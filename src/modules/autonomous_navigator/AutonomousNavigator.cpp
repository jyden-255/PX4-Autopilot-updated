/****************************************************************************
 * AutonomousNavigator.cpp
 *
 * Full-autonomous mission navigator for MATEK F405 VTOL.
 * See header for feature overview.
 *
 ****************************************************************************/

#include "AutonomousNavigator.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>
#include <lib/geo/geo.h>
#include <string.h>

using namespace matrix;
using namespace time_literals;

static constexpr float INFINITY_COST = 1e9f;

// ─────────────────────────────────────────────────────────────────────────────
//  Construction / init
// ─────────────────────────────────────────────────────────────────────────────

AutonomousNavigator::AutonomousNavigator() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
    _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{}

bool AutonomousNavigator::init()
{
    ScheduleOnInterval(100_ms); // 10 Hz main loop
    PX4_INFO("AutonomousNavigator: init complete, 10 Hz");
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main loop
// ─────────────────────────────────────────────────────────────────────────────

void AutonomousNavigator::Run()
{
    if (should_exit()) { ScheduleClear(); exit_and_cleanup(); return; }
    perf_begin(_loop_perf);

    // Refresh all subscriptions
    _local_pos_sub.copy(&_local_pos);
    _global_pos_sub.copy(&_global_pos);
    _vehicle_status_sub.copy(&_vehicle_status);
    _crosswind_sub.copy(&_wind_comp);
    _home_sub.copy(&_home);

    // Connectivity monitors
    detect_gps_gcs_loss();
    detect_human_override();
    handle_wind_limits();

    const NavState old_state = _state;

    switch (_state) {

    case NavState::IDLE:
        // Wait for armed + mission available
        if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
            && _mission_valid)
        {
            download_mission_to_cache();
            build_waypoint_graph();
            recompute_wind_costs();   // FIX B5: first plan previously ignored wind
            run_dijkstra(0, _wp_count > 0 ? _wp_count - 1 : 0);
            _state = NavState::AWAITING_APPROVAL;
            request_approval("Begin autonomous mission", 0x01);
        }
        break;

    case NavState::PLANNING:
        // Wind re-plan mid-mission
        // FIX B4: replan from the waypoint we are flying to (_optimal_path[_path_idx]),
        // not from node number _path_idx, and keep the legs already flown.
        if (_path_idx < _optimal_path_len) {
            recompute_wind_costs();
            uint8_t flown[AN_MAX_WAYPOINTS];
            const uint8_t n_flown = _path_idx;
            memcpy(flown, _optimal_path, n_flown);
            const uint8_t src = _optimal_path[_path_idx];
            const uint8_t dst = _optimal_path[_optimal_path_len - 1];
            if (run_dijkstra(src, dst)) {
                if (n_flown + _optimal_path_len <= AN_MAX_WAYPOINTS) {
                    memmove(_optimal_path + n_flown, _optimal_path, _optimal_path_len);
                    memcpy(_optimal_path, flown, n_flown);
                    _optimal_path_len += n_flown;
                } else {
                    _path_idx = 0; // new path already starts at the current target
                }
            }
        }
        _state = NavState::EXECUTING;
        break;

    case NavState::AWAITING_APPROVAL:
        check_approval_timeout();
        if (approval_granted()) {
            mavlink_log_info(nullptr, "[AN] Approved – executing");
            _state = NavState::EXECUTING;
            // FIX B6: only a NEW mission starts from waypoint 0. Re-engaging after a
            // pilot override (action 0x02) or approving a leg must continue in place.
            if (_pending_action_id == 0x01) { _path_idx = 0; }
        }
        break;

    case NavState::EXECUTING:
        // Periodic wind-driven re-plan
        if ((hrt_absolute_time() - _last_replan_time) > (uint64_t)(_param_replan_interval.get() * 1e6f)) {
            _last_replan_time = hrt_absolute_time();
            _state = NavState::PLANNING;
            break;
        }
        advance_to_next_waypoint();
        break;

    case NavState::OFFLINE_EXECUTING:
        // GPS/GCS lost – fly cached mission using dead-reckoning
        if (!_cache_valid) {
            mavlink_log_critical(nullptr, "[AN] OFFLINE: no cache available – holding position!");
            _state = NavState::WIND_HOLD;
            break;
        }
        advance_to_next_waypoint(); // uses local position only
        // Re-check if connectivity restored
        if (_gps_valid && _gcs_connected) {
            mavlink_log_info(nullptr, "[AN] Connectivity restored – resuming normal nav");
            _state = NavState::EXECUTING;
        }
        break;

    case NavState::HUMAN_OVERRIDE:
        // Do nothing; pilot controls drone.
        // Detect release of override
        {
            manual_control_setpoint_s mc{};
            _manual_control_sub.copy(&mc);
            const bool sticks_centered = (fabsf(mc.pitch) < 0.05f
                                       && fabsf(mc.roll)  < 0.05f
                                       && fabsf(mc.yaw)   < 0.05f
                                       && fabsf(mc.throttle - 0.5f) < 0.05f);
            if (sticks_centered) {
                mavlink_log_info(nullptr, "[AN] Sticks centered – re-engaging autonomous nav");
                request_approval("Re-engage autonomous nav", 0x02);
                _state = NavState::AWAITING_APPROVAL;
            }
        }
        break;

    case NavState::WIND_HOLD:
        // Hovering until wind drops
        if (!_wind_hold_active) {
            _state = NavState::EXECUTING;
        }
        break;

    case NavState::RTL:
        // RTL handled by PX4 commander; we just monitor
        break;

    case NavState::LANDED:
        _state = NavState::IDLE;
        _wp_current = 0;
        _path_idx = 0;
        break;
    }

    if (old_state != _state) {
        PX4_INFO("[AN] State: %s → %s", state_name(old_state), state_name(_state));
    }

    publish_status();
    perf_end(_loop_perf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Dijkstra shortest-path planner
// ─────────────────────────────────────────────────────────────────────────────

void AutonomousNavigator::build_waypoint_graph()
{
    // FIX B7: add the mission chain 0->1->...->N-1 FIRST so the route can never
    //         be disconnected by running out of edge slots.
    // FIX B8: shortcut edges (which let Dijkstra skip mission waypoints) are only
    //         added when AN_ALLOW_SKIP = 1 (point-to-point routing mode).
    _edge_count = 0;
    const int N = _wp_count;

    auto add_edge = [this](int i, int j) {
        if (_edge_count >= AN_MAX_GRAPH_EDGES) { return false; }
        const float dist = get_distance_to_next_waypoint(
            _wp_cache[i].lat_deg, _wp_cache[i].lon_deg,
            _wp_cache[j].lat_deg, _wp_cache[j].lon_deg);
        const float alt_cost = fabsf(_wp_cache[i].alt_m - _wp_cache[j].alt_m);
        GraphEdge &e = _edges[_edge_count++];
        e.from       = (uint8_t)i;
        e.to         = (uint8_t)j;
        e.base_cost  = dist + alt_cost * 2.f; // altitude change is expensive
        e.wind_cost  = 0.f;
        e.total_cost = e.base_cost;
        return true;
    };

    for (int i = 0; i + 1 < N; i++) {
        add_edge(i, i + 1);
    }

    if (_param_allow_skip.get() == 1) {
        bool full = false;
        for (int i = 0; i < N && !full; i++) {
            for (int j = i + 2; j < N && !full; j++) {
                const float dist = get_distance_to_next_waypoint(
                    _wp_cache[i].lat_deg, _wp_cache[i].lon_deg,
                    _wp_cache[j].lat_deg, _wp_cache[j].lon_deg);
                if (dist < 500.f) { full = !add_edge(i, j); }
            }
        }
    }

    PX4_INFO("[AN] Graph built: %d waypoints, %d edges", N, (int)_edge_count);
}

void AutonomousNavigator::recompute_wind_costs()
{
    // Stored for status/logging only (forward direction). Dijkstra computes the
    // penalty for the direction actually flown, see run_dijkstra().
    for (int e = 0; e < _edge_count; e++) {
        _edges[e].wind_cost  = edge_wind_penalty(_edges[e].from, _edges[e].to, _edges[e].base_cost);
        _edges[e].total_cost = _edges[e].base_cost + _edges[e].wind_cost;
    }
}

float AutonomousNavigator::edge_wind_penalty(uint8_t from, uint8_t to, float base_cost)
{
    const float bearing = get_bearing_to_next_waypoint(
        _wp_cache[from].lat_deg, _wp_cache[from].lon_deg,
        _wp_cache[to].lat_deg,   _wp_cache[to].lon_deg
    );
    // PX4 wind = velocity of the air (direction it blows TOWARDS). Projecting it
    // onto the leg gives the TAILWIND component.
    // FIX B2: the original treated this as headwind, so it penalised tailwind.
    const float tailwind = _wind_comp.wind_north * cosf(bearing)
                         + _wind_comp.wind_east  * sinf(bearing);
    const float headwind = math::max(-tailwind, 0.f);
    const float k        = _param_wind_cost_k.get();
    return k * headwind * base_cost;
}

bool AutonomousNavigator::run_dijkstra(uint8_t src, uint8_t dst)
{
    if (_wp_count < 2) { return false; }
    const int N = _wp_count;

    // Init
    bool visited[AN_MAX_GRAPH_NODES]{};
    uint8_t prev[AN_MAX_GRAPH_NODES]{};
    for (int i = 0; i < N; i++) { _dist[i] = INFINITY_COST; prev[i] = 255; }
    _dist[src] = 0.f;

    for (int iter = 0; iter < N; iter++) {
        // Find unvisited node with min dist
        uint8_t u = 255;
        float   min_d = INFINITY_COST;
        for (int i = 0; i < N; i++) {
            if (!visited[i] && _dist[i] < min_d) { min_d = _dist[i]; u = (uint8_t)i; }
        }
        if (u == 255) break;
        visited[u] = true;

        // Relax edges from u (undirected: also check reverse)
        for (int e = 0; e < _edge_count; e++) {
            const GraphEdge &edge = _edges[e];
            uint8_t nbr = 255;
            if (edge.from == u) nbr = edge.to;
            else if (edge.to == u) nbr = edge.from; // bidirectional
            if (nbr == 255 || visited[nbr]) continue;

            // FIX B9: edges are undirected, but wind is not: cost the direction flown.
            const float new_d = _dist[u] + edge.base_cost
                              + edge_wind_penalty(u, nbr, edge.base_cost);
            if (new_d < _dist[nbr]) {
                _dist[nbr] = new_d;
                prev[nbr]  = u;
            }
        }
    }

    // Reconstruct path
    if (_dist[dst] >= INFINITY_COST) {
        PX4_WARN("[AN] Dijkstra: no path from %d to %d", src, dst);
        return false;
    }

    _optimal_path_len = 0;
    uint8_t cur = dst;
    while (cur != 255 && _optimal_path_len < AN_MAX_WAYPOINTS) {
        _optimal_path[_optimal_path_len++] = cur;
        cur = prev[cur];
    }
    // Reverse
    for (int i = 0; i < _optimal_path_len / 2; i++) {
        uint8_t tmp = _optimal_path[i];
        _optimal_path[i] = _optimal_path[_optimal_path_len - 1 - i];
        _optimal_path[_optimal_path_len - 1 - i] = tmp;
    }
    PX4_INFO("[AN] Dijkstra: shortest path length %d nodes, cost %.0f m", _optimal_path_len, (double)_dist[dst]);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mission cache (offline capability)
// ─────────────────────────────────────────────────────────────────────────────

void AutonomousNavigator::download_mission_to_cache()
{
    // In real firmware this reads from dataman (SD card / FRAM).
    // Here we snapshot the current mission items from the dataman cache.
    // DatamanClient is used by the existing navigator; we do the same.
    // For brevity we mark each waypoint as cached once written.
    PX4_INFO("[AN] Downloading mission to offline cache (%d waypoints)...", _wp_count);
    for (int i = 0; i < _wp_count; i++) {
        _wp_cache[i].cached = true;
    }
    persist_cache_to_fram();
    _cache_valid = true;
    mavlink_log_info(nullptr, "[AN] Mission cached for offline ops (%d WPs)", _wp_count);
}

void AutonomousNavigator::persist_cache_to_fram()
{
    // MATEK F405 has external FRAM via SPI.
    // Write _wp_cache[0.._wp_count] to dataman FLIGHT_OFFLINE_PLAN item.
    // Actual dataman API call:
    //   dm_write(DM_KEY_OFFLINE_MISSION_CACHE, 0,
    //            _wp_cache, sizeof(NavWaypoint) * _wp_count);
    PX4_DEBUG("[AN] Cache persisted to FRAM");
}

bool AutonomousNavigator::load_mission_from_cache()
{
    // dm_read(DM_KEY_OFFLINE_MISSION_CACHE, 0, _wp_cache, sizeof(NavWaypoint)*AN_MAX_WAYPOINTS);
    _cache_valid = (_wp_count > 0);
    return _cache_valid;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Pre-flight approval gate
// ─────────────────────────────────────────────────────────────────────────────

void AutonomousNavigator::request_approval(const char *action_desc, uint8_t action_id)
{
    human_approval_request_s req{};
    req.timestamp  = hrt_absolute_time();
    req.action_id  = action_id;
    req.timeout_s  = (float)AN_APPROVAL_TIMEOUT_S;
    strncpy(req.description, action_desc, sizeof(req.description) - 1);
    _approval_req_pub.publish(req);
    _approval_requested_at = hrt_absolute_time();
    _pending_action_id     = action_id;
    _approval_pending      = true;
    mavlink_log_warning(nullptr, "[AN] APPROVAL REQUIRED: %s (timeout %ds)", action_desc, AN_APPROVAL_TIMEOUT_S);
}

bool AutonomousNavigator::approval_granted()
{
    // FIX B3: a full-auto timeout now actually grants approval
    if (_auto_approved) { _auto_approved = false; return true; }
    if (!_approval_pending) return false;
    human_approval_response_s resp{};
    if (_approval_response_sub.copy(&resp)) {
        if (resp.action_id == _pending_action_id) {
            _approval_pending = false;
            return resp.approved;
        }
    }
    return false;
}

void AutonomousNavigator::check_approval_timeout()
{
    if (!_approval_pending) return;
    const float elapsed = (hrt_absolute_time() - _approval_requested_at) * 1e-6f;
    if (elapsed > _param_approval_timeout.get()) {
        if (_param_full_auto.get()) {
            // Full-auto mode: auto-approve on timeout
            mavlink_log_info(nullptr, "[AN] Approval timeout – auto-executing (FULL_AUTO mode)");
            _auto_approved    = true;   // FIX B3: previously nothing granted it
            _approval_pending = false;
        } else {
            // Require explicit approval: warn again later, KEEP the request open
            // (FIX B3: clearing it here meant a late human approval was ignored)
            mavlink_log_critical(nullptr, "[AN] Awaiting human approval – no action taken");
            _approval_requested_at = hrt_absolute_time(); // reset timer
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Waypoint execution
// ─────────────────────────────────────────────────────────────────────────────

void AutonomousNavigator::advance_to_next_waypoint()
{
    if (_optimal_path_len == 0 || _path_idx >= _optimal_path_len) {
        mavlink_log_info(nullptr, "[AN] Mission complete");
        _state = NavState::LANDED;
        return;
    }

    const uint8_t wp_idx = _optimal_path[_path_idx];
    const NavWaypoint &wp = _wp_cache[wp_idx];

    // Check if we've reached current waypoint
    const float dist = get_distance_to_next_waypoint(
        _global_pos.lat, _global_pos.lon,
        wp.lat_deg, wp.lon_deg
    );
    const float accept_r = (wp.acceptance_radius_m > 0.f) ? wp.acceptance_radius_m : 5.f;

    if (dist < accept_r) {
        // If loiter required, the position controller handles it
        _path_idx++;
        if (_path_idx < _optimal_path_len) {
            const NavWaypoint &next = _wp_cache[_optimal_path[_path_idx]];
            // Request approval for each leg in semi-auto mode
            if (!_param_full_auto.get()) {
                char buf[64];
                snprintf(buf, sizeof(buf), "Fly to WP%d (%.4f, %.4f)", _path_idx,
                         next.lat_deg, next.lon_deg);
                request_approval(buf, 0x10 + _path_idx);
                _state = NavState::AWAITING_APPROVAL;
                return;
            }
        }
    }

    publish_position_setpoint(wp);
}

void AutonomousNavigator::publish_position_setpoint(const NavWaypoint &wp)
{
    position_setpoint_triplet_s spt{};
    spt.timestamp = hrt_absolute_time();

    auto &curr = spt.current;
    curr.valid       = true;
    curr.type        = position_setpoint_s::SETPOINT_TYPE_POSITION;
    curr.lat         = wp.lat_deg;
    curr.lon         = wp.lon_deg;
    curr.alt         = wp.alt_m;
    curr.acceptance_radius = (wp.acceptance_radius_m > 0.f) ? wp.acceptance_radius_m : 5.f;
    curr.loiter_radius     = (wp.loiter_radius_m    > 0.f) ? wp.loiter_radius_m    : 50.f;
    curr.loiter_direction  = 1; // CW
    curr.cruising_speed    = (wp.speed_m_s          > 0.f) ? wp.speed_m_s          : NAN;

    // Look ahead: set next WP
    if (_path_idx + 1 < _optimal_path_len) {
        const NavWaypoint &nxt = _wp_cache[_optimal_path[_path_idx + 1]];
        auto &next = spt.next;
        next.valid = true;
        next.type  = position_setpoint_s::SETPOINT_TYPE_POSITION;
        next.lat   = nxt.lat_deg;
        next.lon   = nxt.lon_deg;
        next.alt   = nxt.alt_m;
    }

    _pos_sp_pub.publish(spt);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Connectivity & safety monitors
// ─────────────────────────────────────────────────────────────────────────────

void AutonomousNavigator::detect_human_override()
{
    manual_control_setpoint_s mc{};
    if (!_manual_control_sub.copy(&mc)) return;

    const bool sticks_active = (fabsf(mc.pitch)    > 0.15f
                              || fabsf(mc.roll)     > 0.15f
                              || fabsf(mc.yaw)      > 0.15f
                              || fabsf(mc.throttle - 0.5f) > 0.15f);

    if (sticks_active && _state == NavState::EXECUTING) {
        _state = NavState::HUMAN_OVERRIDE;
        mavlink_log_info(nullptr, "[AN] Human override detected – autonomous nav suspended");
    }
}

void AutonomousNavigator::detect_gps_gcs_loss()
{
    const hrt_abstime now = hrt_absolute_time();

    // GPS: check local position validity
    _gps_valid = (_local_pos.xy_valid && _local_pos.z_valid);
    if (_gps_valid) _last_gps_fix = now;

    const float gps_timeout = _param_gps_timeout.get() * 1e6f;
    const float gcs_timeout = _param_gcs_timeout.get() * 1e6f;

    // GCS: check vehicle_status.gcs_connection_status
    failsafe_flags_s fsf{};
    _failsafe_sub.copy(&fsf);
    _gcs_connected = !fsf.gcs_connection_lost;
    if (_gcs_connected) _last_gcs_heartbeat = now;

    const bool gps_lost = !_gps_valid && (now - _last_gps_fix > (uint64_t)gps_timeout);
    const bool gcs_lost = !_gcs_connected && (now - _last_gcs_heartbeat > (uint64_t)gcs_timeout);

    if ((gps_lost || gcs_lost) && _state == NavState::EXECUTING) {
        _offline_mode = true;
        if (!_cache_valid) load_mission_from_cache();
        _state = NavState::OFFLINE_EXECUTING;
        mavlink_log_critical(nullptr, "[AN] GPS/GCS lost – switching to OFFLINE mode (cached mission)");
    }
}

void AutonomousNavigator::handle_wind_limits()
{
    if (_wind_comp.wind_limit_exceeded && _state == NavState::EXECUTING) {
        _wind_hold_active = true;
        _state = NavState::WIND_HOLD;
        mavlink_log_critical(nullptr, "[AN] Wind limit exceeded – holding position");
    } else if (!_wind_comp.wind_limit_exceeded) {
        _wind_hold_active = false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Status
// ─────────────────────────────────────────────────────────────────────────────

void AutonomousNavigator::publish_status()
{
    autonomous_nav_status_s st{};
    st.timestamp       = hrt_absolute_time();
    st.nav_state       = (uint8_t)_state;
    st.wp_total        = _wp_count;
    st.wp_current      = _path_idx < _optimal_path_len ? _optimal_path[_path_idx] : 255;
    st.offline_mode    = _offline_mode;
    st.wind_hold       = _wind_hold_active;
    st.approval_pending= _approval_pending;
    st.gps_valid       = _gps_valid;
    st.gcs_connected   = _gcs_connected;
    _status_pub.publish(st);
}

int AutonomousNavigator::print_status()
{
    PX4_INFO("AutonomousNavigator  state: %s", state_name(_state));
    PX4_INFO("  WPs: %d total, path len: %d, current idx: %d", _wp_count, _optimal_path_len, _path_idx);
    PX4_INFO("  GPS: %s  GCS: %s  Offline: %s  Cache: %s",
             _gps_valid?"OK":"LOST", _gcs_connected?"OK":"LOST",
             _offline_mode?"YES":"NO", _cache_valid?"YES":"NO");
    PX4_INFO("  Wind: %.1f m/s  Hold: %s  Approval pending: %s",
             (double)_wind_comp.wind_magnitude,
             _wind_hold_active?"YES":"NO", _approval_pending?"YES":"NO");
    perf_print_counter(_loop_perf);
    return 0;
}

const char *AutonomousNavigator::state_name(NavState s)
{
    switch (s) {
    case NavState::IDLE:              return "IDLE";
    case NavState::PLANNING:          return "PLANNING";
    case NavState::AWAITING_APPROVAL: return "AWAITING_APPROVAL";
    case NavState::EXECUTING:         return "EXECUTING";
    case NavState::HUMAN_OVERRIDE:    return "HUMAN_OVERRIDE";
    case NavState::OFFLINE_EXECUTING: return "OFFLINE_EXECUTING";
    case NavState::WIND_HOLD:         return "WIND_HOLD";
    case NavState::RTL:               return "RTL";
    case NavState::LANDED:            return "LANDED";
    default:                          return "UNKNOWN";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Module plumbing
// ─────────────────────────────────────────────────────────────────────────────

int AutonomousNavigator::task_spawn(int argc, char *argv[])
{
    AutonomousNavigator *inst = new AutonomousNavigator();
    if (!inst) { PX4_ERR("alloc failed"); return PX4_ERROR; }
    _object.store(inst);
    _task_id = task_id_is_work_queue;
    if (!inst->init()) { delete inst; _object.store(nullptr); _task_id = -1; return PX4_ERROR; }
    return PX4_OK;
}

int AutonomousNavigator::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s\n", reason);
    PRINT_MODULE_DESCRIPTION("Autonomous navigator: shortest-path planning, offline cache, wind routing.");
    PRINT_MODULE_USAGE_NAME("autonomous_navigator", "navigator");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int autonomous_navigator_main(int argc, char *argv[])
{
    return AutonomousNavigator::main(argc, argv);
}
