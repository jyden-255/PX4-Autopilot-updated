/****************************************************************************
 * AutonomousNavigator.hpp
 *
 * Fully-autonomous mission navigator for MATEK F405 VTOL.
 *
 * Key capabilities
 * ────────────────
 *  1. SHORTEST-PATH PLANNER
 *     Dijkstra over a geofence-aware waypoint graph.  Pre-computes and
 *     caches the optimal path before takeoff.  Mid-mission replanning
 *     if wind or obstacle data changes the cost landscape.
 *
 *  2. OFFLINE MISSION CACHE
 *     Full mission is downloaded to FRAM/SD during pre-arm phase.
 *     On GCS/GPS loss the drone executes from the local cache autonomously.
 *     All waypoints include altitude, speed, loiter radius, and wind-adjusted
 *     cross-track tolerances.
 *
 *  3. PRE-FLIGHT APPROVAL GATE
 *     Every autonomous action is posted to the "approval queue" (published
 *     as human_approval_request_s).  The action executes only when:
 *       (a) human approves via GCS/RC within timeout, OR
 *       (b) drone is in full-auto mode AND timeout expires
 *     Humans can override any queued action at any time.
 *
 *  4. MID-FLIGHT HUMAN OVERRIDE
 *     RC sticks / GCS position override immediately suspend autonomous nav
 *     and transfer control; releasing back triggers smooth re-engagement.
 *
 *  5. WIND-ADJUSTED ROUTING
 *     Path costs are weighted by headwind penalty so the planner prefers
 *     legs with tailwind or crosswind over strong headwind legs.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/crosswind_compensation.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/human_approval_request.h>
#include <uORB/topics/human_approval_response.h>
#include <uORB/topics/autonomous_nav_status.h>
#include <uORB/topics/failsafe_flags.h>

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>

#include <cstdint>
#include <array>

using namespace time_literals;

// ── Compile-time limits (MATEK F405 RAM budget: ~192 KB) ─────────────────────
static constexpr int AN_MAX_WAYPOINTS    = 64;
static constexpr int AN_MAX_GRAPH_NODES  = 64;
static constexpr int AN_MAX_GRAPH_EDGES  = 256;
static constexpr int AN_APPROVAL_TIMEOUT_S = 10; // s before auto-executing

struct NavWaypoint {
    double lat_deg;
    double lon_deg;
    float  alt_m;
    float  acceptance_radius_m;
    float  loiter_radius_m;      // 0 = fly-through
    float  speed_m_s;            // 0 = use default
    uint8_t nav_cmd;             // MAV_CMD_NAV_*
    bool   cached;               // stored in FRAM
};

struct GraphEdge {
    uint8_t from;
    uint8_t to;
    float   base_cost;       // Euclidean distance [m]
    float   wind_cost;       // updated each cycle from wind topic
    float   total_cost;      // base + wind penalty
};

enum class NavState : uint8_t {
    IDLE = 0,
    PLANNING,           // running Dijkstra
    AWAITING_APPROVAL,  // queued, waiting for human OK
    EXECUTING,          // flying waypoints
    HUMAN_OVERRIDE,     // pilot has taken control
    OFFLINE_EXECUTING,  // GPS/GCS lost, flying cached mission
    WIND_HOLD,          // holding position: wind too strong
    RTL,
    LANDED
};

class AutonomousNavigator : public ModuleBase<AutonomousNavigator>,
                             public ModuleParams,
                             public px4::ScheduledWorkItem
{
public:
    AutonomousNavigator();
    ~AutonomousNavigator() override = default;

    static int task_spawn(int argc, char *argv[]);
    static AutonomousNavigator *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    int  print_status() override;

private:
    void Run() override;

    // ── Planning ─────────────────────────────────────────────────────────────
    void  build_waypoint_graph();
    void  recompute_wind_costs();
    bool  run_dijkstra(uint8_t src, uint8_t dst);
    // FIX B2/B9: penalty for the direction actually flown (from -> to)
    float edge_wind_penalty(uint8_t from, uint8_t to, float base_cost);

    // ── Mission cache ─────────────────────────────────────────────────────────
    void  download_mission_to_cache();
    bool  load_mission_from_cache();
    void  persist_cache_to_fram();

    // ── Approval gate ─────────────────────────────────────────────────────────
    void  request_approval(const char *action_desc, uint8_t action_id);
    bool  approval_granted();
    void  check_approval_timeout();

    // ── Execution ─────────────────────────────────────────────────────────────
    void  advance_to_next_waypoint();
    void  publish_position_setpoint(const NavWaypoint &wp);
    void  detect_human_override();
    void  detect_gps_gcs_loss();
    void  handle_wind_limits();

    // ── Status ────────────────────────────────────────────────────────────────
    void  publish_status();

    // ── uORB ─────────────────────────────────────────────────────────────────
    uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _mission_sub{ORB_ID(mission)};
    uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
    uORB::Subscription _crosswind_sub{ORB_ID(crosswind_compensation)};
    uORB::Subscription _home_sub{ORB_ID(home_position)};
    uORB::Subscription _approval_response_sub{ORB_ID(human_approval_response)};
    uORB::Subscription _failsafe_sub{ORB_ID(failsafe_flags)};

    uORB::Publication<position_setpoint_triplet_s> _pos_sp_pub{ORB_ID(position_setpoint_triplet)};
    uORB::Publication<human_approval_request_s>    _approval_req_pub{ORB_ID(human_approval_request)};
    uORB::Publication<autonomous_nav_status_s>     _status_pub{ORB_ID(autonomous_nav_status)};

    // ── State ─────────────────────────────────────────────────────────────────
    NavState _state{NavState::IDLE};
    NavState _prev_state{NavState::IDLE};
    const char *state_name(NavState s);

    // ── Mission cache (FRAM mirror) ───────────────────────────────────────────
    NavWaypoint _wp_cache[AN_MAX_WAYPOINTS]{};
    uint8_t     _wp_count{0};
    uint8_t     _wp_current{0};

    // ── Optimal path from Dijkstra ─────────────────────────────────────────────
    uint8_t _optimal_path[AN_MAX_WAYPOINTS]{};
    uint8_t _optimal_path_len{0};
    uint8_t _path_idx{0};

    // ── Graph ──────────────────────────────────────────────────────────────────
    GraphEdge _edges[AN_MAX_GRAPH_EDGES]{};
    uint16_t  _edge_count{0};   // FIX B7: was uint8_t
    float     _dist[AN_MAX_GRAPH_NODES]{};  // Dijkstra dist array

    // ── Approval gate ──────────────────────────────────────────────────────────
    hrt_abstime _approval_requested_at{0};
    uint8_t     _pending_action_id{0};
    bool        _approval_pending{false};
    bool        _auto_approved{false};    // FIX B3: set when full-auto timeout fires
    bool        _full_auto_mode{false};

    // ── Connectivity monitoring ────────────────────────────────────────────────
    hrt_abstime _last_gcs_heartbeat{0};
    hrt_abstime _last_gps_fix{0};
    bool        _gps_valid{false};
    bool        _gcs_connected{false};
    bool        _offline_mode{false};

    // ── Wind ──────────────────────────────────────────────────────────────────
    crosswind_compensation_s _wind_comp{};
    bool                     _wind_hold_active{false};

    // ── Misc ──────────────────────────────────────────────────────────────────
    vehicle_local_position_s  _local_pos{};
    vehicle_global_position_s _global_pos{};
    vehicle_status_s          _vehicle_status{};
    home_position_s           _home{};

    hrt_abstime _last_replan_time{0};
    bool        _mission_valid{false};
    bool        _cache_valid{false};

    perf_counter_t _loop_perf;

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::AN_REPLAN_INTERVAL>) _param_replan_interval, // s between wind-driven replans
        (ParamFloat<px4::params::AN_WIND_COST_K>)     _param_wind_cost_k,     // headwind cost multiplier
        (ParamFloat<px4::params::AN_GCS_TIMEOUT>)     _param_gcs_timeout,     // s before GCS considered lost
        (ParamFloat<px4::params::AN_GPS_TIMEOUT>)     _param_gps_timeout,     // s before GPS considered lost
        (ParamInt<px4::params::AN_FULL_AUTO>)         _param_full_auto,       // 0=require approval, 1=full auto
        (ParamFloat<px4::params::AN_APPROVAL_TO>)     _param_approval_timeout, // s before auto-execute in full-auto
        (ParamInt<px4::params::AN_ALLOW_SKIP>)        _param_allow_skip        // FIX B8: 0 = fly every waypoint in order
    )
};
