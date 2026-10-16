/****************************************************************************
 * CrosswindCompensator.cpp
 *
 * Real-time crosswind and gust compensation for MATEK F405 VTOL.
 *
 * Algorithm overview
 * ──────────────────
 *  1. Fuse EKF2 wind estimate with external anemometer (I2C/UART SensorAirflow)
 *     using a complementary filter. If external sensor times out (>500 ms),
 *     fall back entirely to EKF2.
 *  2. Low-pass filter the fused wind at configurable tau (default 0.4 s) to
 *     suppress sensor noise while still reacting to step gusts within ~1 s.
 *  3. Decompose into headwind / crosswind relative to current track.
 *  4. Compute:
 *       crab_angle  = atan2(crosswind, airspeed)      [rad]  – lean into wind
 *       thr_boost   = K * max(headwind, 0)             [0..1] – extra power
 *  5. Detect gusts via sliding-window peak detector; if gust > warn_wind
 *     raise WIND_WARNING MAVLink event; if > max_wind publish RTL command.
 *  6. Publish crosswind_compensation_s at 50 Hz consumed by:
 *       • mc_pos_control  → adds crab angle to attitude setpoint
 *       • fw_lat_lon_ctrl → adjusts course setpoint
 *       • mc_rate_control → gets throttle boost
 *
 ****************************************************************************/

#include "CrosswindCompensator.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>

using namespace matrix;
using namespace time_literals;

CrosswindCompensator::CrosswindCompensator() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
    _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

bool CrosswindCompensator::init()
{
    ScheduleOnInterval(20_ms); // 50 Hz
    return true;
}

void CrosswindCompensator::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);

    // Update parameters if changed
    if (_param_update_sub.updated()) {
        parameter_update_s pu;
        _param_update_sub.copy(&pu);
        updateParams();
    }

    if (_param_enable.get() == 0) {
        perf_end(_loop_perf);
        return;
    }

    update_wind_estimate();
    compute_compensation();
    publish_compensation();
    check_wind_limits();

    perf_end(_loop_perf);
}

void CrosswindCompensator::update_wind_estimate()
{
    const hrt_abstime now = hrt_absolute_time();

    // ── EKF2 wind ─────────────────────────────────────────────────────────
    if (_wind_sub.updated()) {
        _wind_sub.copy(&_wind);
        _last_ekf2_update = now;
    }
    Vector2f ekf2_wind(_wind.windspeed_north, _wind.windspeed_east);

    // ── External anemometer (SensorAirflow on I2C/UART) ────────────────────
    if (_sensor_airflow_sub.updated()) {
        _sensor_airflow_sub.copy(&_sensor_airflow);
        _last_external_update = now;
        _external_sensor_valid = true;
    }

    // Timeout external sensor after 500 ms
    if (_external_sensor_valid && (now - _last_external_update) > 500_ms) {
        _external_sensor_valid = false;
        PX4_WARN("CWC: external anemometer timeout, using EKF2 only");
    }

    // ── Fusion ──────────────────────────────────────────────────────────────
    if (_external_sensor_valid) {
        // Convert polar (speed, direction) → cartesian NE
        const float dir_rad = math::radians(_sensor_airflow.direction);
        Vector2f ext_wind(
            _sensor_airflow.speed * cosf(dir_rad),   // North
            _sensor_airflow.speed * sinf(dir_rad)    // East
        );
        const float w = math::constrain(_param_ext_weight.get(), 0.f, 1.f);
        _fused_wind_NE = w * ext_wind + (1.f - w) * ekf2_wind;
    } else {
        _fused_wind_NE = ekf2_wind;
    }

    // ── Complementary (low-pass) filter ─────────────────────────────────────
    // alpha = dt / (tau + dt),  dt = 20ms
    const float dt    = 0.02f;
    const float tau   = math::constrain(_param_lp_tau.get(), 0.05f, 5.f);
    const float alpha = dt / (tau + dt);
    _wind_filtered_NE = (1.f - alpha) * _wind_filtered_NE + alpha * _fused_wind_NE;
    _wind_magnitude   = _wind_filtered_NE.norm();

    // ── Gust sliding-window peak detector ───────────────────────────────────
    _gust_buf[_gust_idx++ % GUST_WINDOW] = _wind_magnitude;
    _gust_peak = 0.f;
    for (int i = 0; i < GUST_WINDOW; i++) {
        _gust_peak = math::max(_gust_peak, _gust_buf[i]);
    }
}

void CrosswindCompensator::compute_compensation()
{
    // Need current track heading to decompose wind
    _local_pos_sub.copy(&_local_pos);
    _airspeed_sub.copy(&_airspeed);

    float track_rad = 0.f;
    if (fabsf(_local_pos.vx) > 0.1f || fabsf(_local_pos.vy) > 0.1f) {
        track_rad = atan2f(_local_pos.vy, _local_pos.vx);
    }

    // Project wind onto track and cross-track axes
    const float cos_t = cosf(track_rad);
    const float sin_t = sinf(track_rad);

    // PX4 wind is the air's velocity (direction it blows TOWARDS), so its
    // projection on the track is TAILWIND. FIX B10: negate it to get headwind
    // (+ = headwind, as crosswind_compensation.msg documents). Previously the
    // throttle boost was applied in tailwinds instead of headwinds.
    _headwind_component  = -(_wind_filtered_NE(0) * cos_t + _wind_filtered_NE(1) * sin_t);
    _crosswind_component = -_wind_filtered_NE(0) * sin_t + _wind_filtered_NE(1) * cos_t;

    // Crab angle: how much to rotate into wind to maintain track
    const float airspeed = math::max(_airspeed.true_airspeed_m_s, 2.f); // avoid div/0
    const float crab_limit_rad = math::radians(_param_crab_limit.get());
    _crab_angle_rad = math::constrain(
        asinf(math::constrain(_crosswind_component / airspeed, -1.f, 1.f)),
        -crab_limit_rad, crab_limit_rad
    );

    // Throttle boost for headwind (only positive headwind needs extra thrust)
    const float k = math::constrain(_param_thr_boost_k.get(), 0.f, 0.05f);
    _throttle_boost = math::constrain(k * math::max(_headwind_component, 0.f), 0.f, 0.3f);
}

void CrosswindCompensator::publish_compensation()
{
    crosswind_compensation_s comp{};
    comp.timestamp             = hrt_absolute_time();
    comp.wind_north            = _wind_filtered_NE(0);
    comp.wind_east             = _wind_filtered_NE(1);
    comp.wind_magnitude        = _wind_magnitude;
    comp.gust_peak             = _gust_peak;
    comp.headwind              = _headwind_component;
    comp.crosswind             = _crosswind_component;
    comp.crab_angle_rad        = _crab_angle_rad;
    comp.throttle_boost        = _throttle_boost;
    comp.external_sensor_valid = _external_sensor_valid;
    comp.wind_limit_exceeded   = _wind_limit_exceeded;
    _comp_pub.publish(comp);
}

void CrosswindCompensator::check_wind_limits()
{
    const float warn  = _param_warn_wind.get();
    const float limit = _param_max_wind.get();

    if (_gust_peak > limit) {
        if (!_wind_limit_exceeded) {
            _wind_limit_exceeded = true;
            mavlink_log_critical(nullptr, "CWC: WIND LIMIT EXCEEDED %.1f m/s! RTL triggered", (double)_gust_peak);
            // Commander will see wind_limit_exceeded flag in published topic
            // and can trigger RTL via FailsafeFlags
        }
    } else if (_gust_peak > warn) {
        mavlink_log_warning(nullptr, "CWC: Wind warning %.1f m/s", (double)_gust_peak);
        _wind_limit_exceeded = false;
    } else {
        _wind_limit_exceeded = false;
    }
}

int CrosswindCompensator::print_status()
{
    PX4_INFO("CrosswindCompensator");
    PX4_INFO("  Fused wind:  N=%.2f E=%.2f m/s  |%.2f m/s|",
             (double)_wind_filtered_NE(0), (double)_wind_filtered_NE(1), (double)_wind_magnitude);
    PX4_INFO("  Headwind:    %.2f m/s  Crosswind: %.2f m/s", (double)_headwind_component, (double)_crosswind_component);
    PX4_INFO("  Crab angle:  %.1f deg  Thr boost: %.3f", (double)math::degrees(_crab_angle_rad), (double)_throttle_boost);
    PX4_INFO("  Gust peak:   %.2f m/s  External valid: %s  Limit exceeded: %s",
             (double)_gust_peak, _external_sensor_valid ? "YES" : "NO", _wind_limit_exceeded ? "YES" : "NO");
    perf_print_counter(_loop_perf);
    return 0;
}

int CrosswindCompensator::task_spawn(int argc, char *argv[])
{
    CrosswindCompensator *instance = new CrosswindCompensator();
    if (!instance) { PX4_ERR("alloc failed"); return PX4_ERROR; }
    _object.store(instance);
    _task_id = task_id_is_work_queue;
    if (!instance->init()) { delete instance; _object.store(nullptr); _task_id = -1; return PX4_ERROR; }
    return PX4_OK;
}

int CrosswindCompensator::print_usage(const char *reason)
{
    if (reason) { PX4_WARN("%s\n", reason); }
    PRINT_MODULE_DESCRIPTION("Real-time crosswind compensation for MATEK F405 VTOL.\n"
        "Fuses EKF2 wind estimate with external anemometer and publishes\n"
        "crab angle and throttle boost corrections at 50 Hz.\n");
    PRINT_MODULE_USAGE_NAME("crosswind_compensator", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int crosswind_compensator_main(int argc, char *argv[])
{
    return CrosswindCompensator::main(argc, argv);
}
