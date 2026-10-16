/****************************************************************************
 * CrosswindCompensator.hpp
 *
 * Real-time crosswind compensation for MATEK F405 VTOL
 *
 * Subscribes to:
 *   - wind (from EKF2)
 *   - external_wind_sensor (SensorAirflow from physical anemometer on I2C/UART)
 *   - vehicle_local_position
 *   - vehicle_attitude
 *
 * Publishes:
 *   - crosswind_compensation (custom uORB) → consumed by mc_pos_control & fw_lat_lon
 *   - wind_compensation_status
 *
 * Design:
 *   Fuses EKF2 wind estimate with live external anemometer data via a
 *   complementary filter. At 50 Hz the compensated wind vector is used to:
 *     (a) Lean the multicopter attitude into the wind (crab-angle correction)
 *     (b) Adjust fixed-wing course setpoint via crab angle
 *     (c) Boost throttle when headwind exceeds threshold
 *     (d) Trigger RTL/hold when wind exceeds safe operational limits
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/wind.h>
#include <uORB/topics/sensor_airflow.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/crosswind_compensation.h>

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>

using namespace matrix;
using namespace time_literals;

class CrosswindCompensator : public ModuleBase<CrosswindCompensator>,
                              public ModuleParams,
                              public px4::ScheduledWorkItem
{
public:
    CrosswindCompensator();
    ~CrosswindCompensator() override = default;

    static int task_spawn(int argc, char *argv[]);
    static CrosswindCompensator *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    int print_status() override;

private:
    void Run() override;
    void parameters_update();
    void update_wind_estimate();
    void compute_compensation();
    void publish_compensation();
    void check_wind_limits();

    // ── uORB Subscriptions ──────────────────────────────────────────────────
    uORB::Subscription _wind_sub{ORB_ID(wind)};
    uORB::Subscription _sensor_airflow_sub{ORB_ID(sensor_airflow)};
    uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _airspeed_sub{ORB_ID(airspeed_validated)};
    uORB::Subscription _param_update_sub{ORB_ID(parameter_update)};

    // ── uORB Publications ───────────────────────────────────────────────────
    uORB::Publication<crosswind_compensation_s> _comp_pub{ORB_ID(crosswind_compensation)};

    // ── Cached topic data ────────────────────────────────────────────────────
    wind_s                  _wind{};
    sensor_airflow_s        _sensor_airflow{};
    vehicle_local_position_s _local_pos{};
    vehicle_attitude_s      _attitude{};
    vehicle_status_s        _vehicle_status{};
    airspeed_validated_s    _airspeed{};

    // ── Wind fusion state ────────────────────────────────────────────────────
    Vector2f _fused_wind_NE{0.f, 0.f};       // [m/s] fused north/east
    Vector2f _wind_filtered_NE{0.f, 0.f};    // low-pass filtered
    float    _wind_magnitude{0.f};            // [m/s]
    float    _crosswind_component{0.f};       // [m/s] perpendicular to track
    float    _headwind_component{0.f};        // [m/s] along track (+ = head)
    float    _crab_angle_rad{0.f};            // [rad] lean / course correction
    float    _throttle_boost{0.f};            // [0..1] additive throttle
    bool     _wind_limit_exceeded{false};
    bool     _external_sensor_valid{false};
    hrt_abstime _last_external_update{0};
    hrt_abstime _last_ekf2_update{0};

    // ── Gust detection (sliding window) ─────────────────────────────────────
    static constexpr int GUST_WINDOW = 20;     // samples @50Hz = 0.4s
    float  _gust_buf[GUST_WINDOW]{};
    int    _gust_idx{0};
    float  _gust_peak{0.f};

    // ── Parameters ───────────────────────────────────────────────────────────
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::CWC_LP_TAU>)       _param_lp_tau,         // complementary filter time constant [s]
        (ParamFloat<px4::params::CWC_EXT_WEIGHT>)   _param_ext_weight,     // weight for external anemometer [0..1]
        (ParamFloat<px4::params::CWC_MAX_WIND>)     _param_max_wind,       // max safe wind [m/s] → triggers RTL
        (ParamFloat<px4::params::CWC_WARN_WIND>)    _param_warn_wind,      // warning threshold [m/s]
        (ParamFloat<px4::params::CWC_THR_BOOST_K>)  _param_thr_boost_k,   // throttle boost gain per m/s headwind
        (ParamFloat<px4::params::CWC_CRAB_LIMIT>)   _param_crab_limit,     // max crab angle [deg]
        (ParamInt<px4::params::CWC_ENABLE>)         _param_enable          // 0=disabled, 1=enabled
    )

    perf_counter_t _loop_perf;
};
