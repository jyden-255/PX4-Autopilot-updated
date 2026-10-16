# CrosswindCompensation
# Published by crosswind_compensator module at 50 Hz.
# Consumed by mc_pos_control, fw_lat_lon_ctrl, mc_rate_control.

uint64 timestamp                # [us] Time since system start

# Fused & filtered wind (EKF2 + external anemometer)
float32 wind_north              # [m/s] North component
float32 wind_east               # [m/s] East component
float32 wind_magnitude          # [m/s] Total wind speed

# Gust detection
float32 gust_peak               # [m/s] Peak wind in last 0.4 s sliding window

# Decomposed relative to current track
float32 headwind                # [m/s] + = headwind, - = tailwind
float32 crosswind               # [m/s] perpendicular to track

# Compensation outputs
float32 crab_angle_rad          # [rad] attitude/course lean into wind
float32 throttle_boost          # [0..1] additive throttle for headwind

# Status flags
bool external_sensor_valid      # True if external anemometer is contributing
bool wind_limit_exceeded        # True if wind > CWC_MAX_WIND → request RTL

# TOPICS crosswind_compensation
