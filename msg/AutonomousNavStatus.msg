# AutonomousNavStatus
# Published by autonomous_navigator at 10 Hz.

uint64 timestamp

uint8 nav_state         # NavState enum value
uint8 wp_total          # Total waypoints in mission
uint8 wp_current        # Current waypoint index in optimal path
bool  offline_mode      # True = flying on cached mission (GPS/GCS lost)
bool  wind_hold         # True = holding position due to wind
bool  approval_pending  # True = waiting for human approval
bool  gps_valid
bool  gcs_connected

# State constants
uint8 STATE_IDLE = 0
uint8 STATE_PLANNING = 1
uint8 STATE_AWAITING_APPROVAL = 2
uint8 STATE_EXECUTING = 3
uint8 STATE_HUMAN_OVERRIDE = 4
uint8 STATE_OFFLINE_EXECUTING = 5
uint8 STATE_WIND_HOLD = 6
uint8 STATE_RTL = 7
uint8 STATE_LANDED = 8

# TOPICS autonomous_nav_status
