# MATEK F405-VTOL — PX4 Enhancement Pack
## Fully Autonomous VTOL Drone | Crosswind Compensation | Offline Mission Cache

---

## What This Is

A drop-in set of new modules, board files, and uORB message definitions that layer on top of PX4-Autopilot-main to give your MATEK F405 VTOL:

1. **Shortest-path planning** — Dijkstra on a geofence-aware waypoint graph, wind-cost weighted
2. **Offline mission cache** — full mission persisted to FRAM/SD; flies autonomously if GCS or GPS is lost
3. **Human approval gate** — every autonomous action is broadcast over MAVLink before execution; humans can approve, reject, or override at any time
4. **Real-time crosswind compensation** — fuses EKF2 wind + external anemometer; outputs crab angle + throttle boost at 50 Hz
5. **Gust detection** — sliding-window peak detector; warning at configurable threshold, RTL above max wind

---

## File Map

```
drone_f405_vtol/
├── boards/matek/f405-vtol/
│   ├── default.cmake              ← Board module list (copy into PX4 boards/matek/)
│   └── init/rc.board_defaults     ← All default parameters for F405-VTOL
│
├── src/modules/
│   ├── crosswind_compensator/
│   │   ├── CrosswindCompensator.hpp   ← Module header
│   │   ├── CrosswindCompensator.cpp   ← 50 Hz wind fusion + crab angle computation
│   │   ├── CMakeLists.txt
│   │   └── module.yaml                ← Parameter definitions (QGroundControl shows these)
│   │
│   └── autonomous_navigator/
│       ├── AutonomousNavigator.hpp    ← State machine + graph structures
│       ├── AutonomousNavigator.cpp    ← Dijkstra, offline cache, approval gate
│       ├── CMakeLists.txt
│       └── module.yaml
│
└── msg/
    ├── crosswind_compensation.msg     ← uORB: wind compensation outputs
    ├── autonomous_nav_status.msg      ← uORB: navigator state machine
    ├── human_approval_request.msg     ← uORB: pre-action approval broadcast
    └── human_approval_response.msg    ← uORB: GCS/RC approval confirmation
```

---

## How To Install Into PX4

```bash
# 1. Clone PX4 (or use your existing checkout)
git clone https://github.com/PX4/PX4-Autopilot --recursive
cd PX4-Autopilot

# 2. Copy board definition
cp -r drone_f405_vtol/boards/matek/f405-vtol  boards/matek/

# 3. Copy new modules
cp -r drone_f405_vtol/src/modules/crosswind_compensator  src/modules/
cp -r drone_f405_vtol/src/modules/autonomous_navigator   src/modules/

# 4. Copy new uORB messages
cp drone_f405_vtol/msg/*.msg  msg/

# 5. Register the new messages in msg/CMakeLists.txt
#    Add these lines to the set(msg_files ...) block:
#      crosswind_compensation.msg
#      autonomous_nav_status.msg
#      human_approval_request.msg
#      human_approval_response.msg

# 6. Build for F405-VTOL
make matek_f405-vtol

# 7. Flash
make matek_f405-vtol upload
```

---

## Feature Details

### 1 — Crosswind Compensator (`crosswind_compensator` module)

Runs at **50 Hz** on the nav_and_controllers work queue.

**Data sources:**
- `wind` uORB topic (EKF2 Kalman filter estimate, always available)
- `sensor_airflow` uORB topic (external anemometer via I2C/UART, optional)

**Algorithm:**
```
fused_wind = ext_weight × anemometer + (1 − ext_weight) × EKF2
filtered   = complementary_filter(fused_wind, tau=CWC_LP_TAU)
headwind   = dot(filtered, track_unit_vector)
crosswind  = cross(filtered, track_unit_vector)
crab_angle = arcsin(crosswind / TAS)          ← fed to attitude setpoint
thr_boost  = CWC_THR_BOOST_K × max(headwind, 0)  ← fed to throttle
```

**Gust detection:** 20-sample sliding window (~0.4 s at 50 Hz). If peak > `CWC_WARN_WIND` → MAVLink warning. If peak > `CWC_MAX_WIND` → sets `wind_limit_exceeded` flag → autonomous_navigator triggers RTL.

**External anemometer wiring (example: Calypso Ultrasonic or FT742-SM):**
- Connect to UART4 (GPS1 port) or I2C1
- PX4 reads it as `SensorAirflow` via `drivers/wind_sensor` (standard PX4 driver)
- Module auto-detects timeout (500 ms) and falls back to EKF2

---

### 2 — Autonomous Navigator (`autonomous_navigator` module)

Runs at **10 Hz** on the nav_and_controllers work queue.

**State machine:**
```
IDLE → PLANNING → AWAITING_APPROVAL → EXECUTING ⇄ HUMAN_OVERRIDE
                                           ↓              ↑
                                   OFFLINE_EXECUTING  (sticks centered)
                                           ↓
                                      WIND_HOLD
                                           ↓
                                          RTL → LANDED → IDLE
```

**Shortest-path (Dijkstra):**
- Builds a bidirectional graph from mission waypoints
- Edge cost = Euclidean distance + altitude change penalty + wind headwind penalty
- Re-runs every `AN_REPLAN_INTERVAL` seconds with fresh wind costs
- Path stored in `_optimal_path[]` array, visited in order

**Offline cache:**
- On arm: full mission copied to `_wp_cache[]` array and persisted to dataman (FRAM)
- If GPS lost (>`AN_GPS_TIMEOUT` s) OR GCS lost (>`AN_GCS_TIMEOUT` s): switches to `OFFLINE_EXECUTING`
- Flies cached waypoints using local position estimate (dead-reckoning from IMU+baro)
- Reconnection auto-detected; resumes normal mode

**Human approval gate:**
- Before each leg (semi-auto mode `AN_FULL_AUTO=0`): publishes `human_approval_request` with description + timeout
- GCS plugin or RC switch publishes `human_approval_response`
- If no response within `AN_APPROVAL_TO` seconds AND `AN_FULL_AUTO=1`: auto-executes
- Humans can take stick control at any time (>15% deflection = immediate override)

---

### 3 — Key Parameters (QGroundControl → Parameters tab)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CWC_ENABLE` | 1 | Enable crosswind compensator |
| `CWC_MAX_WIND` | 15 m/s | Wind limit → triggers RTL |
| `CWC_WARN_WIND` | 10 m/s | Warning threshold |
| `CWC_LP_TAU` | 0.4 s | Wind filter time constant |
| `CWC_EXT_WEIGHT` | 0.6 | Anemometer vs EKF2 blend |
| `CWC_THR_BOOST_K` | 0.012 | Throttle boost per m/s headwind |
| `CWC_CRAB_LIMIT` | 25 deg | Max crab angle |
| `AN_FULL_AUTO` | 0 | 0=require approval, 1=full auto |
| `AN_APPROVAL_TO` | 10 s | Auto-execute timeout (full-auto) |
| `AN_GCS_TIMEOUT` | 10 s | GCS loss → offline mode |
| `AN_GPS_TIMEOUT` | 5 s | GPS loss → offline mode |
| `AN_REPLAN_INTERVAL` | 30 s | Wind-driven re-plan frequency |
| `AN_WIND_COST_K` | 0.5 | Headwind path cost multiplier |

---

### 4 — Human Control (Always Available)

At any moment, the pilot can:
- **Move RC sticks** (>15% deflection) → immediately suspends autonomous nav, takes manual control
- **Release sticks** → drone requests re-approval to re-engage (or auto re-engages in full-auto mode)
- **Send GCS override** → publish `human_approval_response` with `approved=false` to abort any queued action
- **Switch to POSCTL/STABILIZED mode** → bypasses autonomous navigator entirely; standard PX4 manual control

---

### 5 — MATEK F405 Hardware Notes

- **MCU:** STM32F405RGT6 — 168 MHz Cortex-M4, 1 MB Flash, 192 KB SRAM
- **RAM budget:** Dijkstra graph uses ~4 KB; waypoint cache uses ~2 KB — well within limits
- **No F405-VTOL board in stock PX4** — this package creates it, derived from `matek/h743` with F405 flash/RAM constraints
- **PWM outputs:** 9 channels — 4× quad motors, 2× tilt servos, 1× forward motor, 2× elevons
- **Anemometer port:** UART4 (GPS1 connector) recommended; configure `SER_GPS1_BAUD` to match sensor
- **FRAM:** External SPI FRAM (e.g. MB85RS256B) for mission cache persistence across reboots

---

## License

BSD 3-Clause (same as PX4-Autopilot). See PX4-Autopilot/LICENSE.

---

## Testing and known issues

The planner and state machine were modelled in Python (`../sim/`) and tested with 24 unit tests and three benchmark experiments. This found ten bugs (B1–B10). B2–B10 are fixed in this C++ (look for `FIX Bn` comments) and B1 (mission loading) is still open. See `sim/README.md` for details and results.

**Status:** the C++ fixes have passed a syntax check only. Build with `make px4_sitl` and test in SITL before any real flight.
