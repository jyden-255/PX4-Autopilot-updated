# Navigator simulation and test harness

A Python reference model of the `autonomous_navigator` PX4 module, used to test the planner and state machine without building firmware. Each bug found is reproduced by a test against the original logic, fixed, and then checked again, and the fix is copied into the C++.

```bash
pip install pytest
python -m pytest -v      # 24 tests: each bug reproduced in the original, absent in the fix
python benchmark.py      # experiments E1-E3, about 3 seconds, fixed seeds
```

## Bugs found and fixed

| ID | Bug in the original module | Effect | Status |
|---|---|---|---|
| B1 | `_mission_valid` is never set and the mission is never read into `_wp_cache` | Navigator never leaves IDLE, so no autonomous flight | Fixed in the model only. The C++ still needs mission loading through PX4's dataman (open issue) |
| B2 | PX4 wind is the air's velocity, so its projection on the track is tailwind, but the planner treated it as headwind | Planner avoids tailwind legs and prefers headwind | Fixed (C++ + model) |
| B3 | Full-auto approval timeout never granted approval; in manual mode a timeout discarded the request, so a late human approval was ignored | Mission stuck in AWAITING_APPROVAL | Fixed (C++ + model) |
| B4 | Mid-mission replan used the path index as a waypoint ID | Drone could turn back towards the wrong waypoint | Fixed (C++ + model), plus a regression test for an edge case the benchmark found |
| B5 | First plan built before wind costs were computed | Initial route ignores wind | Fixed (C++ + model) |
| B6 | Re-engaging after a pilot override reset `_path_idx` to 0 | Mission restarts from waypoint 0 | Fixed (C++ + model) |
| B7 | Edge budget used up by shortcut edges before later chain edges are added | No route to the end of long missions with a clustered start | Fixed: chain edges reserved first, counter widened to uint16 |
| B8 | Shortcut edges (< 500 m) let Dijkstra skip mission waypoints | Waypoints silently not flown | Fixed: shortcuts only when new param `AN_ALLOW_SKIP = 1` |
| B9 | Wind penalty computed for an edge's stored direction, but edges are traversed both ways | Wrong cost when flying an edge backwards | Fixed: penalty computed for the direction flown |
| B10 | Crosswind compensator had the same tailwind/headwind sign error | Throttle boost applied in tailwind | Fixed (C++) |

Not changed, but flagged to check in SITL:
- **Offline cache:** `persist_cache_to_fram()` is a stub (the `dm_write` call is commented out), so the mission isn't actually persisted.
- **Throttle convention:** recent PX4 versions report `manual_control_setpoint.throttle` in [-1, 1]. If so, the `throttle - 0.5` override checks would fire with the sticks centred.
- **Anemometer direction:** check whether the external anemometer's direction means "blowing from" (meteorological) or "blowing to" before blending it with EKF2 wind.

## Benchmark results (`benchmark_results.txt`)

- **E1, mission completeness** (1,000 random survey missions): the original planner skipped at least one waypoint in 95.4% of missions, and 42.6% of waypoints on average. The fixed planner skipped none.
- **E2, wind-aware routing** (1,000 random 30-node networks, uniform wind 4–12 m/s, airspeed 18 m/s): the original wind cost produced routes 12.2% slower than optimal on average, and over 5% slower in 44% of trips. With the sign fixed, wind-aware routing matched distance-only routing almost exactly. In uniform wind, the shortest route is already the fastest, so the wind term only adds value if wind varies over space (terrain, altitude). The recommended default is a small `AN_WIND_COST_K`, or 0 until spatial wind data exists.
- **E3, end-to-end** (200 random missions with injected GPS loss, pilot overrides and wind holds): the fixed navigator finished 200/200 missions with every waypoint visited in order. The first run of this experiment found a 1/200 failure (a replan after the last waypoint duplicated it), which was fixed and given a regression test.

## Limitations

The model is kinematic: it flies straight lines at airspeed-plus-wind ground speed, with no attitude dynamics, sensor noise or real PX4 timing. It tests the planning and decision logic, not flight control. The C++ fixes passed a syntax check against stand-in headers only. The next step is building with `make px4_sitl` and repeating E3 in PX4 SITL with Gazebo.
