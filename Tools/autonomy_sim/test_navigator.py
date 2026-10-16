"""Tests for the navigator model. Each bug has a pair of tests:
one proving the ORIGINAL code has the bug, one proving the FIX removes it.

Run:  python -m pytest -v    (from the sim/ folder)
"""
import math
import random

import pytest

from navigator_model import (S, Fixes, Inputs, Navigator, Waypoint, apply_wind,
                             build_graph, crab_angle, dijkstra, leg_time,
                             wind_penalty)

ORIG, FIXED = Fixes.original(), Fixes.all()


def line_mission(n=5, spacing=300.0, heading_deg=0.0):
    h = math.radians(heading_deg)
    return [Waypoint(i * spacing * math.cos(h), i * spacing * math.sin(h)) for i in range(n)]


def only(**kw):
    """All fixes ON except the ones named (set False) - isolates one bug."""
    return Fixes(**{**Fixes.all().__dict__, **kw})


# ---------------------------------------------------------------- physics

def test_tailwind_is_faster_than_headwind():
    a, b = Waypoint(0, 0), Waypoint(1000, 0)        # flying north
    north_wind = (8.0, 0.0)                          # air moving north = tailwind
    assert leg_time(a, b, *north_wind) < leg_time(b, a, *north_wind)


def test_crab_angle_matches_geometry():
    assert crab_angle(0.0, 18.0) == 0.0
    assert math.isclose(math.degrees(crab_angle(9.0, 18.0)), 30.0, abs_tol=1e-6)
    assert math.isclose(math.degrees(crab_angle(50.0, 18.0, limit_deg=30)), 30.0)  # clamped


def test_dijkstra_finds_shortest():
    wps = [Waypoint(0, 0), Waypoint(300, 0), Waypoint(600, 0), Waypoint(300, 300)]
    edges = build_graph(wps, only(no_skip_in_mission=False), mission_mode=False)
    path, cost = dijkstra(len(wps), edges, 0, 2)
    assert path[0] == 0 and path[-1] == 2
    assert cost <= 600 + 1e-6


# -------------------------------------------- B1: mission never loaded

def test_B1_original_never_leaves_idle():
    nav = Navigator(line_mission(), fixes=only(mission_loading=False))
    assert nav.run(60, approve_all=True) == S.IDLE


def test_B1_fixed_flies_mission():
    nav = Navigator(line_mission(), fixes=FIXED)
    assert nav.run(600, approve_all=True) == S.LANDED
    assert nav.visited == [0, 1, 2, 3, 4]


# ------------------------------------------------ B2: wind sign flipped

def test_B2_original_penalises_tailwind():
    wps = line_mission(2)                       # one leg heading north
    edges = build_graph(wps, FIXED)
    north_wind = (8.0, 0.0)                     # tailwind for this leg
    assert wind_penalty(edges[0], wps, *north_wind, k=0.5, fixes=only(wind_sign=False)) > 0


def test_B2_fixed_penalises_only_headwind():
    wps = line_mission(2)
    edges = build_graph(wps, FIXED)
    assert wind_penalty(edges[0], wps, 8.0, 0.0, k=0.5, fixes=FIXED) == 0       # tailwind: free
    assert wind_penalty(edges[0], wps, -8.0, 0.0, k=0.5, fixes=FIXED) > 0       # headwind: costly


# ------------------------------------ B3: full-auto approval never fires

def test_B3_original_full_auto_gets_stuck():
    nav = Navigator(line_mission(), fixes=only(full_auto_timeout=False), full_auto=True)
    assert nav.run(120) == S.AWAITING_APPROVAL   # nobody approves, timeout never helps


def test_B3_fixed_full_auto_proceeds_after_timeout():
    nav = Navigator(line_mission(), fixes=FIXED, full_auto=True)
    assert nav.run(600) == S.LANDED


@pytest.mark.parametrize("fixes,expect", [(only(full_auto_timeout=False), S.AWAITING_APPROVAL),
                                          (FIXED, S.EXECUTING)])
def test_B3_late_human_approval_is_accepted(fixes, expect):
    nav = Navigator(line_mission(), fixes=fixes, full_auto=False, approval_timeout_s=10)
    nav.run(15)                                   # past the 10 s timeout, nobody answered
    assert nav.state == S.AWAITING_APPROVAL
    nav.step(Inputs(approval=(0x01, True)))       # the pilot finally approves
    assert nav.state == expect


# -------------------------- B4: replan uses a path index as a node id

def _network():
    # 0 -> 2 -> 4 is the best route; 1 and 3 are detours off to the side
    return [Waypoint(0, 0), Waypoint(200, 400), Waypoint(400, 0),
            Waypoint(600, 400), Waypoint(800, 0)]


@pytest.mark.parametrize("fixes,expect_ok", [(only(replan_source=False), False), (FIXED, True)])
def test_B4_replan_starts_from_current_waypoint(fixes, expect_ok):
    nav = Navigator(_network(), fixes=fixes)
    nav.edges = build_graph(nav.wps, fixes, mission_mode=False)
    nav.path, _ = dijkstra(5, nav.edges, 0, 4)
    assert nav.path == [0, 2, 4]
    nav.path_idx = 1                      # currently heading to waypoint 2
    nav.state = S.PLANNING
    nav.step(Inputs())
    heading_to = nav.path[nav.path_idx]
    assert (heading_to == 2) == expect_ok  # original re-plans from node "1" instead


def test_B4_regression_replan_after_last_waypoint():
    # Found by benchmark E3: a replan firing after the final waypoint was
    # reached appended the last waypoint a second time.
    nav = Navigator(line_mission(3), fixes=FIXED)
    nav.edges = build_graph(nav.wps, FIXED)
    nav.path, nav.path_idx = [0, 1, 2], 3
    nav.state = S.PLANNING
    nav.step(Inputs())
    assert nav.path == [0, 1, 2]


# ------------------------------------------- B5: first plan ignores wind

@pytest.mark.parametrize("fixes,expect_wind", [(only(initial_wind=False), False), (FIXED, True)])
def test_B5_initial_plan_uses_wind(fixes, expect_wind):
    nav = Navigator(line_mission(), fixes=fixes)
    nav.step(Inputs(wind_n=-8.0))         # headwind for a northbound mission
    assert any(e.wind > 0 for e in nav.edges) == expect_wind


# ------------------------------ B6: re-engaging restarts the mission

@pytest.mark.parametrize("fixes,restarts", [(only(resume_after_override=False), True), (FIXED, False)])
def test_B6_resume_after_override(fixes, restarts):
    nav = Navigator(line_mission(6, 300), fixes=fixes, full_auto=True, approval_timeout_s=1)
    nav.state = S.EXECUTING
    nav.edges = build_graph(nav.wps, fixes)
    nav.path = list(range(6))
    nav.path_idx = 3
    nav.step(Inputs(sticks=0.5))          # pilot grabs the sticks
    assert nav.state == S.HUMAN_OVERRIDE
    nav.step(Inputs(sticks=0.0))          # pilot lets go -> asks to re-engage
    nav.step(Inputs(approval=(0x02, True)))
    assert nav.state == S.EXECUTING
    assert (nav.path_idx == 0) == restarts


# --------------------------- B7: edge budget disconnects the graph

def test_B7_clustered_start_disconnects_long_route():
    # 64 waypoints (the maximum): a survey pattern of 22 close points near
    # home, then a long transit leg of 42 points 400 m apart.
    rng = random.Random(1)
    wps = [Waypoint(rng.uniform(0, 300), rng.uniform(0, 300)) for _ in range(22)]
    wps += [Waypoint(400 + 400 * i, 300) for i in range(42)]
    broken = build_graph(wps, only(edge_budget=False, no_skip_in_mission=False), mission_mode=False)
    fixed = build_graph(wps, only(no_skip_in_mission=False), mission_mode=False)
    assert dijkstra(64, broken, 0, 63)[0] == []        # original: no route to the end at all
    assert dijkstra(64, fixed, 0, 63)[0] != []         # fixed: chain reserved, route exists


# -------------------------------- B8: mission waypoints get skipped

def test_B8_original_skips_waypoints():
    wps = [Waypoint(0, 0), Waypoint(200, 150), Waypoint(400, 0)]   # WP1 is a detour
    edges = build_graph(wps, only(no_skip_in_mission=False))
    assert dijkstra(3, edges, 0, 2)[0] == [0, 2]                    # WP1 never flown


def test_B8_fixed_visits_every_waypoint_in_order():
    wps = [Waypoint(0, 0), Waypoint(200, 150), Waypoint(400, 0)]
    edges = build_graph(wps, FIXED)
    assert dijkstra(3, edges, 0, 2)[0] == [0, 1, 2]


# ------------------------------------------------------ safety paths

def test_override_always_wins():
    nav = Navigator(line_mission(), fixes=FIXED, full_auto=True)
    nav.run(15)
    assert nav.state == S.EXECUTING
    nav.step(Inputs(sticks=0.3))
    assert nav.state == S.HUMAN_OVERRIDE
    for _ in range(50):                   # holding the sticks keeps control with the pilot
        nav.step(Inputs(sticks=0.3))
    assert nav.state == S.HUMAN_OVERRIDE


def test_gps_loss_switches_to_offline_and_back():
    nav = Navigator(line_mission(), fixes=FIXED, full_auto=True)
    nav.run(15)
    nav.step(Inputs(gps_valid=False))
    assert nav.state == S.OFFLINE_EXECUTING
    nav.step(Inputs())
    assert nav.state == S.EXECUTING


def test_wind_limit_holds_then_resumes():
    nav = Navigator(line_mission(), fixes=FIXED, full_auto=True)
    nav.run(15)
    nav.step(Inputs(wind_limit_exceeded=True))
    assert nav.state == S.WIND_HOLD
    nav.step(Inputs())
    assert nav.state == S.EXECUTING
