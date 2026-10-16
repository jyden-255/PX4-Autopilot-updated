"""Benchmark experiments for the navigator planner.

E1  Mission completeness: how often does the ORIGINAL planner silently skip
    mission waypoints, vs the fixed planner?
E2  Wind-aware routing: over random networks of safe waypoints, does a
    wind-aware cost actually reduce flight time? Compares:
      - distance only (ignores wind)
      - the ORIGINAL wind cost (sign error, direction-blind)
      - the FIXED wind cost at several values of AN_WIND_COST_K
      - an oracle that plans on true simulated flight time (best possible)
E3  End-to-end: random missions flown by the fixed state machine with random
    GPS dropouts, pilot overrides and wind holds. Does every mission finish
    with every waypoint visited, in order?

All randomness uses fixed seeds, so results are exactly repeatable.
Run:  python benchmark.py            (from the sim/ folder)
"""
from __future__ import annotations

import math
import random
import statistics as st

from navigator_model import (INF, S, Fixes, Inputs, Navigator, Waypoint,
                             apply_wind, build_graph, dijkstra, leg_time,
                             route_time, tailwind)

FIXED = Fixes.all()
ORIG_PLANNER = Fixes(wind_sign=False, no_skip_in_mission=False, edge_budget=False)
TAS = 18.0  # m/s, typical small VTOL cruise airspeed


def random_mission(rng, n):
    """Random-walk survey mission: legs 80-600 m, random turns."""
    wps, x, y, h = [Waypoint(0, 0)], 0.0, 0.0, rng.uniform(0, 2 * math.pi)
    for _ in range(n - 1):
        h += rng.uniform(-2.0, 2.0)
        d = rng.uniform(80, 600)
        x, y = x + d * math.cos(h), y + d * math.sin(h)
        wps.append(Waypoint(x, y, rng.uniform(60, 140)))
    return wps


def e1_mission_completeness(trials=1000, seed=1):
    rng = random.Random(seed)
    skipped_any = {"original": 0, "fixed": 0}
    skipped_frac = {"original": [], "fixed": []}
    for _ in range(trials):
        wps = random_mission(rng, rng.randint(6, 20))
        for name, fx in [("original", ORIG_PLANNER), ("fixed", FIXED)]:
            path, _ = dijkstra(len(wps), build_graph(wps, fx, mission_mode=True), 0, len(wps) - 1)
            missed = len(wps) - len(path) if path else len(wps)
            skipped_any[name] += missed > 0
            skipped_frac[name].append(missed / len(wps))
    return {k: (skipped_any[k] / trials, st.mean(skipped_frac[k])) for k in skipped_any}


def random_network(rng, n=30, w=3000, h=1500):
    pts = sorted(((rng.uniform(0, w), rng.uniform(0, h)) for _ in range(n)))
    return [Waypoint(x, y, 100.0) for x, y in pts]


def e2_wind_routing(trials=1000, seed=2, ks=(0.01, 0.02, 0.05, 0.1, 0.25, 0.5)):
    rng = random.Random(seed)
    names = ["distance only", "original wind cost (k=0.5)"] + [f"fixed wind cost k={k}" for k in ks]
    excess = {n: [] for n in names}
    bad = {n: 0 for n in names}
    used = 0
    for _ in range(trials):
        wps = random_network(rng)
        speed, direc = rng.uniform(4, 12), rng.uniform(0, 2 * math.pi)
        wn, we = speed * math.cos(direc), speed * math.sin(direc)
        n = len(wps)
        edges = build_graph(wps, Fixes(no_skip_in_mission=False), mission_mode=False)

        def true_t(a, b):
            return leg_time(wps[a], wps[b], wn, we, TAS)
        best_path, best = dijkstra(n, edges, 0, n - 1, directed_cost=true_t)
        if not best_path or best >= INF:
            continue
        used += 1

        def evaluate(name, path):
            t = route_time(path, wps, wn, we, TAS)
            ex = (t / best - 1) * 100
            excess[name].append(ex)
            bad[name] += ex > 5

        for e in edges:
            e.wind = 0.0
        evaluate("distance only", dijkstra(n, edges, 0, n - 1)[0])

        apply_wind(edges, wps, wn, we, 0.5, Fixes(wind_sign=False))
        evaluate("original wind cost (k=0.5)", dijkstra(n, edges, 0, n - 1)[0])

        for k in ks:
            def cost(a, b, k=k):  # FIXED: headwind for the direction actually flown
                e_base = math.hypot(wps[b].x - wps[a].x, wps[b].y - wps[a].y)
                head = max(-tailwind(wps[a], wps[b], wn, we), 0.0)
                return e_base + k * head * e_base
            evaluate(f"fixed wind cost k={k}", dijkstra(n, edges, 0, n - 1, directed_cost=cost)[0])
    return used, {n: (st.mean(v), st.median(v), sorted(v)[int(0.9 * len(v))], bad[n] / len(v))
                  for n, v in excess.items()}


def e3_end_to_end(trials=200, seed=3):
    rng = random.Random(seed)
    finished = in_order = 0
    events = {"gps": 0, "override": 0, "wind": 0}
    for _ in range(trials):
        wps = random_mission(rng, rng.randint(4, 10))
        nav = Navigator(wps, fixes=FIXED, full_auto=True, approval_timeout_s=3.0, tas=TAS)
        speed, direc = rng.uniform(0, 10), rng.uniform(0, 2 * math.pi)
        wn, we = speed * math.cos(direc), speed * math.sin(direc)
        script = {}
        for kind in ("gps", "override", "wind"):
            if rng.random() < 0.5:
                start = rng.uniform(20, 120)
                script[kind] = (start, start + rng.uniform(2, 15))
                events[kind] += 1

        def inputs(t, nav, script=script, wn=wn, we=we):
            def on(k):
                return k in script and script[k][0] <= t < script[k][1]
            return Inputs(wind_n=wn, wind_e=we, gps_valid=not on("gps"),
                          sticks=0.4 if on("override") else 0.0,
                          wind_limit_exceeded=on("wind"))
        end = nav.run(3600, inputs=inputs)
        finished += end == S.LANDED
        in_order += nav.visited == list(range(len(wps)))
    return trials, finished, in_order, events


def main():
    print("E1  Mission completeness (1,000 random survey missions, 6-20 waypoints)")
    for k, (any_frac, mean_frac) in e1_mission_completeness().items():
        print(f"    {k:9s}: missions skipping >=1 waypoint {any_frac:6.1%} | "
              f"average share of waypoints skipped {mean_frac:6.1%}")

    used, res = e2_wind_routing()
    print(f"\nE2  Wind-aware routing ({used} random 30-node networks, wind 4-12 m/s, TAS {TAS} m/s)")
    print("    extra flight time vs the time-optimal route (oracle)")
    print(f"    {'planner':32s} {'mean':>7s} {'median':>7s} {'90th pct':>9s} {'>5% slower':>11s}")
    for n, (mean, med, p90, bad) in res.items():
        print(f"    {n:32s} {mean:6.2f}% {med:6.2f}% {p90:8.2f}% {bad:10.1%}")

    trials, fin, order, ev = e3_end_to_end()
    print(f"\nE3  End-to-end, fixed navigator ({trials} random missions; events injected: "
          f"GPS loss {ev['gps']}, pilot override {ev['override']}, wind hold {ev['wind']})")
    print(f"    finished: {fin}/{trials} | every waypoint visited in order: {order}/{trials}")


if __name__ == "__main__":
    main()
