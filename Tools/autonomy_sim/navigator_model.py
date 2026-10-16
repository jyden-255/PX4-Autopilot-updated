"""Python reference model of the AutonomousNavigator PX4 module.

Why this exists: the C++ module runs inside PX4 firmware, which is slow to
build and test. This file re-implements the same logic in plain Python so
every decision can be tested in milliseconds, then the fixes are copied back
into the C++.

Each known bug is behind a flag in `Fixes`. `Fixes.original()` reproduces
the C++ exactly as first written; `Fixes.all()` is the corrected version.
Tests in test_navigator.py prove each bug exists in the original and is gone
in the fixed version.

Coordinates: local metres, x = north, y = east (PX4's NED convention).
Bearing = angle from north, clockwise (atan2(east, north)), as in PX4.
Wind: velocity of the AIR (the direction it blows TOWARDS), as in PX4's
`wind` topic. So projecting wind onto the direction of travel gives the
TAILWIND component (positive = wind pushing you along).
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

INF = 1e9
MAX_EDGES = 256          # AN_MAX_GRAPH_EDGES in the C++
SKIP_RADIUS_M = 500.0    # "direct edges between close waypoints (< 500 m)"
ALT_WEIGHT = 2.0         # base_cost = dist + 2 * |altitude change|


@dataclass(frozen=True)
class Fixes:
    mission_loading: bool = True   # B1: _mission_valid is never set -> never leaves IDLE
    wind_sign: bool = True         # B2: tailwind penalised as if it were headwind
    full_auto_timeout: bool = True # B3: full-auto approval timeout never grants approval
    replan_source: bool = True     # B4: replan starts from path INDEX, not waypoint ID
    initial_wind: bool = True      # B5: first plan ignores wind (costs never computed)
    resume_after_override: bool = True  # B6: re-engaging restarts the mission at WP0
    edge_budget: bool = True       # B7: edge budget runs out before the i->i+1 chain is complete -> no path
    no_skip_in_mission: bool = True     # B8: shortest path may skip mission waypoints

    @staticmethod
    def original() -> "Fixes":
        return Fixes(*([False] * 8))

    @staticmethod
    def all() -> "Fixes":
        return Fixes()


@dataclass
class Waypoint:
    x: float  # north, m
    y: float  # east, m
    alt: float = 100.0
    accept_r: float = 5.0


@dataclass
class Edge:
    a: int
    b: int
    base: float
    wind: float = 0.0

    @property
    def total(self) -> float:
        return self.base + self.wind


# ------------------------------------------------------------ geometry ---

def dist(p: Waypoint, q: Waypoint) -> float:
    return math.hypot(q.x - p.x, q.y - p.y)


def bearing(p: Waypoint, q: Waypoint) -> float:
    return math.atan2(q.y - p.y, q.x - p.x)


def tailwind(p: Waypoint, q: Waypoint, wind_n: float, wind_e: float) -> float:
    b = bearing(p, q)
    return wind_n * math.cos(b) + wind_e * math.sin(b)


def crosswind(p: Waypoint, q: Waypoint, wind_n: float, wind_e: float) -> float:
    b = bearing(p, q)
    return -wind_n * math.sin(b) + wind_e * math.cos(b)


def leg_time(p: Waypoint, q: Waypoint, wind_n: float, wind_e: float,
             tas: float = 18.0, climb_rate: float = 2.5) -> float:
    """Seconds to fly p->q holding the track (crabbing into crosswind).
    Ground speed = sqrt(TAS^2 - crosswind^2) + tailwind."""
    d = dist(p, q)
    if d < 1e-6:
        return abs(q.alt - p.alt) / climb_rate
    cw = crosswind(p, q, wind_n, wind_e)
    if abs(cw) >= tas:
        return INF
    gs = math.sqrt(tas * tas - cw * cw) + tailwind(p, q, wind_n, wind_e)
    if gs <= 0.1:
        return INF
    return d / gs + abs(q.alt - p.alt) / climb_rate


def crab_angle(cross: float, airspeed: float, limit_deg: float = 30.0) -> float:
    """Same maths as CrosswindCompensator::compute_compensation."""
    a = max(airspeed, 2.0)
    ang = math.asin(max(-1.0, min(1.0, cross / a)))
    lim = math.radians(limit_deg)
    return max(-lim, min(lim, ang))


# ------------------------------------------------------------- planner ---

def build_graph(wps: list[Waypoint], fixes: Fixes, mission_mode: bool = True) -> list[Edge]:
    """Mirror of build_waypoint_graph(). With fixes.no_skip_in_mission and
    mission_mode=True only consecutive edges exist, so every waypoint is
    visited in order. mission_mode=False is point-to-point routing over a
    network of safe waypoints, where shortcuts are the point."""
    n = len(wps)
    allow_skip = not (fixes.no_skip_in_mission and mission_mode)

    def base_cost(i, j):
        return dist(wps[i], wps[j]) + abs(wps[i].alt - wps[j].alt) * ALT_WEIGHT

    if fixes.edge_budget:
        # FIX B7: reserve room for the chain i->i+1
        # first, so the route is always connected; shortcuts only fill
        # whatever capacity is left.
        out = [Edge(i, i + 1, base_cost(i, i + 1)) for i in range(n - 1)]
        if allow_skip:
            for i in range(n):
                for j in range(i + 2, n):
                    if len(out) >= MAX_EDGES:
                        return out
                    if dist(wps[i], wps[j]) < SKIP_RADIUS_M:
                        out.append(Edge(i, j, base_cost(i, j)))
        return out

    # ---- original behaviour (kept for comparison) ----
    # The outer guard stops adding edges once 256 - N are used, which with
    # many close waypoints happens before the later chain edges are added.
    # (_edge_count is uint8_t; the guard happens to stop it wrapping.)
    edges: list[Optional[Edge]] = [None] * MAX_EDGES
    count = 0
    for i in range(n):
        if count >= MAX_EDGES - n:
            break
        for j in range(i + 1, n):
            d = dist(wps[i], wps[j])
            base = d + abs(wps[i].alt - wps[j].alt) * ALT_WEIGHT
            if j == i + 1 or (allow_skip and d < SKIP_RADIUS_M):
                # uint8_t: 255 + 1 wraps to 0 and starts overwriting edge 0
                edges[count] = Edge(i, j, base)
                count = (count + 1) & 0xFF
    # C++ only ever iterates edges[0 .. _edge_count-1]
    return [e for e in edges[:count] if e is not None]


def wind_penalty(e: Edge, wps: list[Waypoint], wind_n: float, wind_e: float,
                 k: float, fixes: Fixes) -> float:
    along = tailwind(wps[e.a], wps[e.b], wind_n, wind_e)
    # Original C++ called this projection "headwind"; it is actually tailwind.
    head = max(along, 0.0) if not fixes.wind_sign else max(-along, 0.0)
    return k * head * e.base


def apply_wind(edges: list[Edge], wps, wind_n, wind_e, k, fixes) -> None:
    for e in edges:
        e.wind = wind_penalty(e, wps, wind_n, wind_e, k, fixes)


def dijkstra(n: int, edges: list[Edge], src: int, dst: int,
             directed_cost=None) -> tuple[list[int], float]:
    """Undirected Dijkstra, same result as run_dijkstra() in the C++ (which
    uses an O(N^2) scan; a heap gives identical paths, faster).
    directed_cost(a, b) optionally overrides the cost of travelling a->b."""
    adj: dict[int, list[tuple[int, Edge]]] = {i: [] for i in range(n)}
    for e in edges:
        adj[e.a].append((e.b, e))
        adj[e.b].append((e.a, e))
    distv = [INF] * n
    prev = [-1] * n
    distv[src] = 0.0
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > distv[u]:
            continue
        for v, e in adj[u]:
            c = directed_cost(u, v) if directed_cost else e.total
            if d + c < distv[v]:
                distv[v] = d + c
                prev[v] = u
                heapq.heappush(pq, (distv[v], v))
    if distv[dst] >= INF:
        return [], INF
    path, cur = [], dst
    while cur != -1:
        path.append(cur)
        cur = prev[cur]
    return path[::-1], distv[dst]


def route_time(path: list[int], wps, wind_n, wind_e, tas=18.0) -> float:
    return sum(leg_time(wps[a], wps[b], wind_n, wind_e, tas) for a, b in zip(path, path[1:]))


# ------------------------------------------------------- state machine ---

class S(Enum):
    IDLE = 0
    PLANNING = 1
    AWAITING_APPROVAL = 2
    EXECUTING = 3
    HUMAN_OVERRIDE = 4
    OFFLINE_EXECUTING = 5
    WIND_HOLD = 6
    RTL = 7
    LANDED = 8


@dataclass
class Inputs:
    armed: bool = True
    approval: Optional[tuple[int, bool]] = None  # (action_id, approved)
    sticks: float = 0.0          # max stick deflection, 0..1
    gps_valid: bool = True
    gcs_connected: bool = True
    wind_n: float = 0.0
    wind_e: float = 0.0
    wind_limit_exceeded: bool = False


@dataclass
class Navigator:
    """Tick-by-tick model of AutonomousNavigator::Run() at 10 Hz."""
    wps: list[Waypoint]
    fixes: Fixes = field(default_factory=Fixes.all)
    full_auto: bool = False
    wind_k: float = 0.5
    approval_timeout_s: float = 10.0
    replan_interval_s: float = 30.0
    tas: float = 18.0
    dt: float = 0.1

    def __post_init__(self):
        self.state = S.IDLE
        self.t = 0.0
        self.edges: list[Edge] = []
        self.path: list[int] = []
        self.path_idx = 0
        self.mission_valid = self.fixes.mission_loading  # B1
        self.cache_valid = False
        self.approval_pending = False
        self.auto_approved = False
        self.pending_action = 0
        self.approval_requested_at = 0.0
        self.last_replan = 0.0
        self.pos = Waypoint(self.wps[0].x, self.wps[0].y, self.wps[0].alt)
        self.visited: list[int] = []
        self.log: list[str] = []
        self.wind = (0.0, 0.0)

    # -- helpers mirroring the C++ --
    def _plan(self, src: int, dst: int) -> None:
        path, _ = dijkstra(len(self.wps), self.edges, src, dst)
        if path:
            self.path = path

    def _request_approval(self, action_id: int) -> None:
        self.approval_pending = True
        self.auto_approved = False
        self.pending_action = action_id
        self.approval_requested_at = self.t

    def _approval_granted(self, inp: Inputs) -> bool:
        if self.fixes.full_auto_timeout and self.auto_approved:
            self.auto_approved = False
            return True
        if not self.approval_pending:
            return False
        if inp.approval and inp.approval[0] == self.pending_action:
            self.approval_pending = False
            return inp.approval[1]
        return False

    def _check_approval_timeout(self) -> None:
        if not self.approval_pending:
            return
        if self.t - self.approval_requested_at > self.approval_timeout_s:
            if self.full_auto:
                self.log.append("approval timeout: auto-executing (FULL_AUTO)")
                if self.fixes.full_auto_timeout:
                    self.auto_approved = True
                self.approval_pending = False
            else:
                self.approval_requested_at = self.t  # warn again later
                if not self.fixes.full_auto_timeout:
                    # original: clears the request, so a LATE human approval is ignored
                    self.approval_pending = False

    def _fly_towards(self, wp_idx: int) -> None:
        wp = self.wps[wp_idx]
        d = dist(self.pos, wp)
        if d < 1e-9:
            return
        gs_time = leg_time(self.pos, wp, *self.wind, tas=self.tas, climb_rate=1e9)
        gs = d / gs_time if gs_time < INF else 0.0
        step = min(d, gs * self.dt)
        self.pos = Waypoint(self.pos.x + (wp.x - self.pos.x) / d * step,
                            self.pos.y + (wp.y - self.pos.y) / d * step, wp.alt)

    def _advance(self) -> None:
        if not self.path or self.path_idx >= len(self.path):
            self.state = S.LANDED
            return
        wp_idx = self.path[self.path_idx]
        wp = self.wps[wp_idx]
        if dist(self.pos, wp) < wp.accept_r:
            self.visited.append(wp_idx)
            self.path_idx += 1
            if self.path_idx < len(self.path) and not self.full_auto:
                self._request_approval(0x10 + self.path_idx)
                self.state = S.AWAITING_APPROVAL
                return
            if self.path_idx >= len(self.path):
                return
            wp_idx = self.path[self.path_idx]
        self._fly_towards(wp_idx)

    # -- one 10 Hz tick --
    def step(self, inp: Inputs) -> S:
        self.wind = (inp.wind_n, inp.wind_e)
        # safety monitors run every tick, like the C++
        if inp.sticks > 0.15 and self.state == S.EXECUTING:
            self.state = S.HUMAN_OVERRIDE
        if (not inp.gps_valid or not inp.gcs_connected) and self.state == S.EXECUTING:
            self.state = S.OFFLINE_EXECUTING
        if inp.wind_limit_exceeded and self.state == S.EXECUTING:
            self.state = S.WIND_HOLD

        st = self.state
        if st == S.IDLE:
            if inp.armed and self.mission_valid:
                self.edges = build_graph(self.wps, self.fixes, mission_mode=True)
                if self.fixes.initial_wind:
                    apply_wind(self.edges, self.wps, inp.wind_n, inp.wind_e, self.wind_k, self.fixes)
                self._plan(0, len(self.wps) - 1)
                self.cache_valid = True
                self.state = S.AWAITING_APPROVAL
                self._request_approval(0x01)
        elif st == S.PLANNING:
            apply_wind(self.edges, self.wps, inp.wind_n, inp.wind_e, self.wind_k, self.fixes)
            dst = self.path[-1] if self.path else 0
            if self.fixes.replan_source and self.path_idx >= len(self.path):
                pass  # mission already finished: nothing left to replan
            elif self.fixes.replan_source:
                src = self.path[self.path_idx]
                old_prefix = self.path[:self.path_idx]
                path, _ = dijkstra(len(self.wps), self.edges, src, dst)
                if path:
                    self.path = old_prefix + path
            else:
                self._plan(self.path_idx, dst)  # B4: an index used as a node id
            self.state = S.EXECUTING
        elif st == S.AWAITING_APPROVAL:
            self._check_approval_timeout()
            if self._approval_granted(inp):
                if self.pending_action == 0x01:
                    self.path_idx = 0
                elif self.pending_action == 0x02 and not self.fixes.resume_after_override:
                    self.path_idx = 0  # B6: original resets for every approval
                self.state = S.EXECUTING
        elif st == S.EXECUTING:
            if self.t - self.last_replan > self.replan_interval_s:
                self.last_replan = self.t
                self.state = S.PLANNING
            else:
                self._advance()
        elif st == S.OFFLINE_EXECUTING:
            self._advance()
            if inp.gps_valid and inp.gcs_connected:
                self.state = S.EXECUTING
        elif st == S.HUMAN_OVERRIDE:
            if inp.sticks < 0.05:
                self._request_approval(0x02)
                self.state = S.AWAITING_APPROVAL
        elif st == S.WIND_HOLD:
            if not inp.wind_limit_exceeded:
                self.state = S.EXECUTING
        elif st == S.LANDED:
            pass
        self.t += self.dt
        return self.state

    def run(self, seconds: float, inputs=None, approve_all: bool = False) -> S:
        """Run for `seconds`. inputs(t, nav) -> Inputs lets a test script
        events; approve_all simulates a human approving every request."""
        for _ in range(int(seconds / self.dt)):
            inp = inputs(self.t, self) if inputs else Inputs()
            if approve_all and self.approval_pending and inp.approval is None:
                inp.approval = (self.pending_action, True)
            if self.step(inp) == S.LANDED:
                break
        return self.state
