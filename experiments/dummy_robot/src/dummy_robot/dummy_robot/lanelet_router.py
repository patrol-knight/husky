"""Lanelet2-based local path planner for dummy_robot.

A trimmed port of mission_manager/backend/core/map_server.py's routing
logic -- deliberately duplicated rather than shared. dummy_robot stands in
for a robot's own onboard nav stack, which plans its own path from a goal
position alone; it must never receive a path computed by the backend (see
NavigationGatewayComponent/AnomalyResponseCoordinator, which only ever send
a goal). Only local (x, y) lanelet centerline geometry is needed here --
no lat/lon conversion, no GeoJSON, unlike MapServer's version.
"""

import logging
import math

import yaml

logger = logging.getLogger(__name__)

try:
    from lanelet2 import traffic_rules
    from lanelet2.core import BasicPoint2d
    from lanelet2.geometry import findNearest
    from lanelet2.io import Origin
    from lanelet2.io import load as ll2_load
    from lanelet2.projection import LocalCartesianProjector
    from lanelet2.routing import RoutingGraph

    LANELET2_AVAILABLE = True
except ImportError:
    LANELET2_AVAILABLE = False


class LaneletRouter:
    def __init__(self, osm_path: str, projector_yaml: str) -> None:
        if not LANELET2_AVAILABLE:
            raise RuntimeError("lanelet2 is not installed; cannot plan routes")

        with open(projector_yaml, "r") as f:
            data = yaml.safe_load(f)
        origin = Origin(data["map_origin"]["latitude"], data["map_origin"]["longitude"])
        projector = LocalCartesianProjector(origin)

        lanelet_map = ll2_load(osm_path, projector)
        self._lanelet_layer = lanelet_map.laneletLayer
        tr = traffic_rules.create(traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle)
        self._routing_graph = RoutingGraph(lanelet_map, tr)
        logger.info("LaneletRouter loaded %d lanelets from %s", len(self._lanelet_layer), osm_path)

    def plan_route(self, x0: float, y0: float, x1: float, y1: float) -> list[tuple[float, float]] | None:
        """Local-xy centerline polyline from (x0, y0) to (x1, y1), or None if unreachable."""
        start_ll = self._nearest_lanelet(x0, y0)
        end_ll = self._nearest_lanelet(x1, y1)
        if start_ll is None or end_ll is None:
            return None

        t0 = self._project_onto_centerline(start_ll, x0, y0)
        t1 = self._project_onto_centerline(end_ll, x1, y1)

        if start_ll.id == end_ll.id and t0 <= t1:
            ll_sequence = [start_ll]
        elif start_ll.id == end_ll.id:
            # Goal is "behind" the robot on the same lanelet -- loop around
            # via the next lanelet rather than driving backwards.
            following = list(self._routing_graph.following(start_ll))
            if not following:
                return None
            via_ll = following[0]
            route1 = self._routing_graph.getRoute(start_ll, via_ll, 0)
            route2 = self._routing_graph.getRoute(via_ll, end_ll, 0)
            if route1 is None or route2 is None:
                return None
            ll_sequence = list(route1.shortestPath()) + list(route2.shortestPath())[1:]
        else:
            route = self._routing_graph.getRoute(start_ll, end_ll, 0)
            if route is None:
                return None
            ll_sequence = list(route.shortestPath())

        coords: list[tuple[float, float]] = []
        for i, ll in enumerate(ll_sequence):
            is_first, is_last = i == 0, i == len(ll_sequence) - 1
            if is_first and is_last:
                coords.extend(self._sample_between(ll, t0, t1))
            elif is_first:
                coords.extend(self._sample_between(ll, t0, 1.0)[:-1])
            elif is_last:
                coords.extend(self._sample_between(ll, 0.0, t1))
            else:
                coords.extend((p.x, p.y) for p in list(ll.centerline)[:-1])
        return coords

    def _nearest_lanelet(self, x: float, y: float):
        point = BasicPoint2d(x, y)
        candidates = findNearest(self._lanelet_layer, point, 5)
        if not candidates:
            return None
        best_ll = None
        best_dist = float("inf")
        for dist, ll in candidates:
            yaw = self._lanelet_yaw_near(ll, x, y)
            if best_ll is None:
                ref_yaw = yaw
                best_ll = ll
                best_dist = dist
                continue
            dot = math.cos(yaw) * math.cos(ref_yaw) + math.sin(yaw) * math.sin(ref_yaw)
            if dot > 0 and dist < best_dist:
                best_dist = dist
                best_ll = ll
        return best_ll

    @staticmethod
    def _lanelet_yaw_near(ll, x: float, y: float) -> float:
        pts = list(ll.centerline)
        if len(pts) < 2:
            return 0.0
        best_i, best_d = 0, float("inf")
        for i in range(len(pts) - 1):
            mx, my = (pts[i].x + pts[i + 1].x) / 2, (pts[i].y + pts[i + 1].y) / 2
            d = (mx - x) ** 2 + (my - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        dx = pts[best_i + 1].x - pts[best_i].x
        dy = pts[best_i + 1].y - pts[best_i].y
        return math.atan2(dy, dx)

    @staticmethod
    def _project_onto_centerline(ll, x: float, y: float) -> float:
        pts = list(ll.centerline)
        if len(pts) < 2:
            return 0.0
        arc = LaneletRouter._arc_lengths(pts)
        total = arc[-1]
        if total == 0:
            return 0.0
        best_t, best_d = 0.0, float("inf")
        for i in range(len(pts) - 1):
            ax, ay, bx, by = pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y
            dx, dy = bx - ax, by - ay
            seg_len_sq = dx * dx + dy * dy
            t_seg = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len_sq)) if seg_len_sq > 0 else 0.0
            px, py = ax + t_seg * dx, ay + t_seg * dy
            d = math.hypot(x - px, y - py)
            if d < best_d:
                best_d = d
                best_t = (arc[i] + t_seg * (arc[i + 1] - arc[i])) / total
        return best_t

    @staticmethod
    def _arc_lengths(pts) -> list[float]:
        arc = [0.0]
        for i in range(1, len(pts)):
            arc.append(arc[-1] + math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y))
        return arc

    @staticmethod
    def _sample_between(ll, t_start: float, t_end: float) -> list[tuple[float, float]]:
        pts = list(ll.centerline)
        if len(pts) < 2:
            return [(pts[0].x, pts[0].y)] if pts else []
        arc = LaneletRouter._arc_lengths(pts)
        total = arc[-1]
        if total == 0:
            return [(pts[0].x, pts[0].y)]

        def interp(t: float) -> tuple[float, float]:
            s = t * total
            for i in range(len(pts) - 1):
                if arc[i] <= s <= arc[i + 1]:
                    seg_len = arc[i + 1] - arc[i]
                    alpha = (s - arc[i]) / seg_len if seg_len > 0 else 0.0
                    return (
                        pts[i].x + alpha * (pts[i + 1].x - pts[i].x),
                        pts[i].y + alpha * (pts[i + 1].y - pts[i].y),
                    )
            return (pts[-1].x, pts[-1].y)

        result = [interp(t_start)]
        for i, a in enumerate(arc[1:], start=1):
            t = a / total
            if t_start + 1e-9 < t < t_end - 1e-9:
                result.append((pts[i].x, pts[i].y))
        result.append(interp(t_end))
        return result
