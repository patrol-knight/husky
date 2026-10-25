#include "dummy_robot/lanelet_router.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <yaml-cpp/yaml.h>

namespace dummy_robot
{

namespace
{

std::vector<Point2d> centerline_points(const lanelet::ConstLanelet & ll)
{
  std::vector<Point2d> pts;
  for (const auto & p : ll.centerline()) {
    pts.emplace_back(p.x(), p.y());
  }
  return pts;
}

std::vector<double> arc_lengths(const std::vector<Point2d> & pts)
{
  std::vector<double> arc{0.0};
  for (std::size_t i = 1; i < pts.size(); ++i) {
    arc.push_back(arc.back() + std::hypot(pts[i].first - pts[i - 1].first, pts[i].second - pts[i - 1].second));
  }
  return arc;
}

double lanelet_yaw_near(const lanelet::ConstLanelet & ll, double x, double y)
{
  const auto pts = centerline_points(ll);
  if (pts.size() < 2) {
    return 0.0;
  }
  std::size_t best_i = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    const double mx = (pts[i].first + pts[i + 1].first) / 2.0;
    const double my = (pts[i].second + pts[i + 1].second) / 2.0;
    const double d = (mx - x) * (mx - x) + (my - y) * (my - y);
    if (d < best_d) {
      best_d = d;
      best_i = i;
    }
  }
  return std::atan2(pts[best_i + 1].second - pts[best_i].second, pts[best_i + 1].first - pts[best_i].first);
}

// Normalized arc-length position [0, 1] of (x, y)'s projection onto the centerline.
double project_onto_centerline(const lanelet::ConstLanelet & ll, double x, double y)
{
  const auto pts = centerline_points(ll);
  if (pts.size() < 2) {
    return 0.0;
  }
  const auto arc = arc_lengths(pts);
  const double total = arc.back();
  if (total == 0.0) {
    return 0.0;
  }
  double best_t = 0.0;
  double best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    const auto [ax, ay] = pts[i];
    const auto [bx, by] = pts[i + 1];
    const double dx = bx - ax;
    const double dy = by - ay;
    const double seg_len_sq = dx * dx + dy * dy;
    const double t_seg =
      seg_len_sq > 0.0 ? std::clamp(((x - ax) * dx + (y - ay) * dy) / seg_len_sq, 0.0, 1.0) : 0.0;
    const double d = std::hypot(x - (ax + t_seg * dx), y - (ay + t_seg * dy));
    if (d < best_d) {
      best_d = d;
      best_t = (arc[i] + t_seg * (arc[i + 1] - arc[i])) / total;
    }
  }
  return best_t;
}

std::vector<Point2d> sample_between(const lanelet::ConstLanelet & ll, double t_start, double t_end)
{
  const auto pts = centerline_points(ll);
  if (pts.size() < 2) {
    return pts;
  }
  const auto arc = arc_lengths(pts);
  const double total = arc.back();
  if (total == 0.0) {
    return {pts.front()};
  }

  auto interp = [&](double t) -> Point2d {
      const double s = t * total;
      for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        if (arc[i] <= s && s <= arc[i + 1]) {
          const double seg_len = arc[i + 1] - arc[i];
          const double alpha = seg_len > 0.0 ? (s - arc[i]) / seg_len : 0.0;
          return {
            pts[i].first + alpha * (pts[i + 1].first - pts[i].first),
            pts[i].second + alpha * (pts[i + 1].second - pts[i].second)};
        }
      }
      return pts.back();
    };

  std::vector<Point2d> result{interp(t_start)};
  for (std::size_t i = 1; i < arc.size(); ++i) {
    const double t = arc[i] / total;
    if (t_start + 1e-9 < t && t < t_end - 1e-9) {
      result.push_back(pts[i]);
    }
  }
  result.push_back(interp(t_end));
  return result;
}

}  // namespace

LaneletRouter::LaneletRouter(const std::string & osm_path, const std::string & projector_yaml)
{
  const YAML::Node origin_node = YAML::LoadFile(projector_yaml)["map_origin"];
  lanelet::projection::LocalCartesianProjector projector(lanelet::Origin(
      lanelet::GPSPoint{origin_node["latitude"].as<double>(), origin_node["longitude"].as<double>(), 0.0}));

  map_ = lanelet::load(osm_path, projector);
  traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  routing_graph_ = lanelet::routing::RoutingGraph::build(*map_, *traffic_rules_);
}

std::size_t LaneletRouter::lanelet_count() const
{
  return map_->laneletLayer.size();
}

std::optional<lanelet::ConstLanelet> LaneletRouter::nearest_lanelet(double x, double y) const
{
  const auto candidates = lanelet::geometry::findNearest(map_->laneletLayer, lanelet::BasicPoint2d(x, y), 5);
  if (candidates.empty()) {
    return std::nullopt;
  }
  lanelet::ConstLanelet best_ll = candidates.front().second;
  double best_dist = candidates.front().first;
  const double ref_yaw = lanelet_yaw_near(best_ll, x, y);
  for (std::size_t i = 1; i < candidates.size(); ++i) {
    const auto & [dist, ll] = candidates[i];
    const double yaw = lanelet_yaw_near(ll, x, y);
    const double dot = std::cos(yaw) * std::cos(ref_yaw) + std::sin(yaw) * std::sin(ref_yaw);
    if (dot > 0.0 && dist < best_dist) {
      best_dist = dist;
      best_ll = ll;
    }
  }
  return best_ll;
}

std::optional<std::vector<Point2d>> LaneletRouter::plan_route(double x0, double y0, double x1, double y1) const
{
  const auto start_ll = nearest_lanelet(x0, y0);
  const auto end_ll = nearest_lanelet(x1, y1);
  if (!start_ll || !end_ll) {
    return std::nullopt;
  }

  const double t0 = project_onto_centerline(*start_ll, x0, y0);
  const double t1 = project_onto_centerline(*end_ll, x1, y1);

  std::vector<lanelet::ConstLanelet> sequence;
  if (start_ll->id() == end_ll->id() && t0 <= t1) {
    sequence.push_back(*start_ll);
  } else if (start_ll->id() == end_ll->id()) {
    // Goal is behind on the same lanelet: loop around via the next one instead of reversing.
    const auto following = routing_graph_->following(*start_ll);
    if (following.empty()) {
      return std::nullopt;
    }
    const auto route1 = routing_graph_->getRoute(*start_ll, following.front(), 0);
    const auto route2 = routing_graph_->getRoute(following.front(), *end_ll, 0);
    if (!route1 || !route2) {
      return std::nullopt;
    }
    const auto path1 = route1->shortestPath();
    const auto path2 = route2->shortestPath();
    sequence.assign(path1.begin(), path1.end());
    sequence.insert(sequence.end(), std::next(path2.begin()), path2.end());
  } else {
    const auto route = routing_graph_->getRoute(*start_ll, *end_ll, 0);
    if (!route) {
      return std::nullopt;
    }
    const auto path = route->shortestPath();
    sequence.assign(path.begin(), path.end());
  }

  std::vector<Point2d> coords;
  auto append = [&coords](const std::vector<Point2d> & pts, bool drop_last) {
      coords.insert(coords.end(), pts.begin(), drop_last && !pts.empty() ? std::prev(pts.end()) : pts.end());
    };
  for (std::size_t i = 0; i < sequence.size(); ++i) {
    const bool is_first = i == 0;
    const bool is_last = i + 1 == sequence.size();
    if (is_first && is_last) {
      append(sample_between(sequence[i], t0, t1), false);
    } else if (is_first) {
      append(sample_between(sequence[i], t0, 1.0), true);
    } else if (is_last) {
      append(sample_between(sequence[i], 0.0, t1), false);
    } else {
      append(centerline_points(sequence[i]), true);
    }
  }
  return coords;
}

}  // namespace dummy_robot
