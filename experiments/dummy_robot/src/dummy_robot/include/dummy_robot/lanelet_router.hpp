#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

namespace dummy_robot
{

using Point2d = std::pair<double, double>;

// Trimmed port of mission_manager/backend/core/map_server.py's routing,
// deliberately duplicated: a robot plans its own path from a goal alone.
class LaneletRouter
{
public:
  LaneletRouter(const std::string & osm_path, const std::string & projector_yaml);

  // Local-xy centerline polyline from (x0, y0) to (x1, y1), or nullopt if unreachable.
  std::optional<std::vector<Point2d>> plan_route(double x0, double y0, double x1, double y1) const;

  std::size_t lanelet_count() const;

  // Distance from (x, y) to the nearest lanelet; 0 inside one. Infinity on an empty map.
  double distance_to_map(double x, double y) const;

private:
  std::optional<lanelet::ConstLanelet> nearest_lanelet(double x, double y) const;

  lanelet::LaneletMapPtr map_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  lanelet::routing::RoutingGraphUPtr routing_graph_;
};

}  // namespace dummy_robot
