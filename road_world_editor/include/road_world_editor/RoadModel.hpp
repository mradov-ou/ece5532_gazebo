#pragma once
#include <road_world_editor/GazeboModel.hpp>
#include <road_world_editor/ModelPort.hpp>

#include <array>
#include <tuple>
#include <utility>

using namespace lanelet;

namespace road_world_editor {

template <int Type>
class RoadModel : public GazeboModel
{
  public:
    RoadModel(const Eigen::Vector2d& starting_position, const ModelOrientation& starting_orientation, const std::string& frame_id, std::string model_id = "")
        : GazeboModel(ROAD_MODEL_INFO[Type].mesh_path, ROAD_MODEL_INFO[Type].four_lane_mesh_path, frame_id, model_id)
    {
      control_marker_offset_ = ROAD_MODEL_INFO[Type].control_marker_offset;
      model_position_ = starting_position;
      model_orientation_ = starting_orientation;
      initModel();
    }

    int getModelType() const override { return Type; }

    void initPorts(const std::vector<std::string>& port_ids = std::vector<std::string>()) override {
      auto this_shared = shared_from_this();
      auto port_offsets = ROAD_MODEL_INFO[Type].port_offsets;

      if (port_ids.empty()) {
        for (auto& port_offset : port_offsets) {
          ports_.push_back(std::make_shared<ModelPort>(this_shared, port_offset, ""));
        }
      } else {
        if (port_ids.size() != port_offsets.size()) {
          throw std::runtime_error("Number of port IDs does not match number of ports");
        }
        for (size_t i = 0; i < port_offsets.size(); i++) {
          ports_.push_back(std::make_shared<ModelPort>(this_shared, port_offsets[i], port_ids[i]));
        }
      }
    }

    std::vector<Lanelet> generateLanelet() override {
      switch (Type) {
        case Intersection:
        return intersectionLanelets();
        return std::vector<Lanelet>();
        case Straight100m:
        return straightRoadLanelets(100.0);
        case Straight50m:
        return straightRoadLanelets(50.0);
        case Straight20m:
        return straightRoadLanelets(20.0);
        case Straight10m:
        return straightRoadLanelets(10.0);
        case Curved50m:
        return curvedRoadLanelets(50.0);
        case Curved100m:
        return curvedRoadLanelets(100.0);
        case LaneMerge:
        return laneMergeLanelets();
        default:
        return std::vector<Lanelet>();
      }
    }

    // Builds one lane-boundary LineString running from local (x_start, y_start) to local
    // (x_end, y_end) (straight for constant y, tapered when y_start != y_end), projected into
    // the map frame. `reversed` swaps the point order so the line runs from the end point to
    // the start point instead -- this file's convention for encoding a lanelet's travel
    // direction, since two bounds sharing the same order must travel the same direction.
    LineString3d buildLaneBoundary(double x_start, double y_start, double x_end, double y_end, bool reversed) {
      auto toMapPoint = [this](double x, double y) {
        return Point3d(utils::getId(), utils::to3D(BasicPoint2d(offsetPoint(model_position_, Eigen::Vector2d(x, y), model_orientation_))));
      };
      Point3d p_start = toMapPoint(x_start, y_start);
      Point3d p_end = toMapPoint(x_end, y_end);
      return reversed ? LineString3d(utils::getId(), {p_end, p_start}) : LineString3d(utils::getId(), {p_start, p_end});
    }

    std::vector<Lanelet> intersectionLanelets() {
      constexpr double kLaneWidth = 4.0;
      constexpr double kRightTurnRadius = 18.0;
      constexpr double kLeftTurnRadius = 22.0;
      const auto& port_offsets = ROAD_MODEL_INFO[Intersection].port_offsets;

      auto rotateVec = [](const Eigen::Vector2d& v, double angle_deg) {
        double angle_rad = M_PI / 180.0 * angle_deg;
        double c = cos(angle_rad), s = sin(angle_rad);
        return Eigen::Vector2d(c * v.x() - s * v.y(), s * v.x() + c * v.y());
      };
      auto leftOf = [&](const Eigen::Vector2d& v) { return rotateVec(v, 90.0); };
      auto toPoint3d = [this](const Eigen::Vector2d& local_pt) {
        return Point3d(utils::getId(), utils::to3D(BasicPoint2d(offsetPoint(model_position_, local_pt, model_orientation_))));
      };

      // Builds a one-way lanelet from sampled centerline points paired with the direction of
      // travel at each point, offsetting left/right bounds by half the lane width.
      auto buildLanelet = [&](const std::vector<Eigen::Vector2d>& center_pts, const std::vector<Eigen::Vector2d>& travel_dirs) {
        std::vector<Point3d> left_pts;
        std::vector<Point3d> right_pts;
        for (size_t i = 0; i < center_pts.size(); i++) {
          Eigen::Vector2d lateral = leftOf(travel_dirs[i]) * (kLaneWidth / 2.0);
          left_pts.push_back(toPoint3d(center_pts[i] + lateral));
          right_pts.push_back(toPoint3d(center_pts[i] - lateral));
        }
        Lanelet lanelet;
        lanelet.setLeftBound(LineString3d(utils::getId(), left_pts));
        lanelet.setRightBound(LineString3d(utils::getId(), right_pts));
        return lanelet;
      };

      // Builds a quarter-circle turn lanelet starting at entry_point heading travel_in.
      // sigma = -1.0 sweeps clockwise (right turn), +1.0 sweeps counter-clockwise (left turn).
      auto buildTurn = [&](const Eigen::Vector2d& entry_point, const Eigen::Vector2d& travel_in, double sigma, double radius) {
        Eigen::Vector2d u = rotateVec(travel_in, 90.0 * sigma);
        Eigen::Vector2d center = entry_point + radius * u;
        std::vector<Eigen::Vector2d> center_pts;
        std::vector<Eigen::Vector2d> travel_dirs;
        for (int arc_angle_deg = 0; arc_angle_deg <= 90; arc_angle_deg += 2) {
          center_pts.push_back(center - radius * rotateVec(u, sigma * arc_angle_deg));
          travel_dirs.push_back(rotateVec(travel_in, sigma * arc_angle_deg));
        }
        return buildLanelet(center_pts, travel_dirs);
      };

      std::vector<Lanelet> lanelets;
      for (int entry = 0; entry < 4; entry++) {
        if (!ports_[entry]->isConnected()) {
          continue;  // No road attached at this port, so nothing can enter here.
        }
        Eigen::Vector2d port_pos = port_offsets[entry];
        Eigen::Vector2d d_entry = port_pos.normalized();
        Eigen::Vector2d travel_in = -d_entry;
        Eigen::Vector2d entry_point = port_pos + leftOf(d_entry) * (kLaneWidth / 2.0);

        // Straight-through movement to the opposite port.
        int straight_exit = (entry + 2) % 4;
        if (ports_[straight_exit]->isConnected()) {
          Eigen::Vector2d exit_port_pos = port_offsets[straight_exit];
          Eigen::Vector2d d_exit = exit_port_pos.normalized();
          Eigen::Vector2d exit_point = exit_port_pos - leftOf(d_exit) * (kLaneWidth / 2.0);
          lanelets.push_back(buildLanelet({entry_point, exit_point}, {travel_in, travel_in}));
        }

        // Right turn (18m center radius) to the next port clockwise.
        int right_exit = (entry + 1) % 4;
        if (ports_[right_exit]->isConnected()) {
          lanelets.push_back(buildTurn(entry_point, travel_in, -1.0, kRightTurnRadius));
        }

        // Left turn (22m center radius) to the next port counter-clockwise.
        int left_exit = (entry + 3) % 4;
        if (ports_[left_exit]->isConnected()) {
          lanelets.push_back(buildTurn(entry_point, travel_in, 1.0, kLeftTurnRadius));
        }
      }
      return lanelets;
    }

    // Builds `lane_count` adjacent 4m-wide lanelets covering local y in [0, lane_count * 4]
    // (mirrored onto negative y by the caller when needed), each running the full
    // [x_start, x_end] length. Consecutive lanelets share the exact same boundary LineString
    // object (never a fresh, merely-coincident copy) so lanelet2's routing graph recognizes
    // them as lane-change neighbors. `first_bound` seeds the innermost shared edge -- pass the
    // road's centerline (or its .invert()) to butt this chain up against an opposite-direction
    // chain built the same way.
    std::vector<Lanelet> buildLaneChain(int lane_count, double x_start, double y_start_sign, double x_end, double y_end_sign, bool reversed, const LineString3d& first_bound) {
      std::vector<Lanelet> lanelets;
      LineString3d inner_bound = first_bound;
      for (int i = 0; i < lane_count; i++) {
        LineString3d outer_bound = buildLaneBoundary(x_start, y_start_sign * (i + 1) * 4.0, x_end, y_end_sign * (i + 1) * 4.0, reversed);
        Lanelet lanelet;
        lanelet.setLeftBound(inner_bound);
        lanelet.setRightBound(outer_bound);
        lanelets.push_back(lanelet);
        inner_bound = outer_bound;
      }
      return lanelets;
    }

    std::vector<Lanelet> straightRoadLanelets(const double& road_segment_length) {
      bool four_lane = variant_.find("four_lane") != variant_.npos;
      bool two_way = variant_.find("two_way") != variant_.npos;
      int lanes_per_direction = four_lane ? 2 : 1;

      LineString3d center_lane_marking = buildLaneBoundary(0.0, 0.0, road_segment_length, 0.0, false);

      // The right-hand (negative-y) lanes always travel local +x.
      auto lanelets = buildLaneChain(lanes_per_direction, 0.0, -1.0, road_segment_length, -1.0, false, center_lane_marking);

      // The left-hand (positive-y) lanes travel the opposite direction (-x) when two-way --
      // sharing the centerline as their innermost lanelet's LEFT bound specifically, since
      // lanelet2's OSM writer drops an inverted LineString's point order when it's a
      // rightBound, desyncing that lanelet's two bounds on a save/reload round-trip -- or the
      // same +x direction as the right-hand lanes when one-way (no oncoming traffic).
      auto left_lanelets = buildLaneChain(lanes_per_direction, 0.0, 1.0, road_segment_length, 1.0, two_way, two_way ? center_lane_marking.invert() : center_lane_marking);
      lanelets.insert(lanelets.end(), left_lanelets.begin(), left_lanelets.end());
      return lanelets;
    }

    std::vector<Lanelet> laneMergeLanelets() {
      constexpr double kLength = 50.0;
      // Local x=0 is this model's wide (four-lane) end and x=kLength its narrow (two-lane)
      // end -- see ROAD_MODEL_INFO's port_offsets. Each side's inner (through) lane stays 4m
      // wide for the full length; its outer (merging) lane tapers from 4m wide at the wide end
      // down to a single point at the narrow end, where it disappears into the through lane.
      bool two_way = variant_.find("two_way") != variant_.npos;

      LineString3d center_lane_marking = buildLaneBoundary(0.0, 0.0, kLength, 0.0, false);

      LineString3d right_inner_bound = buildLaneBoundary(0.0, -4.0, kLength, -4.0, false);
      Lanelet right_through;
      right_through.setLeftBound(center_lane_marking);
      right_through.setRightBound(right_inner_bound);

      LineString3d right_outer_bound = buildLaneBoundary(0.0, -8.0, kLength, -4.0, false);
      Lanelet right_merge;
      right_merge.setLeftBound(right_inner_bound);
      right_merge.setRightBound(right_outer_bound);

      // See straightRoadLanelets(): the left-hand side travels -x (oncoming traffic) when
      // two-way, sharing the centerline as its innermost lanelet's LEFT bound specifically to
      // survive an OSM save/reload round-trip; otherwise it travels the same +x direction as
      // the right-hand side (one-way, both ends narrowing from four lanes to two in the same
      // direction of travel).
      LineString3d left_inner_bound = buildLaneBoundary(0.0, 4.0, kLength, 4.0, two_way);
      Lanelet left_through;
      left_through.setLeftBound(two_way ? center_lane_marking.invert() : center_lane_marking);
      left_through.setRightBound(left_inner_bound);

      LineString3d left_outer_bound = buildLaneBoundary(0.0, 8.0, kLength, 4.0, two_way);
      Lanelet left_merge;
      left_merge.setLeftBound(left_inner_bound);
      left_merge.setRightBound(left_outer_bound);

      return std::vector<Lanelet>{right_through, right_merge, left_through, left_merge};
    }

    std::vector<Lanelet> curvedRoadLanelets(const double& curve_radius) {
      Lanelet left_lanelet;
      Lanelet right_lanelet;
      std::vector<Point3d> center_points;
      std::vector<Point3d> left_points;
      std::vector<Point3d> right_points;
      for (int arc_angle_deg = 0; arc_angle_deg <= 90; arc_angle_deg += 2) {
        center_points.push_back(Point3d(utils::getId(), utils::to3D(
          BasicPoint2d(offsetPoint(model_position_,
            Eigen::Vector2d(
              curve_radius * sin(M_PI / 180.0 * arc_angle_deg),
              curve_radius * (cos(M_PI / 180.0 * arc_angle_deg) - 1.0)
            ), model_orientation_
          )))));
        left_points.push_back(Point3d(utils::getId(), utils::to3D(
          BasicPoint2d(offsetPoint(model_position_,
            Eigen::Vector2d(
              (curve_radius + 4.0) * sin(M_PI / 180.0 * (90 - arc_angle_deg)),
              (curve_radius + 4.0) * cos(M_PI / 180.0 * (90 - arc_angle_deg)) - curve_radius
            ), model_orientation_
          )))));
        right_points.push_back(Point3d(utils::getId(), utils::to3D(
          BasicPoint2d(offsetPoint(model_position_,
            Eigen::Vector2d(
              (curve_radius - 4.0) * sin(M_PI / 180.0 * arc_angle_deg),
              (curve_radius - 4.0) * cos(M_PI / 180.0 * arc_angle_deg) - curve_radius
            ), model_orientation_
          )))));
      }
      LineString3d center_lane_marking(utils::getId(), center_points);
      LineString3d left_lane_marking(utils::getId(), left_points);
      LineString3d right_lane_marking(utils::getId(), right_points);

      right_lanelet.setLeftBound(center_lane_marking);
      right_lanelet.setRightBound(right_lane_marking);
      // See straightRoadLanelets(): the inverted shared centerline must be left_lanelet's
      // LEFT bound specifically, or the inversion is lost on an OSM save/reload round-trip.
      left_lanelet.setLeftBound(center_lane_marking.invert());
      left_lanelet.setRightBound(left_lane_marking);
      return std::vector<Lanelet>{left_lanelet, right_lanelet};
    }

    visualization_msgs::msg::Marker leftDirectionMarker() override
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.lifetime.sec = 0;
      m.lifetime.nanosec = 300000000;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.ns = model_id_;
      m.id = 1;
      auto marker_pos = offsetPoint(model_position_, Eigen::Vector2d(0.0, 4.0), model_orientation_);
      m.pose.position.x = marker_pos.x();
      m.pose.position.y = marker_pos.y();
      m.pose.position.z = 1.0;

      bool two_way = variant_.find("two_way") != variant_.npos;
      switch (model_orientation_) {
        case ModelOrientation::EAST:
          m.pose.orientation.w = (two_way ? 0.0 : 1.0);
          m.pose.orientation.z = (two_way ? 1.0 : 0.0);
          break;
        case ModelOrientation::NORTH:
          m.pose.orientation.w = SINCOS_M_PI_4;
          m.pose.orientation.z = (two_way ? -SINCOS_M_PI_4 : SINCOS_M_PI_4);
          break;
        case ModelOrientation::WEST:
          m.pose.orientation.w = (two_way ? 1.0 : 0.0);
          m.pose.orientation.z = (two_way ? 0.0 : 1.0);
          break;
        case ModelOrientation::SOUTH:
          m.pose.orientation.w = SINCOS_M_PI_4;
          m.pose.orientation.z = (two_way ? SINCOS_M_PI_4 : -SINCOS_M_PI_4);
          break;
      }
      m.scale.x = 5.0;
      m.scale.y = 1.0;
      m.scale.z = 1.0;

      if (Type == Intersection) {
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.color.a = 0.0;
      } else {
        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 1.0;
      }
      return m;
    }

    visualization_msgs::msg::Marker rightDirectionMarker() override {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.lifetime.sec = 0;
      m.lifetime.nanosec = 300000000;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.ns = model_id_;
      m.id = 2;
      if (Type == Intersection) {
        return m;
      }
      auto marker_pos = offsetPoint(model_position_, Eigen::Vector2d(0.0, -4.0), model_orientation_);
      m.pose.position.x = marker_pos.x();
      m.pose.position.y = marker_pos.y();
      m.pose.position.z = 1.0;
      switch (model_orientation_) {
        case ModelOrientation::EAST:
          m.pose.orientation.w = 1.0;
          m.pose.orientation.z = 0.0;
          break;
        case ModelOrientation::NORTH:
          m.pose.orientation.w = SINCOS_M_PI_4;
          m.pose.orientation.z = SINCOS_M_PI_4;
          break;
        case ModelOrientation::WEST:
          m.pose.orientation.w = 0;
          m.pose.orientation.z = 1;
          break;
        case ModelOrientation::SOUTH:
          m.pose.orientation.w = SINCOS_M_PI_4;
          m.pose.orientation.z = -SINCOS_M_PI_4;
          break;
      }
      m.scale.x = 5.0;
      m.scale.y = 1.0;
      m.scale.z = 1.0;
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      return m;
      visualization_msgs::msg::Marker marker;
      return marker;
    }
};

namespace detail {

template <int Type>
GazeboModel::SharedPtr makeRoadModel(const Eigen::Vector2d& starting_position, const ModelOrientation& starting_orientation, const std::string& frame_id, const std::string& model_id)
{
  return std::make_shared<RoadModel<Type>>(starting_position, starting_orientation, frame_id, model_id);
}

template <std::size_t... Is>
constexpr auto makeRoadModelTable(std::index_sequence<Is...>)
{
  return std::array<GazeboModel::SharedPtr (*)(const Eigen::Vector2d&, const ModelOrientation&, const std::string&, const std::string&), sizeof...(Is)>{
      &makeRoadModel<Is>...};
}

}  // namespace detail

using RoadModelFactory = GazeboModel::SharedPtr (*)(const Eigen::Vector2d&, const ModelOrientation&, const std::string&, const std::string&);
using RoadModelFactoryTable = std::array<RoadModelFactory, std::tuple_size_v<decltype(ROAD_MODEL_INFO)>>;
extern const RoadModelFactoryTable ROAD_MODEL_FACTORIES;

}  // namespace road_world_editor