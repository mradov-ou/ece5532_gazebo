#pragma once

#include <array>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <interactive_markers/interactive_marker_server.hpp>

namespace road_world_editor {

class GazeboModel;
class ModelPort;

struct RoadModelInfo {
    std::string name;
    std::string mesh_path;
    // Preview mesh to use instead of mesh_path when the model's selected variant is a
    // four-lane one; empty for models with no four-lane preview mesh (e.g. Curved 50m, which
    // has no four-lane variant at all).
    std::string four_lane_mesh_path;
    std::string gazebo_model_path;
    std::vector<std::string> variants;
    Eigen::Vector2d control_marker_offset;
    std::vector<Eigen::Vector2d> port_offsets;
};

static const std::string DEFAULT_ROAD_VARIANT = "two_way_two_lane";
static const std::vector<std::string> ALL_ROAD_VARIANTS{
    "two_way_two_lane",
    "one_way_two_lane",
    "two_way_four_lane",
    "one_way_four_lane",
};
static const std::vector<std::string> TWO_LANE_ROAD_VARIANTS{
    "two_way_two_lane",
    "one_way_two_lane",
};
static const std::vector<std::string> LANE_MERGE_VARIANTS{
    "two_way_merge",
    "one_way_merge",
};

enum RoadModelType {
  Intersection = 0,
  Straight10m = 1,
  Straight20m = 2,
  Straight50m = 3,
  Straight100m = 4,
  Curved50m = 5,
  Curved100m = 6,
  LaneMerge = 7
};

static const std::array<RoadModelInfo, 8> ROAD_MODEL_INFO{{
    {"Intersection",
     "package://road_world_editor/model_meshes/intersection.stl",
     "",
     "package://ece5532_gazebo/models/intersection",
     {},
     Eigen::Vector2d(0, 0),
     {
         Eigen::Vector2d(20, 0),
         Eigen::Vector2d(0, 20),
         Eigen::Vector2d(-20, 0),
         Eigen::Vector2d(0, -20),
     }},
    {"Straight 10m",
     "package://road_world_editor/model_meshes/straight_two_lane_10m.stl",
     "package://road_world_editor/model_meshes/straight_four_lane_10m.stl",
     "package://ece5532_gazebo/models/straight_road_10m",
     ALL_ROAD_VARIANTS,
     Eigen::Vector2d(5, 0),
     {
         Eigen::Vector2d(0, 0),
         Eigen::Vector2d(10, 0),
     }},
    {"Straight 20m",
     "package://road_world_editor/model_meshes/straight_two_lane_20m.stl",
     "package://road_world_editor/model_meshes/straight_four_lane_20m.stl",
     "package://ece5532_gazebo/models/straight_road_20m",
     ALL_ROAD_VARIANTS,
     Eigen::Vector2d(10, 0),
     {
         Eigen::Vector2d(0, 0),
         Eigen::Vector2d(20, 0),
     }},
    {"Straight 50m",
     "package://road_world_editor/model_meshes/straight_two_lane_50m.stl",
     "package://road_world_editor/model_meshes/straight_four_lane_50m.stl",
     "package://ece5532_gazebo/models/straight_road_50m",
     ALL_ROAD_VARIANTS,
     Eigen::Vector2d(25, 0),
     {
         Eigen::Vector2d(0, 0),
         Eigen::Vector2d(50, 0),
     }},
    {"Straight 100m",
     "package://road_world_editor/model_meshes/straight_two_lane_100m.stl",
     "package://road_world_editor/model_meshes/straight_four_lane_100m.stl",
     "package://ece5532_gazebo/models/straight_road_100m",
     ALL_ROAD_VARIANTS,
     Eigen::Vector2d(50, 0),
     {
         Eigen::Vector2d(0, 0),
         Eigen::Vector2d(100, 0),
     }},
    {"Curved 50m",
     "package://road_world_editor/model_meshes/curved_50m.stl",
     "",
     "package://ece5532_gazebo/models/curved_road_50m",
     TWO_LANE_ROAD_VARIANTS,
     Eigen::Vector2d(50 * 0.5 * sqrt(2), 50 * (0.5 * sqrt(2) - 1)),
     {
         Eigen::Vector2d(0, 0),
         Eigen::Vector2d(50, -50),
     }},
    {"Curved 100m",
     "package://road_world_editor/model_meshes/curved_two_lane_100m.stl",
     "package://road_world_editor/model_meshes/curved_four_lane_100m.stl",
     "package://ece5532_gazebo/models/curved_road_100m",
     ALL_ROAD_VARIANTS,
     Eigen::Vector2d(100 * 0.5 * sqrt(2), 100 * (0.5 * sqrt(2) - 1)),
     {
         Eigen::Vector2d(0, 0),
         Eigen::Vector2d(100, -100),
     }},
    {"Lane Merge",
     "package://road_world_editor/model_meshes/lane_merge.stl",
     "",
     "package://ece5532_gazebo/models/lane_merge",
     LANE_MERGE_VARIANTS,
     Eigen::Vector2d(25, 0),
     {
         Eigen::Vector2d(0, 0),
         Eigen::Vector2d(50, 0),
     }},
}};

struct CommonData {
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server;
    rclcpp::Time marker_moved_stamp;
    rclcpp::Time current_time;
    std::weak_ptr<GazeboModel> last_updated_model;
    // While true, GazeboModel/ModelPort skip their usual applyChanges() calls so a caller
    // (e.g. loading a whole file) can batch many inserts/connections into a single update
    // instead of flooding interactive_markers with one update per model/port.
    bool batch_marker_updates = false;
};
extern CommonData g_common_data;

static inline void applyMarkerChangesUnlessBatching()
{
  if (!g_common_data.batch_marker_updates) {
    g_common_data.marker_server->applyChanges();
  }
}

enum class ModelOrientation : uint8_t {
  EAST = 0,
  NORTH = 1,
  WEST = 2,
  SOUTH = 3,
};

static inline double squaredDist(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) { return (Eigen::Vector2d(p1.x, p1.y) - Eigen::Vector2d(p2.x, p2.y)).squaredNorm(); }
static inline std::string generateUuid() { return boost::uuids::to_string(boost::uuids::random_generator()()); }
static inline ModelOrientation clockwiseRotation(const ModelOrientation& old_orientation)
{
  switch (old_orientation) {
    case ModelOrientation::EAST:
      return ModelOrientation::SOUTH;
    case ModelOrientation::NORTH:
      return ModelOrientation::EAST;
    case ModelOrientation::WEST:
      return ModelOrientation::NORTH;
    case ModelOrientation::SOUTH:
    default:
      return ModelOrientation::WEST;
  }
}
static inline ModelOrientation counterClockwiseRotation(const ModelOrientation& old_orientation)
{
  switch (old_orientation) {
    case ModelOrientation::EAST:
      return ModelOrientation::NORTH;
    case ModelOrientation::NORTH:
      return ModelOrientation::WEST;
    case ModelOrientation::WEST:
      return ModelOrientation::SOUTH;
    case ModelOrientation::SOUTH:
    default:
      return ModelOrientation::EAST;
  }
}
static inline Eigen::Vector2d offsetPoint(const Eigen::Vector2d& start_point, const Eigen::Vector2d& offset, const ModelOrientation& orientation)
{
  Eigen::Isometry2d rot;
  rot.setIdentity();
  switch (orientation) {
    case ModelOrientation::NORTH:
      rot.rotate(M_PI_2);
      break;
    case ModelOrientation::WEST:
      rot.rotate(M_PI);
      break;
    case ModelOrientation::SOUTH:
      rot.rotate(-M_PI_2);
      break;
  }
  return start_point + rot * offset;
}

}  // namespace road_world_editor