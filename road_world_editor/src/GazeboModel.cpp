#include <road_world_editor/GazeboModel.hpp>
#include <road_world_editor/ModelPort.hpp>

namespace road_world_editor {

GazeboModel::GazeboModel(const std::string& mesh_path, const std::string& frame_id, const std::string& model_id)
    : mesh_path_(mesh_path), frame_id_(frame_id), model_id_(model_id.empty() ? generateUuid() : model_id)
{
}

void GazeboModel::initModel()
{
  initMarkers();
  moveMarkers();
  applyMarkerChangesUnlessBatching();
}

void GazeboModel::move(const Eigen::Vector2d& new_model_pos)
{
  if (moved_) {
    return;
  }

  model_position_ = new_model_pos;
  moved_ = true;
  moveMarkers();
  for (auto port : ports_) {
    port->moveWithParent();
  }
}

void GazeboModel::setModelPosition(const Eigen::Vector2d& new_position)
{
  initiateMove();
  move(new_position);
  g_common_data.last_updated_model = shared_from_this();
  g_common_data.marker_moved_stamp = g_common_data.current_time;
  g_common_data.marker_server->applyChanges();
}

void GazeboModel::initiateMove()
{
  if (!moved_) {
    return;
  }
  moved_ = false;
  for (auto port : ports_) {
    port->initiateMove();
  }
}

void GazeboModel::rotateClockwise() { rotate(clockwiseRotation(model_orientation_)); }

void GazeboModel::rotateCounterClockwise() { rotate(counterClockwiseRotation(model_orientation_)); }

void GazeboModel::disconnectAllPorts()
{
  for (auto& port : ports_) {
    port->breakConnection();
  }
}

void GazeboModel::rotate(const ModelOrientation& new_model_orientation)
{
  model_orientation_ = new_model_orientation;
  moveMarkers();

  for (auto& port : ports_) {
    port->breakConnection();
  }
  moved_ = true;
  initiateMove();
  move(model_position_);
  g_common_data.marker_server->applyChanges();
}

std::vector<visualization_msgs::msg::Marker> GazeboModel::getPortMarkers()
{
  std::vector<visualization_msgs::msg::Marker> markers;
  for (auto port : ports_) {
    markers.push_back(port->marker_);
  }
  return markers;
}

void GazeboModel::initMarkers()
{
  mesh_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  mesh_marker_.mesh_resource = mesh_path_;
  mesh_marker_.ns = model_id_;
  mesh_marker_.id = 0;
  mesh_marker_.header.frame_id = frame_id_;
  mesh_marker_.lifetime.sec = 0;
  mesh_marker_.lifetime.nanosec = 300000000;
  mesh_marker_.scale.x = 1.0;
  mesh_marker_.scale.y = 1.0;
  mesh_marker_.scale.z = 1.0;
  mesh_marker_.color.r = 1.0f;
  mesh_marker_.color.g = 1.0f;
  mesh_marker_.color.b = 1.0f;
  mesh_marker_.color.a = 1.0f;

  control_marker_.header.frame_id = frame_id_;
  control_marker_.name = model_id_;
  control_marker_.description = "Move model";
  control_marker_.scale = 1.0;

  visualization_msgs::msg::Marker cube_marker;
  cube_marker.type = visualization_msgs::msg::Marker::CUBE;
  cube_marker.scale.x = 2.0;
  cube_marker.scale.y = 2.0;
  cube_marker.scale.z = 2.0;
  cube_marker.color.r = 0.0;
  cube_marker.color.g = 1.0;
  cube_marker.color.b = 0.0;
  cube_marker.color.a = 1.0;

  // Rotating the control's local x-axis (its plane normal for MOVE_PLANE) onto
  // the world z-axis constrains dragging to the world x/y plane.
  visualization_msgs::msg::InteractiveMarkerControl move_control;
  move_control.orientation.w = 0.7071067811865476;
  move_control.orientation.x = 0.0;
  move_control.orientation.y = 0.7071067811865476;
  move_control.orientation.z = 0.0;
  move_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  move_control.always_visible = true;
  move_control.markers.push_back(cube_marker);
  control_marker_.controls.push_back(move_control);

  g_common_data.marker_server->insert(control_marker_);
  g_common_data.marker_server->setCallback(control_marker_.name, std::bind(&GazeboModel::processMarkerFeedback, this, std::placeholders::_1));
  applyMarkerChangesUnlessBatching();
}

void GazeboModel::processMarkerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  auto new_model_pos = offsetPoint(Eigen::Vector2d(feedback->pose.position.x, feedback->pose.position.y), -control_marker_offset_, model_orientation_);

  if (g_common_data.marker_moved_stamp.get_clock_type() != g_common_data.current_time.get_clock_type()) {
    g_common_data.marker_moved_stamp = g_common_data.current_time;
    return;
  }

  if ((g_common_data.current_time - g_common_data.marker_moved_stamp).seconds() > 0.1) {
    initiateMove();
    move(new_model_pos);
    g_common_data.marker_moved_stamp = g_common_data.current_time;
    g_common_data.last_updated_model = shared_from_this();
    g_common_data.marker_server->applyChanges();
  }
}

void GazeboModel::moveMarkers()
{
  mesh_marker_.pose.position.x = model_position_.x();
  mesh_marker_.pose.position.y = model_position_.y();
  switch (model_orientation_) {
    case ModelOrientation::EAST:
      mesh_marker_.pose.orientation.w = 1.0;
      mesh_marker_.pose.orientation.z = 0.0;
      break;
    case ModelOrientation::NORTH:
      mesh_marker_.pose.orientation.w = SINCOS_M_PI_4;
      mesh_marker_.pose.orientation.z = SINCOS_M_PI_4;
      break;
    case ModelOrientation::WEST:
      mesh_marker_.pose.orientation.w = 0;
      mesh_marker_.pose.orientation.z = 1;
      break;
    case ModelOrientation::SOUTH:
      mesh_marker_.pose.orientation.w = SINCOS_M_PI_4;
      mesh_marker_.pose.orientation.z = -SINCOS_M_PI_4;
      break;
  }

  auto marker_pos_v = offsetPoint(model_position_, control_marker_offset_, model_orientation_);
  control_marker_.pose.position.x = marker_pos_v.x();
  control_marker_.pose.position.y = marker_pos_v.y();
  g_common_data.marker_server->setPose(control_marker_.name, control_marker_.pose);
}

}  // namespace road_world_editor