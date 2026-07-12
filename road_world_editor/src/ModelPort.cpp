#include <road_world_editor/ModelPort.hpp>
#include <road_world_editor/GazeboModel.hpp>

namespace road_world_editor {

ModelPort::ModelPort(std::shared_ptr<GazeboModel> parent_model, Eigen::Vector2d rel_pos, const std::string& port_id)
    : rel_pos_(rel_pos), port_id_(port_id.empty() ? generateUuid() : port_id), parent_model_(parent_model)
{
  initMarker();
}

/**
 * @brief Update the position of this marker to align with parent model
 */
void ModelPort::moveWithParent()
{
  if (moved_) {
    return;
  }

  moved_ = true;
  auto marker_pos_v = offsetPoint(parent_model_->model_position_, rel_pos_, parent_model_->model_orientation_);
  marker_.pose.position.x = marker_pos_v.x();
  marker_.pose.position.y = marker_pos_v.y();

  // Move each connected port and their parents
  if (auto other = connected_port_.lock()) {
    other->moveAbsolute(marker_.pose.position);
  }
}

void ModelPort::initiateMove()
{
  if (!moved_) {
    return;
  }

  moved_ = false;
  parent_model_->initiateMove();
  if (auto other = connected_port_.lock()) {
    other->initiateMove();
  }
}

/**
 * @brief Force the marker position to a new point and then move parent accordingly
 */
void ModelPort::moveAbsolute(const geometry_msgs::msg::Point& new_position)
{
  if (moved_) {
    return;
  }

  moved_ = true;
  marker_.pose.position = new_position;
  parent_model_->move(offsetPoint(Eigen::Vector2d(new_position.x, new_position.y), -rel_pos_, parent_model_->model_orientation_));
}

void ModelPort::breakConnection()
{
  if (auto other = connected_port_.lock()) {
    other->connected_port_.reset();
    other->marker_.color.r = 0.0f;
    other->marker_.color.g = 1.0f;
    other->marker_.color.b = 1.0f;
    connected_port_.reset();
  }
  marker_.color.r = 0.0f;
  marker_.color.g = 1.0f;
  marker_.color.b = 1.0f;
}

bool ModelPort::isConnected() { return connected_port_.lock() != nullptr; }

std::string ModelPort::getConnectedPortId() const
{
  auto other = connected_port_.lock();
  return other ? other->port_id_ : "";
}

void ModelPort::connectTo(const SharedPtr& other)
{
  if (other.get() == this) {
    RCLCPP_WARN(rclcpp::get_logger("ModelPort"), "Cannot connect to self.");
    return;
  }
  moved_ = true;
  initiateMove();
  moveAbsolute(other->getPosition());
  applyMarkerChangesUnlessBatching();

  connected_port_ = other;
  other->connected_port_ = shared_from_this();
  marker_.color.r = 1.0f;
  marker_.color.g = 0.0f;
  marker_.color.b = 0.0f;
  other->marker_.color.r = 1.0f;
  other->marker_.color.g = 0.0f;
  other->marker_.color.b = 0.0f;
}

void ModelPort::initMarker()
{
  marker_.type = visualization_msgs::msg::Marker::SPHERE;
  marker_.ns = port_id_;
  marker_.id = 0;
  marker_.header.frame_id = parent_model_->frame_id_;
  marker_.lifetime.sec = 0;
  marker_.lifetime.nanosec = 300000000;
  marker_.scale.x = 2.0;
  marker_.scale.y = 2.0;
  marker_.scale.z = 2.0;
  marker_.color.r = 0.0f;
  marker_.color.g = 1.0f;
  marker_.color.b = 1.0f;
  marker_.color.a = 1.0f;

  auto marker_pos_v = offsetPoint(parent_model_->model_position_, rel_pos_, parent_model_->model_orientation_);
  marker_.pose.position.x = marker_pos_v.x();
  marker_.pose.position.y = marker_pos_v.y();
}

}  // namespace road_world_editor