#pragma once
#include <road_world_editor/common.hpp>

namespace road_world_editor {

class ModelPort : public std::enable_shared_from_this<ModelPort>
{
  public:
    friend class GazeboModel;
    using SharedPtr = std::shared_ptr<ModelPort>;
    ModelPort(std::shared_ptr<GazeboModel> parent_model, Eigen::Vector2d rel_pos, const std::string& port_id = "");

    const geometry_msgs::msg::Point& getPosition() { return marker_.pose.position; }
    const std::string& getPortId() const { return port_id_; }
    std::string getConnectedPortId() const;
    bool isConnected();
    void connectTo(const SharedPtr& other);

  private:
    void moveWithParent();                                            /*!< Update the position of this marker to align with parent model */
    void moveAbsolute(const geometry_msgs::msg::Point& new_position); /*!< Force the marker position to a new point and then move parent accordingly */
    void breakConnection();
    void initiateMove();

    std::shared_ptr<GazeboModel> parent_model_;
    Eigen::Vector2d rel_pos_;
    std::string port_id_;
    visualization_msgs::msg::Marker marker_;
    std::weak_ptr<ModelPort> connected_port_;
    bool moved_ = true;
    void initMarker();
};

}  // namespace road_world_editor