#pragma once
#include <road_world_editor/common.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_io/Io.h>

namespace road_world_editor {

class GazeboModel : public std::enable_shared_from_this<GazeboModel>
{
  public:
    friend class ModelPort;
    using SharedPtr = std::shared_ptr<GazeboModel>;
    GazeboModel(const std::string& mesh_path, const std::string& frame_id, const std::string& model_id = "");
    virtual ~GazeboModel() {}
    virtual void initPorts(const std::vector<std::string>& port_ids = std::vector<std::string>()) = 0;
    virtual int getModelType() const = 0;

    std::vector<visualization_msgs::msg::Marker> getPortMarkers();
    std::vector<std::shared_ptr<ModelPort>>& getPorts() { return ports_; }
    const std::vector<visualization_msgs::msg::Marker> getModelMarkers() { return {mesh_marker_, leftDirectionMarker(), rightDirectionMarker()}; }
    const geometry_msgs::msg::Point& getControlMarkerPosition() const { return control_marker_.pose.position; }
    const Eigen::Vector2d& getModelPosition() const { return model_position_; }
    const ModelOrientation& getModelOrientation() const { return model_orientation_; }
    const std::string& getModelId() const { return model_id_; }
    const std::string& getVariant() const { return variant_; }
    void setVariant(const std::string& variant) { variant_ = variant; }
    void setModelPosition(const Eigen::Vector2d& new_position);
    void rotateClockwise();
    void rotateCounterClockwise();
    void disconnectAllPorts();
    virtual std::vector<lanelet::Lanelet> generateLanelet() = 0;

  protected:
    std::string model_id_;
    std::string frame_id_;
    std::string variant_ = DEFAULT_ROAD_VARIANT;
    Eigen::Vector2d control_marker_offset_;
    std::vector<std::shared_ptr<ModelPort>> ports_;
    Eigen::Vector2d model_position_;
    ModelOrientation model_orientation_;
    void initModel();
    virtual visualization_msgs::msg::Marker leftDirectionMarker() = 0;
    virtual visualization_msgs::msg::Marker rightDirectionMarker() = 0;
    static constexpr double SINCOS_M_PI_4 = 0.707106781186547;

  private:
    std::string mesh_path_;
    bool moved_ = true;
    visualization_msgs::msg::Marker mesh_marker_;
    visualization_msgs::msg::InteractiveMarker control_marker_;

    void initiateMove();
    void move(const Eigen::Vector2d& new_model_pos);
    void rotate(const ModelOrientation& new_model_orientation);
    void moveMarkers();
    virtual void initMarkers();
    void processMarkerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
};

}  // namespace road_world_editor