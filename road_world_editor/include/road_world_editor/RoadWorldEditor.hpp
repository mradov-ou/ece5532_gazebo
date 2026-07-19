#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <road_world_editor/RoadModel.hpp>

#ifndef Q_MOC_RUN
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#endif

namespace road_world_editor {

class RoadWorldEditor : public rviz_common::Panel
{
    Q_OBJECT

  public:
    explicit RoadWorldEditor(QWidget* parent = nullptr);
    ~RoadWorldEditor();

    void onInitialize() override;

  private Q_SLOTS:
    void addModelClicked();
    void saveClicked();
    void loadClicked();
    void exportGazeboClicked();
    void exportLaneletClicked();
    void positionEditFinished();
    void variantSelected(int index);
    void disconnectClicked();
    void rotateCwClicked();
    void rotateCcwClicked();
    void deleteClicked();

  private:
    QPushButton* add_model_btn_;
    QPushButton* save_btn_;
    QPushButton* load_btn_;
    QPushButton* export_gazebo_btn_;
    QPushButton* export_lanelet_btn_;
    QPushButton* disconnect_btn_;
    QPushButton* delete_btn_;
    QPushButton* rotate_cw_btn_;
    QPushButton* rotate_ccw_btn_;
    QComboBox* model_type_combo_;
    QComboBox* variant_combo_;
    QLineEdit* x_edit_;
    QLineEdit* y_edit_;
    QLineEdit* latitude_edit_;
    QLineEdit* longitude_edit_;
    QLineEdit* altitude_edit_;
    std::weak_ptr<GazeboModel> displayed_model_;
    Eigen::Vector2d displayed_position_ = Eigen::Vector2d::Zero();
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::TimerBase::SharedPtr marker_refresh_timer_;
    std::string frame_id_;
    std::vector<GazeboModel::SharedPtr> models_;

    bool last_is_marker_moving_ = false;
    bool isMarkerMoving();
    void snapPorts();
    QString loadDialogDir() const;
    void storeDialogDir(const QString& filename);
    void refreshPositionEdits();
    bool getReferenceCoordinates(double& latitude, double& longitude, double& altitude);
    visualization_msgs::msg::Marker selectionIndicatorMarker(GazeboModel::SharedPtr active_model);

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
    void recvInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
};

}  // namespace road_world_editor
