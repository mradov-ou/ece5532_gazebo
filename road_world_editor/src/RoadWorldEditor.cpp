#include <road_world_editor/ModelPort.hpp>
#include <road_world_editor/RoadWorldEditor.hpp>

#include <rviz_common/display_context.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>

#include <algorithm>
#include <fstream>
#include <sstream>

using namespace visualization_msgs::msg;
using namespace interactive_markers;

namespace road_world_editor {

CommonData g_common_data;

const RoadModelFactoryTable ROAD_MODEL_FACTORIES = detail::makeRoadModelTable(std::make_index_sequence<std::tuple_size_v<decltype(ROAD_MODEL_INFO)>>{});

RoadWorldEditor::RoadWorldEditor(QWidget* parent) : rviz_common::Panel(parent)
{
  add_model_btn_ = new QPushButton("Add Model");
  save_btn_ = new QPushButton("Save");
  load_btn_ = new QPushButton("Load");
  export_btn_ = new QPushButton("Export World");
  model_type_combo_ = new QComboBox();

  for (auto& info : ROAD_MODEL_INFO) {
    model_type_combo_->addItem(QString::fromStdString(info.name));
  }

  disconnect_btn_ = new QPushButton("Disconnect");
  delete_btn_ = new QPushButton("Delete Model");
  rotate_cw_btn_ = new QPushButton("Rotate CW");
  rotate_ccw_btn_ = new QPushButton("Rotate CCW");
  x_edit_ = new QLineEdit();
  y_edit_ = new QLineEdit();

  variant_combo_ = new QComboBox();
  variant_combo_->setEnabled(false);

  auto* file_btn_layout = new QHBoxLayout;
  file_btn_layout->addWidget(save_btn_);
  file_btn_layout->addWidget(load_btn_);

  auto* position_layout = new QHBoxLayout;
  position_layout->addWidget(new QLabel("X:"));
  position_layout->addWidget(x_edit_);
  position_layout->addWidget(new QLabel("Y:"));
  position_layout->addWidget(y_edit_);

  auto* rotate_btn_layout = new QHBoxLayout;
  rotate_btn_layout->addWidget(rotate_ccw_btn_);
  rotate_btn_layout->addWidget(rotate_cw_btn_);

  auto* variant_layout = new QHBoxLayout;
  variant_layout->addWidget(new QLabel("Variant:"));
  variant_layout->addWidget(variant_combo_);

  auto* layout = new QVBoxLayout;
  layout->addWidget(add_model_btn_);
  layout->addWidget(model_type_combo_);
  layout->addLayout(position_layout);
  layout->addLayout(variant_layout);
  layout->addLayout(rotate_btn_layout);

  auto* model_btn_layout = new QHBoxLayout;
  model_btn_layout->addWidget(disconnect_btn_);
  model_btn_layout->addWidget(delete_btn_);
  layout->addLayout(model_btn_layout);
  layout->addLayout(file_btn_layout);
  layout->addWidget(export_btn_);
  setLayout(layout);

  connect(add_model_btn_, &QPushButton::clicked, this, &RoadWorldEditor::addModelClicked);
  connect(save_btn_, &QPushButton::clicked, this, &RoadWorldEditor::saveClicked);
  connect(load_btn_, &QPushButton::clicked, this, &RoadWorldEditor::loadClicked);
  connect(export_btn_, &QPushButton::clicked, this, &RoadWorldEditor::exportClicked);
  connect(disconnect_btn_, &QPushButton::clicked, this, &RoadWorldEditor::disconnectClicked);
  connect(delete_btn_, &QPushButton::clicked, this, &RoadWorldEditor::deleteClicked);
  connect(rotate_cw_btn_, &QPushButton::clicked, this, &RoadWorldEditor::rotateCwClicked);
  connect(rotate_ccw_btn_, &QPushButton::clicked, this, &RoadWorldEditor::rotateCcwClicked);
  connect(x_edit_, &QLineEdit::editingFinished, this, &RoadWorldEditor::positionEditFinished);
  connect(y_edit_, &QLineEdit::editingFinished, this, &RoadWorldEditor::positionEditFinished);
  connect(variant_combo_, QOverload<int>::of(&QComboBox::activated), this, &RoadWorldEditor::variantSelected);
}

RoadWorldEditor::~RoadWorldEditor() {}

void RoadWorldEditor::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  frame_id_ = node_->declare_parameter<std::string>("frame_id", "map");

  g_common_data.marker_server = std::make_unique<InteractiveMarkerServer>("interactive_markers", node_);
  pub_markers_ = node_->create_publisher<MarkerArray>("markers", 1);

  marker_refresh_timer_ = rclcpp::create_timer(node_, node_->get_clock(), std::chrono::milliseconds(50), [this]() {
    MarkerArray marker_array_msg;
    for (auto& model : models_) {
      auto new_markers = model->getPortMarkers();
      marker_array_msg.markers.insert(marker_array_msg.markers.end(), new_markers.begin(), new_markers.end());
    }

    int mesh_marker_id = 0;
    for (auto& model : models_) {
      Marker mesh_marker = model->getMeshMarker();
      mesh_marker.id = mesh_marker_id++;
      marker_array_msg.markers.push_back(mesh_marker);
    }

    if (auto active_model = g_common_data.last_updated_model.lock()) {
      marker_array_msg.markers.push_back(selectionIndicatorMarker(active_model));
    }
    pub_markers_->publish(marker_array_msg);

    g_common_data.current_time = node_->get_clock()->now();
    bool current_marker_moving = isMarkerMoving();
    if (last_is_marker_moving_ && !current_marker_moving) {
      snapPorts();
    }
    last_is_marker_moving_ = current_marker_moving;

    refreshPositionEdits();
  });
}

Marker RoadWorldEditor::selectionIndicatorMarker(GazeboModel::SharedPtr active_model)
{
  Marker m;
  m.header.frame_id = frame_id_;
  m.lifetime.sec = 0;
  m.lifetime.nanosec = 300000000;
  m.type = Marker::ARROW;
  m.ns = "active_model";
  m.points.resize(2);
  m.points[0] = m.points[1] = active_model->getControlMarkerPosition();
  m.points[0].z = 35.0;
  m.points[1].z = 15.0;
  m.scale.x = 3.0;
  m.scale.y = 4.0;
  m.scale.z = 5.0;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 1.0;
  m.color.a = 1.0;
  return m;
}

bool RoadWorldEditor::isMarkerMoving()
{
  if (g_common_data.marker_moved_stamp.get_clock_type() != g_common_data.current_time.get_clock_type()) {
    return false;
  } else if ((g_common_data.current_time - g_common_data.marker_moved_stamp).seconds() > 0.25) {
    return false;
  }
  return true;
}

void RoadWorldEditor::snapPorts()
{
  auto active_model = g_common_data.last_updated_model.lock();
  if (!active_model) {
    RCLCPP_WARN(node_->get_logger(), "Active model not set");
    return;
  }

  auto& active_model_ports = active_model->getPorts();
  for (auto& active_port : active_model_ports) {
    if (active_port->isConnected()) {
      continue;
    }

    for (auto& model : models_) {
      if (model == active_model) {
        continue;
      }

      for (auto& port : model->getPorts()) {
        if (port->isConnected()) {
          continue;
        }
        if (squaredDist(port->getPosition(), active_port->getPosition()) < (10.0 * 10.0)) {
          active_port->connectTo(port);
        }
      }
    }
  }
}

void RoadWorldEditor::addModelClicked()
{
  Eigen::Vector2d starting_position = Eigen::Vector2d::Zero();
  ModelOrientation starting_orientation = ModelOrientation::EAST;

  if (auto active_model = g_common_data.last_updated_model.lock()) {
    starting_position = active_model->getModelPosition() + Eigen::Vector2d(50, 50);
    starting_orientation = active_model->getModelOrientation();
  }

  int model_type_index = model_type_combo_->currentIndex();
  auto new_model = ROAD_MODEL_FACTORIES[model_type_index](starting_position, starting_orientation, frame_id_, "");

  new_model->initPorts();
  models_.push_back(new_model);
  g_common_data.last_updated_model = new_model;
}

void RoadWorldEditor::refreshPositionEdits()
{
  auto active_model = g_common_data.last_updated_model.lock();
  if (!active_model) {
    if (displayed_model_.lock()) {
      displayed_model_.reset();
      x_edit_->clear();
      y_edit_->clear();
      variant_combo_->setEnabled(false);
    }
    return;
  }

  const auto& position = active_model->getModelPosition();
  bool model_changed = displayed_model_.lock() != active_model;
  bool position_changed = position.x() != displayed_position_.x() || position.y() != displayed_position_.y();
  if (!model_changed && !position_changed) {
    return;
  }

  // Don't overwrite a value the user is currently typing in
  if (!model_changed && (x_edit_->hasFocus() || y_edit_->hasFocus())) {
    return;
  }

  if (model_changed) {
    const auto& variants = ROAD_MODEL_INFO[active_model->getModelType()].variants;
    variant_combo_->clear();
    for (auto& variant : variants) {
      variant_combo_->addItem(QString::fromStdString(variant));
    }
    variant_combo_->setEnabled(!variants.empty());
    auto it = std::find(variants.begin(), variants.end(), active_model->getVariant());
    if (it != variants.end()) {
      variant_combo_->setCurrentIndex(std::distance(variants.begin(), it));
    }
  }

  displayed_model_ = active_model;
  displayed_position_ = position;
  x_edit_->setText(QString::number(position.x(), 'f', 2));
  y_edit_->setText(QString::number(position.y(), 'f', 2));
}

void RoadWorldEditor::positionEditFinished()
{
  auto active_model = g_common_data.last_updated_model.lock();
  if (!active_model) {
    return;
  }

  bool x_ok = false;
  bool y_ok = false;
  double x = x_edit_->text().toDouble(&x_ok);
  double y = y_edit_->text().toDouble(&y_ok);
  if (!x_ok || !y_ok) {
    return;
  }

  const auto& current_position = active_model->getModelPosition();
  if (x == current_position.x() && y == current_position.y()) {
    return;
  }

  active_model->setModelPosition(Eigen::Vector2d(x, y));
}

void RoadWorldEditor::variantSelected(int index)
{
  auto active_model = g_common_data.last_updated_model.lock();
  if (!active_model) {
    return;
  }
  const auto& variants = ROAD_MODEL_INFO[active_model->getModelType()].variants;
  if (index < 0 || index >= static_cast<int>(variants.size())) {
    return;
  }
  active_model->setVariant(variants[index]);
}

void RoadWorldEditor::disconnectClicked()
{
  if (auto active_model = g_common_data.last_updated_model.lock()) {
    active_model->disconnectAllPorts();
  }
}

void RoadWorldEditor::rotateCwClicked()
{
  if (auto active_model = g_common_data.last_updated_model.lock()) {
    active_model->rotateClockwise();
  }
}

void RoadWorldEditor::rotateCcwClicked()
{
  if (auto active_model = g_common_data.last_updated_model.lock()) {
    active_model->rotateCounterClockwise();
  }
}

void RoadWorldEditor::deleteClicked()
{
  auto active_model = g_common_data.last_updated_model.lock();
  if (!active_model) {
    return;
  }

  active_model->disconnectAllPorts();
  g_common_data.last_updated_model.reset();
  g_common_data.marker_server->erase(active_model->getModelId());
  g_common_data.marker_server->applyChanges();
  models_.erase(std::remove(models_.begin(), models_.end(), active_model), models_.end());

  displayed_model_.reset();
  x_edit_->clear();
  y_edit_->clear();
  variant_combo_->clear();
  variant_combo_->setEnabled(false);
}

QString RoadWorldEditor::loadDialogDir() const
{
  QFile dir_file(QDir::home().filePath(".rviz2/road_world_editor_dialog_dir"));
  if (dir_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QString dir = QString::fromUtf8(dir_file.readAll()).trimmed();
    if (!dir.isEmpty() && QDir(dir).exists()) {
      return dir;
    }
  }
  return QDir::homePath();
}

void RoadWorldEditor::storeDialogDir(const QString& filename)
{
  QDir::home().mkpath(".rviz2");
  QFile dir_file(QDir::home().filePath(".rviz2/road_world_editor_dialog_dir"));
  if (dir_file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    dir_file.write(QFileInfo(filename).absolutePath().toUtf8());
  }
}

void RoadWorldEditor::saveClicked()
{
  QString filename = QFileDialog::getSaveFileName(this, "Save Road World", loadDialogDir(), "JSON files (*.json)");
  if (filename.isEmpty()) {
    return;
  }
  if (QFileInfo(filename).suffix().isEmpty()) {
    filename += ".json";
  }
  storeDialogDir(filename);

  nlohmann::json root;
  auto& models_json = root["models"] = nlohmann::json::array();
  for (auto& model : models_) {
    nlohmann::json model_json;
    model_json["type"] = model->getModelType();
    model_json["id"] = model->getModelId();
    model_json["position"]["x"] = model->getModelPosition().x();
    model_json["position"]["y"] = model->getModelPosition().y();
    model_json["orientation"] = static_cast<int>(model->getModelOrientation());
    model_json["variant"] = model->getVariant();
    auto& ports_json = model_json["ports"] = nlohmann::json::array();
    for (auto& port : model->getPorts()) {
      ports_json.push_back({{"id", port->getPortId()}, {"connected_id", port->getConnectedPortId()}});
    }
    models_json.push_back(model_json);
  }

  std::ofstream file(filename.toStdString());
  file << root.dump(2) << std::endl;
  if (!file) {
    QMessageBox::warning(this, "Save Road World", QString("Failed to write %1").arg(filename));
  }
}

static const char* orientationToYaw(const ModelOrientation& orientation)
{
  switch (orientation) {
    case ModelOrientation::NORTH:
      return "1.57079632679";
    case ModelOrientation::WEST:
      return "3.14159265359";
    case ModelOrientation::SOUTH:
      return "-1.57079632679";
    case ModelOrientation::EAST:
    default:
      return "0";
  }
}

void RoadWorldEditor::exportClicked()
{
  QString filename = QFileDialog::getSaveFileName(this, "Export Gazebo World", loadDialogDir(), "SDF world files (*.sdf)");
  if (filename.isEmpty()) {
    return;
  }
  if (QFileInfo(filename).suffix().isEmpty()) {
    filename += ".sdf";
  }
  storeDialogDir(filename);

  std::string world_sdf;
  try {
    std::string template_path = ament_index_cpp::get_package_share_directory("ece5532_gazebo") + "/worlds/road_world_template.sdf";
    std::ifstream template_file(template_path);
    if (!template_file) {
      throw std::runtime_error("Failed to open " + template_path);
    }
    std::stringstream buffer;
    buffer << template_file.rdbuf();
    world_sdf = buffer.str();
  } catch (const std::exception& e) {
    QMessageBox::warning(this, "Export Gazebo World", QString("Failed to read world template:\n%1").arg(e.what()));
    return;
  }

  size_t insert_pos = world_sdf.rfind("</world>");
  if (insert_pos == std::string::npos) {
    QMessageBox::warning(this, "Export Gazebo World", "World template has no </world> tag");
    return;
  }

  std::ostringstream models_sdf;
  for (auto& model : models_) {
    const auto& info = ROAD_MODEL_INFO[model->getModelType()];
    const auto& position = model->getModelPosition();
    models_sdf << "    <model name=\"" << model->getModelId() << "\">\n"
               << "      <static>true</static>\n"
               << "      <self_collide>false</self_collide>\n"
               << "      <pose>" << position.x() << " " << position.y() << " 0 0 0 " << orientationToYaw(model->getModelOrientation()) << "</pose>\n"
               << "      <include merge=\"true\">\n"
               << "        <uri>" << info.gazebo_model_path << "</uri>\n";
    bool variant_supported = std::find(info.variants.begin(), info.variants.end(), model->getVariant()) != info.variants.end();
    if (variant_supported && model->getVariant() != DEFAULT_ROAD_VARIANT) {
      std::string mesh_uri = info.gazebo_model_path + "/meshes/" + model->getVariant() + ".obj";
      models_sdf << "        <experimental:params>\n"
                 << "          <visual element_id=\"road::road_visual\" action=\"modify\">\n"
                 << "            <geometry>\n"
                 << "              <mesh>\n"
                 << "                <uri>" << mesh_uri << "</uri>\n"
                 << "              </mesh>\n"
                 << "            </geometry>\n"
                 << "          </visual>\n"
                 << "          <collision element_id=\"road::road_collision\" action=\"modify\">\n"
                 << "            <geometry>\n"
                 << "              <mesh>\n"
                 << "                <uri>" << mesh_uri << "</uri>\n"
                 << "              </mesh>\n"
                 << "            </geometry>\n"
                 << "          </collision>\n"
                 << "        </experimental:params>\n";
    }
    models_sdf << "      </include>\n"
               << "    </model>\n\n";
  }
  world_sdf.insert(insert_pos, models_sdf.str());

  std::ofstream file(filename.toStdString());
  file << world_sdf;
  if (!file) {
    QMessageBox::warning(this, "Export Gazebo World", QString("Failed to write %1").arg(filename));
  }
}

void RoadWorldEditor::loadClicked()
{
  QString filename = QFileDialog::getOpenFileName(this, "Load Road World", loadDialogDir(), "JSON files (*.json)");
  if (filename.isEmpty()) {
    return;
  }
  storeDialogDir(filename);

  nlohmann::json root;
  try {
    std::ifstream file(filename.toStdString());
    if (!file) {
      throw std::runtime_error("Failed to open file");
    }
    root = nlohmann::json::parse(file);
  } catch (const std::exception& e) {
    QMessageBox::warning(this, "Load Road World", QString("Failed to load %1:\n%2").arg(filename, e.what()));
    return;
  }

  // Replace the current scene with the loaded one
  g_common_data.last_updated_model.reset();
  models_.clear();
  g_common_data.marker_server->clear();
  g_common_data.marker_server->applyChanges();

  std::map<std::string, ModelPort::SharedPtr> ports_by_id;
  std::vector<std::pair<ModelPort::SharedPtr, std::string>> pending_connections;

  // Loading can create dozens of models and connections; batch all the resulting
  // interactive marker inserts/updates into a single applyChanges() at the end instead of
  // one per model/connection, which otherwise floods interactive_markers with a burst of
  // updates large enough to corrupt its client-side update queue (seen as a crash a moment
  // after loading a large road world).
  g_common_data.batch_marker_updates = true;

  try {
    for (auto& model_json : root.at("models")) {
      int model_type = model_json.at("type");
      if (model_type < 0 || model_type >= static_cast<int>(ROAD_MODEL_FACTORIES.size())) {
        throw std::runtime_error("Invalid model type: " + std::to_string(model_type));
      }

      Eigen::Vector2d position(model_json.at("position").at("x"), model_json.at("position").at("y"));
      auto orientation = static_cast<ModelOrientation>(model_json.at("orientation").get<int>());
      auto model = ROAD_MODEL_FACTORIES[model_type](position, orientation, frame_id_, model_json.at("id").get<std::string>());
      const auto& variants = ROAD_MODEL_INFO[model_type].variants;
      std::string variant = model_json.value("variant", DEFAULT_ROAD_VARIANT);
      if (std::find(variants.begin(), variants.end(), variant) == variants.end()) {
        variant = DEFAULT_ROAD_VARIANT;
      }
      model->setVariant(variant);

      std::vector<std::string> port_ids;
      std::vector<std::string> connected_ids;
      for (auto& port_json : model_json.at("ports")) {
        port_ids.push_back(port_json.at("id"));
        connected_ids.push_back(port_json.at("connected_id"));
      }
      model->initPorts(port_ids);

      auto& ports = model->getPorts();
      for (size_t i = 0; i < ports.size(); i++) {
        ports_by_id[port_ids[i]] = ports[i];
        if (!connected_ids[i].empty()) {
          pending_connections.emplace_back(ports[i], connected_ids[i]);
        }
      }
      models_.push_back(model);
    }
  } catch (const std::exception& e) {
    g_common_data.batch_marker_updates = false;
    models_.clear();
    g_common_data.marker_server->clear();
    g_common_data.marker_server->applyChanges();
    QMessageBox::warning(this, "Load Road World", QString("Invalid road world file %1:\n%2").arg(filename, e.what()));
    return;
  }

  for (auto& [port, connected_id] : pending_connections) {
    if (port->isConnected()) {
      continue;
    }
    auto it = ports_by_id.find(connected_id);
    if (it == ports_by_id.end()) {
      RCLCPP_WARN(node_->get_logger(), "Connected port ID %s not found in file", connected_id.c_str());
      continue;
    }
    port->connectTo(it->second);
  }

  g_common_data.batch_marker_updates = false;
  g_common_data.marker_server->applyChanges();
}

}  // namespace road_world_editor

PLUGINLIB_EXPORT_CLASS(road_world_editor::RoadWorldEditor, rviz_common::Panel)
