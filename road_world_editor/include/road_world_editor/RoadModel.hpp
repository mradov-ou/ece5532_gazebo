#pragma once
#include <road_world_editor/GazeboModel.hpp>

#include <array>
#include <tuple>
#include <utility>

namespace road_world_editor {

template <int Type>
class RoadModel : public GazeboModel
{
  public:
    RoadModel(const Eigen::Vector2d& starting_position, const ModelOrientation& starting_orientation, const std::string& frame_id, std::string model_id = "")
        : GazeboModel(ROAD_MODEL_INFO[Type].mesh_path, frame_id, model_id)
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