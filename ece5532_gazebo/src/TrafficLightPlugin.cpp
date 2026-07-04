/*******************************************************************************
BSD 2-Clause License

Copyright (c) 2026, Micho Radovnikovich

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "TrafficLightPlugin.hpp"

#include <sstream>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointPositionReset.hh>

using namespace gz;
using namespace gz::sim;

namespace ece5532_gazebo {

TrafficLightPlugin::TrafficLightPlugin() {}
TrafficLightPlugin::~TrafficLightPlugin() {}

void TrafficLightPlugin::Configure(const Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, EntityComponentManager& ecm, EventManager& /*eventMgr*/)
{
  this->model_ = Model(entity);
  if (!this->model_.Valid(ecm)) {
    gzerr << "TrafficLightPlugin plugin must be attached to a model entity. Failed to initialize." << std::endl;
    return;
  }

  this->joints_[RED] = this->model_.JointByName(ecm, "red_switch");
  this->joints_[YELLOW] = this->model_.JointByName(ecm, "yellow_switch");
  this->joints_[GREEN] = this->model_.JointByName(ecm, "green_switch");

  for (int i = RED; i <= GREEN; i++) {
    if (this->joints_[i] == kNullEntity) {
      gzerr << "TrafficLightPlugin: missing required switch joint entity" << std::endl;
      return;
    }
  }

  // Parse optional light sequence from SDF <sequence_entry> child elements.
  // Each entry is a space-delimited string: "color duration [flashing]"
  // where color is "red", "yellow", or "green", duration is seconds,
  // and flashing is "true" or "false" (optional, defaults to false).
  auto seq_elem = sdf->FindElement("sequence_entry");
  while (seq_elem) {
    std::string entry_str = seq_elem->Get<std::string>();
    std::istringstream iss(entry_str);
    std::string color_str;
    double duration;
    bool flashing = false;
    if (iss >> color_str >> duration) {
      iss >> std::boolalpha >> flashing;
      LightColor color = LightColor::RED;
      if (color_str == "green")
        color = LightColor::GREEN;
      else if (color_str == "yellow")
        color = LightColor::YELLOW;
      this->light_sequence_.emplace_back(color, duration, flashing);
    }
    seq_elem = seq_elem->GetNextElement("sequence_entry");
  }

  if (this->light_sequence_.empty()) {
    gzmsg << "TrafficLightPlugin [" << this->model_.Name(ecm) << "]: no sequence_entry elements in SDF, using default sequence" << std::endl;
    this->light_sequence_.emplace_back(LightColor::GREEN, 4.0, false);
    this->light_sequence_.emplace_back(LightColor::YELLOW, 2.0, false);
    this->light_sequence_.emplace_back(LightColor::RED, 6.0, false);
  }

  for (const auto& entry : this->light_sequence_) {
    this->cycle_time_ += entry.duration;
  }
}

void TrafficLightPlugin::PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm)
{
  if (info.paused) return;
  sequence_timestamp_ = std::fmod(std::chrono::duration<double>(info.simTime).count(), cycle_time_);
  advanceSequence(ecm);
}

void TrafficLightPlugin::advanceSequence(EntityComponentManager& ecm)
{
  double time_thres = cycle_time_;
  for (auto it = light_sequence_.rbegin(); it != light_sequence_.rend(); ++it) {
    time_thres -= it->duration;
    if (sequence_timestamp_ >= time_thres) {
      setColor(ecm, it->color, it->flashing);
      break;
    }
  }
}

void TrafficLightPlugin::setColor(EntityComponentManager& ecm, LightColor color, bool flashing)
{
  bool flash_off = flashing && (((int)(10 * sequence_timestamp_) % 10) > 5);
  for (int i = RED; i <= GREEN; i++) {
    double pos = (flash_off || i != color) ? LIGHT_OFF_POS_ : LIGHT_ON_POS_;
    setLightPosition(ecm, joints_[i], pos);
  }
}

void TrafficLightPlugin::setLightPosition(EntityComponentManager& ecm, Entity joint, double pos) { ecm.SetComponentData<components::JointPositionReset>(joint, {pos}); }

}  // namespace ece5532_gazebo

GZ_ADD_PLUGIN(ece5532_gazebo::TrafficLightPlugin, gz::sim::System, ece5532_gazebo::TrafficLightPlugin::ISystemConfigure, ece5532_gazebo::TrafficLightPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(ece5532_gazebo::TrafficLightPlugin, "gz::sim::systems::TrafficLightPlugin")
