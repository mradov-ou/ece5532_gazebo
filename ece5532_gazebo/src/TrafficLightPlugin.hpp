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

#pragma once

#include <atomic>
#include <vector>

#include <gz/msgs.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace ece5532_gazebo {

typedef enum { RED = 0, YELLOW, GREEN } LightColor;

struct LightSequenceEntry {
    LightColor color;
    double duration;
    bool flashing;
    LightSequenceEntry(LightColor c = LightColor::RED, double d = 1.0, bool f = false) : color(c), duration(d), flashing(f) {}
};

class TrafficLightPlugin : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
  public:
    TrafficLightPlugin();
    ~TrafficLightPlugin() override;

    void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& eventMgr) override;

    void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

    void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

  private:
    void setLightPosition(gz::sim::EntityComponentManager& ecm, gz::sim::Entity joint, double pos);
    void setColor(gz::sim::EntityComponentManager& ecm, LightColor color, bool flashing);
    void advanceSequence(gz::sim::EntityComponentManager& ecm);

    gz::sim::Model model_;
    gz::transport::Node node_;
    gz::sim::Entity joints_[3]{gz::sim::kNullEntity, gz::sim::kNullEntity, gz::sim::kNullEntity};

    std::vector<LightSequenceEntry> light_sequence_;
    double sequence_timestamp_ = 0.0;
    double cycle_time_ = 0.0;

    bool publish_ground_truth_pose_ = false;
    gz::sim::Entity light_fixture_link_ = gz::sim::kNullEntity;
    gz::transport::Node::Publisher pub_pose_;
    gz::msgs::Pose_V posev_msg_;

    static constexpr double LIGHT_ON_POS_ = 0.101;
    static constexpr double LIGHT_OFF_POS_ = 0.0;
};

}  // namespace ece5532_gazebo
