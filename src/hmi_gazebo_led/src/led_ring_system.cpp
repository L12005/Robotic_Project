#include <chrono>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/msgs/convert/Color.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace hmi::gazebo
{
namespace
{
double WrapIndexDistance(double from, double to, int count)
{
  const double raw = std::fabs(from - to);
  return std::min(raw, static_cast<double>(count) - raw);
}

double GlowLevelForDistance(double distance)
{
  if (distance < 0.5)
    return 1.0;
  if (distance < 1.5)
    return 0.45;
  if (distance < 2.5)
    return 0.18;
  return 0.03;
}

double Clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}
}  // namespace

class LedRingSystem final:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->modelEntity = _entity;
    if (_sdf)
    {
      if (_sdf->HasElement("topic"))
        this->topic = _sdf->Get<std::string>("topic");
      if (_sdf->HasElement("segment_prefix"))
        this->segmentPrefix = _sdf->Get<std::string>("segment_prefix");
      if (_sdf->HasElement("segment_count"))
        this->segmentCount = _sdf->Get<int>("segment_count");
    }

    this->transportNode.Subscribe(this->topic, &LedRingSystem::OnModeMsg, this);
    this->ResolveSegments(_ecm);
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    if (this->segmentEntities.size() != static_cast<std::size_t>(this->segmentCount))
      this->ResolveSegments(_ecm);

    if (this->segmentEntities.size() != static_cast<std::size_t>(this->segmentCount))
      return;

    this->SyncPendingMode(_info.simTime);
    const double simSec = std::chrono::duration<double>(_info.simTime).count();

    for (int i = 0; i < this->segmentCount; ++i)
    {
      const auto color = this->SegmentColor(i, simSec);
      gz::msgs::Visual visualMsg;
      auto *material = visualMsg.mutable_material();
      gz::msgs::Set(material->mutable_ambient(), this->ScaledColor(color, 0.20));
      gz::msgs::Set(material->mutable_diffuse(), this->ScaledColor(color, 0.55));
      gz::msgs::Set(material->mutable_emissive(), color);
      _ecm.SetComponentData<gz::sim::components::VisualCmd>(
          this->segmentEntities[static_cast<std::size_t>(i)], visualMsg);
      _ecm.SetChanged(
          this->segmentEntities[static_cast<std::size_t>(i)],
          gz::sim::components::VisualCmd::typeId,
          gz::sim::ComponentState::OneTimeChange);
    }
  }

  private: void OnModeMsg(const gz::msgs::StringMsg &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pendingMode = _msg.data();
    this->hasPendingMode = true;
  }

  private: void SyncPendingMode(const std::chrono::steady_clock::duration &_simTime)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->hasPendingMode)
      return;

    if (this->currentMode != this->pendingMode)
    {
      this->currentMode = this->pendingMode;
      this->modeStartedAt = _simTime;
    }
    this->hasPendingMode = false;
  }

  private: void ResolveSegments(gz::sim::EntityComponentManager &_ecm)
  {
    this->segmentEntities.clear();
    this->segmentEntities.reserve(static_cast<std::size_t>(this->segmentCount));

    for (int i = 0; i < this->segmentCount; ++i)
    {
      const std::string scopedName =
          "base_link::" + this->segmentPrefix + (i < 10 ? "0" : "") + std::to_string(i);
      const std::unordered_set<gz::sim::Entity> entities =
          gz::sim::entitiesFromScopedName(scopedName, _ecm, this->modelEntity);
      if (entities.empty())
        continue;

      const auto entity = *entities.begin();
      this->segmentEntities.push_back(entity);

      gz::msgs::Visual initialVisual;
      _ecm.SetComponentData<gz::sim::components::VisualCmd>(entity, initialVisual);
      _ecm.SetChanged(
          entity,
          gz::sim::components::VisualCmd::typeId,
          gz::sim::ComponentState::OneTimeChange);
    }
  }

  private: gz::math::Color SegmentColor(int index, double simSec) const
  {
    if (this->currentMode == "white_dim")
      return this->ScaledColor(gz::math::Color(1.0, 1.0, 1.0, 1.0), 0.18);

    if (this->currentMode == "white_steady")
      return this->ScaledColor(gz::math::Color(1.0, 1.0, 1.0, 1.0), 0.68);

    if (this->currentMode == "green_steady")
      return this->ScaledColor(gz::math::Color(0.10, 1.0, 0.20, 1.0), 0.78);

    if (this->currentMode == "red_fast_blink")
    {
      const double phase = std::fmod(simSec * 7.0, 1.0);
      const double intensity = phase < 0.5 ? 1.0 : 0.06;
      return this->ScaledColor(gz::math::Color(1.0, 0.12, 0.10, 1.0), intensity);
    }

    if (this->currentMode == "green_fade_to_white")
    {
      const double startedAt = std::chrono::duration<double>(this->modeStartedAt).count();
      const double progress = Clamp01((simSec - startedAt) / 1.2);
      const gz::math::Color green = this->ScaledColor(gz::math::Color(0.10, 1.0, 0.20, 1.0), 0.78);
      const gz::math::Color white = this->ScaledColor(gz::math::Color(1.0, 1.0, 1.0, 1.0), 0.68);
      return this->Lerp(green, white, progress);
    }

    if (this->currentMode == "green_ccw_flow")
      return this->FlowColor(index, simSec, +1.0);

    if (this->currentMode == "green_cw_flow")
      return this->FlowColor(index, simSec, -1.0);

    if (this->currentMode == "green_backward_flow")
      return this->BackwardFlowColor(index, simSec);

    return this->ScaledColor(gz::math::Color(1.0, 1.0, 1.0, 1.0), 0.18);
  }

  private: gz::math::Color FlowColor(int index, double simSec, double direction) const
  {
    const double speed = 6.5;
    double center = std::fmod(simSec * speed * direction, static_cast<double>(this->segmentCount));
    if (center < 0.0)
      center += this->segmentCount;

    const double level = GlowLevelForDistance(
        WrapIndexDistance(static_cast<double>(index), center, this->segmentCount));
    return this->ScaledColor(gz::math::Color(0.10, 1.0, 0.20, 1.0), level);
  }

  private: gz::math::Color BackwardFlowColor(int index, double simSec) const
  {
    const double rearCenter = static_cast<double>(this->segmentCount) / 2.0;
    const double travel = std::fmod(simSec * 3.6, rearCenter);
    const double leftPulse = travel;
    const double rightPulse = std::fmod(static_cast<double>(this->segmentCount) - travel,
                                        static_cast<double>(this->segmentCount));
    const double distance = std::min(
        WrapIndexDistance(static_cast<double>(index), leftPulse, this->segmentCount),
        WrapIndexDistance(static_cast<double>(index), rightPulse, this->segmentCount));
    const double rearDistance = WrapIndexDistance(static_cast<double>(index), rearCenter, this->segmentCount);
    const double rearBias = rearDistance < 2.5 ? 0.10 : 0.0;
    const double level = std::min(1.0, GlowLevelForDistance(distance) + rearBias);
    return this->ScaledColor(gz::math::Color(0.10, 1.0, 0.20, 1.0), level);
  }

  private: gz::math::Color ScaledColor(const gz::math::Color &base, double intensity) const
  {
    return gz::math::Color(
        Clamp01(base.R() * intensity),
        Clamp01(base.G() * intensity),
        Clamp01(base.B() * intensity),
        1.0);
  }

  private: gz::math::Color Lerp(
      const gz::math::Color &from,
      const gz::math::Color &to,
      double t) const
  {
    return gz::math::Color(
        from.R() + (to.R() - from.R()) * t,
        from.G() + (to.G() - from.G()) * t,
        from.B() + (to.B() - from.B()) * t,
        1.0);
  }

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::transport::Node transportNode;
  private: std::mutex mutex;
  private: std::string topic{"/hmi/visual/led_mode"};
  private: std::string segmentPrefix{"led_seg_"};
  private: int segmentCount{25};
  private: std::vector<gz::sim::Entity> segmentEntities;
  private: std::string currentMode{"white_dim"};
  private: std::string pendingMode{"white_dim"};
  private: bool hasPendingMode{false};
  private: std::chrono::steady_clock::duration modeStartedAt{};
};
}  // namespace hmi::gazebo

GZ_ADD_PLUGIN(
    hmi::gazebo::LedRingSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(hmi::gazebo::LedRingSystem, "hmi::gazebo::LedRingSystem")
