// MorphTerramechanics: force-level soil emulation inside named zones (see terramechanics_model.hpp).
// Applies one thing to the simulation: a resisting torque about each wheel's spin axis, equal to the
// compaction (and optional slip-loss) resistance times the wheel radius. Rolling resistance is a
// moment from the forward-shifted ground reaction, so it is applied as an external torque on the
// wheel link, not as a joint torque and not as a force at the contact point (which the rigid
// friction constraint would simply cancel). Sinkage and soil slip are published, not rendered.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <sstream>
#include <string>
#include <vector>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include "terramechanics_model.hpp"

namespace morph_sim {
class MorphTerramechanics final : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  struct Zone {std::string name, soilName; double xmin, xmax, ymin, ymax; terramechanics::Soil soil;};
  struct RutPoint {double x, y, depth;};
  struct Wheel {
    std::string name; gz::sim::Entity link{gz::sim::kNullEntity}, joint{gz::sim::kNullEntity}, spring{gz::sim::kNullEntity};
    std::deque<RutPoint> rut; double thrust{0.}, normal{0.}, torque{0.}, omega{0.}; int zone{-1}; bool inRut{false}; terramechanics::State state;
  };
  gz::sim::Model model{gz::sim::kNullEntity};
  std::vector<Zone> zones; std::vector<Wheel> wheels; terramechanics::Config config;
  double radius{.075}, width{.045}, unsprungWeight{7.16}, speedScale{.3}, thrustFilter{.1}, rutSpacing{.02}, captureRadius{.045}, lastPublish{-1.};
  size_t rutMemory{600};
  gz::transport::Node node; gz::transport::Node::Publisher status;

 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf, gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    model = gz::sim::Model(entity);
    auto get = [&](const char *key, double fallback) {return sdf->HasElement(key) ? sdf->Get<double>(key) : fallback;};
    radius = get("wheel_radius", radius); width = get("wheel_width", width); unsprungWeight = get("unsprung_weight", unsprungWeight);
    speedScale = std::max(1e-3, get("speed_scale", speedScale)); thrustFilter = std::max(1e-3, get("thrust_filter", thrustFilter));
    rutSpacing = std::max(1e-3, get("rut_point_spacing", rutSpacing)); captureRadius = get("rut_capture_radius", captureRadius);
    rutMemory = static_cast<size_t>(std::max(1., get("rut_memory_points", 600.)));
    config.sinkageLimitFraction = get("sinkage_limit_radius_fraction", .5); config.reloadFraction = get("reload_resistance_fraction", .1);
    config.slipSinkageGain = get("slip_sinkage_gain", 0.); config.slipDrag = get("slip_drag_enabled", 0.); config.slipLimit = get("slip_ratio_limit", .9);
    auto copy = sdf->Clone();
    for (auto e = copy->HasElement("wheel") ? copy->GetElement("wheel") : nullptr; e; e = e->GetNextElement("wheel")) {
      Wheel w; w.name = e->Get<std::string>(); wheels.push_back(w);
    }
    for (auto e = copy->HasElement("zone") ? copy->GetElement("zone") : nullptr; e; e = e->GetNextElement("zone")) {
      Zone z; z.name = e->Get<std::string>("name"); z.soilName = e->Get<std::string>("soil");
      z.xmin = e->Get<double>("xmin"); z.xmax = e->Get<double>("xmax"); z.ymin = e->Get<double>("ymin"); z.ymax = e->Get<double>("ymax");
      z.soil = {e->Get<double>("kc"), e->Get<double>("kphi"), e->Get<double>("n"), e->Get<double>("cohesion"), e->Get<double>("phi"), e->Get<double>("shear_length")};
      if (!(z.soil.n > 0. && z.soil.n < 3.) || !(z.soil.kc + width * z.soil.kphi > 0.) || !(z.soil.shearLength > 0.)) {gzerr << "MorphTerramechanics: invalid soil for zone " << z.name << "\n"; continue;}
      zones.push_back(z);
    }
    status = node.Advertise<gz::msgs::StringMsg>("/morph/terramechanics/status");
    gzmsg << "MorphTerramechanics: " << zones.size() << " soil zone(s), " << wheels.size() << " wheel(s); force-level emulation over rigid contact\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0. || dt > .1) return;
    for (auto &w : wheels) {
      if (w.link == gz::sim::kNullEntity) {
        w.link = model.LinkByName(ecm, w.name + "_wheel"); w.joint = model.JointByName(ecm, w.name + "_wheel_joint"); w.spring = model.JointByName(ecm, w.name + "_spring_joint");
        if (w.link != gz::sim::kNullEntity) gz::sim::Link(w.link).EnableVelocityChecks(ecm, true);
      }
      if (w.link == gz::sim::kNullEntity || w.joint == gz::sim::kNullEntity || w.spring == gz::sim::kNullEntity) continue;
      const gz::math::Pose3d pose = gz::sim::worldPose(w.link, ecm);
      const double x = pose.Pos().X(), y = pose.Pos().Y();
      w.zone = -1;
      for (size_t i = 0; i < zones.size(); ++i) if (x >= zones[i].xmin && x < zones[i].xmax && y >= zones[i].ymin && y < zones[i].ymax) {w.zone = static_cast<int>(i); break;}
      auto velocity = ecm.Component<gz::sim::components::JointVelocity>(w.joint);
      if (!velocity) {ecm.CreateComponent(w.joint, gz::sim::components::JointVelocity()); continue;}
      w.omega = velocity->Data().empty() ? 0. : velocity->Data()[0];
      // Suspension writes its force command earlier in this PreUpdate (negative when compressed).
      auto spring = ecm.Component<gz::sim::components::JointForceCmd>(w.spring);
      const double springForce = spring && !spring->Data().empty() ? spring->Data()[0] : 0.;
      w.normal = std::max(0., -springForce + unsprungWeight);
      auto command = ecm.Component<gz::sim::components::JointForceCmd>(w.joint);
      const double motor = command && !command->Data().empty() ? command->Data()[0] : 0.;
      if (!std::isfinite(w.omega) || !std::isfinite(motor) || !std::isfinite(w.normal)) continue;
      if (w.zone < 0) {w.state = terramechanics::State(); w.thrust = 0.; w.torque = 0.; w.inRut = false; continue;}
      // Soil thrust demand: motor torque over the radius, less the slip-loss part that is only an energy charge.
      const double demand = std::max(0., std::abs(motor) / radius - w.state.slipResistance);
      w.thrust += (demand - w.thrust) * dt / (thrustFilter + dt);
      // Deepest rut sample left by another wheel within one capture radius.
      double rutDepth = 0.; const double r2 = captureRadius * captureRadius;
      for (const auto &other : wheels) {
        if (&other == &w) continue;
        for (const auto &p : other.rut) {const double dx = p.x - x, dy = p.y - y; if (dx * dx + dy * dy <= r2) rutDepth = std::max(rutDepth, p.depth);}
      }
      w.inRut = rutDepth > 0.;
      w.state = terramechanics::WheelSoilState(w.normal, w.thrust, radius, width, zones[w.zone].soil, config, rutDepth);
      if (w.rut.empty() || std::hypot(w.rut.back().x - x, w.rut.back().y - y) >= rutSpacing) {
        w.rut.push_back({x, y, w.state.rutDepth}); if (w.rut.size() > rutMemory) w.rut.pop_front();
      }
      w.torque = -std::tanh(w.omega / speedScale) * w.state.resistance * radius;
      if (!std::isfinite(w.torque)) {w.torque = 0.; continue;}
      const gz::math::Vector3d axis = pose.Rot().RotateVector(gz::math::Vector3d(0, 1, 0));
      gz::sim::Link(w.link).AddWorldWrench(ecm, gz::math::Vector3d::Zero, axis * w.torque);
    }
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (now - lastPublish >= .0495 || now < lastPublish) {
      lastPublish = now; std::ostringstream out;
      out << "{\"sim_time_s\":" << now << ",\"scope\":\"force-level emulation over rigid contact; sinkage and soil slip are model states\",\"wheels\":{";
      for (size_t i = 0; i < wheels.size(); ++i) {
        const auto &w = wheels[i];
        out << (i ? "," : "") << "\"" << w.name << "\":{\"zone\":" << (w.zone >= 0 ? "\"" + zones[w.zone].name + "\"" : std::string("null"))
            << ",\"soil\":" << (w.zone >= 0 ? "\"" + zones[w.zone].soilName + "\"" : std::string("null")) << ",\"normal_N\":" << w.normal << ",\"thrust_N\":" << w.thrust
            << ",\"sinkage_m\":" << w.state.sinkage << ",\"soil_slip\":" << w.state.slip << ",\"compaction_N\":" << w.state.compaction << ",\"slip_loss_N\":" << w.state.slipResistance
            << ",\"thrust_limit_N\":" << w.state.thrustLimit << ",\"resisting_torque_Nm\":" << w.torque << ",\"in_rut\":" << (w.inRut ? "true" : "false") << "}";
      }
      out << "}}"; gz::msgs::StringMsg msg; msg.set_data(out.str()); status.Publish(msg);
    }
  }
};
}  // namespace morph_sim
GZ_ADD_PLUGIN(morph_sim::MorphTerramechanics, gz::sim::System, morph_sim::MorphTerramechanics::ISystemConfigure, morph_sim::MorphTerramechanics::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(morph_sim::MorphTerramechanics, "morph_sim::MorphTerramechanics")
