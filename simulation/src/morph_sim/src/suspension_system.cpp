#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <sstream>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <sdf/Element.hh>

namespace morph_sim {
class SuspensionSystem final : public gz::sim::System,
 public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 struct Spring {std::string name; gz::sim::Entity entity{gz::sim::kNullEntity};double temperature{22.},heat{0.},effectiveC{85.};};
 gz::sim::Model model{gz::sim::kNullEntity};
 std::vector<Spring> springs;
 gz::transport::Node node;
 std::mutex mutex;
 gz::transport::Node::Publisher telemetry;
 double ambient{22.},capacity{180.},resistance{6.},tempCoefficient{.012},lastPublish{0.};
 double sealFriction{.25},bumpStiffness{12000.},forceCap{150.},bumpLower{-.008},bumpUpper{.025},sealVelocity{.002},adjustmentTime{.5},referenceTemperature{22.};
 double requestedK{1800.}, requestedC{65.}, k{1800.}, c{65.};
 public:
 void Configure(const gz::sim::Entity &entity,
  const std::shared_ptr<const sdf::Element> &sdf,
  gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
  model=gz::sim::Model(entity);
  if(sdf->HasElement("ambient"))ambient=sdf->Get<double>("ambient");
  if(sdf->HasElement("thermal_capacity"))capacity=std::max(1.,sdf->Get<double>("thermal_capacity"));
  if(sdf->HasElement("thermal_resistance"))resistance=std::max(.01,sdf->Get<double>("thermal_resistance"));
  if(sdf->HasElement("temperature_coefficient"))tempCoefficient=std::clamp(sdf->Get<double>("temperature_coefficient"),0.,.05);
  if(sdf->HasElement("seal_friction"))sealFriction=sdf->Get<double>("seal_friction");
  if(sdf->HasElement("bump_stiffness"))bumpStiffness=sdf->Get<double>("bump_stiffness");
  if(sdf->HasElement("force_cap"))forceCap=sdf->Get<double>("force_cap");
  if(sdf->HasElement("bump_lower"))bumpLower=sdf->Get<double>("bump_lower");
  if(sdf->HasElement("bump_upper"))bumpUpper=sdf->Get<double>("bump_upper");
  if(sdf->HasElement("seal_velocity_scale"))sealVelocity=sdf->Get<double>("seal_velocity_scale");
  if(sdf->HasElement("adjustment_time"))adjustmentTime=sdf->Get<double>("adjustment_time");
  if(sdf->HasElement("reference_temperature"))referenceTemperature=sdf->Get<double>("reference_temperature");
  telemetry=node.Advertise<gz::msgs::StringMsg>("/morph/suspension/telemetry");
  if (sdf->HasElement("stiffness")) requestedK=sdf->Get<double>("stiffness");
  if (sdf->HasElement("damping")) requestedC=sdf->Get<double>("damping");
  requestedK=std::clamp(requestedK,500.,4000.); requestedC=std::clamp(requestedC,20.,180.);
  k=requestedK;c=requestedC;
  if (sdf->HasElement("joint")) {
   auto sdfCopy=sdf->Clone();
   auto element=sdfCopy->GetElement("joint");
   while(element){springs.push_back({element->Get<std::string>(),gz::sim::kNullEntity});element=element->GetNextElement("joint");}
  }
  for(auto &spring:springs)spring.temperature=ambient;
  node.Subscribe("/morph/suspension/ambient",&SuspensionSystem::OnAmbient,this);
  node.Subscribe("/morph/suspension/stiffness",&SuspensionSystem::OnStiffness,this);
  node.Subscribe("/morph/suspension/damping",&SuspensionSystem::OnDamping,this);
 }
 void OnAmbient(const gz::msgs::Double &m){if(std::isfinite(m.data())){std::lock_guard<std::mutex> lock(mutex);ambient=std::clamp(m.data(),-30.,60.);}}
 void OnStiffness(const gz::msgs::Double &m){if(std::isfinite(m.data())){std::lock_guard<std::mutex> lock(mutex);requestedK=std::clamp(m.data(),500.,4000.);}}
 void OnDamping(const gz::msgs::Double &m){if(std::isfinite(m.data())){std::lock_guard<std::mutex> lock(mutex);requestedC=std::clamp(m.data(),20.,180.);}}
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
  if(info.paused)return;
  const double dt=std::chrono::duration<double>(info.dt).count();
  if(dt<=0. || dt>.1)return;
  double ambientNow;
  {std::lock_guard<std::mutex> lock(mutex);ambientNow=ambient;const double alpha=1.-std::exp(-dt/adjustmentTime);k+=alpha*(requestedK-k);c+=alpha*(requestedC-c);}
  for(auto &s:springs){
   if(s.entity==gz::sim::kNullEntity)s.entity=model.JointByName(ecm,s.name);
   if(s.entity==gz::sim::kNullEntity)continue;
   auto q=ecm.Component<gz::sim::components::JointPosition>(s.entity);
   auto v=ecm.Component<gz::sim::components::JointVelocity>(s.entity);
   if(!q)ecm.CreateComponent(s.entity,gz::sim::components::JointPosition());
   if(!v)ecm.CreateComponent(s.entity,gz::sim::components::JointVelocity());
   if(!q || !v || q->Data().empty() || v->Data().empty())continue;
   const double pos=q->Data()[0],vel=v->Data()[0];
   // Oil viscosity falls with temperature. Passive viscous/seal terms only:
   // dissipated heat >= 0; cooling returns energy to the ambient reservoir.
   s.effectiveC=c*std::clamp(std::exp(-tempCoefficient*(s.temperature-referenceTemperature)),.35,1.8);
   const double dissipative=s.effectiveC*vel+sealFriction*std::tanh(vel/sealVelocity);
   const double bump=bumpStiffness*(std::max(0.,pos-bumpUpper)+std::min(0.,pos-bumpLower));
   double force=-k*pos-dissipative-bump;
   s.heat=std::min(forceCap*std::abs(vel),std::max(0.,dissipative*vel));
   s.temperature+=dt*(s.heat-(s.temperature-ambientNow)/resistance)/capacity;
   if(!std::isfinite(force))force=0.;force=std::clamp(force,-forceCap,forceCap);
   auto cmd=ecm.Component<gz::sim::components::JointForceCmd>(s.entity);
   if(!cmd)ecm.CreateComponent(s.entity,gz::sim::components::JointForceCmd({force}));
   else cmd->Data()={force};
  }
  const double now=std::chrono::duration<double>(info.simTime).count();
  if(now-lastPublish>=.05 || now<lastPublish){
   lastPublish=now;std::ostringstream out;
   out << "{\"sim_time_s\":" << now << ",\"parameter_status\":\"assumed\",\"temperature_C\":[";
   for(size_t i=0;i<springs.size();++i){if(i)out<<",";out<<springs[i].temperature;}
   out<<"],\"dissipated_W\":[";
   for(size_t i=0;i<springs.size();++i){if(i)out<<",";out<<springs[i].heat;}
   out<<"],\"effective_damping_Ns_m\":[";
   for(size_t i=0;i<springs.size();++i){if(i)out<<",";out<<springs[i].effectiveC;}
   out<<"]}";gz::msgs::StringMsg msg;msg.set_data(out.str());telemetry.Publish(msg);
  }
 }
};
}
GZ_ADD_PLUGIN(morph_sim::SuspensionSystem,gz::sim::System,
 morph_sim::SuspensionSystem::ISystemConfigure,
 morph_sim::SuspensionSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(morph_sim::SuspensionSystem,"morph_sim::SuspensionSystem")
