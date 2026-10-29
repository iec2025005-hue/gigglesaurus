// Local simulated drive-driver watchdog. Hardware/firmware equivalence is unvalidated.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <stdexcept>
#include <vector>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
namespace morph_sim {
class DriveWatchdog final: public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 struct Axis {std::string name;gz::sim::Entity entity{gz::sim::kNullEntity};double hold{0.};bool captured{false};};
 gz::sim::Model model{gz::sim::kNullEntity};std::vector<Axis> wheels,positions;
 gz::transport::Node node;gz::transport::Node::Publisher telemetry;std::mutex mutex;
 bool ready{false},trace{false},poweredHold{false},wasLive{false};std::chrono::steady_clock::time_point received{};
 double timeout{.2},torqueLimit{.5},brakeGain{.21875},lastPublish{-1.},energy{0.},holdKp{0.},holdEnergy{0.},captureSpeed{.2};
 public:
 void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,gz::sim::EntityComponentManager &,gz::sim::EventManager &)override{
  if(sdf->HasElement("trace"))trace=sdf->Get<bool>("trace");
  poweredHold=sdf->Get<std::string>("mode")=="powered_hold";holdKp=sdf->Get<double>("hold_kp");
  if(!std::isfinite(holdKp) || holdKp<=0)throw std::runtime_error("Invalid watchdog hold gain");
  captureSpeed=sdf->Get<double>("capture_speed");
  if(!std::isfinite(captureSpeed) || captureSpeed<=0)throw std::runtime_error("Invalid hold capture speed");
  model=gz::sim::Model(entity);timeout=sdf->Get<double>("timeout");torqueLimit=sdf->Get<double>("torque_limit");brakeGain=sdf->Get<double>("brake_gain");
  if(!(timeout>0 && timeout<=.2 && torqueLimit>0 && torqueLimit<=.5 && brakeGain>0))throw std::runtime_error("Invalid native watchdog safety settings");
  for(const auto &key:{"wheel","position_joint"}){
   if(!sdf->HasElement(key))continue;auto copy=sdf->Clone();auto element=copy->GetElement(key);
   while(element){(std::string(key)=="wheel"?wheels:positions).push_back({element->Get<std::string>(),gz::sim::kNullEntity});element=element->GetNextElement(key);}
  }
  node.Subscribe("/morph/actuator/watchdog",&DriveWatchdog::Heartbeat,this);
  telemetry=node.Advertise<gz::msgs::StringMsg>("/morph/watchdog/status");
 }
 void Heartbeat(const gz::msgs::Boolean &message){std::lock_guard<std::mutex> lock(mutex);ready=message.data();received=std::chrono::steady_clock::now();}
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm)override{
  if(info.paused)return;
  bool live;{std::lock_guard<std::mutex> lock(mutex);live=ready && std::chrono::duration<double>(std::chrono::steady_clock::now()-received).count()<=timeout;}
  double power=0.,holdPower=0.,maxTorque=0.;size_t measured=0;
  for(auto &wheel:wheels){
   if(wheel.entity==gz::sim::kNullEntity)wheel.entity=model.JointByName(ecm,wheel.name);
   if(wheel.entity==gz::sim::kNullEntity)continue;
   auto velocity=ecm.Component<gz::sim::components::JointVelocity>(wheel.entity);
   if(!velocity){ecm.CreateComponent(wheel.entity,gz::sim::components::JointVelocity());continue;}
   auto position=ecm.Component<gz::sim::components::JointPosition>(wheel.entity);
   if(!position){ecm.CreateComponent(wheel.entity,gz::sim::components::JointPosition());continue;}
   if(live){wheel.captured=false;continue;}
   double effort=0.;
   if(!velocity->Data().empty() && std::isfinite(velocity->Data()[0])){
    const double speed=velocity->Data()[0];effort=-brakeGain*speed;
    if(poweredHold && !position->Data().empty() && std::isfinite(position->Data()[0])){
     const double q=position->Data()[0];
     if(!wheel.captured){wheel.hold=q;if(std::abs(speed)<=captureSpeed)wheel.captured=true;}
     if(wheel.captured)effort-=holdKp*(q-wheel.hold);
    }
    effort=std::clamp(effort,-torqueLimit,torqueLimit);power+=std::max(0.,-effort*speed);holdPower+=std::max(0.,effort*speed);maxTorque=std::max(maxTorque,std::abs(effort));++measured;
   }
   auto command=ecm.Component<gz::sim::components::JointForceCmd>(wheel.entity);
   if(command)command->Data()={effort};else ecm.CreateComponent(wheel.entity,gz::sim::components::JointForceCmd({effort}));
  }
  if(!live)for(auto &axis:positions){
   if(axis.entity==gz::sim::kNullEntity)axis.entity=model.JointByName(ecm,axis.name);
   if(axis.entity==gz::sim::kNullEntity)continue;
   auto velocity=ecm.Component<gz::sim::components::JointVelocity>(axis.entity);
   if(!velocity){ecm.CreateComponent(axis.entity,gz::sim::components::JointVelocity());continue;}
   if(velocity->Data().empty() || !std::isfinite(velocity->Data()[0]))continue;
   auto command=ecm.Component<gz::sim::components::JointVelocityCmd>(axis.entity);
   if(command)command->Data()={0.};else ecm.CreateComponent(axis.entity,gz::sim::components::JointVelocityCmd({0.}));
  }
  const double dt=std::chrono::duration<double>(info.dt).count();if(dt>0 && dt<=.1){energy+=power*dt;holdEnergy+=holdPower*dt;}wasLive=live;
  const double now=std::chrono::duration<double>(info.simTime).count();
  if(now-lastPublish>=.1 || now<lastPublish){lastPublish=now;std::ostringstream out;out<<"{\"sim_time_s\":"<<now<<",\"heartbeat_fresh\":"<<(live?"true":"false")<<",\"locally_measured_wheels\":"<<measured<<",\"brake_power_W\":"<<power<<",\"brake_energy_J\":"<<energy<<",\"powered_hold\":"<<(poweredHold?"true":"false")<<",\"hold_mechanical_power_W\":"<<holdPower<<",\"hold_mechanical_energy_J\":"<<holdEnergy<<",\"max_torque_Nm\":"<<maxTorque<<",\"parameter_status\":\"assumed\",\"scope\":\"simulated local drive watchdog; powered hold requires healthy supply and encoder; hardware equivalence and electrical/thermal allocation unvalidated\"}";if(trace){
    std::string payload=out.str();payload.pop_back();out.str("");out.clear();out<<payload<<",\"position_axes\":[";
    bool first=true;for(const auto &axis:positions){
     auto v=ecm.Component<gz::sim::components::JointVelocity>(axis.entity);auto q=ecm.Component<gz::sim::components::JointPosition>(axis.entity);auto command=ecm.Component<gz::sim::components::JointVelocityCmd>(axis.entity);
     if(!v || !q || !command || v->Data().empty() || q->Data().empty() || command->Data().empty())continue;
     if(!std::isfinite(v->Data()[0]) || !std::isfinite(q->Data()[0]) || !std::isfinite(command->Data()[0]))continue;
     if(!first)out<<",";first=false;out<<"{\"joint\":\""<<axis.name<<"\",\"position\":"<<q->Data()[0]<<",\"velocity\":"<<v->Data()[0]<<",\"native_velocity_command\":"<<command->Data()[0]<<"}";
    }out<<"]}";
   }
   gz::msgs::StringMsg message;message.set_data(out.str());telemetry.Publish(message);}
 }
};
}
GZ_ADD_PLUGIN(morph_sim::DriveWatchdog,gz::sim::System,morph_sim::DriveWatchdog::ISystemConfigure,morph_sim::DriveWatchdog::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(morph_sim::DriveWatchdog,"morph_sim::DriveWatchdog")
