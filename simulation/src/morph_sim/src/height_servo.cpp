// Saturated physical-force height servo. Gains are simulation tuning, not motor identification.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
namespace morph_sim {
class HeightServo final:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate{
 struct Axis {std::string name;gz::sim::Entity joint{gz::sim::kNullEntity},spring{gz::sim::kNullEntity};double target{0.},hold{0.},force{0.},mass{.03},damping{10.},friction{.1},lower{-.015},upper{.015},limit{100.},speed{.025};bool captured{false},fault{true};std::chrono::steady_clock::time_point received{};};
 gz::sim::Model model{gz::sim::kNullEntity};std::array<Axis,4> axes;gz::transport::Node node;gz::transport::Node::Publisher telemetry;std::mutex mutex;
 double kp{1000.},kd{12.},timeout{.2},lastPublish{-1.},work{0.};bool ready{false};std::chrono::steady_clock::time_point heartbeat{};
 public:
 void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,gz::sim::EntityComponentManager &,gz::sim::EventManager &)override{
  model=gz::sim::Model(entity);kp=sdf->Get<double>("kp");kd=sdf->Get<double>("kd");timeout=sdf->Get<double>("timeout");
  if(!(kp>0 && kd>=0 && timeout>0 && timeout<=.2))throw std::runtime_error("Invalid height servo tuning");
  auto sdfCopy=sdf->Clone();
  std::array<std::string,4> names={"fl","fr","rl","rr"};
  for(size_t i=0;i<axes.size();++i){auto &axis=axes[i];axis.name=names[i];auto config=sdfCopy->GetElement(axis.name);
   axis.mass=config->Get<double>("mass");axis.damping=config->Get<double>("damping");axis.friction=config->Get<double>("friction");axis.lower=config->Get<double>("lower");axis.upper=config->Get<double>("upper");axis.limit=config->Get<double>("force_limit");axis.speed=config->Get<double>("speed_limit");
   if(!(axis.mass>0 && axis.limit>0 && axis.limit<=100 && axis.speed>0 && axis.speed<=.025))throw std::runtime_error("Height servo exceeds existing actuator bounds");
   std::function<void(const gz::msgs::Double &)> callback=[this,i](const gz::msgs::Double &message){if(std::isfinite(message.data())){std::lock_guard<std::mutex> lock(mutex);axes[i].target=std::clamp(message.data(),axes[i].lower,axes[i].upper);axes[i].received=std::chrono::steady_clock::now();}};node.Subscribe("/morph/height/"+axis.name+"/target",callback);
  }
  node.Subscribe("/morph/actuator/watchdog",&HeightServo::Heartbeat,this);telemetry=node.Advertise<gz::msgs::StringMsg>("/morph/height/status");
 }
 void Heartbeat(const gz::msgs::Boolean &message){std::lock_guard<std::mutex> lock(mutex);ready=message.data();heartbeat=std::chrono::steady_clock::now();}
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm)override{
  if(info.paused)return;const double dt=std::chrono::duration<double>(info.dt).count();if(dt<=0 || dt>.1)return;
  const auto wall=std::chrono::steady_clock::now();std::array<double,4> requested{};std::array<bool,4> live{};
  {std::lock_guard<std::mutex> lock(mutex);for(size_t i=0;i<axes.size();++i){requested[i]=axes[i].target;live[i]=ready && std::chrono::duration<double>(wall-heartbeat).count()<=timeout && std::chrono::duration<double>(wall-axes[i].received).count()<=timeout;}}
  double gravityAlong=9.81;auto world=ecm.EntityByComponents(gz::sim::components::World());auto gravity=ecm.Component<gz::sim::components::Gravity>(world);auto base=model.LinkByName(ecm,"base_link");
  if(gravity && base!=gz::sim::kNullEntity)gravityAlong=-gz::sim::worldPose(base,ecm).Rot().RotateVectorReverse(gravity->Data()).Z();
  double power=0.;std::ostringstream state;state<<"[";bool first=true;
  for(size_t i=0;i<axes.size();++i){auto &axis=axes[i];
   if(axis.joint==gz::sim::kNullEntity)axis.joint=model.JointByName(ecm,axis.name+"_height_joint");
   if(axis.spring==gz::sim::kNullEntity)axis.spring=model.JointByName(ecm,axis.name+"_spring_joint");
   if(axis.joint==gz::sim::kNullEntity || axis.spring==gz::sim::kNullEntity)continue;
   auto q=ecm.Component<gz::sim::components::JointPosition>(axis.joint);auto v=ecm.Component<gz::sim::components::JointVelocity>(axis.joint);
   if(!q)ecm.CreateComponent(axis.joint,gz::sim::components::JointPosition());if(!v)ecm.CreateComponent(axis.joint,gz::sim::components::JointVelocity());
   if(!q || !v || q->Data().empty() || v->Data().empty())continue;double position=q->Data()[0],velocity=v->Data()[0];if(!std::isfinite(position) || !std::isfinite(velocity))continue;
   if(!axis.captured || (!live[i] && !axis.fault)){axis.hold=std::clamp(position,axis.lower,axis.upper);axis.captured=true;}axis.fault=!live[i];
   const double target=live[i]?requested[i]:axis.hold;
   // Disable the kinematic velocity backend for this axis. Its ROS position
   // interface remains claimed and bounded; this force servo is the sole plant actuator.
   auto velocityCommand=ecm.Component<gz::sim::components::JointVelocityCmd>(axis.joint);
   if(velocityCommand)velocityCommand->Data().clear();else ecm.CreateComponent(axis.joint,gz::sim::components::JointVelocityCmd());
   auto spring=ecm.Component<gz::sim::components::JointForceCmd>(axis.spring);const double springForce=spring && !spring->Data().empty()?spring->Data()[0]:0.;
   const double feedforward=-springForce-axis.mass*gravityAlong+axis.damping*velocity+axis.friction*std::tanh(velocity/.001);
   // Backward-Euler regularization avoids an explicit high-gain update on the light carriage.
   const double denominator=1.+kd*dt/axis.mass+kp*dt*dt/axis.mass;
   double corrective=(kp*(target-position-dt*velocity)-kd*velocity)/denominator;
   if(std::abs(velocity)>=axis.speed && corrective*velocity>0)corrective=0.;
   axis.force=std::clamp(feedforward+corrective,-axis.limit,axis.limit);
   if(!std::isfinite(axis.force))axis.force=0.;auto force=ecm.Component<gz::sim::components::JointForceCmd>(axis.joint);
   if(force)force->Data()={axis.force};else ecm.CreateComponent(axis.joint,gz::sim::components::JointForceCmd({axis.force}));
   power+=axis.force*velocity;
   if(!first)state<<",";first=false;state<<"{\"joint\":\""<<axis.name<<"_height_joint\",\"target_m\":"<<target<<",\"position_m\":"<<position<<",\"velocity_m_s\":"<<velocity<<",\"force_N\":"<<axis.force<<",\"fault_hold\":"<<(axis.fault?"true":"false")<<"}";
  }state<<"]";work+=power*dt;double now=std::chrono::duration<double>(info.simTime).count();
  if(now-lastPublish>=.1 || now<lastPublish){lastPublish=now;std::ostringstream out;out<<"{\"sim_time_s\":"<<now<<",\"parameter_status\":\"assumed\",\"scope\":\"force-limited simulated height servo; uncalibrated motor and thermal behavior\",\"mechanical_power_W\":"<<power<<",\"signed_mechanical_work_J\":"<<work<<",\"axes\":"<<state.str()<<"}";gz::msgs::StringMsg message;message.set_data(out.str());telemetry.Publish(message);}
 }
};
}
GZ_ADD_PLUGIN(morph_sim::HeightServo,gz::sim::System,morph_sim::HeightServo::ISystemConfigure,morph_sim::HeightServo::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(morph_sim::HeightServo,"morph_sim::HeightServo")
