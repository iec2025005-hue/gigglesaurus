// Reuse installed ros_gz conversion code, with explicit transport-first teardown.
// Keep the ROS context and owning node alive until Gazebo callbacks are removed.
#include <chrono>
#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>
namespace {
volatile std::sig_atomic_t stopRequested=0;
void Stop(int){stopRequested=1;}
class LifecycleBridge final : public ros_gz_bridge::RosGzBridge {
 public:
 void Quiesce(){
  if(heartbeat_timer_)heartbeat_timer_->cancel();
  // Unsubscribe while callback owners and ROS publishers/context still exist.
  if(gz_node_)for(const auto &topic:gz_node_->SubscribedTopics())gz_node_->Unsubscribe(topic);
  handles_.clear();services_.clear();gz_node_.reset();
  RCLCPP_INFO(get_logger(),"Bridge transport quiesced before ROS context shutdown");
 }
};
}
int main(int argc,char **argv){
 rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
 std::signal(SIGINT,Stop);std::signal(SIGTERM,Stop);
 auto bridge=std::make_shared<LifecycleBridge>();
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(bridge);
 try{
  while(!stopRequested && rclcpp::ok())executor.spin_once(std::chrono::milliseconds(10));
  executor.remove_node(bridge);bridge->Quiesce();bridge.reset();
  if(rclcpp::ok())rclcpp::shutdown();
  return 0;
 }catch(const std::exception &error){
  RCLCPP_ERROR(bridge->get_logger(),"Bridge failure: %s",error.what());
  executor.remove_node(bridge);bridge->Quiesce();bridge.reset();
  if(rclcpp::ok())rclcpp::shutdown();return 1;
 }
}
