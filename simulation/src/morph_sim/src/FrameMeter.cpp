// Renderer event instrumentation; excludes compositor/display scan-out latency.
#include <QCoreApplication>
#include <QMetaObject>
#include <gz/msgs/boolean.pb.h>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <vector>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Plugin.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
class FrameMeter final : public gz::gui::Plugin {
 Q_OBJECT
 using Clock=std::chrono::steady_clock;
 Clock::time_point start{},lastFrame{},lastReport{Clock::now()};
 std::vector<double> intervals,durations;
 gz::transport::Node node;
 gz::transport::Node::Publisher publisher;
 bool started{false},seenFrame{false};
 bool Shutdown(const gz::msgs::Boolean &request,gz::msgs::Boolean &reply){
  reply.set_data(request.data());
  if(request.data())QMetaObject::invokeMethod(QCoreApplication::instance(),[](){QCoreApplication::quit();},Qt::QueuedConnection);
  return true;
 }
 protected:
 void LoadConfig(const tinyxml2::XMLElement *)override{
  title="Renderer timing";
  node.Advertise("/morph/gui/shutdown",&FrameMeter::Shutdown,this);
  publisher=node.Advertise<gz::msgs::StringMsg>("/morph/gui/frame_timing");
  auto window=gz::gui::App()->findChild<gz::gui::MainWindow *>();
  if(window)window->installEventFilter(this);
 }
 bool eventFilter(QObject *object,QEvent *event)override{
  auto now=Clock::now();
  if(event->type()==gz::gui::events::PreRender::kType){start=now;started=true;}
  if(event->type()==gz::gui::events::Render::kType){
   if(started){durations.push_back(std::chrono::duration<double,std::milli>(now-start).count());started=false;}
   if(seenFrame)intervals.push_back(std::chrono::duration<double,std::milli>(now-lastFrame).count());
   lastFrame=now;seenFrame=true;
   if(std::chrono::duration<double>(now-lastReport).count()>=1. && !intervals.empty()){
    auto percentile=[](std::vector<double> values){std::sort(values.begin(),values.end());return values[static_cast<size_t>(.95*(values.size()-1))];};
    double total=0.;for(double value:intervals)total+=value;
    std::ostringstream out;out<<"{\"observer_wall_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()<<",\"frames\":"<<intervals.size()<<",\"mean_frame_interval_ms\":"<<total/intervals.size()<<",\"frame_interval_p95_ms\":"<<percentile(intervals)<<",\"render_section_p95_ms\":"<<(durations.empty()?0.:percentile(durations))<<",\"scope\":\"Gazebo GUI PreRender to Render event section and rendered-event intervals; excludes compositor, scan-out and asynchronous GPU completion\"}";
    gz::msgs::StringMsg message;message.set_data(out.str());publisher.Publish(message);intervals.clear();durations.clear();lastReport=now;
   }
  }
  return QObject::eventFilter(object,event);
 }
};
GZ_ADD_PLUGIN(FrameMeter,gz::gui::Plugin)
#include "FrameMeter.moc"
