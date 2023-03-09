#include <mapviz_plugins/autopilot_mission_plugin.h>
#include <mapviz/mapviz_plugin.h>

#if QT_VERSION >= 0x050000
#include <QGuiApplication>
#else
#include <QApplication>
#endif

// ROS libraries
#include <ros/ros.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::AutopilotMissionPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  AutopilotMissionPlugin::AutopilotMissionPlugin():
    config_widget_(new QWidget())
    {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    QObject::connect(ui_.MissionPlanAcceptbutton, SIGNAL(clicked()), this, SLOT(AcceptMissionPlan()));
    QObject::connect(ui_.SwarmEventAcceptbutton, SIGNAL(clicked()), this, SLOT(AcceptSwarmEvent()));
    
    ui_.MissionPlanAcceptbutton->setEnabled(false);
    ui_.SwarmEventAcceptbutton->setEnabled(false);

    mission_plan_debug_sub_ = node_.subscribe("/mission_plan_debug", 1, &AutopilotMissionPlugin::MissionPlanCB, this);
    swarm_event_debug_sub_ = node_.subscribe("/swarm_event_debug", 1, &AutopilotMissionPlugin::SwarmEventCB, this);

    confirm_mission_plan_ = node_.serviceClient<std_srvs::SetBool>("/confirm_mission_plan");
    confirm_swarm_event_ = node_.serviceClient<std_srvs::SetBool>("/confirm_swarm_event");
  }

  QWidget* AutopilotMissionPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool AutopilotMissionPlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = static_cast< mapviz::MapCanvas* >(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;

    return true;
  }

  AutopilotMissionPlugin::~AutopilotMissionPlugin()
  {
  }

  void AutopilotMissionPlugin::MissionPlanCB(const nav_msgs::Path &msg)
  {
    ui_.MissionPlanAcceptbutton->setEnabled(true);
    mission_plan_timer_ = node_.createTimer(ros::Duration(1), &AutopilotMissionPlugin::flashingMissionPlan, this);
  }

  void AutopilotMissionPlugin::SwarmEventCB(const nav_msgs::Path &msg)
  {
    ui_.SwarmEventAcceptbutton->setEnabled(true);
    swarm_event_timer_ = node_.createTimer(ros::Duration(1), &AutopilotMissionPlugin::flashingSwarmEvent, this);
  }

  void AutopilotMissionPlugin::AcceptMissionPlan()
  {
    QPalette p_text;

    std_srvs::SetBool sb;
    sb.request.data = true;
    confirm_mission_plan_.call(sb);

    mission_plan_timer_.stop();

    p_text = config_widget_->palette();
    p_text.setColor(QPalette::Text, QColor(Qt::black));
    ui_.MissionPlan->setAutoFillBackground(true);
    ui_.MissionPlan->setPalette(p_text);
    ui_.MissionPlan->update();
    mission_plan_flashing_ = false;

    ui_.MissionPlanAcceptbutton->setEnabled(false);
  }

  void AutopilotMissionPlugin::AcceptSwarmEvent()
  {
    QPalette p_text;

    std_srvs::SetBool sb;
    sb.request.data = true;
    confirm_swarm_event_.call(sb);

    swarm_event_timer_.stop();

    p_text = config_widget_->palette();
    p_text.setColor(QPalette::Text, QColor(Qt::black));
    ui_.SwarmEvent->setAutoFillBackground(true);
    ui_.SwarmEvent->setPalette(p_text);
    ui_.SwarmEvent->update();
    swarm_event_flashing_ = false;

    ui_.SwarmEventAcceptbutton->setEnabled(false);
  }

  void AutopilotMissionPlugin::flashingMissionPlan(const ros::TimerEvent& event)
  {
      QPalette p_text;

      if(!mission_plan_flashing_)
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::green));
        ui_.MissionPlan->setAutoFillBackground(true);
        ui_.MissionPlan->setPalette(p_text);
        ui_.MissionPlan->update();
        mission_plan_flashing_ = true;
      }
      else
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::black));
        ui_.MissionPlan->setAutoFillBackground(true);
        ui_.MissionPlan->setPalette(p_text);
        ui_.MissionPlan->update();
        mission_plan_flashing_ = false;
      }
  }

  void AutopilotMissionPlugin::flashingSwarmEvent(const ros::TimerEvent& event)
  {
      QPalette p_text;

      if(!swarm_event_flashing_)
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::green));
        ui_.SwarmEvent->setAutoFillBackground(true);
        ui_.SwarmEvent->setPalette(p_text);
        ui_.SwarmEvent->update();
        swarm_event_flashing_ = true;
      }
      else
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::black));
        ui_.SwarmEvent->setAutoFillBackground(true);
        ui_.SwarmEvent->setPalette(p_text);
        ui_.SwarmEvent->update();
        swarm_event_flashing_ = false;
      }
  }

  void AutopilotMissionPlugin::PrintError(const std::string& message)
  {
  }

  void AutopilotMissionPlugin::PrintInfo(const std::string& message)
  {
  }

  void AutopilotMissionPlugin::PrintWarning(const std::string& message)
  {
  }

  void AutopilotMissionPlugin::Draw(double x, double y, double scale)
  {
  }

  void AutopilotMissionPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
  }
  void AutopilotMissionPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
  }

}
