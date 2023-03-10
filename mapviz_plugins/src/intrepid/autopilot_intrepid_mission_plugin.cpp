#include <mapviz_plugins/intrepid/autopilot_intrepid_mission_plugin.h>
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

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::AutopilotIntrepidMissionPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  AutopilotIntrepidMissionPlugin::AutopilotIntrepidMissionPlugin():
    config_widget_(new QWidget())
    {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    QObject::connect(ui_.CommandAcceptButton, SIGNAL(clicked()), this, SLOT(AcceptCommand()));

    ui_.CommandAcceptButton->setEnabled(false);

    mission_command_request_sub_ = node_.subscribe("mission_command_request", 1, &AutopilotIntrepidMissionPlugin::MissionCommandRequestCB, this);

    accept_command_pub_ = node_.advertise<std_msgs::Empty>("accept_command", 1);
  }

  QWidget* AutopilotIntrepidMissionPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool AutopilotIntrepidMissionPlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = static_cast< mapviz::MapCanvas* >(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;

    return true;
  }

  AutopilotIntrepidMissionPlugin::~AutopilotIntrepidMissionPlugin()
  {
  }

  void AutopilotIntrepidMissionPlugin::MissionCommandRequestCB(const std_msgs::String &msg)
  {
    ui_.CommandAcceptButton->setEnabled(true);

    if(msg.data == "goto")
      goto_timer_ = node_.createTimer(ros::Duration(1), &AutopilotIntrepidMissionPlugin::flashingGoTo, this);
    else if(msg.data == "pickup")
      pick_up_timer_ = node_.createTimer(ros::Duration(1), &AutopilotIntrepidMissionPlugin::flashingPickUp, this);
    else if(msg.data == "putdown")
      put_down_timer_ = node_.createTimer(ros::Duration(1), &AutopilotIntrepidMissionPlugin::flashingPutDown, this);
    else if (msg.data == "scan_area")
      scan_area_timer_ = node_.createTimer(ros::Duration(1), &AutopilotIntrepidMissionPlugin::flashingScanArea, this);
  }

  void AutopilotIntrepidMissionPlugin::AcceptCommand()
  {
    QPalette p_text;

    std_msgs::Empty msg;
    accept_command_pub_.publish(msg);

    goto_timer_.stop();
    scan_area_timer_.stop();
    pick_up_timer_.stop();
    put_down_timer_.stop();

    p_text = config_widget_->palette();
    p_text.setColor(QPalette::Text, QColor(Qt::black));

    ui_.GoTo->setAutoFillBackground(true);
    ui_.GoTo->setPalette(p_text);
    ui_.GoTo->update();
    ui_.ScanArea->setAutoFillBackground(true);
    ui_.ScanArea->setPalette(p_text);
    ui_.ScanArea->update();
    ui_.PickUp->setAutoFillBackground(true);
    ui_.PickUp->setPalette(p_text);
    ui_.PickUp->update();
    ui_.PutDown->setAutoFillBackground(true);
    ui_.PutDown->setPalette(p_text);
    ui_.PutDown->update();

    goto_flashing_ = false;
    scan_area_flashing_ = false;
    pick_up_flashing_ = false;
    put_down_flashing_ = false;

    ui_.CommandAcceptButton->setEnabled(false);
  }

  void AutopilotIntrepidMissionPlugin::flashingGoTo(const ros::TimerEvent& event)
  {
      QPalette p_text;

      if(!goto_flashing_)
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::green));
        ui_.GoTo->setAutoFillBackground(true);
        ui_.GoTo->setPalette(p_text);
        ui_.GoTo->update();
        goto_flashing_ = true;
      }
      else
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::black));
        ui_.GoTo->setAutoFillBackground(true);
        ui_.GoTo->setPalette(p_text);
        ui_.GoTo->update();
        goto_flashing_ = false;
      }
  }

  void AutopilotIntrepidMissionPlugin::flashingScanArea(const ros::TimerEvent& event)
  {
      QPalette p_text;

      if(!scan_area_flashing_)
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::green));
        ui_.ScanArea->setAutoFillBackground(true);
        ui_.ScanArea->setPalette(p_text);
        ui_.ScanArea->update();
        scan_area_flashing_ = true;
      }
      else
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::black));
        ui_.ScanArea->setAutoFillBackground(true);
        ui_.ScanArea->setPalette(p_text);
        ui_.ScanArea->update();
        scan_area_flashing_ = false;
      }
  }

  void AutopilotIntrepidMissionPlugin::flashingPickUp(const ros::TimerEvent& event)
  {
      QPalette p_text;

      if(!pick_up_flashing_)
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::green));
        ui_.PickUp->setAutoFillBackground(true);
        ui_.PickUp->setPalette(p_text);
        ui_.PickUp->update();
        pick_up_flashing_ = true;
      }
      else
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::black));
        ui_.PickUp->setAutoFillBackground(true);
        ui_.PickUp->setPalette(p_text);
        ui_.PickUp->update();
        pick_up_flashing_ = false;
      }
  }

  void AutopilotIntrepidMissionPlugin::flashingPutDown(const ros::TimerEvent& event)
  {
      QPalette p_text;

      if(!put_down_flashing_)
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::green));
        ui_.PutDown->setAutoFillBackground(true);
        ui_.PutDown->setPalette(p_text);
        ui_.PutDown->update();
        put_down_flashing_ = true;
      }
      else
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::black));
        ui_.PutDown->setAutoFillBackground(true);
        ui_.PutDown->setPalette(p_text);
        ui_.PutDown->update();
        put_down_flashing_ = false;
      }
  }

  void AutopilotIntrepidMissionPlugin::PrintError(const std::string& message)
  {
  }

  void AutopilotIntrepidMissionPlugin::PrintInfo(const std::string& message)
  {
  }

  void AutopilotIntrepidMissionPlugin::PrintWarning(const std::string& message)
  {
  }

  void AutopilotIntrepidMissionPlugin::Draw(double x, double y, double scale)
  {
  }

  void AutopilotIntrepidMissionPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
  }
  void AutopilotIntrepidMissionPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
  }

}
