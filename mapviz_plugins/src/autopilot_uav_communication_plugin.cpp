#include <mapviz_plugins/autopilot_uav_communication_plugin.h>
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

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::AutopilotUAVCommunicationPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  AutopilotUAVCommunicationPlugin::AutopilotUAVCommunicationPlugin():
    config_widget_(new QWidget())
    {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    QObject::connect(ui_.LandGrantbutton, SIGNAL(clicked()), this, SLOT(AcceptLanding()));
    QObject::connect(ui_.TakeOffGrantbutton, SIGNAL(clicked()), this, SLOT(AcceptTakeOff()));
    ui_.LandGrantbutton->setEnabled(false);
    ui_.TakeOffGrantbutton->setEnabled(false);

    uav_land_request_sub_ = node_.subscribe("land_request", 1, &AutopilotUAVCommunicationPlugin::landRequestCB, this);
    uav_take_off_request_sub_ = node_.subscribe("take_off_request", 1, &AutopilotUAVCommunicationPlugin::takeOffRequestCB, this);

    uav_land_granted_pub_ = node_.advertise<std_msgs::Empty>("land_granted", 1);
    uav_take_off_granted_pub_ = node_.advertise<std_msgs::Empty>("take_off_granted", 1);
  }

  QWidget* AutopilotUAVCommunicationPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool AutopilotUAVCommunicationPlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = static_cast< mapviz::MapCanvas* >(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;

    return true;
  }

  AutopilotUAVCommunicationPlugin::~AutopilotUAVCommunicationPlugin()
  {
  }

  void AutopilotUAVCommunicationPlugin::takeOffRequestCB(const std_msgs::Empty &msg)
  {
    ui_.TakeOffGrantbutton->setEnabled(true);
    take_off_request_timer_ = node_.createTimer(ros::Duration(1), &AutopilotUAVCommunicationPlugin::flashingTakeOffRequest, this);
  }

  void AutopilotUAVCommunicationPlugin::landRequestCB(const std_msgs::Empty &msg)
  {
    if(!land_req_received_)
    {
      land_req_received_ = true;
      ui_.LandGrantbutton->setEnabled(true);
      land_request_timer_ = node_.createTimer(ros::Duration(1), &AutopilotUAVCommunicationPlugin::flashingLandRequest, this);
    }
  }

  void AutopilotUAVCommunicationPlugin::AcceptTakeOff()
  {
    std_msgs::Empty msg;
    QPalette p_text;

    uav_take_off_granted_pub_.publish(msg);

    take_off_request_timer_.stop();

    p_text = config_widget_->palette();
    p_text.setColor(QPalette::Text, QColor(Qt::black));
    ui_.TakeOffRequest->setAutoFillBackground(true);
    ui_.TakeOffRequest->setPalette(p_text);
    ui_.TakeOffRequest->update();
    take_off_flashing_ = false;

    ui_.TakeOffGrantbutton->setEnabled(false);
  }

  void AutopilotUAVCommunicationPlugin::AcceptLanding()
  {
    std_msgs::Empty msg;
    QPalette p_text;

    uav_land_granted_pub_.publish(msg);

    land_request_timer_.stop();

    p_text = config_widget_->palette();
    p_text.setColor(QPalette::Text, QColor(Qt::black));
    ui_.LandRequest->setAutoFillBackground(true);
    ui_.LandRequest->setPalette(p_text);
    ui_.LandRequest->update();
    land_flashing_ = false;

    ui_.LandGrantbutton->setEnabled(false);
    land_req_received_ = false;
  }

  void AutopilotUAVCommunicationPlugin::flashingTakeOffRequest(const ros::TimerEvent& event)
  {
      QPalette p_text;

      if(!take_off_flashing_)
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::green));
        ui_.TakeOffRequest->setAutoFillBackground(true);
        ui_.TakeOffRequest->setPalette(p_text);
        ui_.TakeOffRequest->update();
        take_off_flashing_ = true;
      }
      else
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::black));
        ui_.TakeOffRequest->setAutoFillBackground(true);
        ui_.TakeOffRequest->setPalette(p_text);
        ui_.TakeOffRequest->update();
        take_off_flashing_ = false;
      }
  }

  void AutopilotUAVCommunicationPlugin::flashingLandRequest(const ros::TimerEvent& event)
  {
      QPalette p_text;

      if(!land_flashing_)
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::green));
        ui_.LandRequest->setAutoFillBackground(true);
        ui_.LandRequest->setPalette(p_text);
        ui_.LandRequest->update();
        land_flashing_ = true;
      }
      else
      {
        p_text = config_widget_->palette();
        p_text.setColor(QPalette::Text, QColor(Qt::black));
        ui_.LandRequest->setAutoFillBackground(true);
        ui_.LandRequest->setPalette(p_text);
        ui_.LandRequest->update();
        land_flashing_ = false;
      }
  }

  void AutopilotUAVCommunicationPlugin::PrintError(const std::string& message)
  {
  }

  void AutopilotUAVCommunicationPlugin::PrintInfo(const std::string& message)
  {
  }

  void AutopilotUAVCommunicationPlugin::PrintWarning(const std::string& message)
  {
  }

  void AutopilotUAVCommunicationPlugin::Draw(double x, double y, double scale)
  {
  }

  void AutopilotUAVCommunicationPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
  }
  void AutopilotUAVCommunicationPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
  }

}
