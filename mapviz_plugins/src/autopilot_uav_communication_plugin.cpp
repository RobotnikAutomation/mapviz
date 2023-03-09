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
    // Set robot_status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.UAVOpenGripperButton, SIGNAL(clicked()), this, SLOT(UAVOpenGripperCB()));
    QObject::connect(ui_.UAVCloseGripperButton, SIGNAL(clicked()), this, SLOT(UAVCloseGripperCB()));
    QObject::connect(ui_.UAVContinueButton, SIGNAL(clicked()), this, SLOT(ContinueButtonCB()));
    QObject::connect(ui_.CloseValue, SIGNAL(valueChanged(int)), this, SLOT(CloseValueChanged(int)));
    QObject::connect(ui_.OpenValue, SIGNAL(valueChanged(int)), this, SLOT(OPenValueChanged(int)));
    ui_.UAVContinueButton->setEnabled(false);

    uav_land_request_sub_ = node_.subscribe("land_request", 1, &AutopilotUAVCommunicationPlugin::landRequestCB, this);
    uav_take_off_request_sub_ = node_.subscribe("take_off_request", 1, &AutopilotUAVCommunicationPlugin::takeOffRequestCB, this);
    uav_status_sub_ = node_.subscribe("uav_status", 1, &AutopilotUAVCommunicationPlugin::uavStatusCB, this);
    close_gripper_request_sub_ = node_.subscribe("close_gripper_request", 1, &AutopilotUAVCommunicationPlugin::closeGripperRequestCB, this);

    uav_land_granted_pub_ = node_.advertise<std_msgs::Empty>("land_granted", 1);
    uav_take_off_granted_pub_ = node_.advertise<std_msgs::Empty>("take_off_granted", 1);
    gripper_closed_pub_ = node_.advertise<std_msgs::Empty>("gripper_closed", 1);

    // open_uav_gripper_pub_ = node_.advertise<std_msgs::Empty>("open_uav_gripper", 1);
    // close_uav_gripper_pub_ = node_.advertise<std_msgs::Empty>("close_uav_gripper", 1);
    move_uav_gripper_pub_ = node_.advertise<std_msgs::Int64>("move_uav_gripper", 1);
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

  void AutopilotUAVCommunicationPlugin::uavStatusCB(const std_msgs::String &msg)
  {
    PrintErrorHelper(ui_.status, msg.data);
  }

  void AutopilotUAVCommunicationPlugin::takeOffRequestCB(const std_msgs::Empty &msg)
  {
    if(!take_off_req_received_)
    {
      take_off_req_received_ = true;
      ui_.UAVContinueButton->setEnabled(true);
      take_off_request_timer_ = node_.createTimer(ros::Duration(1), &AutopilotUAVCommunicationPlugin::flashingTakeOffRequest, this);
    }
  }

  void AutopilotUAVCommunicationPlugin::landRequestCB(const std_msgs::Empty &msg)
  {
    if(!land_req_received_)
    {
      land_req_received_ = true;
      ui_.UAVContinueButton->setEnabled(true);
      land_request_timer_ = node_.createTimer(ros::Duration(1), &AutopilotUAVCommunicationPlugin::flashingLandRequest, this);
    }
  }

  void AutopilotUAVCommunicationPlugin::closeGripperRequestCB(const std_msgs::Empty &msg)
  {
    if(!close_gripper_req_received_)
    {
      close_gripper_req_received_ = true;
      ui_.UAVContinueButton->setEnabled(true);
    }
  }

  void AutopilotUAVCommunicationPlugin::UAVOpenGripperCB()
  {
    std_msgs::Int64 msg;
    msg.data = open_value_;

    move_uav_gripper_pub_.publish(msg);
  }

  void AutopilotUAVCommunicationPlugin::UAVCloseGripperCB()
  {
    std_msgs::Int64 msg;
    msg.data = close_value_;

    move_uav_gripper_pub_.publish(msg);
  }

  void AutopilotUAVCommunicationPlugin::ContinueButtonCB()
  {
    std_msgs::Empty msg;
    QPalette p_text;

    if(take_off_req_received_ == true)
    {
      uav_take_off_granted_pub_.publish(msg);

      take_off_request_timer_.stop();

      p_text = config_widget_->palette();
      p_text.setColor(QPalette::Text, QColor(Qt::black));
      ui_.TakeOffRequest->setAutoFillBackground(true);
      ui_.TakeOffRequest->setPalette(p_text);
      ui_.TakeOffRequest->update();
      take_off_flashing_ = false;

      take_off_req_received_ = false;
    }
    else if(land_req_received_ == true)
    {
      uav_land_granted_pub_.publish(msg);

      land_request_timer_.stop();

      p_text = config_widget_->palette();
      p_text.setColor(QPalette::Text, QColor(Qt::black));
      ui_.LandRequest->setAutoFillBackground(true);
      ui_.LandRequest->setPalette(p_text);
      ui_.LandRequest->update();
      land_flashing_ = false;

      land_req_received_ = false;
    }
    else if(close_gripper_req_received_ == true)
    {
      gripper_closed_pub_.publish(msg);
      close_gripper_req_received_ = false;
    }

    ui_.UAVContinueButton->setEnabled(false);
  }

  void AutopilotUAVCommunicationPlugin::OpenValueChanged(const int value)
  {
    open_value_ = value;
  }

  void AutopilotUAVCommunicationPlugin::CloseValueChanged(const int value)
  {
    close_value_ = value;
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
