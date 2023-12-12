#include <mapviz_plugins/autopilot_emergency_button_plugin.h>
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

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::AutopilotEmergencyButtonPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  AutopilotEmergencyButtonPlugin::AutopilotEmergencyButtonPlugin():
    config_widget_(new QWidget()),
    emergency_value_(false),
    is_estop_(false),
    is_rearm_(true)
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    QPalette p_emergency(config_widget_->palette());
    p_emergency.setColor(QPalette::Button, QColor(Qt::red));
    ui_.emergencybutton->setAutoFillBackground(true);
    ui_.emergencybutton->setPalette(p_emergency);
    ui_.emergencybutton->update();

    QPalette p_rearm(config_widget_->palette());
    p_rearm.setColor(QPalette::Button, QColor(Qt::green));
    ui_.rearmbutton->setAutoFillBackground(true);
    ui_.rearmbutton->setPalette(p_rearm);
    ui_.rearmbutton->update();
    ui_.rearmbutton->setEnabled(false);

    QObject::connect(ui_.emergencybutton, SIGNAL(clicked()), this, SLOT(EmergencyStop()));
    QObject::connect(ui_.rearmbutton, SIGNAL(clicked()), this, SLOT(Rearm()));

    emergency_stop_pub_ = node_.advertise<std_msgs::Bool>("autopilot/emergency_stop", 1, true);
  }

  QWidget* AutopilotEmergencyButtonPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool AutopilotEmergencyButtonPlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = static_cast< mapviz::MapCanvas* >(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;
    PrintInfo("OK");

    return true;
  }

  AutopilotEmergencyButtonPlugin::~AutopilotEmergencyButtonPlugin()
  {
  }

  void AutopilotEmergencyButtonPlugin::EmergencyStop()
  {
    std_msgs::Bool msg;

    if(is_estop_)
      return;

    msg.data = true;

    emergency_stop_pub_.publish(msg);
    ui_.emergencybutton->setEnabled(false);
    is_estop_ = true;
    ui_.rearmbutton->setEnabled(true);
    is_rearm_ = false;
  }

  void AutopilotEmergencyButtonPlugin::Rearm()
  {
    std_msgs::Bool msg;

    if(is_rearm_)
      return;

    msg.data = false;

    emergency_stop_pub_.publish(msg);
    ui_.emergencybutton->setEnabled(true);
    is_estop_ = false;
    ui_.rearmbutton->setEnabled(false);
    is_rearm_ = true;
  }

  void AutopilotEmergencyButtonPlugin::PrintError(const std::string& message)
  {
  }

  void AutopilotEmergencyButtonPlugin::PrintInfo(const std::string& message)
  {
  }

  void AutopilotEmergencyButtonPlugin::PrintWarning(const std::string& message)
  {
  }

  void AutopilotEmergencyButtonPlugin::Draw(double x, double y, double scale)
  {
  }

  void AutopilotEmergencyButtonPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
  }
  void AutopilotEmergencyButtonPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
  }

}
