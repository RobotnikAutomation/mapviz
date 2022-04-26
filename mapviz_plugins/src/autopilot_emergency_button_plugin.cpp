#include <mapviz_plugins/autopilot_emergency_button_plugin.h>
#include <mapviz/mapviz_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDateTime>
#include <QMouseEvent>
#include <QTextStream>
#include <QPainter>

#if QT_VERSION >= 0x050000
#include <QGuiApplication>
#else
#include <QApplication>
#endif

// ROS libraries
#include <ros/ros.h>
#include <ros/master.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::AutopilotEmergencyButtonPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  AutopilotEmergencyButtonPlugin::AutopilotEmergencyButtonPlugin():
    config_widget_(new QWidget())
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    QPalette p_emergency(config_widget_->palette());
    p_emergency.setColor(QPalette::Background, Qt::red);
    ui_.emergencybutton->setPalette(p_emergency);

    QObject::connect(ui_.emergencybutton, SIGNAL(clicked()), this, SLOT(EmergencyStop()));
  }

  AutopilotEmergencyButtonPlugin::~AutopilotEmergencyButtonPlugin()
  {
  }

  void AutopilotEmergencyButtonPlugin::EmergencyStop()
  {
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

  QWidget* AutopilotEmergencyButtonPlugin::GetConfigWidget(QWidget* parent)
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
