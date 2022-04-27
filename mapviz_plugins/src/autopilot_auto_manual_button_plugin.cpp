#include <mapviz_plugins/autopilot_auto_manual_button_plugin.h>
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

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::AutopilotAutoManualButtonPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  AutopilotAutoManualButtonPlugin::AutopilotAutoManualButtonPlugin():
    config_widget_(new QWidget()),
    is_auto_(false),
    is_manual_(false)
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    p_auto_ = config_widget_->palette();
    p_auto_.setColor(QPalette::Button, QColor(Qt::red));
    ui_.automaticmodebutton->setAutoFillBackground(true);
    ui_.automaticmodebutton->setPalette(p_auto_);
    ui_.automaticmodebutton->update();

    p_manual_ = config_widget_->palette();
    p_manual_.setColor(QPalette::Button, QColor(Qt::red));
    ui_.manualmodebutton->setAutoFillBackground(true);
    ui_.manualmodebutton->setPalette(p_manual_);
    ui_.manualmodebutton->update();

    QObject::connect(ui_.automaticmodebutton, SIGNAL(clicked()), this, SLOT(AutomaticMode()));
    QObject::connect(ui_.manualmodebutton, SIGNAL(clicked()), this, SLOT(ManualMode()));

    pause_navigation_pub_ = node_.advertise<std_msgs::Bool>("pause_navigation", 1, true);
  }

  QWidget* AutopilotAutoManualButtonPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool AutopilotAutoManualButtonPlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = static_cast< mapviz::MapCanvas* >(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;
    PrintInfo("OK");

    return true;
  }

  AutopilotAutoManualButtonPlugin::~AutopilotAutoManualButtonPlugin()
  {
  }

  void AutopilotAutoManualButtonPlugin::AutomaticMode()
  {
    std_msgs::Bool msg;

    if(is_auto_)
      return;

    msg.data = false;

    p_auto_.setColor(QPalette::Button, QColor(Qt::green));
    ui_.automaticmodebutton->setAutoFillBackground(true);
    ui_.automaticmodebutton->setPalette(p_auto_);
    ui_.automaticmodebutton->update();
    is_auto_ = true;

    p_manual_.setColor(QPalette::Button, QColor(Qt::red));
    ui_.manualmodebutton->setAutoFillBackground(true);
    ui_.manualmodebutton->setPalette(p_manual_);
    ui_.manualmodebutton->update();
    is_manual_=false;

    pause_navigation_pub_.publish(msg);
  }

  void AutopilotAutoManualButtonPlugin::ManualMode()
  {
    std_msgs::Bool msg;

    if(is_manual_)
      return;

    msg.data = true;

    p_auto_.setColor(QPalette::Button, QColor(Qt::red));
    ui_.automaticmodebutton->setAutoFillBackground(true);
    ui_.automaticmodebutton->setPalette(p_auto_);
    ui_.automaticmodebutton->update();
    is_auto_ = false;

    p_manual_.setColor(QPalette::Button, QColor(Qt::green));
    ui_.manualmodebutton->setAutoFillBackground(true);
    ui_.manualmodebutton->setPalette(p_manual_);
    ui_.manualmodebutton->update();
    is_manual_=true;

    pause_navigation_pub_.publish(msg);
  }

  void AutopilotAutoManualButtonPlugin::PrintError(const std::string& message)
  {
  }

  void AutopilotAutoManualButtonPlugin::PrintInfo(const std::string& message)
  {
  }

  void AutopilotAutoManualButtonPlugin::PrintWarning(const std::string& message)
  {
  }

  void AutopilotAutoManualButtonPlugin::Draw(double x, double y, double scale)
  {
  }

  void AutopilotAutoManualButtonPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
  }
  void AutopilotAutoManualButtonPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
  }

}
