#ifndef MAPVIZ_PLUGINS_AUTOPILOT_EMERGENCY_BUTTON_PLUGIN_H_
#define MAPVIZ_PLUGINS_AUTOPILOT_EMERGENCY_BUTTON_PLUGIN_H_

// C++ standard libraries
#include <string>
#include <list>

#include <mapviz/mapviz_plugin.h>

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// QT autogenerated files
#include "ui_autopilot_emergency_button_config.h"

namespace mapviz_plugins
{
  class AutopilotEmergencyButtonPlugin : public mapviz::MapvizPlugin
  {
    Q_OBJECT

   public:
    AutopilotEmergencyButtonPlugin();
    virtual ~AutopilotEmergencyButtonPlugin();

    bool Initialize(QGLWidget* canvas);
    void Shutdown()
    {
    }

    void Draw(double x, double y, double scale);
    void Transform() { };

    void LoadConfig(const YAML::Node& node, const std::string& path);
    void SaveConfig(YAML::Emitter& emitter, const std::string& path);

    QWidget* GetConfigWidget(QWidget* parent);

    void PrintError(const std::string& message);
    void PrintInfo(const std::string& message);
    void PrintWarning(const std::string& message);

    protected Q_SLOTS:
    void EmergencyStop();

    private:
    Ui::autopilot_emergency_button_config ui_;
    QWidget* config_widget_;

    ros::Publisher emergency_stop_pub_;
    };

    }

    #endif // MAPVIZ_PLUGINS_AUTOPILOT_EMERGENCY_BUTTON_PLUGIN_H_
