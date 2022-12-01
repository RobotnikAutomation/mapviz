#ifndef MAPVIZ_PLUGINS_AUTOPILOT_UAV_COMMUNICATION_PLUGIN_H_
#define MAPVIZ_PLUGINS_AUTOPILOT_UAV_COMMUNICATION_PLUGIN_H_

#include <mapviz/mapviz_plugin.h>

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// Mapviz libraries
#include <mapviz/map_canvas.h>
//
// QT autogenerated files
#include "ui_autopilot_uav_communication_config.h"

namespace mapviz_plugins
{
  class AutopilotUAVCommunicationPlugin : public mapviz::MapvizPlugin
  {
    Q_OBJECT

   public:
    AutopilotUAVCommunicationPlugin();
    virtual ~AutopilotUAVCommunicationPlugin();

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

    bool SupportsPainting()
    {
      return true;
    }

    protected Q_SLOTS:
    void takeOffRequestCB(const std_msgs::Empty &msg);
    void landRequestCB(const std_msgs::Empty &msg);
    void AcceptTakeOff();
    void AcceptLanding();
    void flashingTakeOffRequest(const ros::TimerEvent& event);
    void flashingLandRequest(const ros::TimerEvent& event);

    private:
    Ui::autopilot_uav_communication_config ui_;
    QWidget* config_widget_;
    QPalette p_text_;
    mapviz::MapCanvas* map_canvas_;

    ros::Timer land_request_timer_, take_off_request_timer_;

    ros::Subscriber uav_land_request_sub_, uav_take_off_request_sub_;
    ros::Publisher uav_land_granted_pub_, uav_take_off_granted_pub_, pause_navigation_pub_;

    bool take_off_flashing_ = false, land_flashing_ = false, land_req_received_ = false;
    };

    }

    #endif // MAPVIZ_PLUGINS_AUTOPILOT_UAV_COMMUNICATION_PLUGIN_H_
