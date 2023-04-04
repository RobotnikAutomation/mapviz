// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************
#include <iostream>
#include <locale>
#include <string.h>

#include <mapviz_plugins/send_gps_goal_plugin.h>
#include <mapviz/mapviz_plugin.h>

#include <QClipboard>
#include <QDateTime>
#include <QMouseEvent>
#include <QTextStream>

#if QT_VERSION >= 0x050000
#include <QGuiApplication>
#else
#include <QApplication>
#endif

// ROS Libraries
#include <ros/ros.h>

// Mapviz Libraries
#include <mapviz/select_frame_dialog.h>

//
#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform.h>
#include <swri_transform_util/transform_util.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::SendGPSGoalPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

SendGPSGoalPlugin::SendGPSGoalPlugin()
  : config_widget_(new QWidget()),
  map_canvas_(NULL),
  is_mouse_down_(false),
  max_ms_(Q_INT64_C(500)),
  max_distance_(2.0)
{
  ui_.setupUi(config_widget_);
  ui_.goal_color->setColor(Qt::green);

  QObject::connect(ui_.cancel, SIGNAL(clicked()), this,
                   SLOT(Cancel()));
  QObject::connect(ui_.send_goal, SIGNAL(clicked()),
                   this, SLOT(SendGoal()));
  QObject::connect(ui_.goal_color, SIGNAL(colorEdited(const QColor&)),
                   this, SLOT(SetColor(const QColor&)));
  gps_goal_namespace_ = "autopilot_gps_goal_navigator/gps_goal_action";
  gps_goal_ac_.reset(
      new actionlib::SimpleActionClient<autopilot_msgs::GPSGoalAction>(node_, gps_goal_namespace_, true));

  goal_sub_ = node_.subscribe("autopilot_gps_goal_navigator/gps_goal", 1, &SendGPSGoalPlugin::goalCallback, this);

  accept_mission_pub_ = node_.advertise<std_msgs::Empty>("accept_goal", 1, true);
  cancel_pub_ = node_.advertise<std_msgs::Empty>("cancel", 1, true);
}

SendGPSGoalPlugin::~SendGPSGoalPlugin()
{
  if (map_canvas_)
  {
    map_canvas_->removeEventFilter(this);
  }
}

QWidget* SendGPSGoalPlugin::GetConfigWidget(QWidget* parent)
{
  config_widget_->setParent(parent);

  return config_widget_;
}

bool SendGPSGoalPlugin::Initialize(QGLWidget* canvas)
{
  map_canvas_ = static_cast< mapviz::MapCanvas* >(canvas);
  map_canvas_->installEventFilter(this);

  initialized_ = true;
  PrintInfo("OK");

  return true;
}

bool SendGPSGoalPlugin::eventFilter(QObject* object, QEvent* event)
{
  if(!this->Visible())
  {
    ROS_DEBUG("Ignoring mouse event, since coordinate picker plugin is hidden");
    return false;
  }

  switch (event->type())
  {
    case QEvent::MouseButtonPress:
      return handleMousePress(static_cast< QMouseEvent* >(event));
    case QEvent::MouseButtonRelease:
      return handleMouseRelease(static_cast< QMouseEvent* >(event));
    case QEvent::MouseMove:
      return handleMouseMove(static_cast< QMouseEvent* >(event));
    default:
      return false;
  }
}

bool SendGPSGoalPlugin::handleMousePress(QMouseEvent* event)
{
  if (event->button() == Qt::LeftButton)
  {
    is_mouse_down_ = true;
#if QT_VERSION >= 0x050000
    mouse_down_pos_ = event->localPos();
#else
    mouse_down_pos_ = event->posF();
#endif
    mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
    return false;
  }
  // Let other plugins process this event too
  return false;
}

bool SendGPSGoalPlugin::handleMouseRelease(QMouseEvent* event)
{
  if (is_mouse_down_)
  {
    #if QT_VERSION >= 0x050000
        qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
    #else
        qreal distance = QLineF(mouse_down_pos_, event->posF()).length();
    #endif
    qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

    // Only fire the event if the mouse has moved less than the maximum distance
    // and was held for shorter than the maximum time..  This prevents click
    // events from being fired if the user is dragging the mouse across the map
    // or just holding the cursor in place.
    if (msecsDiff < max_ms_ && distance <= max_distance_)
    {
#if QT_VERSION >= 0x050000
      QPointF point = event->localPos();
#else
      QPointF point = event->posF();
#endif

      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
      tf::Vector3 position(transformed.x(), transformed.y(), 0.0);

      vertices_.clear();
      vertices_.push_back(position);

      if (!tf_manager_->LocalXyUtil()->Initialized())
      {
        return false;
      }
      autopilot_msgs::GPSGoalGoal goal;
      double latitude;
      double longitude;

      tf_manager_->LocalXyUtil()->ToWgs84(position.x(), position.y(), latitude, longitude);

      goal.target.latitude = latitude;
      goal.target.longitude = longitude;

      std::string str_position = std::to_string(latitude) + " " + std::to_string(longitude);
      ui_.gps_point->setText(QString::fromStdString(str_position));

      if(gps_goal_ac_->isServerConnected())
      gps_goal_ac_->sendGoal(goal);
    }
  }
  is_mouse_down_ = false;
  // Let other plugins process this event too
  return false;
}

bool SendGPSGoalPlugin::handleMouseMove(QMouseEvent* event)
{
  // Let other plugins process this event too
  return false;
}

void SendGPSGoalPlugin::goalCallback(const geographic_msgs::GeoPoint& msg)
{
  std::string str_position = std::to_string(msg.latitude) + " " + std::to_string(msg.longitude);
  ui_.gps_point->setText(QString::fromStdString(str_position));

  if (!tf_manager_->LocalXyUtil()->Initialized())
  {
    return;
  }

  vertices_.clear();

  double x;
  double y;
  tf_manager_->LocalXyUtil()->ToLocalXy(msg.latitude, msg.longitude, x, y);
  tf::Vector3 position(x, y, 0.0);
  vertices_.push_back(position);

  return;
}

void SendGPSGoalPlugin::Cancel()
{
  std_msgs::Empty msg;
  vertices_.clear();
  ui_.gps_point->clear();
  cancel_pub_.publish(msg);
  map_canvas_->installEventFilter(this);
  return;
}

void SendGPSGoalPlugin::SendGoal()
{
  if(!ui_.gps_point->text().isEmpty())
  {
    std_msgs::Empty msg;
    accept_mission_pub_.publish(msg);
    map_canvas_->removeEventFilter(this);
  }
  return;
}

void SendGPSGoalPlugin::Draw(double x, double y, double scale)
{
  const QColor color = ui_.goal_color->color();

  glBegin(GL_LINES);

  glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);

  glEnd();

  // Draw vertices
  glPointSize(20);
  glBegin(GL_POINTS);

  for (const auto& vertex: vertices_)
  {
    glVertex2d(vertex.x(), vertex.y());
  }
  glEnd();

  PrintInfo("OK");
}

void SendGPSGoalPlugin::Paint(QPainter* painter, double x, double y, double scale)
{
  //set the draw color for the text to be the same as the rest
  QColor color = ui_.goal_color->color();
  QPen pen(QBrush(color), 1);
  painter->setPen(pen);
}

void SendGPSGoalPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{
}

void SendGPSGoalPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{
}

void SendGPSGoalPlugin::PrintError(const std::string& message)
{
  PrintErrorHelper(ui_.status, message, 1.0);
}

void SendGPSGoalPlugin::PrintInfo(const std::string& message)
{
  PrintInfoHelper(ui_.status, message, 1.0);
}

void SendGPSGoalPlugin::PrintWarning(const std::string& message)
{
  PrintWarningHelper(ui_.status, message, 1.0);
}

} // namespace mapviz_plugins
