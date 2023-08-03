// *****************************************************************************
//
// Copyright (c) 2016, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/fiqugs/autopilot_fiqugs_area_exploration_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QGLWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPalette>

#include <opencv2/core/core.hpp>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/master.h>
#include <mapviz/select_frame_dialog.h>
#include <swri_transform_util/frames.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::AutopilotFiqugsAreaExplorationPlugin, mapviz::MapvizPlugin)

namespace stu = swri_transform_util;

namespace mapviz_plugins
{
  AutopilotFiqugsAreaExplorationPlugin::AutopilotFiqugsAreaExplorationPlugin() :
    config_widget_(new QWidget()),
    map_canvas_(NULL),
    selected_point_(-1),
    is_mouse_down_(false),
    creating_polygon_(false),
    max_ms_(Q_INT64_C(500)),
    max_distance_(2.0)
  {
    ui_.setupUi(config_widget_);

    ui_.color->setColor(Qt::green);
    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);
    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.selectframe, SIGNAL(clicked()), this,
                     SLOT(SelectFrame()));
    QObject::connect(ui_.frame, SIGNAL(editingFinished()), this,
                     SLOT(FrameEdited()));
    QObject::connect(ui_.publish, SIGNAL(clicked()), this,
                     SLOT(PublishPolygon()));
    QObject::connect(ui_.clear, SIGNAL(clicked()), this,
                     SLOT(Clear()));
    QObject::connect(ui_.createPolygon, SIGNAL(clicked()), this,
                     SLOT(CreatePolygon()));
    QObject::connect(ui_.sendButton, SIGNAL(clicked()), this,
                     SLOT(Send()));
    QObject::connect(ui_.cancelButton, SIGNAL(clicked()), this,
                     SLOT(Cancel()));

    cartesian_area_namespace_ = "autopilot_cartesian_area_navigator/cartesian_area_action";
    cartesian_area_ac_.reset(
       new actionlib::SimpleActionClient<autopilot_msgs::CartesianAreaAction>(node_, cartesian_area_namespace_, true));

    accept_mission_pub_ = node_.advertise<std_msgs::Empty>("accept_goal", 1, true);
    cancel_pub_ = node_.advertise<std_msgs::Empty>("cancel", 1, true);
  }

  AutopilotFiqugsAreaExplorationPlugin::~AutopilotFiqugsAreaExplorationPlugin()
  {
    if (map_canvas_)
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  void AutopilotFiqugsAreaExplorationPlugin::SelectFrame()
  {
    std::string frame = mapviz::SelectFrameDialog::selectFrame(tf_);
    if (!frame.empty())
    {
      ui_.frame->setText(QString::fromStdString(frame));
      FrameEdited();
    }
  }

  void AutopilotFiqugsAreaExplorationPlugin::FrameEdited()
  {
    source_frame_ = ui_.frame->text().toStdString();
    PrintWarning("Waiting for transform.");

    ROS_INFO("Setting target frame to to %s", source_frame_.c_str());

    initialized_ = true;
  }

  void AutopilotFiqugsAreaExplorationPlugin::PublishPolygon()
  {
    autopilot_msgs::CartesianArea area;
    area.target.header.stamp = ros::Time::now();
    area.target.header.frame_id = ui_.frame->text().toStdString();

    for (const auto& vertex: vertices_)
    {
      geometry_msgs::Point32 point;
      point.x = vertex.x();
      point.y = vertex.y();
      point.z = 0;
      area.target.polygon.points.push_back(point);
    }

    if(cartesian_area_ac_->isServerConnected())
    cartesian_area_ac_->sendGoal(area);

    creating_polygon_ = false;
    ui_.createPolygon->setEnabled(true);
    ui_.clear->setEnabled(false);
  }

  void AutopilotFiqugsAreaExplorationPlugin::Clear()
  {
    vertices_.clear();
    transformed_vertices_.clear();
    creating_polygon_ = false;
    ui_.createPolygon->setEnabled(true);
    ui_.clear->setEnabled(false);
    ui_.publish->setEnabled(false);
  }

  void AutopilotFiqugsAreaExplorationPlugin::CreatePolygon()
  {
    creating_polygon_ = true;
    ui_.createPolygon->setEnabled(false);
    ui_.clear->setEnabled(true);
  }

  void AutopilotFiqugsAreaExplorationPlugin::Cancel()
  {
    std_msgs::Empty msg;
    cancel_pub_.publish(msg);
    return;
  }

  void AutopilotFiqugsAreaExplorationPlugin::Send()
  {
    std_msgs::Empty msg;
    accept_mission_pub_.publish(msg);
    return;
  }

  void AutopilotFiqugsAreaExplorationPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message, 1.0);
  }

  void AutopilotFiqugsAreaExplorationPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message, 1.0);
  }

  void AutopilotFiqugsAreaExplorationPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message, 1.0);
  }

  QWidget* AutopilotFiqugsAreaExplorationPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool AutopilotFiqugsAreaExplorationPlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = static_cast<mapviz::MapCanvas*>(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;
    return true;
  }

  bool AutopilotFiqugsAreaExplorationPlugin::eventFilter(QObject *object, QEvent* event)
  {
    switch (event->type())
    {
      case QEvent::MouseButtonPress:
        return handleMousePress(static_cast<QMouseEvent*>(event));
      case QEvent::MouseButtonRelease:
        return handleMouseRelease(static_cast<QMouseEvent*>(event));
      case QEvent::MouseMove:
        return handleMouseMove(static_cast<QMouseEvent*>(event));
      default:
        return false;
    }
  }

  bool AutopilotFiqugsAreaExplorationPlugin::handleMousePress(QMouseEvent* event)
  {
    if(!this->Visible() || !creating_polygon_)
    {
      ROS_DEBUG("Ignoring mouse press, since draw polygon plugin is hidden or you are creating a polygon");
      return false;
    }

    selected_point_ = -1;
    int closest_point = 0;
    double closest_distance = std::numeric_limits<double>::max();

#if QT_VERSION >= 0x050000
    QPointF point = event->localPos();
#else
    QPointF point = event->posF();
#endif
    stu::Transform transform;
    std::string frame = ui_.frame->text().toStdString();
    if (tf_manager_->GetTransform(target_frame_, frame, transform))
    {
      for (size_t i = 0; i < vertices_.size(); i++)
      {
        tf::Vector3 vertex = vertices_[i];
        vertex = transform * vertex;

        QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(QPointF(vertex.x(), vertex.y()));

        double distance = QLineF(transformed, point).length();

        if (distance < closest_distance)
        {
          closest_distance = distance;
          closest_point = static_cast<int>(i);
        }
      }
    }

    if (event->button() == Qt::LeftButton)
    {
      if (closest_distance < 15)
      {
        selected_point_ = closest_point;
        return true;
      }
      else
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
    }
    else if (event->button() == Qt::RightButton)
    {
      if (closest_distance < 15)
      {
        vertices_.erase(vertices_.begin() + closest_point);
        transformed_vertices_.resize(vertices_.size());
        return true;
      }
    }

    return false;
  }

  bool AutopilotFiqugsAreaExplorationPlugin::handleMouseRelease(QMouseEvent* event)
  {
    std::string frame = ui_.frame->text().toStdString();
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
    {
#if QT_VERSION >= 0x050000
      QPointF point = event->localPos();
#else
      QPointF point = event->posF();
#endif
      stu::Transform transform;
      if (tf_manager_->GetTransform(frame, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        vertices_[selected_point_].setX(position.x());
        vertices_[selected_point_].setY(position.y());
      }

      selected_point_ = -1;
      return true;
    }
    else if (is_mouse_down_)
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
        ROS_INFO("mouse point at %f, %f -> %f, %f", point.x(), point.y(), transformed.x(), transformed.y());

        stu::Transform transform;
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);

        if (tf_manager_->GetTransform(frame, target_frame_, transform))
        {
          position = transform * position;
          vertices_.push_back(position);
          transformed_vertices_.resize(vertices_.size());
          ROS_INFO("Adding vertex at %lf, %lf %s", position.x(), position.y(), frame.c_str());
        }
      }
    }
    is_mouse_down_ = false;

    return false;
  }

  bool AutopilotFiqugsAreaExplorationPlugin::handleMouseMove(QMouseEvent* event)
  {
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
    {
#if QT_VERSION >= 0x050000
      QPointF point = event->localPos();
#else
      QPointF point = event->posF();
#endif
      stu::Transform transform;
      std::string frame = ui_.frame->text().toStdString();
      if (tf_manager_->GetTransform(frame, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        vertices_[selected_point_].setY(position.y());
        vertices_[selected_point_].setX(position.x());
      }

      return true;
    }
    return false;
  }

  void AutopilotFiqugsAreaExplorationPlugin::Draw(double x, double y, double scale)
  {
    stu::Transform transform;
    std::string frame = ui_.frame->text().toStdString();
    if (!tf_manager_->GetTransform(target_frame_, frame, transform))
    {
      return;
    }

    // Transform polygon
    for (size_t i = 0; i < vertices_.size(); i++)
    {
      transformed_vertices_[i] = transform * vertices_[i];
    }

    glLineWidth(1);
    const QColor color = ui_.color->color();
    glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
    glBegin(GL_LINE_STRIP);

    for (const auto& vertex: transformed_vertices_)
    {
      glVertex2d(vertex.x(), vertex.y());
    }

    glEnd();

    glBegin(GL_LINES);

    glColor4d(color.redF(), color.greenF(), color.blueF(), 0.25);

    if (transformed_vertices_.size() > 2)
    {
      glVertex2d(transformed_vertices_.front().x(), transformed_vertices_.front().y());
      glVertex2d(transformed_vertices_.back().x(), transformed_vertices_.back().y());
      ui_.publish->setEnabled(true);
    }

    glEnd();

    // Draw vertices
    glPointSize(9);
    glBegin(GL_POINTS);

    for (const auto& vertex: transformed_vertices_)
    {
      glVertex2d(vertex.x(), vertex.y());
    }
    glEnd();

    PrintInfo("OK");
  }

  void AutopilotFiqugsAreaExplorationPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["frame"])
    {
      node["frame"] >> source_frame_;
      ui_.frame->setText(source_frame_.c_str());
    }

    if (node["polygon_topic"])
    {
      std::string polygon_topic;
      node["polygon_topic"] >> polygon_topic;
      ui_.topic->setText(polygon_topic.c_str());
    }
    if (node["color"])
    {
      std::string color;
      node["color"] >> color;
      ui_.color->setColor(QColor(color.c_str()));
    }
  }

  void AutopilotFiqugsAreaExplorationPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string frame = ui_.frame->text().toStdString();
    emitter << YAML::Key << "frame" << YAML::Value << frame;

    std::string polygon_topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "polygon_topic" << YAML::Value << polygon_topic;

    std::string color = ui_.color->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;
  }
}
