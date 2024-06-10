#include <mapviz_plugins/autopilot_waypoints_plugin.h>
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

#include <mapviz/select_topic_dialog.h>
#include <mapviz/select_frame_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::AutopilotWaypointsPlugin, mapviz::MapvizPlugin)

bool mouse_pressed = false;

namespace mapviz_plugins
{
  AutopilotWaypointsPlugin::AutopilotWaypointsPlugin():
    config_widget_(new QWidget()),
    selected_point_(-1),
    index_(-1),
    is_mouse_down_(false),
    add_point_flag_(false),
    remove_point_flag_(false),
    max_ms_(Q_INT64_C(500)),
    max_distance_(2.0),
    map_canvas_(NULL)
  {
    modify_waypoints_client_ = node_.serviceClient<autopilot_msgs::ModifyGPSWaypoint>("autopilot_gps_waypoints_navigator/modify_waypoint");
    add_waypoint_client_ = node_.serviceClient<autopilot_msgs::AddGPSWaypoint>("autopilot_gps_waypoints_navigator/add_waypoint");

    ui_.setupUi(config_widget_);
    ui_.path_color->setColor(Qt::green);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.cancel, SIGNAL(clicked()), this, SLOT(Cancel()));
    QObject::connect(ui_.send_goal, SIGNAL(clicked()), this, SLOT(SendGoal()));
    QObject::connect(ui_.add_points, SIGNAL(clicked()), this, SLOT(AddPoint()));
    QObject::connect(ui_.remove_point, SIGNAL(clicked()), this, SLOT(RemovePoint()));
    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
    QObject::connect(ui_.path_color, SIGNAL(colorEdited(const QColor&)), this, SLOT(SetColor(const QColor&)));
    QObject::connect(ui_.alpha, SIGNAL(valueChanged(double)), this, SLOT(AlphaChanged(double)));

    gps_goal_namespace_ = "autopilot_gps_goal_navigator/gps_goal_action";
    gps_goal_ac_.reset(
        new actionlib::SimpleActionClient<autopilot_msgs::GPSGoalAction>(node_, gps_goal_namespace_, true));

    // goal_sub_ = node_.subscribe("autopilot_gps_goal_navigator/gps_goal", 1, &AutopilotWaypointsPlugin::goalCallback, this);

    accept_mission_pub_ = node_.advertise<std_msgs::Empty>("waypoints/accept_goal", 1, true);
    cancel_pub_ = node_.advertise<std_msgs::Empty>("waypoints/cancel", 1, true);
  }

  AutopilotWaypointsPlugin::~AutopilotWaypointsPlugin()
  {
    if (map_canvas_)
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  void AutopilotWaypointsPlugin::Clear()
  {
    vertices_.clear();
  }

  void AutopilotWaypointsPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic =
        mapviz::SelectTopicDialog::selectTopic("autopilot_msgs/AutopilotGeoPath");

    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void AutopilotWaypointsPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      Clear();
      has_message_ = false;
      PrintWarning("No messages received.");

      waypoints_sub_.shutdown();

      topic_ = topic;
      if (!topic.empty())
      {
        waypoints_sub_ = node_.subscribe(topic_, 1, &AutopilotWaypointsPlugin::waypointsCallback, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void AutopilotWaypointsPlugin::waypointsCallback(const autopilot_msgs::AutopilotGeoPathConstPtr& waypoints)
  {
    if (!tf_manager_->LocalXyUtil()->Initialized())
    {
      return;
    }

    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    if (is_mouse_down_)
      return;

    if(index_ >= 0)
      if(waypoints->poses[index_].pose.pose.position != new_coords_)
      {
        index_ = -1;
        return;
      }

    Clear();

    waypoints_array_ = waypoints->poses;

    for (unsigned int i = 0; i < waypoints->poses.size(); i++)
    {
      double x;
      double y;
      tf_manager_->LocalXyUtil()->ToLocalXy(waypoints->poses[i].pose.pose.position.latitude, waypoints->poses[i].pose.pose.position.longitude, x, y);
      tf::Vector3 position(x, y, 0.0);
      vertices_.push_back(position);
    }
    return;
  }

  void AutopilotWaypointsPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void AutopilotWaypointsPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void AutopilotWaypointsPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* AutopilotWaypointsPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool AutopilotWaypointsPlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = static_cast< mapviz::MapCanvas* >(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;

    return true;
  }

  bool AutopilotWaypointsPlugin::eventFilter(QObject* object, QEvent* event)
  {
    if(!this->Visible())
    {
      ROS_DEBUG("Ignoring mouse event, since measuring plugin is hidden");
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

  bool AutopilotWaypointsPlugin::handleMousePress(QMouseEvent* event)
  {
    selected_point_ = -1;
    int closest_point = 0;
    double closest_distance = std::numeric_limits<double>::max();
  #if QT_VERSION >= 0x050000
    QPointF point = event->localPos();
  #else
    QPointF point = event->posF();
  #endif
    ROS_DEBUG("Map point: %f %f", point.x(), point.y());
    for (size_t i = 0; i < vertices_.size(); i++)
    {
      tf::Vector3 vertex = vertices_[i];
      QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(QPointF(vertex.x(), vertex.y()));

      double distance = QLineF(transformed, point).length();

      if (distance < closest_distance)
      {
        closest_distance = distance;
        closest_point = static_cast<int>(i);
      }
    }
    if (event->button() == Qt::LeftButton)
    {
      is_mouse_down_ = true;
      #if QT_VERSION >= 0x050000
      mouse_down_pos_ = event->localPos();
      #else
      mouse_down_pos_ = event->posF();
      #endif
      mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
      if (closest_distance < 15)
      {
        selected_point_ = closest_point;
        return true;
      }
      // else
      // {
        return false;
      // }
    }
    else if (event->button() == Qt::RightButton)
    {
      if (add_point_flag_ && closest_distance < 15)
      {
        autopilot_msgs::ModifyGPSWaypoint srv;
        srv.request.remove = false;
        srv.request.add = true;
        srv.request.place = "before";
        srv.request.index = closest_point;
        if(!modify_waypoints_client_.call(srv))
        {
          ROS_ERROR("Failed to call service ModifyGPSWaypoint");
          return false;
        }
        return true;
      }
    }
    // Let other plugins process this event too
    return false;
  }

  bool AutopilotWaypointsPlugin::handleMouseRelease(QMouseEvent* event)
  {
    autopilot_msgs::ModifyGPSWaypoint srv;
    autopilot_msgs::GPSGoalGoal goal;
    double latitude;
    double longitude;

    if (is_mouse_down_)
    {
      #if QT_VERSION >= 0x050000
          qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
      #else
          qreal distance = QLineF(mouse_down_pos_, event->posF()).length();
      #endif

      qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

      #if QT_VERSION >= 0x050000
          QPointF point = event->localPos();
      #else
          QPointF point = event->posF();
      #endif
          QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);

      tf::Vector3 position(transformed.x(), transformed.y(), 0.0);

      if (!tf_manager_->LocalXyUtil()->Initialized())
      {
        return false;
      }

      tf_manager_->LocalXyUtil()->ToWgs84(position.x(), position.y(), latitude, longitude);

      if(selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
      {

        if (msecsDiff < max_ms_ && distance <= max_distance_)
        {
          if(add_point_flag_)
          {
            srv.request.remove = false;
            srv.request.add = true;
            srv.request.place = "after";
            srv.request.index = selected_point_;

            if(!modify_waypoints_client_.call(srv))
            {
              ROS_ERROR("Failed to call service ModifyGPSWaypoint");
              return false;
            }
          }
          else if(remove_point_flag_)
          {
            srv.request.remove = true;
            srv.request.add = false;
            srv.request.index = selected_point_;

            if(!modify_waypoints_client_.call(srv))
            {
              ROS_ERROR("Failed to call service ModifyGPSWaypoint");
              return false;
            }
          }
        }
        else
        {
          srv.request.remove = false;
          srv.request.add = false;
          srv.request.index = selected_point_;
          srv.request.new_pose.position.latitude = latitude;
          srv.request.new_pose.position.longitude = longitude;

          index_ = selected_point_;
          new_coords_.latitude = latitude;
          new_coords_.longitude = longitude;

          if(!modify_waypoints_client_.call(srv))
          {
            ROS_ERROR("Failed to call service ModifyGPSWaypoint");
            return false;
          }

          is_mouse_down_ = false;
          selected_point_ = -1;
          return true;
        }
      }
      else
      {
        if (msecsDiff < max_ms_ && distance <= max_distance_)
        {
          if(remove_point_flag_)
            return false;
          else if(add_point_flag_)
          {
            srv.request.remove = false;
            srv.request.add = true;
            srv.request.place = "after";
            srv.request.index = vertices_.size()-1;
            srv.request.new_pose.position.latitude = latitude;
            srv.request.new_pose.position.longitude = longitude;

            if(!modify_waypoints_client_.call(srv))
            {
              ROS_ERROR("Failed to call service ModifyGPSWaypoint");
              return false;
            }
          }
          else
          {
            if (!tf_manager_->LocalXyUtil()->Initialized())
            {
              return false;
            }
            goal.target.latitude = latitude;
            goal.target.longitude = longitude;
            goal.start_from_previous_point = false;

            if(vertices_.size() > 1)
            {
              tf::Vector3 position(vertices_.back().x(), vertices_.back().y(), 0.0);

              tf_manager_->LocalXyUtil()->ToWgs84(position.x(), position.y(), latitude, longitude);

              goal.start_from_previous_point = true;
              goal.previous_point.latitude = latitude;
              goal.previous_point.longitude = longitude;
            }

            if(gps_goal_ac_->isServerConnected())
            gps_goal_ac_->sendGoal(goal);
          }
        }
      }
    }
    selected_point_ = -1;
    is_mouse_down_ = false;
    // Let other plugins process this event too
    return false;
  }

  bool AutopilotWaypointsPlugin::handleMouseMove(QMouseEvent* event)
  {
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
    {
  #if QT_VERSION >= 0x050000
      QPointF point = event->localPos();
  #else
      QPointF point = event->posF();
  #endif
      std::string frame = target_frame_;
      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
      tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
      vertices_[selected_point_].setY(position.y());
      vertices_[selected_point_].setX(position.x());
      return true;
    }// Let other plugins process this event too
    return false;
  }

  void AutopilotWaypointsPlugin::AddPoint()
  {
    if(add_point_flag_==false)
    {
      add_point_flag_ = true;
      remove_point_flag_ = false;

      p_addpoint_ = config_widget_->palette();
      p_addpoint_.setColor(QPalette::Button, QColor(Qt::green));
      ui_.add_points->setAutoFillBackground(true);
      ui_.add_points->setPalette(p_addpoint_);
      ui_.add_points->update();

      p_remove_ = config_widget_->palette();
      p_remove_.setColor(QPalette::Button, QColor(Qt::lightGray).lighter(125));
      ui_.remove_point->setAutoFillBackground(true);
      ui_.remove_point->setPalette(p_remove_);
      ui_.remove_point->update();
    }
    else
    {
      add_point_flag_ = false;

      p_addpoint_ = config_widget_->palette();
      p_addpoint_.setColor(QPalette::Button, QColor(Qt::lightGray).lighter(125));
      ui_.add_points->setAutoFillBackground(true);
      ui_.add_points->setPalette(p_addpoint_);
      ui_.add_points->update();
    }

    return;
  }

  void AutopilotWaypointsPlugin::RemovePoint()
  {
    if(remove_point_flag_==false)
    {
      remove_point_flag_ = true;
      add_point_flag_ = false;

      p_remove_ = config_widget_->palette();
      p_remove_.setColor(QPalette::Button, QColor(Qt::green));
      ui_.remove_point->setAutoFillBackground(true);
      ui_.remove_point->setPalette(p_remove_);
      ui_.remove_point->update();

      p_addpoint_ = config_widget_->palette();
      p_addpoint_.setColor(QPalette::Button, QColor(Qt::lightGray).lighter(125));
      ui_.add_points->setAutoFillBackground(true);
      ui_.add_points->setPalette(p_addpoint_);
      ui_.add_points->update();
    }
    else
    {
      remove_point_flag_ = false;

      p_remove_ = config_widget_->palette();
      p_remove_.setColor(QPalette::Button, QColor(Qt::lightGray).lighter(125));
      ui_.remove_point->setAutoFillBackground(true);
      ui_.remove_point->setPalette(p_remove_);
      ui_.remove_point->update();
    }

    return;
  }

  void AutopilotWaypointsPlugin::Cancel()
  {
    std_msgs::Empty msg;
    cancel_pub_.publish(msg);
    Clear();
    map_canvas_->installEventFilter(this);
    return;
  }

  void AutopilotWaypointsPlugin::SendGoal()
  {
    if(vertices_.size() > 0)
    {
      std_msgs::Empty msg;
      accept_mission_pub_.publish(msg);
      map_canvas_->removeEventFilter(this);
    }
    return;
  }

  void AutopilotWaypointsPlugin::Draw(double x, double y, double scale)
  {
    int iterator = 0;
    glLineWidth(10);
    QColor color = ui_.path_color->color();
    // glColor4d(color.redF(), color.greenF(), color.blueF(), ui_.alpha->value()/2.0);
    glBegin(GL_LINE_STRIP);

    for (const auto& vertex: vertices_)
    {
      if(waypoints_array_[iterator].visited == true)
        color = QColor(Qt::green);
      else
        color = ui_.path_color->color();

      glColor4d(color.redF(), color.greenF(), color.blueF(), ui_.alpha->value()/2.0);

      glVertex2d(vertex.x(), vertex.y());

      iterator++;
    }

    glEnd();

    glBegin(GL_LINES);

    // glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);

    glEnd();

    iterator = 0;

    // Draw vertices
    glPointSize(20);
    glBegin(GL_POINTS);

    for (const auto& vertex: vertices_)
    {
      if(waypoints_array_[iterator].visited == true)
        color = QColor(Qt::green);
      else
        color = ui_.path_color->color();

      glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);

      glVertex2d(vertex.x(), vertex.y());

      iterator++;
    }
    glEnd();

    PrintInfo("OK");
  }

  void AutopilotWaypointsPlugin::Paint(QPainter* painter, double x, double y, double scale)
  {
    //set the draw color for the text to be the same as the rest
    QColor color = ui_.path_color->color();
    double alpha = ui_.alpha->value()*2.0 < 1.0 ? ui_.alpha->value()*2.0 : 1.0;
    color.setAlphaF(alpha);
    QPen pen(QBrush(color), 1);
    painter->setPen(pen);
  }

  void AutopilotWaypointsPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (swri_yaml_util::FindValue(node, "topic"))
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(topic.c_str());
      TopicEdited();
    }

    if (swri_yaml_util::FindValue(node, "color"))
    {
      std::string color;
      node["color"] >> color;
      QColor qcolor(color.c_str());
      // SetColor(qcolor);
      ui_.path_color->setColor(qcolor);
    }

    if (node["alpha"])
    {
      double alpha;
      node["alpha"] >> alpha;
      ui_.alpha->setValue(alpha);
      AlphaChanged(alpha);
    }
  }

  void AutopilotWaypointsPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;

    std::string color = ui_.path_color->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;

    emitter << YAML::Key << "alpha" << YAML::Value << ui_.alpha->value();
  }
}
