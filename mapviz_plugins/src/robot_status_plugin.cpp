#include <mapviz_plugins/robot_status_plugin.h>
#include <GL/glut.h>

// C++ standard libraries
#include <algorithm>
#include <cstdio>
#include <vector>

// QT libraries
#include <QDebug>
#include <QDialog>
#include <QGLWidget>

// ROS libraries
#include <ros/master.h>

#include <mapviz/select_topic_dialog.h>
#include <mapviz/select_frame_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::RobotStatusPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  RobotStatusPlugin::RobotStatusPlugin():
      config_widget_(new QWidget())
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);
    // Set robot_status text red
    QPalette p3(ui_.robot_status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.robot_status->setPalette(p3);

    heartbeat_counter_ = 0;
    listener_counter_ = 0;
    filename_ = "$(find autopilot_interface)/config/BadConnection.png";
    LoadImage();

    heartbeat_publisher_ = node_.advertise<std_msgs::Empty>("autopilot_heartbeat", 1, true);
    heartbeat_subscriber_ = node_.subscribe("robot_heartbeat", 1, &RobotStatusPlugin::RobotStatusCallback, this);
    }

  RobotStatusPlugin::~RobotStatusPlugin()
  {
  }

  void RobotStatusPlugin::RobotStatusCallback(const std_msgs::EmptyConstPtr& msg)
  {
    heartbeat_counter_++;
  }

  void RobotStatusPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.robot_status, message);
  }

  void RobotStatusPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.robot_status, message);
  }

  void RobotStatusPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.robot_status, message);
  }

  QWidget* RobotStatusPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);
    return config_widget_;
  }

  bool RobotStatusPlugin::Initialize(QGLWidget* canvas)
  {
    initialized_ = true;
    canvas_ = canvas;

    startTimer(100);

    return true;
  }

  void RobotStatusPlugin::timerEvent(QTimerEvent*)
  {
    std_msgs::Empty msg;
    heartbeat_publisher_.publish(msg);

    listener_counter_++;

    if(listener_counter_ < 10)
      return;

    // if(heartbeat_counter_ == 0)
    // {
    //   PrintError("Disconnected");
    //   filename_ = "$(find autopilot_interface)/config/NoConnection.png";
    //   LoadImage();
    // }
    if(heartbeat_counter_ <= 3)
    {
        PrintError("Bad Connection");
        filename_ = "$(find autopilot_interface)/config/BadConnection.png";
        LoadImage();
    }else if(heartbeat_counter_ >= 7)
    {
        PrintInfo("Connected");
        filename_ = "$(find autopilot_interface)/config/GoodConnection.png";
        LoadImage();
    }else
    {
        PrintInfo("Connected");
        filename_ = "$(find autopilot_interface)/config/MediumConnection.png";
        LoadImage();
    }
    listener_counter_ = heartbeat_counter_ = 0;
  }

  void RobotStatusPlugin::Shutdown()
  {
  }

  void RobotStatusPlugin::Draw(double x, double y, double scale)
  {
    if (texture_loaded_)
    {
      // Calculate the correct offsets and dimensions
      double x_offset = 30;
      double y_offset = 30;
      double width = image_.width();
      double height = image_.height();

      // Calculate the correct render position
      double x_left_pos = x_offset;
      double x_right_pos = x_offset + width;
      double y_upper_pos = canvas_->height() - height - y_offset;
      double y_bottom_pos = canvas_->height() - y_offset;

      glColor3f(1.0f, 1.0f, 1.0f);
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture_id_));

      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      glOrtho(0, canvas_->width(), canvas_->height(), 0, -0.5f, 0.5f);

      glRasterPos2d(x_left_pos, y_upper_pos);

      glBegin(GL_QUADS);

      glTexCoord2f(0, 1); glVertex2d(x_left_pos, y_upper_pos);
      glTexCoord2f(1, 1); glVertex2d(x_right_pos, y_upper_pos);
      glTexCoord2f(1, 0); glVertex2d(x_right_pos, y_bottom_pos);
      glTexCoord2f(0, 0); glVertex2d(x_left_pos, y_bottom_pos);

      glEnd();

      glPopMatrix();

      glDisable(GL_TEXTURE_2D);
    }
  }

  void RobotStatusPlugin::LoadImage()
  {
    try
    {
      QImage nullImage;
      image_ = nullImage;

      const std::string prefix = "$(find ";
      std::string real_filename;
      size_t spos = filename_.find(prefix);
      bool has_close = spos != -1 ? filename_.find(')', spos) != -1: false;
      if (spos != -1 && spos + prefix.length() < filename_.size() && has_close)
      {
        std::string package = filename_.substr(spos + prefix.length());
        package = package.substr(0, package.find(")"));

        real_filename = ros::package::getPath(package) + filename_.substr(filename_.find(')')+1);
      }
      else
      {
        real_filename = filename_;
      }


      if (image_.load(real_filename.c_str()))
      {
        int width = image_.width();
        int height = image_.height();

        float max_dim = std::max(width, height);
        dimension_ = static_cast<int>(std::pow(2, std::ceil(std::log(max_dim) / std::log(2.0f))));

        if (width != dimension_ || height != dimension_)
        {
          image_ = image_.scaled(dimension_, dimension_, Qt::IgnoreAspectRatio, Qt::FastTransformation);
        }

        image_ = QGLWidget::convertToGLFormat(image_);

        GLuint ids[1];
        glGenTextures(1, &ids[0]);
        texture_id_ = ids[0];

        glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture_id_));
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dimension_, dimension_, 1, GL_RGBA, GL_UNSIGNED_BYTE, image_.bits());

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        texture_loaded_ = true;
      }
    }
    catch (const std::exception& e)
    {
      PrintError("Failed to load image.  Exception occured.");
    }
  }

  void RobotStatusPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
  }

  void RobotStatusPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
  }
}
