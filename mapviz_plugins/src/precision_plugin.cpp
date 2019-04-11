#include <mapviz_plugins/precision_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <algorithm>
#include <vector>
#include <list>
#include <string>

// QT libraries
#include <QGLWidget>
#include <QPalette>
#include <QImage>
#include <QFileDialog>

// ROS libraries
#include <ros/master.h>

#include <swri_image_util/geometry_util.h>
#include <swri_geometry_util/geometry_util.h>

#include <mapviz/select_topic_dialog.h>

#include "/usr/local/include/gdal.h"
// #include <gdal/gdal.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::PrecisionPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  // PUBLIC
  PrecisionPlugin::PrecisionPlugin() :
    config_widget_(new QWidget())
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.browse, SIGNAL(clicked()), this,
                     SLOT(SelectFile()));
    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this,
                     SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this,
                     SLOT(TopicEdited()));

    state_pub_ = nh_.advertise<std_msgs::String>("zone", 100);
  }

  PrecisionPlugin::~PrecisionPlugin()
  {
  }

  bool PrecisionPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    return true;
  }

  void PrecisionPlugin::Draw(double x, double y, double scale) {}
  void PrecisionPlugin::Transform() {}

  QWidget* PrecisionPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  void PrecisionPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["file"])
    {
      node["file"] >> shapefile_;
      ui_.file->setText(shapefile_.c_str());
    }

    if (node["topic"])
    {
      std::string topic;
      node["topic"] >> topic_;
      ui_.topic->setText(topic_.c_str());
    }

    FileEdited();
    TopicEdited();
  }

  void PrecisionPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "file" << YAML::Value << shapefile_;
    emitter << YAML::Key << "topic" << YAML::Value << topic_;
  }

  // PROTECTED
  void PrecisionPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void PrecisionPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void PrecisionPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  // PROTECTED Q_SLOTS
  void PrecisionPlugin::SelectFile()
  {
    QFileDialog dialog(config_widget_, "Select Shapefile");
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilter(tr("Shapefiles (*.shp)"));

    dialog.exec();

    if (dialog.result() == QDialog::Accepted && dialog.selectedFiles().count() == 1)
    {
      ui_.file->setText(dialog.selectedFiles().first());
      shapefile_ = dialog.selectedFiles().first().toStdString();
      FileEdited();
    }
  }

  void PrecisionPlugin::FileEdited()
  {
    // Open shapefile
    GDALAllRegister();
    GDALDataset *poDS;
    OGRLayer *poLayer;
    OGRFeature *poFeature;

    poDS = (GDALDataset*) GDALOpenEx(shapefile_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( poDS == NULL )
    {
      ROS_ERROR("Failed to open shapefile.");
      return;
    } else {
      ROS_INFO("Successfully opened shapefile.");
    }

    // Isolate polygons with designation
    poLayer = poDS->GetLayer(0);

    poLayer->ResetReading();
    while( (poFeature = poLayer->GetNextFeature()) != NULL )
    {
      std::string name;
      std::string polygon;

      if(poFeature->IsFieldSet(10))
      {
        name = poFeature->GetFieldAsString(2);
        if(name.size() == 0)
        {
          name = poFeature->GetFieldAsString(3);
        }
        polygon = poFeature->GetFieldAsString(10);
        polygon = polygon.substr(10, polygon.size()-12);
      }

      if(polygon.size())
      {
        Polygon temp;

        temp.name = name;
        temp.polygon = StringToPolygon(polygon);

        zones_.push_back(temp);
      }
    }

    ROS_INFO("Number of polygons saved: %d", zones_.size());
  }

  std::vector<cv::Vec2d> PrecisionPlugin::StringToPolygon(std::string polygon)
  {

    std::vector<cv::Vec2d> poly;
    std::vector<std::string> points;
    cv::Vec2d point;

    std::string comma = ",";
    std::string space = " ";
    std::string x_str;
    std::string y_str;

    size_t pos = 0;
    while ((pos = polygon.find(comma)) != std::string::npos)
    {
      points.push_back(polygon.substr(0, pos));
      polygon.erase(0, pos + comma.length());
    }

    for(auto xy : points)
    {
      pos = xy.find(space);
      x_str = xy.substr(0, pos);
      xy.erase(0, pos + space.length());
      y_str = xy;

      std::setlocale(LC_NUMERIC,"C");
      point[0] = stod(x_str);
      point[1] = stod(y_str);

      poly.push_back(point);
    }

    return poly;
  }

  void PrecisionPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic =
        mapviz::SelectTopicDialog::selectTopic("sensor_msgs/NavSatFix");

    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void PrecisionPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();

    if (topic != topic_ || !initialized_)
    {
      initialized_ = false;
      has_message_ = false;
      PrintWarning("No messages recieved.");
      navsat_sub_.shutdown();

      topic_ = topic;
      if(!topic.empty())
      {
        navsat_sub_ = nh_.subscribe(topic_, 1, &PrecisionPlugin::NavSatFixCallback, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  //PRIVATE
  void PrecisionPlugin::NavSatFixCallback(
    const sensor_msgs::NavSatFixConstPtr navsat) // TODO
  {
    std_msgs::String msg;

    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;

      PrintInfo("OK.");
    }

    cv::Vec2d point;

    point[0] = navsat->longitude;
    point[1] = navsat->latitude;

    // Check if in polygons
    std::string designation;
    for(auto zone : zones_)
    {
      if(swri_geometry_util::PointInPolygon(zone.polygon, point))
      {
        if(designation != "bjork")
        {
          designation = zone.name;
        }
      }
    }

    if(designation == "") designation = "None";

    // ROS_INFO("Designation: %s", designation.c_str());
    msg.data = designation;

    state_pub_.publish(msg);
  }

}
