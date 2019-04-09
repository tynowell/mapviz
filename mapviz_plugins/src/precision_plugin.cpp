#include <mapviz_plugins/precision_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <algorithm>
#include <vector>

// QT libraries
#include <QGLWidget>
#include <QPalette>
#include <QImage>
#include <QFileDialog>

// ROS libraries
#include <ros/master.h>

#include <swri_image_util/geometry_util.h>

#include <mapviz/select_topic_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::PrecisionPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  // PUBLIC
  PrecisionPlugin::PrecisionPlugin() :
    config_widget_(new QWidget()),
    anchor_(TOP_LEFT),
    units_(PIXELS),
    offset_x_(0),
    offset_y_(0),
    width_(320),
    height_(240),
    has_image_(false),
    last_width_(0),
    last_height_(0),
    original_aspect_ratio_(1.0)
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
    QObject::connect(ui_.anchor, SIGNAL(activated(QString)), this,
                     SLOT(SetAnchor(QString)));
    QObject::connect(ui_.width, SIGNAL(valueChanged(double)), this,
                     SLOT(SetWidth(double)));
    QObject::connect(ui_.height, SIGNAL(valueChanged(double)), this,
                     SLOT(SetHeight(double)));
    QObject::connect(ui_.units, SIGNAL(activated(QString)), this,
                     SLOT(SetUnits(QString)));
    QObject::connect(ui_.offset_x, SIGNAL(valueChanged(int)), this,
                     SLOT(SetOffsetX(int)));
    QObject::connect(ui_.offset_y, SIGNAL(valueChanged(int)), this,
                     SLOT(SetOffsetY(int)));
    QObject::connect(ui_.keep_ratio, SIGNAL(toggled(bool)), this,
                     SLOT(KeepRatioChanged(bool)));

    ui_.width->setKeyboardTracking(false);
    ui_.height->setKeyboardTracking(false);
  }

  PrecisionPlugin::~PrecisionPlugin()
  {
  }

  bool PrecisionPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    return true;
  }

  void PrecisionPlugin::Draw(double x, double y, double scale)
  {
    // Calculate the correct offsets and dimensions
    double x_offset = offset_x_;
    double y_offset = offset_y_;
    double width = width_;
    double height = height_;

    if (units_ == PERCENT)
    {
      x_offset = offset_x_ * canvas_->width() / 100.0;
      y_offset = offset_y_ * canvas_->height() / 100.0;
      width = width_ * canvas_->width() / 100.0;
      height = height_ * canvas_->height() / 100.0;
    }

    if( ui_.keep_ratio->isChecked() )
    {
      height = original_aspect_ratio_ * width;
    }

    // Scale the source image if necessary
    if (width != last_width_ || height != last_height_)
    {
      ScaleImage(width, height);
    }

    // Calculate the correct render position
    double x_pos = 0;
    double y_pos = 0;
    if (anchor_ == TOP_LEFT)
    {
      x_pos = x_offset;
      y_pos = y_offset;
    }
    else if (anchor_ == TOP_CENTER)
    {
      x_pos = (canvas_->width() - width) / 2.0 + x_offset;
      y_pos = y_offset;
    }
    else if (anchor_ == TOP_RIGHT)
    {
      x_pos = canvas_->width() - width - x_offset;
      y_pos = y_offset;
    }
    else if (anchor_ == CENTER_LEFT)
    {
      x_pos = x_offset;
      y_pos = (canvas_->height() - height) / 2.0 + y_offset;
    }
    else if (anchor_ == CENTER)
    {
      x_pos = (canvas_->width() - width) / 2.0 + x_offset;
      y_pos = (canvas_->height() - height) / 2.0 + y_offset;
    }
    else if (anchor_ == CENTER_RIGHT)
    {
      x_pos = canvas_->width() - width - x_offset;
      y_pos = (canvas_->height() - height) / 2.0 + y_offset;
    }
    else if (anchor_ == BOTTOM_LEFT)
    {
      x_pos = x_offset;
      y_pos = canvas_->height() - height - y_offset;
    }
    else if (anchor_ == BOTTOM_CENTER)
    {
      x_pos = (canvas_->width() - width) / 2.0 + x_offset;
      y_pos = canvas_->height() - height - y_offset;
    }
    else if (anchor_ == BOTTOM_RIGHT)
    {
      x_pos = canvas_->width() - width - x_offset;
      y_pos = canvas_->height() - height - y_offset;
    }

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, canvas_->width(), canvas_->height(), 0, -0.5f, 0.5f);

    glRasterPos2d(x_pos, y_pos);

    DrawIplImage(&scaled_image_);

    glPopMatrix();

    last_width_ = width;
    last_height_ = height;
  }

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

    if (node["offset_x"])
    {
      node["offset_x"] >> offset_x_;
      ui_.offset_x->setValue(offset_x_);
    }

    if (node["offset_y"])
    {
      node["offset_y"] >> offset_y_;
      ui_.offset_y->setValue(offset_y_);
    }

    if (node["width"])
    {
      node["width"] >> width_;
      ui_.width->setValue(width_);
    }

    if (node["height"])
    {
      node["height"] >> height_;
      ui_.height->setValue(height_);
    }

    if (node["keep_ratio"])
    {
      bool keep;
      node["keep_ratio"] >> keep;
      ui_.keep_ratio->setChecked( keep );
    }

    FileEdited();
    TopicEdited();
  }

  void PrecisionPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "file" << YAML::Value << shapefile_;
    emitter << YAML::Key << "topic" << YAML::Value << topic_;
    emitter << YAML::Key << "width" << YAML::Value << width_;
    emitter << YAML::Key << "height" << YAML::Value << height_;
    emitter << YAML::Key << "offset_x" << YAML::Value << offset_x_;
    emitter << YAML::Key << "offset_y" << YAML::Value << offset_y_;
    emitter << YAML::Key << "keep_ratio" << YAML::Value << ui_.keep_ratio->isChecked();
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

  void PrecisionPlugin::FileEdited() // TODO
  {
    // Open shapefile

    // Isolate polygons with designation

    // Convert to list of std::vector<cv::Vec2d>

    return;
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

    if (topic != topic_)
    {
      initialized_ = false;
      has_message_ = false;
      PrintWarning("No messages recieved.");
      navsat_sub_.shutdown();

      topic_ = topic;
      if(!topic.empty())
      {
        navsat_sub_ = node_.subscribe(topic_, 1, &PrecisionPlugin::NavSatFixCallback, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void PrecisionPlugin::SetOffsetX(int offset)
  {
    offset_x_ = offset;
  }

  void PrecisionPlugin::SetOffsetY(int offset)
  {
    offset_y_ = offset;
  }

  void PrecisionPlugin::SetWidth(double width)
  {
    width_ = width;
  }

  void PrecisionPlugin::SetHeight(double height)
  {
    height_ = height;
  }

  void PrecisionPlugin::SetAnchor(QString anchor)
  {
    if (anchor == "top left")
    {
      anchor_ = TOP_LEFT;
    }
    else if (anchor == "top center")
    {
      anchor_ = TOP_CENTER;
    }
    else if (anchor == "top right")
    {
      anchor_ = TOP_RIGHT;
    }
    else if (anchor == "center left")
    {
      anchor_ = CENTER_LEFT;
    }
    else if (anchor == "center")
    {
      anchor_ = CENTER;
    }
    else if (anchor == "center right")
    {
      anchor_ = CENTER_RIGHT;
    }
    else if (anchor == "bottom left")
    {
      anchor_ = BOTTOM_LEFT;
    }
    else if (anchor == "bottom center")
    {
      anchor_ = BOTTOM_CENTER;
    }
    else if (anchor == "bottom right")
    {
      anchor_ = BOTTOM_RIGHT;
    }
  }

  void PrecisionPlugin::SetUnits(QString units)
  {
    // do this in both cases to avoid image clamping
    ui_.width->setMaximum(10000);
    ui_.height->setMaximum(10000);

    if (units == "pixels")
    {
      ui_.width->setDecimals(0);
      ui_.height->setDecimals(0);
      units_ = PIXELS;
      width_  = width_ * double(canvas_->width()) / 100.0;
      height_ = height_ * double(canvas_->height()) / 100.0;
      ui_.width->setSuffix(" px");
      ui_.height->setSuffix(" px");
    }
    else if (units == "percent")
    {
      ui_.width->setDecimals(1);
      ui_.height->setDecimals(1);
      units_ = PERCENT;
      width_ = width_ * 100.0 / double(canvas_->width());
      height_ =  height_ * 100.0 / double(canvas_->height());
      ui_.width->setSuffix(" %");
      ui_.height->setSuffix(" %");
    }
    ui_.width->setValue( width_ );
    ui_.height->setValue( height_ );

    if( units_ == PERCENT)
    {
      ui_.width->setMaximum(100);
      ui_.height->setMaximum(100);
    }
  }

  void PrecisionPlugin::KeepRatioChanged(bool checked)
  {
    ui_.height->setEnabled( !checked );
    if( checked )
    {
      ui_.height->setValue( width_ * original_aspect_ratio_ );
    }
  }

  //PRIVATE
  void PrecisionPlugin::NavSatFixCallback(
    const sensor_msgs::NavSatFixConstPtr navsat) // TODO
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

    double x;
    double y;
    tf_manager_->LocalXyUtil()->ToLocalXy(navsat->latitude, navsat->longitude, x, y);

    // Convert x, y to cv::Vec2d

    // Check if in polygons

    // if state change
    //    play audio
    //    change image
  }

  void PrecisionPlugin::LoadImage(const std::string& filename)
  {
    try
    {
      QImage nullImage;
      image_ = nullImage;

      if (texture_loaded_)
      {
        GLuint ids[1];
        ids[0] = static_cast<GLuint>(texture_id_);
        glDeleteTextures(1, &ids[0]);
        texture_loaded_ = false;
      }

      if (image_.load(filename.c_str()))
      {
        int width = image_.width();
        int height = image_.height();
        image_ratio_ = (double)height / (double)width;

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
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dimension_, dimension_, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_.bits());

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        texture_loaded_ = true;
      }
      else
      {
        PrintError("Failed to load image.");
      }
    }
    catch (const std::exception& e)
    {
      PrintError("Failed to load image.  Exception occured.");
    }
  }

  void PrecisionPlugin::ScaleImage(double width, double height)
  {
    if (!has_image_)
    {
      return;
    }

    cv::resize(cv_image_->image, scaled_image_, cvSize2D32f(width, height), 0, 0, CV_INTER_AREA);
  }

  void PrecisionPlugin::DrawIplImage(cv::Mat *image)
  {
    // TODO(malban) glTexture2D may be more efficient than glDrawPixels

    if (image == NULL || image->cols == 0 || image->rows == 0)
    {
      return;
    }

    GLenum format;
    switch (image->channels())
    {
      case 1:
        format = GL_LUMINANCE;
        break;
      case 2:
        format = GL_LUMINANCE_ALPHA;
        break;
      case 3:
        format = GL_BGR;
        break;
      default:
        return;
    }

    glPixelZoom(1.0f, -1.0f);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glDrawPixels(image->cols, image->rows, format, GL_UNSIGNED_BYTE, image->ptr());

    PrintInfo("OK");
  }

  std::string PrecisionPlugin::AnchorToString(Anchor anchor)
  {
    std::string anchor_string = "top left";

    if (anchor == TOP_LEFT)
    {
      anchor_string = "top left";
    }
    else if (anchor == TOP_CENTER)
    {
      anchor_string = "top center";
    }
    else if (anchor == TOP_RIGHT)
    {
      anchor_string = "top right";
    }
    else if (anchor == CENTER_LEFT)
    {
      anchor_string = "center left";
    }
    else if (anchor == CENTER)
    {
      anchor_string = "center";
    }
    else if (anchor == CENTER_RIGHT)
    {
      anchor_string = "center right";
    }
    else if (anchor == BOTTOM_LEFT)
    {
      anchor_string = "bottom left";
    }
    else if (anchor == BOTTOM_CENTER)
    {
      anchor_string = "bottom center";
    }
    else if (anchor == BOTTOM_RIGHT)
    {
      anchor_string = "bottom right";
    }

    return anchor_string;
  }

  std::string PrecisionPlugin::UnitsToString(Units units)
  {
    std::string units_string = "pixels";

    if (units == PIXELS)
    {
      units_string = "pixels";
    }
    else if (units == PERCENT)
    {
      units_string = "percent";
    }

    return units_string;
  }
}
