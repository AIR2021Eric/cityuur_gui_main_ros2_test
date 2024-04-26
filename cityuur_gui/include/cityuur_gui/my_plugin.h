#ifndef CITYUUR_GUI_MY_PLUGIN_H
#define CITYUUR_GUI_MY_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_my_plugin.h>

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QWidget>
#include <QImage>

namespace cityuur_gui 
{

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

  // subscriber callback
  virtual void callbackImageFront(const sensor_msgs::Image::ConstPtr& msg);
  virtual void callbackStatus(const std_msgs::String::ConstPtr& msg);
  virtual void callbackMaxThrust(const std_msgs::String::ConstPtr& msg);
  virtual void callbackPressure(const std_msgs::String::ConstPtr& msg);

  bool eventFilter(QObject *target, QEvent *event);

  void publishcomm(std::string, std::string, std::string); 
  // Common method/function
  QString timeToQString(int);

public slots:
  void timer_on();
  void onTimerStop();
  void onTimerStart();
  void onTimerReset();

  void pump_on();

  virtual void onRotateLeft();
  virtual void onRotateRight();

protected:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;

  /* node handler */
  ros::NodeHandle n;

  /* publisher */
  ros::Publisher pub_cmd_;

  /* subscriber */
  ros::Subscriber sub_status_1;
  ros::Subscriber sub_status_2;
  ros::Subscriber sub_status_3;

  QTimer *timer;
  int timer_limit;
  int timer_counter;

  image_transport::Subscriber sub_image_front_;
  cv::Mat conversion_mat_top_;

  enum RotateState {
    ROTATE_0 = 0,
    ROTATE_90 = 1,
    ROTATE_180 = 2,
    ROTATE_270 = 3,

    ROTATE_STATE_COUNT
  };

  void syncRotateLabel();
  RotateState rotate_state_;

  int servo_1_angle;
  int servo_2_angle;
  int servo_3_angle;

  QTimer *pump_timer;
  bool isPumpOn;
  int pump_time;

};
}  // namespace cityuur_gui
#endif // CITYUUR_GUI_MY_PLUGIN_H
