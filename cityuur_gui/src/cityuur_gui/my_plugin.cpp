#include "cityuur_gui/my_plugin.h"
#include <pluginlib/class_list_macros.h>

#include <ros/package.h>

#include "curos_msgs/ManiCMD.h"

#include <QStringList>
#include <QString>
#include <QTimer>

#include <QDebug>

#include <QKeyEvent>

namespace cityuur_gui 
{

// Constructor
MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
  , rotate_state_(ROTATE_180)
{

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  // add event filter (eg keyboard)
  widget_->installEventFilter(this);

  // dealing with image
  ui_.image_frame_top->setOuterLayout(ui_.image_layout_top);
  ui_.image_frame_top->setImage(QImage());

  
  // **change publish topic here
  pub_cmd_ = n.advertise<std_msgs::String>("/mani_cmd", 10);

  // **change subscribe topic here
  sub_status_1 = n.subscribe("/status/drive_mode", 1, &MyPlugin::callbackStatus, this);
  sub_status_2 = n.subscribe("/status/max_thrust", 1, &MyPlugin::callbackMaxThrust, this);
  sub_status_3 = n.subscribe("/status/pressure", 1, &MyPlugin::callbackPressure, this);
  
  // camera
  QList<QString> transports;
  image_transport::ImageTransport it(n);
  image_transport::TransportHints hints("compressed");
  try {
    sub_image_front_ = it.subscribe("/front_cam/image_raw", 1, &MyPlugin::callbackImageFront, this, hints);
  } catch (image_transport::TransportLoadException& e) {
    std::cout << "Transport Load error: " << e.what() << std::endl;
  }

  // initialize with timer
  timer = new QTimer(this);
  timer_limit = 900;
  timer_counter = 900;
  ui_.timer_display->setText("15:00");

  connect(timer, SIGNAL(timeout()), this, SLOT(timer_on()));
  connect(ui_.timer_stop_push_button, SIGNAL(clicked(bool)), this, SLOT(onTimerStop()));
  connect(ui_.timer_start_push_button, SIGNAL(clicked(bool)), this, SLOT(onTimerStart()));
  connect(ui_.timer_reset_push_button, SIGNAL(clicked(bool)), this, SLOT(onTimerReset()));
  
  connect(ui_.rotate_left_push_button, SIGNAL(clicked(bool)), this, SLOT(onRotateLeft()));
  connect(ui_.rotate_right_push_button, SIGNAL(clicked(bool)), this, SLOT(onRotateRight()));

  syncRotateLabel();

  // ur logo
  QPixmap pix("/home/ur/curos_ws/src/cityuur_gui/resource/BlueSUR_copy.png");
  int w = ui_.ur_logo->width();
  int h = ui_.ur_logo->height();
  ui_.ur_logo->setPixmap(pix.scaled(w,h,Qt::KeepAspectRatio));

  // initial status
  servo_1_angle = 0;
  servo_2_angle = 0;
  servo_3_angle = 0;
  ui_.mani_value_3->setText(QString::number(servo_2_angle));
  ui_.mani_value_4->setText(QString::number(servo_1_angle));
  ui_.mani_value_5->setText(QString::number(servo_3_angle));

  pump_timer = new QTimer(this);
  connect(pump_timer, SIGNAL(timeout()), this, SLOT(pump_on()));
  isPumpOn = false;
  pump_time = 0;
  ui_.mani_value_7->setText(QString("OFF"));
  ui_.mani_value_8->setText(QString::number(pump_time));
  
}

void MyPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here
  pub_cmd_.shutdown();
  sub_image_front_.shutdown();
  sub_status_1.shutdown();
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

void MyPlugin::callbackImageFront(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
  conversion_mat_top_ = cv_ptr->image;

  // Handle rotation
  switch(rotate_state_)
  {
    case ROTATE_90:
    {
      cv::Mat tmp;
      cv::transpose(conversion_mat_top_, tmp);
      cv::flip(tmp, conversion_mat_top_, 1);
      break;
    }
    case ROTATE_180:
    {
      cv::Mat tmp;
      cv::flip(conversion_mat_top_, tmp, -1);
      conversion_mat_top_ = tmp;
      break;
    }
    case ROTATE_270:
    {
      cv::Mat tmp;
      cv::transpose(conversion_mat_top_, tmp);
      cv::flip(tmp, conversion_mat_top_, 0);
      break;
    }
    default:
      break;
  }

  QImage image(conversion_mat_top_.data, conversion_mat_top_.cols, conversion_mat_top_.rows, conversion_mat_top_.step[0], QImage::Format_RGB888);
  ui_.image_frame_top->setImage(image);
}
void MyPlugin::callbackStatus(const std_msgs::String::ConstPtr& msg)
{
  QString str = QString::fromStdString(msg->data.c_str());
  ui_.status_value_1->setText(str);
}
void MyPlugin::callbackMaxThrust(const std_msgs::String::ConstPtr& msg)
{
  QString str = QString::fromStdString(msg->data.c_str());
  ui_.status_value_2->setText(str);
}
void MyPlugin::callbackPressure(const std_msgs::String::ConstPtr& msg)
{
  QString str = QString::fromStdString(msg->data.c_str());
  ui_.sensor_value_1->setText(str);
}





/* TIMER */
void MyPlugin::timer_on()
{
  timer_counter --;
  ui_.timer_display->setText(timeToQString(timer_counter));
  if (timer_counter == 0)
  {
    timer->stop();
    ui_.timer_display->setText("TIME'S UP");
  }
}

void MyPlugin::onTimerStart()
{
  timer->start(1000);
}

void MyPlugin::onTimerStop()
{
  timer->stop();
}

void MyPlugin::onTimerReset()
{
  if (timer->isActive())
    timer->stop();
  timer_counter = timer_limit;
  ui_.timer_display->setText(timeToQString(timer_counter));
}

/* PUMP ON */
void MyPlugin::pump_on()
{
  pump_time ++;
  ui_.mani_value_8->setText(QString::number(pump_time));
  if (pump_time == 27)
  {
    ui_.mani_value_7->setText("OFF");
    pump_timer->stop();
  }
}

/* IMAGE ROTATE */
void MyPlugin::onRotateLeft()
{
  int m = rotate_state_ - 1;
  if(m < 0)
    m = ROTATE_STATE_COUNT-1;

  rotate_state_ = static_cast<RotateState>(m);
  syncRotateLabel();
}

void MyPlugin::onRotateRight()
{
  rotate_state_ = static_cast<RotateState>((rotate_state_ + 1) % ROTATE_STATE_COUNT);
  syncRotateLabel();
}

void MyPlugin::syncRotateLabel()
{
  switch(rotate_state_)
  {
    default:
    case ROTATE_0:   ui_.rotate_label->setText("0°"); break;
    case ROTATE_90:  ui_.rotate_label->setText("90°"); break;
    case ROTATE_180: ui_.rotate_label->setText("180°"); break;
    case ROTATE_270: ui_.rotate_label->setText("270°"); break;
  }
}
/* END of IMAGE ROTATE*/

/* KEYBOARD */
bool MyPlugin::eventFilter(QObject *target, QEvent *event) {
  if(event->type() == QEvent::KeyPress)
  {
    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
    std_msgs::String msg;

    // **change keybinds
    if(0x31 == keyEvent->key())
    {
      msg.data = "Finger1Open";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'1' Finger1Open"));
      ui_.mani_value_2->setText("OPEN");
    }
    if (0x32 == keyEvent->key())
    {
      msg.data = "Finger1Close";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'2' Finger1Close"));
      ui_.mani_value_2->setText("CLOSE");
    }
    if (0x33 == keyEvent->key())
    {
      msg.data = "Finger2Open";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'3' Finger2Open"));
      ui_.mani_value_6->setText("OPEN");
    }
    if (0x34 == keyEvent->key())
    {
      msg.data = "Finger2Close";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'4' Finger2Close"));
      ui_.mani_value_6->setText("CLOSE");
    }
    if (0x35 == keyEvent->key())
    {
      msg.data = "Finger3Open";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'5' Finger3Open"));
    }
    if (0x36 == keyEvent->key())
    {
      msg.data = "Finger3Close";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'6' Finger3Close"));
    }
    if (0x37 == keyEvent->key())
    {
      msg.data = "RDropOpen";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'7' RDropOpen"));
      ui_.mani_value_1->setText("OPEN");
    }
    if (0x38 == keyEvent->key())
    {
      msg.data = "RDropClose";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'8' RDropCLose"));
      ui_.mani_value_1->setText("CLOSE");
    }
    if (0x39 == keyEvent->key())
    {
      msg.data = "LLiftBag";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'9' LLiftBag"));
    }
    if (0x30 == keyEvent->key())
    {
      msg.data = "RLiftBag";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'0' RLiftBah"));
    }
    if (0x57 == keyEvent->key())
    {
      msg.data = "CameraAngleUp";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'W' CameraAngleUp"));
    }
    if (0x53 == keyEvent->key())
    {
      msg.data = "CameraAngleDown";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'S' CameraAngleDown"));
    }
    if (0x4a == keyEvent->key())
    {
      msg.data = "Servo1AngleLeft";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'J' Servo1AngleLeft"));
      servo_1_angle += 5;
      if (servo_1_angle >= 270)
        servo_1_angle = 270;
      ui_.mani_value_4->setText(QString::number(servo_1_angle));
    }
    if (0x4b == keyEvent->key())
    {
      msg.data = "Servo1AngleRight";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'K' Servo1AngleRight"));
      servo_1_angle -= 5;
      if (servo_1_angle <= 0)
        servo_1_angle = 0;
      ui_.mani_value_4->setText(QString::number(servo_1_angle));
    }
    if (0x4e == keyEvent->key())
    {
      msg.data = "Servo2AngleLeft";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'N' Servo2AngleLeft"));
      servo_2_angle += 5;
      if (servo_2_angle >= 270)
        servo_2_angle = 270;
      ui_.mani_value_3->setText(QString::number(servo_2_angle));
    }
    if (0x4d == keyEvent->key())
    {
      msg.data = "Servo2AngleRight";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'M' Servo2AngleRight"));
      servo_2_angle -= 5;
      if (servo_2_angle <= 0)
        servo_2_angle = 0;
      ui_.mani_value_3->setText(QString::number(servo_2_angle));
    }
    if (0x56 == keyEvent->key())
    {
      msg.data = "Servo3AngleUp";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'V' Servo3AngleLeft"));
      ui_.mani_value_5->setText(QString::number('ON'));
    }
    if (0x42 == keyEvent->key())
    {
      msg.data = "Servo3AngleDown";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'B' Servo3AngleRight"));
      ui_.mani_value_5->setText(QString::number('OFF'));
    }
    if (0x55 == keyEvent->key())
    {
      msg.data = "SearchLight1On";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'U' SearchLight1On"));
    }
    if (0x49 == keyEvent->key())
    {
      msg.data = "SearchLight1Off";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'I' SearchLight1Off"));
    }
    // if (0x4f == keyEvent->key())
    // {
    //   msg.data = "SearchLight2On";
    //   pub_cmd_.publish(msg);
    //   ui_.command_browser->append(QString("MANI:'O' SearchLight2On"));
    // }
    // if (0x50 == keyEvent->key())
    // {
    //   msg.data = "SearchLight2Off";
    //   pub_cmd_.publish(msg);
    //   ui_.command_browser->append(QString("MANI:'P' SearchLight2Off"));
    // }
    if (0x54 == keyEvent->key())
    {
      msg.data = "UVLightOn";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'T' UVLightOn"));
    }
    if (0x59 == keyEvent->key())
    {
      msg.data = "UVLightOff";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'Y' UVLightOff"));
    }
    if (0x47 == keyEvent->key())
    {
      msg.data = "PumpOn";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'G' PumpOn"));
      isPumpOn = true;
      pump_time = 0;
      pump_timer->start(1000);
      ui_.mani_value_7->setText("ON");
    }
    if (0x48 == keyEvent->key())
    {
      msg.data = "PumpOff";
      pub_cmd_.publish(msg);
      ui_.command_browser->append(QString("MANI:'H' PumpOff"));
      isPumpOn = false;
      pump_timer->stop();
      ui_.mani_value_7->setText("OFF");
    }
  }
  return rqt_gui_cpp::Plugin::eventFilter(target, event);
}
/* END of KEYBOARD */

void MyPlugin::publishcomm(std::string topic, std::string data, std::string msgtype="None")
{
  std_msgs::String msg;
  msg.data = data;
  if (msgtype == "command")
  {
    if (topic == "mani_cmd")
    {
      pub_cmd_.publish(msg);
      return;
    }
  }
}

QString MyPlugin::timeToQString(int i)
{
  int m, s;

  m = i / 60;
  s = i % 60;

  // QString str = QString("%1:%2").arg(m).arg(s);
  return QString("%1:%2").arg(m, 2, 10, QLatin1Char('0')).arg(s, 2, 10, QLatin1Char('0'));
}



} // namespace


// PLUGINLIB_DECLARE_CLASS(cityuur_gui, MyPlugin, cityuur_gui::MyPlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(cityuur_gui::MyPlugin, rqt_gui_cpp::Plugin)