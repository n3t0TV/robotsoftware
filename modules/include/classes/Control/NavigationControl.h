#include <ros/ros.h>

#include <modules/joystick_msg.h>
#include <modules/vp_whereiam_msg.h>
#include <std_msgs/Bool.h>
#include <cmath>

static const float MAX_ANG_RATE = 40.0;
static const float E_THRESHOLD = 0.05;
static const float MIN_CONFIDENCE = 0.60;
static const float EXCEPTION_TIME = 0.2;
static const float MAX_VEL = 300;

enum Classifier { CROSS = 0, ROAD, SIDEWALK };

inline float clang(float a, float b, float x){
    if (x<a) return a;
    if (x>b) return b;
    return x;
}

class NavigationControl {
 public:
  NavigationControl(ros::NodeHandle);
  ~NavigationControl(){};

 private:
  ros::NodeHandle nh;
  ros::Subscriber sub_teleops, sub_whereiam, sub_follow_person_;
  ros::Subscriber speech_cntrl_sub_;
  ros::Publisher pub_control;
  ros::Duration last_msg_elapsed_time;
  ros::Time last_whereiam_msg_timeStamp;
  std::string node_name;
  modules::joystick_msg control_msg;
  float angular_rate_inferred{0.0};
  float vel_inferred{0.0};
  bool inferring{false};
  int ilumination_ = 0;

  void TeleopsCallback(const modules::joystick_msg msg);
  void WhereIAmCallback(const modules::vp_whereiam_msg msg);
  void WhereIAmFollowPersonCallback(const modules::vp_whereiam_msg msg);
  void SpeechCntrlCallback(const std_msgs::Bool msg);
};

NavigationControl::NavigationControl(ros::NodeHandle nh_priv) {
  node_name = ros::this_node::getName();
  int log_level;
  nh_priv.param("log_level", log_level, 0);
  ros::console::levels::Level console_level;
  console_level = (ros::console::levels::Level)log_level;
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  pub_control = nh.advertise<modules::joystick_msg>("control_topic", 1);
  sub_teleops = nh.subscribe("teleops_topic", 5,
                             &NavigationControl::TeleopsCallback, this);
  sub_whereiam = nh.subscribe("video_processing/whereiam", 1,
                              &NavigationControl::WhereIAmCallback, this);

  sub_follow_person_ = nh.subscribe("follow_person_topic", 1,
                              &NavigationControl::WhereIAmFollowPersonCallback, this);
  speech_cntrl_sub_ = nh.subscribe("speech_recognize_node/robot_ctrl", 1,
                              &NavigationControl::SpeechCntrlCallback, this);
}

void NavigationControl::TeleopsCallback(const modules::joystick_msg msg) {
  last_msg_elapsed_time = ros::Time::now() - last_whereiam_msg_timeStamp;
  if (last_msg_elapsed_time.toSec() > EXCEPTION_TIME) {
    // whereiam delay or crash
    inferring = false;
    angular_rate_inferred = 0.0;
  }

  control_msg = msg;
  control_msg.Inferring = inferring;
  control_msg.Illumination = ilumination_;
  if (inferring && control_msg.Speed >= 0) {
    control_msg.AngularRate = (control_msg.AngularRate != 0.0)
                                  ? control_msg.AngularRate
                                  : angular_rate_inferred;
    control_msg.Speed = vel_inferred;
    //return;
  }
  pub_control.publish(control_msg);
}

void NavigationControl::WhereIAmCallback(const modules::vp_whereiam_msg msg) {
  if (control_msg.AutoPilot && msg.cls_id == SIDEWALK &&
      msg.cls_confidence >= MIN_CONFIDENCE) {
    inferring = true;
    if (std::abs(msg.cdir) > E_THRESHOLD)
      angular_rate_inferred = msg.cdir * MAX_ANG_RATE;
    else
      angular_rate_inferred = 0.0;
  } else {
    inferring = false;
    angular_rate_inferred = 0.0;
  }
  last_whereiam_msg_timeStamp = ros::Time::now();
}

void NavigationControl::WhereIAmFollowPersonCallback(const modules::vp_whereiam_msg msg)
{
    if (control_msg.AutoPilot) {
        inferring = true;
        angular_rate_inferred = clang(-30, 30, msg.cdir * MAX_ANG_RATE);
        vel_inferred  = std::min(msg.vel*MAX_VEL, 250.0F);
    } else {
        inferring = false;
        angular_rate_inferred = 0.0;
        vel_inferred = 0.0;
    }

    last_whereiam_msg_timeStamp = ros::Time::now();
}

void NavigationControl::SpeechCntrlCallback(const std_msgs::Bool msg)
{
  ilumination_ = msg.data ? 100 : 0;
}
