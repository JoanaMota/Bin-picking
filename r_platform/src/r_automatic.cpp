/**
 *      @file  r_automatic.cpp
 *      @brief
 *
 *
 *      @author   Luís Sarmento - sarmento@ua.pt
 *
 *    @internal
 *      Created  2-May-2018
 *      Company  University of Aveiro
 *
 * =====================================================================================
 */

//=======================================================================================
//================================ LIBS / DEFINES
//=======================================
//=======================================================================================

#include "geometry_msgs/Twist.h"
#include "r_platform/navi.h"
#include <ros/ros.h>

//=======================================================================================
//===================================== CLASS
//===========================================
//=======================================================================================

bool robot_allowed = true;

class TeleopRobonuc {
public:
  TeleopRobonuc();

  const r_platform::navi &vel_msg() const;

  void publish_vel_msg();

private:
  void autoNav(const geometry_msgs::Twist::ConstPtr &vel);

  ros::NodeHandle nh_;

  int linear_,
      angular_; // id of angular and linear axis (position in the array)
  float l_scale_, a_scale_; // linear and angular scale
  ros::Publisher vel_pub_;
  ros::Subscriber vel_sub_;

  r_platform::navi vel_msg_;
};

TeleopRobonuc::TeleopRobonuc()
    : linear_(1), angular_(3), l_scale_(0.025), a_scale_(0.025) {

  vel_pub_ = nh_.advertise<r_platform::navi>("navi_commands", 20);

  vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10,
                                                 &TeleopRobonuc::autoNav, this);
}

const r_platform::navi &TeleopRobonuc::vel_msg() const { return vel_msg_; }

void TeleopRobonuc::publish_vel_msg() { vel_pub_.publish(vel_msg()); }

//=======================================================================================
//=================================== CALLBACKS
//=========================================
//=======================================================================================

// tentar definir um rate de publicação fixo, e quando entra no loop é que vai
// fazer subscribe.. Assim garantiam-se os 20 Hz

void TeleopRobonuc::autoNav(const geometry_msgs::Twist::ConstPtr &vel) {

  vel_msg_.linear_vel = vel->linear.x * 0.5;
  vel_msg_.angular_vel = vel->angular.z * 0.5;
  vel_msg_.robot = 0;
}

//=======================================================================================
//====================================== MAIN
//===========================================
//=======================================================================================

int main(int argc, char **argv) {

  ros::init(argc, argv, "teleop_robonuc");

  TeleopRobonuc teleop_robonuc;

  ros::Rate loop_rate(20);

  while (ros::ok()) {

    teleop_robonuc.publish_vel_msg();

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
