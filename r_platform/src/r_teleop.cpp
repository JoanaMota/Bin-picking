/**
 *      @file  r_teleop.cpp
 *      @brief  
 *
 *      Subscribeer to /joy topic and publisher in the /navi_commands
 *      This node is responsible for controling the platform with the Xbox remote controller
 *
 *      @author   Bruno Vieira - bruno.v@ua.pt
 *
 *    @internal
 *      Created  20-Jun-2017
 *      Company  University of Aveiro
 *    Copyright  Copyright (c) 2017, Live session user
 *
 * =====================================================================================
 */

//=======================================================================================
//================================ LIBS / DEFINES =======================================
//=======================================================================================


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "r_platform/navi.h"


//=======================================================================================
//===================================== CLASS ===========================================
//=======================================================================================


bool robot_allowed=true;

class TeleopRobonuc
{
public:
  TeleopRobonuc();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_; // id of angular and linear axis (position in the array)
  float l_scale_, a_scale_; // linear and angular scale
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopRobonuc::TeleopRobonuc():
  linear_(1),
  angular_(3),
  l_scale_(0.025),
  a_scale_(0.025)
{

  //nh_.param("axis_linear", linear_, linear_);
  //nh_.param("axis_angular", angular_, angular_);
  //nh_.param("scale_angular", a_scale_, a_scale_);
  //nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<r_platform::navi>("navi_commands", 20);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 20, &TeleopRobonuc::joyCallback, this);
}


//=======================================================================================
//=================================== CALLBACKS =========================================
//=======================================================================================

// tentar definir um rate de publicação fixo, e quando entra no loop é que vai fazer subscribe.. Assim garantiam-se os 20 Hz

void TeleopRobonuc::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  
    r_platform::navi vel_msg;

    // decrease velocity rate
    if (joy->buttons[4] == 1 && a_scale_ > 0 && l_scale_ > 0.0)
    {
      l_scale_= l_scale_ - 0.025;
      a_scale_= a_scale_ - 0.025;
      ROS_INFO("DEC v_rate l[%f] a[%f]", l_scale_, a_scale_);
    }

    //increase velocity rate
    if (joy->buttons[5] == 1 && a_scale_ < 0.5 && l_scale_ < 0.5)
    {
      l_scale_= l_scale_ + 0.025;
      a_scale_= a_scale_ + 0.025;
      ROS_INFO("INC v_rate l[%f] a[%f]", l_scale_, a_scale_);
    }

    // deadman switch
    //if (joy->buttons[6] == 1)
	if (joy->axes[2] == -1)
    { 
      vel_msg.linear_vel = l_scale_*joy->axes[linear_];
      vel_msg.angular_vel = a_scale_*joy->axes[angular_];
	
	if(joy->buttons[0] == 0 && joy->buttons[1] == 0 && joy->buttons[2] == 0 && joy->buttons[3] == 0 &&joy->buttons[7] == 0){
		robot_allowed=true;
	}

	//Robot Action
	if(joy->buttons[0] == 1 && robot_allowed ){ 
		vel_msg.robot = 1;
		robot_allowed=false;
	}else if(joy->buttons[1] == 1 && robot_allowed ){
		vel_msg.robot = 2;
		robot_allowed=false;
	}else if(joy->buttons[2] == 1 && robot_allowed ){
		vel_msg.robot = 3;
		robot_allowed=false;
	}else if(joy->buttons[3] == 1 && robot_allowed ){
		vel_msg.robot = 4;
		robot_allowed=false;
	}else if(joy->buttons[7] == 1 && robot_allowed ){
		vel_msg.robot = 5;
		robot_allowed=false;
	}

    }else
    {
      vel_msg.linear_vel = 0;
      vel_msg.angular_vel = 0;
      vel_msg.robot = 0;
    }

    vel_pub_.publish(vel_msg);

}


//=======================================================================================
//====================================== MAIN ===========================================
//=======================================================================================

int main(int argc, char** argv)
{

  ros::init(argc, argv, "teleop_robonuc");

  TeleopRobonuc teleop_robonuc;

  ros::spin();
}
 
