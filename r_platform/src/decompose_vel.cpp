/**
 *      @file  decompose_vel.cpp
 *      @brief
 *
 *      Conversion from linear and angular velocities to wheel pulses.
 * Subscriber to linear and angular velocities,
 *      conversion of those values into a message containing Left/Right pulses
 * to Publish into /client_messages (message for Arduino Leonardo ETH)
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
//===================================== LIBS
//============================================
//=======================================================================================

#include "r_platform/navi.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <sstream>

//=======================================================================================
//================================= DECLARATIONS
//========================================
//=======================================================================================

std_msgs::String generate_motor_message(float, float);
ros::Publisher chatter_pub;

int MSG_FREQ; // message frequency, defines the rate of the feedback loop, se
              // the time in the velocity is relative to it
int P_TURN;   // number of pulses in a full wheel rotation
float LENGTH; // distance between wheels, in meters
float RADIUS; // wheel radius, in meters

//=======================================================================================
//=================================== CALLBACKS
//=========================================
//=======================================================================================

/**
 * @brief  Callback used to process topic "navi_commands" subscription
 * @param[in]
 * @author B.Vieira
 */
void chatterCallback(const r_platform::navi msg) {

  // ROS_INFO("I heard: [%f] [%f] \n", msg.linear_vel, msg.angular_vel);

  std_msgs::String msg_mot =
      generate_motor_message(msg.linear_vel, msg.angular_vel);

  // publishes generated message on "/client_messages"
  chatter_pub.publish(msg_mot);
}

//=======================================================================================
//================================= MAIN ROUTINE
//========================================
//=======================================================================================

int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber vel_sub = n.subscribe("navi_commands", 1000, chatterCallback);
  chatter_pub = n.advertise<std_msgs::String>("/client_messages", 1000);

  n.getParam("/msg_freq", MSG_FREQ);
  n.getParam("/wheel_radius", RADIUS);
  n.getParam("/base_length", LENGTH);
  n.getParam("/pulses_1turn", P_TURN);

  ros::spin();

  return 0;
}

//=======================================================================================
//================================== FUNCTIONS
//==========================================
//=======================================================================================

/**
 *
 * @brief  Generation of message to the Arduino (in left/right pulses) given
 * linear and angular velocities
 * @param[in] linear - linear velocity
 * @param[in] angular - angular velocity
 * @return Returns gen_msg - generated message for the Arduino Leonardo ETH
 * @author B.Vieira
 */
std_msgs::String generate_motor_message(float V_linear, float V_angular) {
  std_msgs::String gen_msg;

  float vR, vL, vR_rps, vL_rps;
  int pulse1, pulse2;
  char D1 = 'F', D2 = 'F';

  vR = (2.0 * V_linear + V_angular * LENGTH) / (2.0 * RADIUS); // 1/s
  vL = (2.0 * V_linear - V_angular * LENGTH) / (2.0 * RADIUS); // 1/s

  vR_rps = (vR / (2.0 * M_PI)) * P_TURN;
  vL_rps = (vL / (2.0 * M_PI)) * P_TURN;

  pulse1 = round(vR_rps / MSG_FREQ);
  pulse2 = round(vL_rps / MSG_FREQ);

  // =====================================================
  // ==================== DIRECTION DEFINITION ===========
  // =====================================================

  if (pulse1 > 0)
    D1 = 'F';
  else {
    D1 = 'B';
    pulse1 = abs(pulse1);
  }

  if (pulse2 > 0)
    D2 = 'F';
  else {
    D2 = 'B';
    pulse2 = abs(pulse2);
  }

  // string concatenation
  std::stringstream ss;
  ss << "MM" << D1 << pulse1 << "-" << D2 << pulse2 << ">";
  //   ss << "MM" << 'F' << 100 << "-" << 'F' << 100 << ">";
  gen_msg.data = ss.str();

  return gen_msg;
}
