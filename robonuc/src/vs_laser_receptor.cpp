#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include "robonuc/robonuc_com.h"
#include "robonuc/conjunto.h"
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include "robonuc/alarm.h"
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/JointState.h>


robonuc::alarm ala;


float dist_lvl0_l0, dist_lvl1_l0, dist_lvl0_l1, dist_lvl1_l1;


class LaserReceptor
{
public:

//====================================================================================================================================================
//****************************************************************************************************************************************************
// Função que é responsavel por invocar tópicos ROS
//****************************************************************************************************************************************************
//====================================================================================================================================================
	LaserReceptor()
	{

		subscriber_laser0 = n.subscribe("/scan0", 1000, &LaserReceptor::Laser0callback, this);
		subscriber_laser1 = n.subscribe("/scan1", 1000, &LaserReceptor::Laser1callback, this);
		subscriber_joints = n.subscribe("/joint_states", 1000, &LaserReceptor::JointsState, this);

		publisher_alarm = n.advertise<robonuc::alarm>("alarm", 1000);

		n.getParam("/d_level0_l0",dist_lvl0_l0);
		n.getParam("/d_level1_l0",dist_lvl1_l0);
		n.getParam("/d_level0_l1",dist_lvl0_l1);
		n.getParam("/d_level1_l1",dist_lvl1_l1);

	} 

	void Laser0callback (const sensor_msgs::LaserScan laser0)
	{
		int ALARM0 = 0; 
		int c0 = 0;
		int sev0 = 0;
		float min_value_l0=99.99;

		for (unsigned int i=0; i<laser0.ranges.size();i++)
		{
			if(laser0.ranges[i] < min_value_l0) min_value_l0 = laser0.ranges[i];

			if(laser0.ranges[i] < dist_lvl1_l0){
				sev0 = 2;
				c0++;
			}else if(laser0.ranges[i] < dist_lvl0_l0){
				if(sev0!=2)
					sev0 = 1;
				c0++;
			}
		}

		//ROS_INFO("%d,%d,%f",sev0,c0,min_value_l0);

		if(c0 > 3){ //Não é ruído
			ala.sever0 = sev0;
			ala.alalaser0 = 1;
			publisher_alarm.publish(ala);
			if(sev0 == 1)
				ROS_WARN("Laser0: %02f < %02f < %02f\n",dist_lvl1_l0,min_value_l0,dist_lvl0_l0);
			if(sev0 == 2)
				ROS_ERROR("Laser0: %02f < %02f < %02f\n",min_value_l0,dist_lvl1_l0,dist_lvl0_l0);	
		}else{
			ala.alalaser0 = 0;
			publisher_alarm.publish(ala);
		}
	}

	void JointsState (const sensor_msgs::JointState joint_state)
	{
		//ROS_ERROR("OLHA EU!%f",joint_state.position[2]);

	}

	void Laser1callback (const sensor_msgs::LaserScan laser1)
	{
		int ALARM1 = 0;
		int c1 = 0;
		int sev1 = 0;
		float min_value_l1=99.99;

		for (unsigned int i=0; i<laser1.ranges.size();i++)
		{
			if(laser1.ranges[i] < min_value_l1 && laser1.ranges[i] > 0.020) min_value_l1 = laser1.ranges[i];

			if(laser1.ranges[i] < dist_lvl1_l1 && laser1.ranges[i] > 0.020){
				sev1 = 2;
				c1++;
			}else if(laser1.ranges[i] < dist_lvl0_l1 && laser1.ranges[i] > 0.020){
				if(sev1!=2)
					sev1 = 1;
				c1++;
			}
		}

		//ROS_INFO("%d,%d,%f",sev1,c1,min_value_l1);


		if(c1 > 3){ //Não é ruído
			ala.sever1 = sev1;
			ala.alalaser1 = 1;
			publisher_alarm.publish(ala);
			if(sev1 == 1)
				ROS_WARN("Laser1: %2f < %2f < %2f\n",dist_lvl1_l1,min_value_l1,dist_lvl0_l1);
			if(sev1 == 2)
				ROS_ERROR("Laser1: %f < %2f < %2f\n",min_value_l1,dist_lvl1_l1,dist_lvl0_l1);
		}else{
			ala.alalaser1 = 0;
			publisher_alarm.publish(ala);
		}
	}

protected:
	ros::NodeHandle n;
	ros::Subscriber subscriber_laser0;
	ros::Subscriber subscriber_laser1;
	ros::Subscriber subscriber_joints;
	ros::Publisher publisher_alarm;
};

int main(int argc, char **argv)
{

	ros::init (argc, argv, "vs_laser_receptor");
	  
	LaserReceptor laserreceptor;


	ros::spin();

	return 0;

}

