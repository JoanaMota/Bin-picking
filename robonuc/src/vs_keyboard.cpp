#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <math.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Int8.h>
#include "robonuc/conjunto.h"
#include <sstream>
#include <ros/callback_queue.h> 
#include <iostream>

using namespace std;

robonuc::conjunto mymsg;
robonuc::platmov mmsg;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<robonuc::conjunto>("chatter", 1000);
	ros::Publisher plat_mov_pub = n.advertise<robonuc::platmov>("platmov", 1000);

	while (ros::ok())

	{
		int id;
		int pos;
		int io;
		float x,y,z;
		float j1,j2,j3,j4,j5,j6;
		int c;

		//cout << "Please select the command id.. \n";
		cin >> id;

		switch (id) {
		    case 1:
			ROS_INFO("Please select the default position.. ");
			cin >> pos;
			ROS_INFO("Default Pos Move Selected ID - %d Pos - %d", id, pos); 
			break;
		    case 2:
			ROS_INFO("Please select the default position.. ");
			cout << "		x - ";
			cin >> x;
			cout << "		y - ";
			cin >> y;
			cout << "		z - ";
			cin >> z;
			ROS_INFO("Point Selected (x,y,z) = (%1.3f,%1.3f,%1.3f)",x,y,z); 
			ROS_INFO("Abs Coord Move Selected ID - %d",id); 
			break;
		    case 3:
			ROS_INFO("Please select the joit values.. ");
			cout << "		j1 - ";
			cin >> j1;
			cout << "		j2 - ";
			cin >> j2;
			cout << "		j3 - ";
			cin >> j3;
			cout << "		j4 - ";
			cin >> j4;
			cout << "		j5 - ";
			cin >> j5;
			cout << "		j6 - ";
			cin >> j6;
			ROS_INFO("Joints (j1,j2,j3,j4,j5,j6) ="); 
			ROS_INFO("= (%1.3f,%1.3f,%1.3f,%1.3f,%1.3f,%1.3f)",j1,j2,j3,j4,j5,j6); 
			ROS_INFO("Joints Move Selected ID - %d",id); 
			break;
		    case 4:
			ROS_INFO("Get Current Manipulator Position - %d",id);
			break;
		    case 5:
			ROS_INFO("Get Current System Position - %d",id);
			break;
		    case 6:
			ROS_INFO("Execute a default aplication program - %d",id);
			break;
		    case 7:
			ROS_INFO("Please select the IO code: 1-Read, 2-ON, 3-OFF ");
			cin >> pos;
			ROS_INFO("Please select the IO Number ");
			cin >> io;
			ROS_INFO("Monitoring IO's - %d",id);
			break;
		    case 8:
			ROS_INFO("Platform Movement - Arrows to move, 'q' to exit)");
			c=0;
			// Set terminal to raw mode 
 			system("stty raw"); 
			do{
				c=0;
				c=getchar();
				if(c==65 | c==66 | c==67 | c==68){
					mmsg.dir=c;
					plat_mov_pub.publish(mmsg);
				}
			}while( c != 'q' );
			// Reset terminal to normal "cooked" mode 
			system("stty cooked");
			break;
		    default:
			ROS_WARN("Unrecognized Command ID - %d",id); 
			    break;
		}			

		mymsg.keyboard.msg_id=id;

		mymsg.keyboard.pos_id=pos;

		mymsg.keyboard.io_num=io;

		mymsg.keyboard.x=x;
		mymsg.keyboard.y=y;
		mymsg.keyboard.z=z;

		mymsg.keyboard.j1=j1;
		mymsg.keyboard.j2=j2;
		mymsg.keyboard.j3=j3;
		mymsg.keyboard.j4=j4;
		mymsg.keyboard.j5=j5;
		mymsg.keyboard.j6=j6;


		ROS_INFO("Command ID %d Sent From Keyboard",id); 

//		Publicação do tópicos
		chatter_pub.publish(mymsg);
	}

	return 0;

}
