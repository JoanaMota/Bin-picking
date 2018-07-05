 /*************************************************************************
 * Author: Abhinav Jain
 * Contact: abhinavjain241@gmail.com, abhinav.jain@heig-vd.ch
 * Date: 28/06/2016
 *
 * This file contains source code to the client node of the ROS package
 * comm_tcp developed at LaRA (Laboratory of Robotics and Automation)
 * as part of my project during an internship from May 2016 - July 2016.
 *
 * (C) All rights reserved. LaRA, HEIG-VD, 2016 (http://lara.populus.ch/)
 ***************************************************************************/
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "comm_tcp/pid.h"

#define MESSAGE_FREQ 20

void error(const char *msg) {
    perror(msg);
    exit(0);
}

class Listener {
private:
	//listener_constructor();
    char topic_message[256];
public:
    void callback(const std_msgs::String::ConstPtr& msg);
    char* getMessageValue();
};

//Listener::listener_constructor(){
//	char topic_message[256]= {0};
//}

void Listener::callback(const std_msgs::String::ConstPtr& msg) {
    memset(topic_message, 0, 256);
    strcpy(topic_message, msg->data.c_str());
    //ROS_INFO("I heard:[%s]", msg->data.c_str());
}

char* Listener::getMessageValue() {
    return topic_message;
}




int main(int argc, char *argv[]) {
	
	// ROS node declaration
	ros::init(argc, argv, "CPU1_client");
	ros::NodeHandle nh;
	ros::Rate loop_rate(MESSAGE_FREQ); // set the rate as defined in the macro MESSAGE_FREQ

	//constructor
	Listener listener;

	ros::Subscriber client_sub = nh.subscribe("/client_messages", 1, &Listener::callback, &listener);

	//to publish msgs for rqt_plot
	ros::Publisher chatter_pub = nh.advertise<comm_tcp::pid>("pid_data", 1000);
	// message from created pid type
	comm_tcp::pid msg;
	std::size_t found, found1, found2, found3, found4;
	std::string str_buffer;
	//==================

    int sockfd, portno, n, choice = 1;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];
    bool echoMode = true;
    
    
   	// if (argc < 3) {
    //   fprintf(stderr,"Usage: $ rosrun comm_tcp client_node <hostname> <port> --arguments\nArguments:\n -e : Echo mode\n");
    //   exit(0);
    //}
    
    //if (argc > 3)
	//	if (strcmp(argv[3], "-e") == 0)
	//		echoMode = true;
			
    //portno = atoi(argv[2]);
	portno = 50000;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    
    if (sockfd < 0) 
        error("ERROR opening socket");
        
    //server = gethostbyname(argv[1]);
    server = gethostbyname("192.168.0.10");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
        
    std::cout << "How do you want the client to behave?:\n1. Be able to send messages manually\n2. Subscribe to /client_messages and send whatever's available there\nYour choice:";
    //td::cin >> choice;
    
    choice=2;
    
	while(ros::ok()) {
        bzero(buffer,256);
        if (choice == 1) {
            printf("Please enter the message: ");
            fgets(buffer,255,stdin);
	    printf("%s\n",buffer);
        } else if (choice == 2) {
            strcpy(buffer, listener.getMessageValue());
            loop_rate.sleep();
        }
	    n = write(sockfd,buffer,strlen(buffer));
	    if (n < 0) 
	         error("ERROR writing to socket");
	    if (echoMode) {

		bzero(buffer, 256);
		n = read(sockfd,buffer,255);
		if (n < 0)
			error("ERROR reading reply");
		printf("%s\n", buffer);
	
		// conversion of array buffer to string
		str_buffer = (std::string)buffer;
		

		if(str_buffer.find("RQT") != std::string::npos!=NULL) // find for RQT inside string
		{
			
			found = str_buffer.find("RQT");
			found1 = str_buffer.find(",,");
			found2 = str_buffer.find("!!");
			found3 = str_buffer.find("==");
			found4 = str_buffer.find("++");

			//std::cout<<"<<<<<<<<" + str_buffer.substr(found1+2,(found2-found1)-2) + ">>>>>>>>>>>>>";

			msg.encoder_read1 = strtof((str_buffer.substr(found+3,(found1-found)-3)).c_str(),0);
			msg.setpoint1 = strtof((str_buffer.substr(found1+2,(found2-found1)-2)).c_str(),0);

			msg.encoder_read2 = strtof((str_buffer.substr(found2+2,(found3-found2)-2)).c_str(),0);
			msg.setpoint2 = strtof((str_buffer.substr(found3+2,(found4-found3)-2)).c_str(),0);

			std::cout<<"<<<<<<<<" + str_buffer.substr(found2+2,(found3-found2)-2) + ">>>>>>>>>>>>>\n";
			std::cout<<"<<<<<<<<" + str_buffer.substr(found3+2,(found4-found3)-2) + ">>>>>>>>>>>>>";
		}
		// publish message to subscribe on RQT PLOT
		chatter_pub.publish(msg);
		//======================
	    }

	 	
	    ros::spinOnce();
	}
	
	return 0;
}
