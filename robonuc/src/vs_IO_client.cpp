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
#include "robonuc/conjunto.h"
#include <cstdlib>
#include <iostream>
#include <string>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <wchar.h>

using namespace std;

robonuc::ios msg; 

int sockfd, portno, n;
bool echoMode = false;

void error(const char *msg) {
    perror(msg);
    exit(0);
}

class Listener 
{
private:

	char topic_message[256];

public:
	Listener()
	{
		client_sub = nh.subscribe("io_client_messages", 1000, &Listener::callback, this);
	}

	void callback(const robonuc::ios msg) 
	{

    		char buffer[256];

//		Shift no código para todas as funções e i/o corresponderem a um char possível de codificar através da tabela ascii
		int code_io = msg.code + 21;
		ROS_INFO("INO Code %d",code_io-21);


		char c = (char)code_io;
		const char *d = &c;

		ROS_INFO("Char Codificado: %c ",c);

		    bzero(buffer,256);

		    strcpy(buffer, d);

		    n = write(sockfd,buffer,1);
		    if (n < 0) 
			 error("ERROR writing to socket");

		    if (code_io<40 & code_io>30) {

			    bzero(buffer, 256);
			    n = read(sockfd,buffer,255);
			    if (n < 0)
					error("ERROR reading reply");
			    ROS_WARN("%s",buffer);
		    }

	}

	char* getMessageValue();

private:
	ros::NodeHandle nh;
	ros::Subscriber client_sub;
};


int main(int argc, char *argv[]) {
	
	// ROS node declaration
	ros::init(argc, argv, "TCPclient_node_io");

	struct sockaddr_in serv_addr;
	struct hostent *server;
	portno = 50000;

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
    
	if (sockfd < 0) 
		error("ERROR opening socket");
	server = gethostbyname("192.168.0.50");

	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, 
	(char *)&serv_addr.sin_addr.s_addr,
	server->h_length);
	serv_addr.sin_port = htons(portno);
    
	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
		error("ERROR connecting");

	cout << "Connected!!" << endl;

//	Constructor
	Listener listener;	
	
	ros::spin();
	
	return 0;
}
