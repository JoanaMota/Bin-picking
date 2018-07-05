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
#include "robonuc/alarm.h"
#include "r_platform/navi.h"

    r_platform::navi com;


//Define of the Plataform Position and Orientation
float sx = 0.000;
float sy = 0.000;
float sz = 0.75;
float sr = 1.571;

bool robot_allowed=true;

std::string s4, s5;
std::string chr4 ,chr5;

using namespace tf;

robonuc::alarm ala;
robonuc::platpos msg;

int first_time = 0;

class PlatformSim
{
public:
	tf::Transform t1;
	tf::TransformListener listener;



	PlatformSim()
	{

		subscriber_teclado = n.subscribe("chatter", 1000, &PlatformSim::ReceberTeclado, this);
		subscriber_posicao = n.subscribe("platmov", 1000, &PlatformSim::ReceberPosicao, this);
		subscriber_seg = n.subscribe("alarm", 1000, &PlatformSim::Seguranca, this);
		toframe_pub = n.advertise<robonuc::platpos>("toframe", 1000);
		client = n.serviceClient<robonuc::robonuc_com>("servcom");
		joy_sub= n.subscribe<r_platform::navi>("navi_commands", 1000, &PlatformSim::Comanda, this);
		
		t1.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		t1.setRotation(q); 

	}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função que lê o estado da parametrização de segurança: quais os lasers a usar para segurança e se a segurança com base nos lasers está ativa
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	void SafetyParam (void)
	{

		if (n.getParam("/safetyl0", s4))
		{
			chr4 = const_cast<char*> (s4.c_str());
			ROS_INFO("Got param /safetyl0: %s", s4.c_str());
		}else{
			ROS_ERROR("Failed to get param '/safetyl0'");
		}

		if (n.getParam("/safetyl1", s5))
		{
			chr5 = const_cast<char*> (s5.c_str());
			ROS_INFO("Got param /safetyl1: %s", s5.c_str());
		}else{
			ROS_ERROR("Failed to get param '/safetyl1'");
		}


	}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Callback de receção dos comandos do comando da xBox
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	void Comanda (const r_platform::navi msg)
	{

			if(msg.linear_vel > 0.05){
				SystemMovement(65,msg.linear_vel);
				if(msg.angular_vel>0.05){
					SystemMovement(68,msg.angular_vel);
				}else if(msg.angular_vel<-0.05){
					SystemMovement(67,-msg.angular_vel);
				}
			}else if(msg.linear_vel < -0.05){
				SystemMovement(66,-msg.linear_vel);
				if(msg.angular_vel>0.05){
					SystemMovement(68,msg.angular_vel);
				}else if(msg.angular_vel<-0.05){
					SystemMovement(67,-msg.angular_vel);
				}

			}else if(msg.angular_vel>0.05){
				SystemMovement(68,msg.angular_vel);
			}else if(msg.angular_vel<-0.05){
				SystemMovement(67,-msg.angular_vel);
			}

		if( robot_allowed && msg.robot>0){

			ROS_INFO("Command Received on Platform_Sim from joy"); 

			robot_allowed=false;

			if (first_time == 0 )
			{ 
				SafetyParam();
				first_time++;
			}
		
			int f_robot = msg.robot; 

			robonuc::robonuc_com srv;
			
			if(msg.robot==5){

				srv.request.reqtofanuc = 6;

			}else{
				srv.request.reqtofanuc = 1;
				srv.request.posnumber = msg.robot;
			}


			if (client.call(srv))
			{
				ROS_INFO("Response received on platform_sim - %ld", (long int)srv.response.respfromfanuc);			
			}
			else
			{
				ROS_ERROR("Failed to call service robonuc_com");		
			}

			robot_allowed=true;
		}

	}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função de cálculo da posição relativa do sistema e envio dessa posição para o nodo vs_frame
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	void SystemMovement(int command, float scale)	
	{
		float displacement, turn_angle;

		if(scale==1){
			displacement = 0.1;
			turn_angle = M_PI/10;
		}else{
			displacement = 0.075*scale;
			turn_angle = scale*M_PI/13.5;
		}

		Transform t_mov;
		Quaternion q;

		switch (command) {
		    case 65:
			t_mov.setOrigin( tf::Vector3(displacement, 0.0, 0.0) );
		  	q.setRPY(0, 0, 0);
		  	t_mov.setRotation(q);
			break;
		    case 66:
			t_mov.setOrigin( tf::Vector3(-displacement, 0.0, 0.0) );
		  	q.setRPY(0, 0, 0);
		  	t_mov.setRotation(q);
			break;
		    case 67:
			t_mov.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		  	q.setRPY(0, 0, -turn_angle);
		  	t_mov.setRotation(q);
			break;
		    case 68:
			t_mov.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		  	q.setRPY(0, 0, turn_angle);
		  	t_mov.setRotation(q);
			break;
		    default:

			break;
		}

		Transform t;

		t = t1*t_mov;
		t1 = t;

		float x = t.getOrigin().x();
		float y = t.getOrigin().y();
		double roll, pitch, yaw;
		t1.getBasis().getRPY(roll, pitch, yaw);

		msg.x = x;
		msg.y = y;
		msg.w = yaw;		
		toframe_pub.publish(msg);


	}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Callback acionada pela receção dos comandos traduzidos pelas setas do teclado em modo de navegação (8)
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	void ReceberPosicao (const robonuc::platmov mmsg)
	{
		
		SystemMovement(mmsg.dir,1);
		
	}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Callback de encaminhamento dos comandos do teclado para o nodo vs_fanuc_client
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	void ReceberTeclado (const robonuc::conjunto mymsg)
	{
		if(robot_allowed){

			robot_allowed=false;

			if (first_time == 0 )
			{ 
				SafetyParam();
				first_time++;
			}
		
			int com_id = mymsg.keyboard.msg_id; 
			int pos_id = mymsg.keyboard.pos_id;
			int io = mymsg.keyboard.io_num;
			float x = mymsg.keyboard.x;
			float y = mymsg.keyboard.y;
			float z = mymsg.keyboard.z;
			float j1 = mymsg.keyboard.j1;
			float j2 = mymsg.keyboard.j2;
			float j3 = mymsg.keyboard.j3;
			float j4 = mymsg.keyboard.j4;
			float j5 = mymsg.keyboard.j5;
			float j6 = mymsg.keyboard.j6;

			ROS_INFO("Command Received on Platform_Sim - %d",com_id); 

			robonuc::robonuc_com srv;

			srv.request.reqtofanuc = com_id;
			srv.request.posnumber = pos_id;
			srv.request.ionumber = io;
			srv.request.x = x;
			srv.request.y = y;
			srv.request.z = z;
			srv.request.j1 = j1;
			srv.request.j2 = j2;
			srv.request.j3 = j3;
			srv.request.j4 = j4;
			srv.request.j5 = j5;
			srv.request.j6 = j6;
			srv.request.sx = sx;
			srv.request.sy = sy;
			srv.request.sz = sz;
			srv.request.sr = sr;

			if (client.call(srv))
			{
				ROS_INFO("Response received on platform_sim - %ld", (long int)srv.response.respfromfanuc);			
			}
			else
			{
				ROS_ERROR("Failed to call service robonuc_com");
			}

			robot_allowed=true;
		}

	}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Callback acionada pela receção do tópico publicado pelo nodo de verificação dos dados dos lasers
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	void Seguranca (const robonuc::alarm mala)
	{
//		Em função do estado de alarme recebido pelo tópicoe, e se a parametrização da segurança estiver configurada para se utilizar o laser0 como dispositivo de segurança, os parametros de larme são alterados
		if (mala.alalaser0 == 1 && chr4.compare("1") == 0){
			n.setParam("ala0", "1"); //Alarme ON
			if(mala.sever0 == 2 ){ //Severidade Elevada
				n.setParam("ala_sev0", "2");
			}else{
				n.setParam("ala_sev0", "1");  
			}
		}else{
			n.setParam("ala0", "0");
			n.setParam("ala_sev0", "0");
		}

		if (mala.alalaser1 == 1  && chr5.compare("1") == 0){
			n.setParam("ala1", "1"); //Alarme ON
			if(mala.sever1 == 2 ){  //Severidade Elevada
				n.setParam("ala_sev1", "2");
			}else{
				n.setParam("ala_sev1", "1");
			}
		}else{
			n.setParam("ala1", "0");
			n.setParam("ala_sev1", "0");
		}
	}


protected:
	ros::NodeHandle n;
	ros::Subscriber subscriber_teclado;
	ros::Subscriber subscriber_posicao;
	ros::Subscriber subscriber_seg;
	ros::ServiceClient client;
	ros::Publisher toframe_pub;
	ros::Subscriber joy_sub;
};

int main(int argc, char **argv)
{
	//ROS init
        ros::init (argc, argv, "vs_platform_sim");
	 
	//Call Constructor
	PlatformSim platformsim;

	//MultiThreadedSpinner para actualizar parametros enquanto as callbacks estaõ a executar a rotina
	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	spinner.spin();

	return 0;

}

