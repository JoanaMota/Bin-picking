#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "robonuc/conjunto.h"

using namespace tf;

//Posição inicial do sistema enquanto não chega nenhum tópico com a sua posição do nodo vs_platform_sim 
float x = 0.0;
float y = 0.0;
float w = 0.0;

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Callback associa à receção de um tópico do nodo vs_platform_sim com a posição atual do sistema
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void PosCallback(const robonuc::platpos msg)
{
	x = msg.x;
	y = msg.y;
	w = msg.w;
}


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "my_tf_broadcaster");
	ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("toframe", 1000, PosCallback);

	tf::TransformBroadcaster br0;
	tf::Transform transform0;

	tf::TransformBroadcaster br1;
	tf::Transform transform1;

	tf::TransformBroadcaster br2;
	tf::Transform transform2;

	ros::Rate rate(10.0);

	while (node.ok()){

//		Publicação da tf do referencial do manipulador para o laser 1 da frente da plataforma UTM
		transform0.setOrigin( tf::Vector3(0.120, 0.0, 0.06) ); //Meters
		transform0.setRotation( tf::Quaternion(0, 0, 0, 1) );
		br0.sendTransform(tf::StampedTransform(transform0, ros::Time::now(), "base_link", "laser1"));

//		Publicação da tf do referencial do manipulador para o laser 0 atrás da plataforma URG
		transform1.setOrigin( tf::Vector3(-0.630, 0.0, -0.250) ); //Meters
		transform1.setRotation( tf::Quaternion(0, 0, 1, 0) );
		br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "base_link", "laser0"));

//		Publicação da tf do referencial global para o referencial do manipulador
		transform2.setOrigin( tf::Vector3(x, y, 0.75) ); //Meters
		Quaternion q2;
		q2.setRPY(0, 0, w);
		transform2.setRotation( q2 );
		br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "base_link"));

		rate.sleep();
		ros::spinOnce();
	}

	return 0;
};




