#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/PoseArray.h>
#include <sstream>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
#include <moveit_msgs/GetPositionIK.h>
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
//#include <industrial_trajectory_filters/filter_base.h>
//#include <industrial_trajectory_filters/uniform_sample_filter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "robonuc/robonuc_com.h"
#include <cstdlib>
#include "robonuc/conjunto.h"

robonuc::conjunto mymsg;

struct RobotCoord 
{
	float x;
	float y;
	float z;
	float rw;
	float rx;
	float ry;
	float rz;
};

void id_position(moveit::planning_interface::MoveGroup &group, int pos_id);
void joints_position(moveit::planning_interface::MoveGroup &group, float j1, float j2, float j3,float j4, float j5, float j6);
void xyz_position(moveit::planning_interface::MoveGroup &group,float x, float y, float z);
void move_seg(moveit::planning_interface::MoveGroup &group);
RobotCoord getmanpos(moveit::planning_interface::MoveGroup &group);
RobotCoord getsyspos(moveit::planning_interface::MoveGroup &group, float platx , float platy , float platz , float platr); 
void pick(moveit::planning_interface::MoveGroup &group,double x,double y, double z);
void monitoring_ios(int function, int ionumber);
void verify_seg (void);
void insert_laser(moveit::planning_interface::MoveGroup &group);

class FanucCommands
{
public:
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<robonuc::conjunto>("ar_client_messages", 1000);
	ros::ServiceServer ss = n.advertiseService("servcom", &FanucCommands::platform_callback, this);
	bool platform_callback(robonuc::robonuc_com::Request  &req , robonuc::robonuc_com::Response &res);
	ros::Publisher pub_co = n.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função de movimento do manipulador para posições pré-definidas e bastante usuais
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void id_position(moveit::planning_interface::MoveGroup &group, int pos_id)
{
  
	double max_velocity_scaling_factor=0.11;
	double max_acceleration_scaling_factor=1;	

	group.setMaxVelocityScalingFactor (max_velocity_scaling_factor);

	std::map<std::string, double> joints;

	switch (pos_id) {
	    case 1:	//Zero hardware
		joints["joint_1"] = 0.00;
		joints["joint_2"] = 0.00;
		joints["joint_3"] = 0.00;
		joints["joint_4"] = 0.00;
		joints["joint_5"] = 0.00;
		joints["joint_6"] = 0.00;
		ROS_INFO("Home Position Selected ID - %d",pos_id); 
		break;
	    case 2:	//Zero harware com 5a junta a 90 graus
		joints["joint_1"] = 0.00;
		joints["joint_2"] = 0.00;
		joints["joint_3"] = 0.00;
		joints["joint_4"] = 0.00;
		joints["joint_5"] = -1.571;
		joints["joint_6"] = 0.00;
		ROS_INFO("Ready Position Selected ID - %d",pos_id); 
		break;
	    case 3:	//Posição de navegação
		joints["joint_1"] = 0.00;
		joints["joint_2"] = -1.571;
		joints["joint_3"] = -0.75;
		joints["joint_4"] = 0.00;
		joints["joint_5"] = -0.82;
		joints["joint_6"] = 0.00;
		ROS_INFO("Travel Position Selected ID - %d",pos_id); 
		break;
	    case 4:	//Posição default para pick de objetos na superfície
		joints["joint_1"] = 0.00;
		joints["joint_2"] = -0.779;
		joints["joint_3"] = 2.992;
		joints["joint_4"] = 0.00;
		joints["joint_5"] = 0.9364;
		joints["joint_6"] = 3.14159;
		ROS_INFO("Back Position Selected ID - %d",pos_id); 
		break;
	    default:
		ROS_WARN("Unrecognized Request ID"); 
		    break;
	}

//	double valores;
//	valores=group.getPlanningTime ();
//	std::cout<<"TIME,TIME="<<valores<<"\n";
		  
	group.setJointValueTarget(joints);

	move_seg(group);

}


//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função de movimento do manipuladorpara um valor espeçífico no espaço de juntas
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void joints_position(moveit::planning_interface::MoveGroup &group, float j1, float j2, float j3,float j4, float j5, float j6)
{
  
	double max_velocity_scaling_factor=0.11;

	group.setMaxVelocityScalingFactor (max_velocity_scaling_factor);

	std::map<std::string, double> joints;

	joints["joint_1"] = j1;
	joints["joint_2"] = j2;
	joints["joint_3"] = j3;
	joints["joint_4"] = j4;
	joints["joint_5"] = j5;
	joints["joint_6"] = j4;
	ROS_INFO("Calculating Joint Position");

//	double valores;
//	valores=group.getPlanningTime ();
//	std::cout<<"TIME,TIME="<<valores<<"\n";
		  
	group.setJointValueTarget(joints);

	move_seg(group);

}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função de movimento do manipuladorpara um valor espeçífico no espaço cartesiano
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void xyz_position(moveit::planning_interface::MoveGroup &group,float x, float y, float z)
{
	group.setEndEffectorLink("tool0");



	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//sleep(4.0);

////******************************************************************************************************
////Planning to a Pose goal
////******************************************************************************************************
// We can plan a motion for this group to a desired pose for the end-effector.
	geometry_msgs::Pose target_pose1;

	target_pose1.position.x = x;
	target_pose1.position.y = y;
	target_pose1.position.z = z;

	//Orientation
	double angle =M_PI;
	Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
    
	target_pose1.orientation.x = quat.x();
	target_pose1.orientation.y = quat.y();
	target_pose1.orientation.z = quat.z();
	target_pose1.orientation.w = quat.w();
  
	group.setPoseTarget(target_pose1);


// Now, we call the planner to compute the plan and visualize it.
// Note that we are just planning, not asking move_group to actually move the robot.
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);
 

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"SUCCESS":"FAILED"); 

	move_seg(group);

}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função que disputa o movimento do robô com verificação do estado da segurança
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void move_seg(moveit::planning_interface::MoveGroup &group)
{
	std::string s0;
	const char* chr0;

	std::string s1;
	const char* chr1;

	std::string s3;
	const char* chr3;

//	Leitura do parâmetro de ativação da segurança associada aos lasers
	if (n.getParam("/robonuc_safety", s3))
	{
		chr3 = s3.c_str();
			ROS_INFO("Got param Safety: %s", s3.c_str());
	}else{
		ROS_ERROR("Failed to get param '/robonuc_safety'");
	}

//	Verificação se a segurança associada aos lasers está ativa
	if (*chr3 == 49){
	
//		Acesso aos parametros com a severidade do alarme gerado pelos lasers
//		Laser0 - Atrás da plataforma URG
		if (n.getParam("/ala0", s0))
		{
			chr0 = s0.c_str();
			ROS_INFO("Got param0: %s", s0.c_str());
		}else{
			ROS_ERROR("Failed to get param '/ala0'");
		}

//		Laser1 - A frente da plataforma UTM
		if (n.getParam("/ala1", s1))
		{
			chr1 = s1.c_str();
				ROS_INFO("Got param1: %s", s1.c_str());
		}else{
			ROS_ERROR("Failed to get param '/ala1'");
		}

//		Se a severidade for "1" o manipulador vai mover-se devagar
//		É necessário meter um input (DI109) do controlador a 1
		if(*chr0==49 | *chr1==49) monitoring_ios(2, 4);

//		Mais uma verificação do estado de alarme para o caso da severidade ser elevada (1)
//		O manipulador ficará à espera que o obstáculo saia da frente do laser para prosseguir
		verify_seg ();

//		Finalmente o robô pode-se mover
		group.move();

//		Aqui volta-se a meter o input 109 a 0
		if(*chr0==49 | *chr1==49) monitoring_ios(3, 4);

	}else{
//		Caso não haja segurança ativa move-se logo
		group.move();

	}

}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função que pede ao controlador que retorne a posição do manipulador
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
RobotCoord getmanpos(moveit::planning_interface::MoveGroup &group)
{
	group.setEndEffectorLink("tool0");
	
	geometry_msgs::PoseStamped robot_pose;
	robot_pose = group.getCurrentPose();

	geometry_msgs::Pose current_position;
	current_position = robot_pose.pose;

	/*Retrive position and orientation */
	geometry_msgs::Point exact_pose = current_position.position;
	geometry_msgs::Quaternion exact_orientation = current_position.orientation;

	RobotCoord actualpos;

	actualpos.x = exact_pose.x;
	actualpos.y = exact_pose.y;
	actualpos.z = exact_pose.z;
	actualpos.rw = exact_orientation.w;
	actualpos.rx = exact_orientation.x;
	actualpos.ry = exact_orientation.y;
	actualpos.rz = exact_orientation.z;

	return {actualpos};

}


//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função que pede ao controlador que retorne a posição do robô para fazer o cálculo da posição da ponta do sistema robótico no referencial global
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
RobotCoord getsyspos(moveit::planning_interface::MoveGroup &group, float platx , float platy , float platz , float platr)
{
	group.setEndEffectorLink("tool0");
	
	geometry_msgs::PoseStamped robot_pose;
	robot_pose = group.getCurrentPose();

	geometry_msgs::Pose current_position;
	current_position = robot_pose.pose;

	/*Retrive position and orientation */
	geometry_msgs::Point exact_pose = current_position.position;
	geometry_msgs::Quaternion exact_orientation = current_position.orientation;

	RobotCoord actualpos;

	actualpos.x = platx + cos(platr)*exact_pose.x + sin(platr)*exact_pose.y;
	actualpos.y = platy + sin(platr)*exact_pose.x + cos(platr)*exact_pose.y;
	actualpos.z = exact_pose.z + platz;
	actualpos.rw = exact_orientation.w;
	actualpos.rx = exact_orientation.x;
	actualpos.ry = exact_orientation.y;
	actualpos.rz = exact_orientation.z;

	return {actualpos};

}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função de envio de um tópico par ao nodo responsável por alterar o estado dos I/='s do robo
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void monitoring_ios(int function, int ionumber)
{	
//	Codificação do valor: a função pode ser ler(1), ligar(2) ou desligar(3) um I/O
	int cod = function*10 + ionumber;
	mymsg.ios.code = cod;


	ROS_INFO("Setting IOs - code: %d",cod); 
	chatter_pub.publish(mymsg);
}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função que insere à cena de planeamento uma caixa verde para detecção de colisão
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void insert_laser(moveit::planning_interface::MoveGroup &group)
{	

	group.setEndEffectorLink("tool0");

	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = "base_link";

	co.id = "table";
	co.operation = moveit_msgs::CollisionObject::REMOVE;
	pub_co.publish(co);

	co.operation = moveit_msgs::CollisionObject::ADD;
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.073;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.073;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.100;
	co.primitive_poses.resize(1);
	co.primitive_poses[0].position.x = 0.135;
	co.primitive_poses[0].position.y = 0;
	co.primitive_poses[0].position.z = 0.051;
	co.primitive_poses[0].orientation.w = 0;
	pub_co.publish(co);

}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
/**
 *     @brief  Função que verifica o estado de segurança e caso corresponda à severidade elevada fica em ciclo contínuo à espera que deixe de estar em alarme para que o manipulador possa prosseguir com o movimento
 */ 
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void verify_seg (void)
{
	std::string s0, s1;

	const char* chr0;
	const char* chr1;

	do{			
		if (n.getParam("/ala_sev0", s0))
		{
			chr0 = s0.c_str();
//			ROS_INFO("Got param: %s", s.c_str());
		}else{
			ROS_ERROR("Failed to get param '/ala_sev0'");
		}

		if (n.getParam("/ala_sev1", s1))
		{
			chr1 = s1.c_str();
//			ROS_INFO("Got param: %s", s.c_str());
		}else{
			ROS_ERROR("Failed to get param '/ala_sev1'");
		}

	}while( *chr0 == 50 | *chr1 == 50 );
}

protected:

};


bool FanucCommands::platform_callback(robonuc::robonuc_com::Request  &req , robonuc::robonuc_com::Response &res)
{
	int request_id;
	int pos_id,io;
	float x,y,z;
	float j1,j2,j3,j4,j5,j6;
	float platx,platy,platz,platr;

	request_id = req.reqtofanuc;
	pos_id = req.posnumber;
	io = req.ionumber;
	x = req.x;
	y = req.y;
	z = req.z;
	j1 = req.j1;
	j2 = req.j2;
	j3 = req.j3;
	j4 = req.j4;
	j5 = req.j5;
	j6 = req.j6;
	platx = req.sx;
	platy = req.sy;
	platz = req.sz;
	platr = req.sr;
	

	moveit::planning_interface::MoveGroup group("manipulator");

	insert_laser(group);

	if(request_id==1){

		id_position( group, pos_id);
		ROS_INFO("Responce sent from fanuc - %d",request_id); 
		res.respfromfanuc = request_id;

	}else if(request_id==2){

		xyz_position(group, x, y, z);
		ROS_INFO("Responce sent from fanuc - %d",request_id); 
		res.respfromfanuc = request_id;

	}else if(request_id==3){

		joints_position( group, j1,j2,j3,j4,j5,j6);
		ROS_INFO("Responce sent from fanuc - %d",request_id); 
		res.respfromfanuc = request_id;

	}else if(request_id==4){
	
		RobotCoord pos = getmanpos(group);
		ROS_INFO("Manip.xyz %1.3f , %1.3f , %1.3f",pos.x,pos.y,pos.z);
		ROS_INFO("Manip.wxyz %1.3f , %1.3f , %1.3f , %1.3f",pos.rw,pos.rx,pos.ry,pos.rz);
		ROS_INFO("Responce sent from fanuc - %d",request_id);
		res.respfromfanuc = request_id;

	}else if(request_id==5){

		RobotCoord syspos = getsyspos(group, platx , platy , platz , platr);
		ROS_INFO("Syst.xyz %1.3f , %1.3f , %1.3f",syspos.x,syspos.y,syspos.z);
		ROS_INFO("Syst.wxyz %1.3f , %1.3f , %1.3f , %1.3f",syspos.rw,syspos.rx,syspos.ry,syspos.rz);
		ROS_INFO("Responce sent from fanuc - %d",request_id);
		res.respfromfanuc = request_id;

	}else if(request_id==6){

		ROS_INFO("Running Demo 0!!");
		id_position( group, 2);
		id_position( group, 4);
		xyz_position(group, -0.45, 0.048, 0.102);
		xyz_position(group, -0.45, 0.048, 0.052);
		monitoring_ios(2, 8);
		xyz_position(group, -0.45, 0.048, 0.120);
		id_position( group, 2);
		monitoring_ios(3, 8);
		id_position( group, 4);
		xyz_position(group, -0.45, 0.0, 0.102);
		xyz_position(group, -0.45, 0.0, 0.052);
		monitoring_ios(2, 8);
		xyz_position(group, -0.45, 0.0, 0.120);
		id_position( group, 2);
		monitoring_ios(3, 8);
		id_position( group, 4);
		xyz_position(group, -0.45, -0.048, 0.102);
		xyz_position(group, -0.45, -0.048, 0.052);
		monitoring_ios(2, 8);
		xyz_position(group, -0.45, -0.048, 0.102);
		id_position( group, 2);
		monitoring_ios(3, 8);
		id_position( group, 3);
		res.respfromfanuc = request_id;

		//xyz_position(group, -0.403, -0.0, 0.102); //Stots do lado do robo

	}else if(request_id==7){

		monitoring_ios(pos_id, io);
		ROS_INFO("Fanuc Monitoring IOS");
		res.respfromfanuc = request_id;

	}
	return true;

}





//******************************************************************************************************
//MAIN
//******************************************************************************************************

int main(int argc, char **argv)
{
//	ROS init
	ros::init (argc, argv, "vs_fanuc_client");

//	start a multithreaded background "spinner", so our node can process ROS messages
//	- this lets us know when the move is completed
	ros::AsyncSpinner spinner(2);
	spinner.start();

	FanucCommands a;
	
	ros::waitForShutdown();

    return 0;

}

