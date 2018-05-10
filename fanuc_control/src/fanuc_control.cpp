#define _MAIN_
#include "../include/fanuc_control/fanuc_control.h"
#include "../../bin_picking/include/bin_picking/header_pcl.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
float X, Y, Z, normal_X, normal_Y, normal_Z;


void centroid (const geometry_msgs::Vector3 centroid_robot_base)
{
    X = centroid_robot_base.x;
    Y = centroid_robot_base.y;
    Z = centroid_robot_base.z;

    cout << "X: " << X << endl;
    cout << "Y: " << Y << endl;
    cout << "Z: " << Z << endl;
}

void normal (const geometry_msgs::Vector3 normal_robot_base)
{
    normal_X = normal_robot_base.x;
    normal_Y = normal_robot_base.y;
    normal_Z = normal_robot_base.z;

    // cout << "W: " << normal_X << endl;
    // cout << "P: " << normal_Y << endl;
    // cout << "R: " << normal_Z << endl;
}

int main (int argc, char* argv[])
{
    int result;             /** Checks the return value from the function
                            check_for_input(argc,argv);
                            If the result == -1, then, the node will shutdown **/
    
    // this function will check for terminal arguments //
    result = check_for_input(argc,argv);
    if(result == -1)
        return 0;
 
    ros::init(argc , argv , "fanuc_control");

    ros::NodeHandle n;
    // Subscribe centroid in relation to the base of the Robot published by the pointTFtransfer.cpp
    ros::Subscriber sub_centroid = n.subscribe<geometry_msgs::Vector3> ("/centroid_in_robot_base", 1, centroid);
    // Subscribe normal in relation to the base of the Robot published by the pointTFtransfer.cpp
    ros::Subscriber sub_normal = n.subscribe<geometry_msgs::Vector3> ("/normal_in_robot_base", 1, normal);
    
    ros::Publisher fanuc_cart  = n.advertise<std_msgs::String>("fanuc_cart", 1000);
    ros::Publisher fanuc_joint = n.advertise<std_msgs::String>("fanuc_joint", 1000);


    ros::Rate loop_rate(20);

    // dataReceiver data;

    try
    {

        boost::asio::io_service io_service;
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(Global_IP,Global_Port);
        tcp::resolver::iterator iterator = resolver.resolve(query);


        // Communication Initiation //
        // robotCom c(io_service, iterator); // Necessary in included file!!

        char tppname[1024] = "PARIJOANA";

        robotCom c(io_service, iterator);
        // c.runtpp(tppname);
        
        //MOVE TO JOINTS - Inicial position 
        // c.movtojpos("-0.000 0.000 -0.000 0.000 -90.000 -0.000 1 40 1");
        
        //MOVE TO POINT -- for Kinect to acquire point cloud
        // c.movtocpos("420.000 0.000 280.000 -179.996 0.013 -1.003 0 1 1 0 0 0 1 50 1");
        stringstream ss_input_arg;

        //ATENCAO CONVERTER PARA A PONTA DO ROBOT
        // ss_input_arg << X << Y << Z << "-179.996 0.013 -0.003 0 1 1 0 0 0 1 10 1";
        // string input_arg = ss_input_arg.str();
        // c.movtocpos(input_arg);
        //130.188-162.5

        //MENOS 23 NO EIXO DO X

        //Obj 1-- azul direita 2
        // c.movtocpos("424.652 188.668 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        // c.movtocpos("401.652 188.668 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        //Obj 2
        // c.movtocpos("535.592 188.57 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        // c.movtocpos("512.592 188.57 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        //Obj 3
        // c.movtocpos("424.632 -3.93306 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        // c.movtocpos("401.632 -3.93306 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        //Obj 4
        // c.movtocpos("313.645 196.748 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        //Obj 5
        // c.movtocpos("626.401 180.54 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        //Obj 6
        // c.movtocpos("320.416 -4.74655 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        //Obj 7
        // c.movtocpos("528.921 -18.2747 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        //Obj 8
        // c.movtocpos("627.086 -13.5831 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");
        
        // c.getcrcpos();
        c.movtocpos("382.62 163.825 -136 -179.996 0.013 -0.003 0 1 1 0 0 0 1 40 1");




        std_msgs::String value_cart;
        std_msgs::String value_joint;

        while(ros::ok())
        {
            ros::spinOnce();

            value_cart.data = "";
            value_joint.data = "";

            value_cart.data = c.getcrcpos();
            value_joint.data = c.getcrjpos();

            cout << value_cart.data << endl;

            fanuc_cart.publish(value_cart);
            fanuc_joint.publish(value_joint);

            loop_rate.sleep();
        }
        c.motionstop();
        c.close();

    }catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    ros::spinOnce();

    return 0;
}



string Int2Str(int a)
{
    ostringstream temp;
    temp<<a;
    return temp.str();
}
string Int2Str(double a)
{
    ostringstream temp;
    temp<<a;
    return temp.str();
}
string Cnvert2CSV(string tmp)
{
    vector <string> fields, final;
    string tmp1 = "", result = "";
    bool comma = false;

    // Rearranges the string for the first time (substitues " " for ", ") //
    split( fields, tmp, boost::is_any_of( " " ) );
    for(size_t i=0; i<fields.size(); i++)
    {
        if(fields[i] != "")
        {
            if(fields[i] == "N" || fields[i] == "D" || fields[i] == "B" || fields[i] == "N," || fields[i] == "D," || fields[i] == "B,")
                fields[i] = "0";
            else if(fields[i] == "T" || fields[i] == "U" || fields[i] == "F" || fields[i] == "T," || fields[i] == "U," || fields[i] == "F,")
                fields[i] = "1";

            if(comma)
                tmp1 += ", " + fields[i];
            else
            {
                tmp1 += fields[i];
                comma = true;
            }
        }
    }

    // Rearranges the string for the second time (substitues ",, " for ", ") //
    split( final, tmp1, boost::is_any_of( ",, " ) );
    result += final[0];
    for(size_t i=1; i<final.size(); i++)
        if(final[i] != "")
            result += ", " + final[i];


    return result;
}