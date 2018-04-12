#define _MAIN_
#include "fanuc_control.h"



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
        
        //MOVE TO JOINTS 
        c.movtojpos("-0.000 0.000 -0.000 0.000 -90.000 -0.000 0 10 1");

        std_msgs::String value_cart;
        std_msgs::String value_joint;

        while(ros::ok())
        {
            ros::spinOnce();

            
            value_cart.data = "";
            value_joint.data = "";

            value_cart.data = c.getcrcpos();
            value_joint.data = c.getcrjpos();

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

    while(ros::ok())
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