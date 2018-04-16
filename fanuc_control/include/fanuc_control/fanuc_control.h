/*
=============================================================================================
=============================================================================================
    ooo        ooooo       .o.         .oooooo.   ooooooooo.     .oooooo.    .oooooo..o 
    `88.       .888'      .888.       d8P'  `Y8b  `888   `Y88.  d8P'  `Y8b  d8P'    `Y8 
     888b     d'888      .8"888.     888           888   .d88' 888      888 Y88bo.      
     8 Y88. .P  888     .8' `888.    888           888ooo88P'  888      888  `"Y8888o.  
     8  `888'   888    .88ooo8888.   888           888`88b.    888      888      `"Y88b 
     8    Y     888   .8'     `888.  `88b    ooo   888  `88b.  `88b    d88' oo     .d8P 
    o8o        o888o o88o     o8888o  `Y8bood8P'  o888o  o888o  `Y8bood8P'  8""88888P'  
=============================================================================================
=============================================================================================
This is the list of Macros used in the script. It is possible that changes need to be made in
the future.
/*******************************************************************************************/
/*******************************************************************************************/
/***/                                                                                   /***/
/***/#define FANUC_SMALL    "192.168.0.231"     /* IP Adress of Small Fanuc             /***/
/***/#define FANUC_BIG      "192.168.0.230"     /* IP Adress of Big Fanuc               /***/
/***/#define PORT           "4900"              /* Serial Port to communicate           /***/
/***/                                                                                   /***/
/*******************************************************************************************/
/*******************************************************************************************/
/*
=============================================================================================
=============================================================================================
          o8o                        oooo                    .o8                    
          `"'                        `888                   "888                    
         oooo  ooo. .oo.    .ooooo.   888  oooo  oooo   .oooo888   .ooooo.   .oooo.o
         `888  `888P"Y88b  d88' `"Y8  888  `888  `888  d88' `888  d88' `88b d88(  "8
          888   888   888  888        888   888   888  888   888  888ooo888 `"Y88b. 
          888   888   888  888   .o8  888   888   888  888   888  888    .o o.  )88b
         o888o o888o o888o `Y8bod8P' o888o  `V88V"V8P' `Y8bod88P" `Y8bod8P' 8""888P'
=============================================================================================
=============================================================================================
*/
#include <ros/ros.h>                        /** Necessary to use ros                      **/
#include "std_msgs/String.h"                /** To publish strings                        **/
#include <string.h>                         /** Allows the usage of strcmp function       **/
#include <string>                           /** Defines the string variable type          **/
#include <sstream>                          /** This include allows the use of ostringstream
                                            which allows the conversion from int to string**/
#include <boost/asio.hpp>                   /** For TCP/IP communication                  **/
#include <boost/asio/ip/tcp.hpp>            /** For TCP/IP communication                  **/
#include <boost/algorithm/string.hpp>       /** More string functions (to rearrange)      **/
#include <vector>                           /** Allows the use of vectors                 **/

// #include <imu_network/sensors_network.h>    /** Allows to subscribe to topic_raw_data     **/
// #include <imu_network/filtered_imu_network.h> /** Allows to subscribe to topic_filtered   **/

#include <geometry_msgs/Point.h>            /** To read the points taken from the track_arrow
                                            node from the arrow_detection package **/

using namespace std;                        /** Defines the basic namespace as std        **/
using boost::asio::ip::tcp;
/*
=============================================================================================
=============================================================================================
    .o88o.                                       .    o8o                                 
    888 `"                                     .o8    `"'                                 
   o888oo  oooo  oooo  ooo. .oo.    .ooooo.  .o888oo oooo   .ooooo.  ooo. .oo.    .oooo.o 
    888    `888  `888  `888P"Y88b  d88' `"Y8   888   `888  d88' `88b `888P"Y88b  d88(  "8 
    888     888   888   888   888  888         888    888  888   888  888   888  `"Y88b.  
    888     888   888   888   888  888   .o8   888 .  888  888   888  888   888  o.  )88b 
   o888o    `V88V"V8P' o888o o888o `Y8bod8P'   "888" o888o `Y8bod8P' o888o o888o 8""888P'
=============================================================================================
=============================================================================================
*/
/*********************/
/***   debug.cpp   ***/
/*********************/
/** These function print out the inputed string with the color in the function name. There is
an overload for each of them. If a function is called only with a string, it will print out 
the string with the color as bold, however if it is called with an int dummy (0,1,2,...) the
printed string will be slim (as opposed to bold).                                         **/
void red_txt(string msg);
void green_txt(string msg);
void yellow_txt(string msg);
void blue_txt(string msg);
void white_txt(string msg);

void red_txt(string msg, int dummy);
void green_txt(string msg, int dummy);
void yellow_txt(string msg, int dummy);
void blue_txt(string msg, int dummy);
void white_txt(string msg, int dummy);

void end_of_function(void);                 /** Final of a function (gives space)         **/
/*****************************/
/***   fanuc_control.cpp   ***/
/*****************************/
string Int2Str(int a);                      /** This function converts int to string      **/
string Int2Str(double a);                   /** Overload                                  **/
string Cnvert2CSV(string tmp);              /** Converts string from fanuc to string compatible
                                            with csv files (num1, num2, num3, ...)        **/
// void rawCallback(const imu_network::sensors_network::ConstPtr& msg); /** topic_raw_data  **/
// void filtCallback(const imu_network::sensors_network::ConstPtr& msg); /** filtered_data   **/
void print( vector <string> & v );  //TEMP
/*************************/
/***   inputData.cpp   ***/
/*************************/
int check_for_input(int argc, char* argv[]);/** This function checks the terminal input   **/
void terminal_help(void);                   /** This function shows the help menu         **/
void terminal_debug(void);                  /** Sets the debug global variables           **/
void manual_definition(void);               /** Sets the IP and PORT manually             **/
void detail_printing(void);                 /** Prints the details of the connection      **/


/************************/
/***   robotCom.cpp   ***/
/************************/
void thread_fanuc_command(void);            /** Will create the socket to connect to FANUC**/
/*
=============================================================================================
=============================================================================================
                         oooo                                                 
                         `888                                                 
                .ooooo.   888   .oooo.    .oooo.o  .oooo.o  .ooooo.   .oooo.o 
               d88' `"Y8  888  `P  )88b  d88(  "8 d88(  "8 d88' `88b d88(  "8 
               888        888   .oP"888  `"Y88b.  `"Y88b.  888ooo888 `"Y88b.  
               888   .o8  888  d8(  888  o.  )88b o.  )88b 888    .o o.  )88b 
               `Y8bod8P' o888o `Y888""8o 8""888P' 8""888P' `Y8bod8P' 8""888P' 
=============================================================================================
=============================================================================================
*/
/* Classes */
class Position                              /** class that will save the FANUC position  **/
{
  public:
    //MANDATORY!
    /***************************************************************************************/
    /***************************************************************************************/
    /***                                                                                 ***/
    /***    The variables listed below are perfected for cartesian type.                 ***/
    /***    However, if there is the need to use joint type coordinates, then there is   ***/
    /*** no need to use the redundacies variables, so the variables will mean the        ***/
    /*** following:                                                                      ***/
    /*** x->J1                              w ->J4                             conf1->J7 ***/
    /*** y->J2                              p ->J5                                       ***/
    /*** z->J3                              r ->J6                                       ***/
    /***                                                                                 ***/
    /***    For safety purposes the unused variables will have NULL value                ***/
    /***                                                                                 ***/
    /***************************************************************************************/
    /***************************************************************************************/
    double x, y, z;                         /** cartesian position (space)                **/
    double w, p, r;                         /** euler angles                              **/
    int conf1, conf2, conf3;                /** redundancies (FLIP/UP/FRONT)              **/
    int turn1, turn2, turn3;                /** redundancies (Joint4/Joint5/Joint6)       **/
    bool is_full;                           /** true if struct is full (help for shync)   **/
    //NOT MANDATORY
    int motionType;                         /** 0->Linear+Fine | 1->Joint+Fine            **/
    int motionSpeed;                        /** [1-4000] mm/sec (motionType == 0)
                                                [1-100]       % (motionType == 1)         **/
    int operationType;                      /** 0->synch       | 1->asynch                **/

    Position();                             /** constructor **/
    Position(double xtemp, double ytemp, double ztemp);
    Position(double xtemp, double ytemp, double ztemp,
             double wtemp, double ptemp, double rtemp);
    Position(double xtemp, double ytemp, double ztemp,
             double wtemp, double ptemp, double rtemp,
             int c1, int c2, int c3, int t1, int t2, int t3);
    Position(double xtemp, double ytemp, double ztemp,
             double wtemp, double ptemp, double rtemp,
             int c1, int c2, int c3, int t1, int t2, int t3,
             int mt, int ms, int ot);
    void Validation(void);                  /** Will validate the values                  **/
    void PrintValues();                     /** Prints the values from the class          **/
    string Cnvert2Str(string);              /** Converts all values to a single string    **/

};
class robotCom
{
  public:
    robotCom(boost::asio::io_service& io_service,tcp::resolver::iterator endpoint_iterator);
    void movtocpos(string msg);
    void movtojpos(string msg);
    string getcrcpos (void);
    string getcrjpos (void);
    bool checkcpos(string input);
    bool checkjpos(string input);
    void listtpp(void);
    void runtpp(char *tppname);
    void motionstop(void);
    void write(char* msg, int length);
    void close();
    

  private:
    char data_[1000];
    void handle_connect(const boost::system::error_code& error,tcp::resolver::iterator endpoint_iterator);
    void do_write(char*msg,int length);
    void do_close();

  private:
    
    boost::asio::io_service& io_service_;
    tcp::socket socket_;
    
};
class dataReceiver
{
    
    ros::NodeHandle nh_;                    //handler for ROS node
    ros::Subscriber sub_raw_;
    ros::Subscriber sub_pts_;
    ros::Subscriber sub_filt_;
    ros::Publisher fanuc_raw;
    ros::Publisher fanuc_filt;
    ros::Publisher fanuc_pts;

  private:
    //variable declaration for calc. purposes
    int Ax, Ay, Az;                         //accelerometers
    int Gx, Gy, Gz;                         //gyroscopes
    int Mx, My, Mz;                         //magnetometers
    double lax, lay, laz;
    double avx, avy, avz;
    double adx, ady, adz; 
    // imu_network::sensors_network received_values_;
  public:
    // dataReceiver();
    // void sensor_receiver_raw(const imu_network::sensors_network::ConstPtr& msg);
    // void sensor_receiver_filt(const imu_network::filtered_imu_network::ConstPtr& msg);
    void points_received(const geometry_msgs::Point& msg);
};
/*
=============================================================================================
=============================================================================================
                               oooo             .o8                 oooo                               
                               `888            "888                 `888                               
                     .oooooooo  888   .ooooo.   888oooo.   .oooo.    888                               
                    888' `88b   888  d88' `88b  d88' `88b `P  )88b   888                               
                    888   888   888  888   888  888   888  .oP"888   888                               
                    `88bod8P'   888  888   888  888   888 d8(  888   888                               
                    `8oooooo.  o888o `Y8bod8P'  `Y8bod8P' `Y888""8o o888o                              
                    d"     YD                                                                          
                    "Y88888P'                                                                          
                                    o8o             .o8       oooo                     
                                    `"'            "888       `888                     
    oooo    ooo  .oooo.   oooo d8b oooo   .oooo.    888oooo.   888   .ooooo.   .oooo.o 
     `88.  .8'  `P  )88b  `888""8P `888  `P  )88b   d88' `88b  888  d88' `88b d88(  "8 
      `88..8'    .oP"888   888      888   .oP"888   888   888  888  888ooo888 `"Y88b.  
       `888'    d8(  888   888      888  d8(  888   888   888  888  888    .o o.  )88b 
        `8'     `Y888""8o d888b    o888o `Y888""8o  `Y8bod8P' o888o `Y8bod8P' 8""888P' 
=============================================================================================
=============================================================================================
This is the list of global variables used in this node. This variables are based upon the
definition of MACROS. The MACROS section defines the overall MACROS necessary to run this
node. However, they are later converted into global variables.
/*******************************************************************************************/
/*******************************************************************************************/
#ifdef _MAIN_
    /** Dinamic Variables - used to set IP and Port **/
    string Global_IP   = FANUC_SMALL;                   /** Contains the IP adress        **/
    string Global_Port = PORT;                          /** Contains the port number      **/


    /** Debug Variables **/
    bool Global_debug_show_functions = false;           /** Debug: show functions names on
                                                        terminal                          **/
    bool Global_debug_stop_functions = false;           /** Debug: stop at the beggining of
                                                        a function                        **/
    bool Global_debug_print_structs = false;            /** Debug: show struct values     **/
    bool Global_debug_print_input_vars = false;         /** Debug: show input of function **/
    bool Global_debug_show_results = false;             /** Shows the result of a given 
                                                        function or operation             **/
    
    bool Global_keep_going = true;                      /** Variable that knows when to stop
                                                        the measures                      **/
    string m1 = "";
    string m2 = "";
    string m3 = "";
    string m4 = "";

    int delete_this = 0;
    int a1, a2, a3, a4, a5, a6;
#else
    extern string Global_str_fanuc_small;
    extern string Global_str_fanuc_big;
    extern string Global_str_fanuc_port;

    extern string Global_IP;
    extern string Global_Port;

    extern bool Global_debug_show_functions;
    extern bool Global_debug_stop_functions;
    extern bool Global_debug_print_structs;
    extern bool Global_debug_print_input_vars;
    extern bool Global_debug_show_results;

    extern bool Global_keep_going;
    extern string m1;
    extern string m2;
    extern string m3;
    extern string m4;

    extern int delete_this;
    extern int a1, a2, a3, a4, a5, a6;

#endif
/*******************************************************************************************/
/*******************************************************************************************/