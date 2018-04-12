#include "fanuc_control.h"

/*
class dataReceiver
{
    
    ros::NodeHandle nh_;                    //handler for ROS node
    ros::Subscriber sub_raw_;
    ros::Subscriber sub_filt_;
    ros::Publisher fanuc_raw;
    ros::Publisher fanuc_filt;

  private:
    //variable declaration for calc. purposes
    int Ax, Ay, Az;                         //accelerometers
    int Gx, Gy, Gz;                         //gyroscopes
    int Mx, My, Mz;                         //magnetometers
    imu_network::sensors_network received_values_;
  public:
    dataReceiver();
    void sensor_receiver_data(const imu_network::sensors_network::ConstPtr& msg);
};
*/


// dataReceiver::dataReceiver()
// {
//     sub_raw_    = nh_.subscribe("topic_raw_data", 1000, &dataReceiver::sensor_receiver_raw, this);
//                                         //subscribes the sensors data
//                                         //every time that the topic is 
//                                         //updated, the function 
//                                         //sensor_receiver is called
//     sub_filt_   = nh_.subscribe("topic_filtered_imu", 1000, &dataReceiver::sensor_receiver_filt, this);
//     sub_pts_    = nh_.subscribe("find_arrow_state", 1000, &dataReceiver::points_received, this);
//     fanuc_raw   = nh_.advertise<std_msgs::String>("fanuc_raw", 1000);
//     fanuc_filt  = nh_.advertise<std_msgs::String>("fanuc_filt", 1000);
//     fanuc_pts  = nh_.advertise<std_msgs::String>("fanuc_pts", 1000);
// }


// void dataReceiver::sensor_receiver_raw(const imu_network::sensors_network::ConstPtr& msg)
// {
//     int total = msg->sensors_val[0].total_number;
//     string result;
//     for(int i=0; i<total; i++)
//     {
//         Ax = msg->sensors_val[i].S_Ax;        Ay = msg->sensors_val[i].S_Ay;        Az = msg->sensors_val[i].S_Az;
//         Gx = msg->sensors_val[i].S_Gx;        Gy = msg->sensors_val[i].S_Gy;        Gz = msg->sensors_val[i].S_Gz;
//         Mx = msg->sensors_val[i].S_Mx;        My = msg->sensors_val[i].S_My;        Mz = msg->sensors_val[i].S_Mz;

//         //result += Int2Str(msg->sensors_val[i].header.stamp) + ", ";
//         result += Int2Str(Ax) + ", " + Int2Str(Ay) + ", " + Int2Str(Az) + ", ";
//         result += Int2Str(Gx) + ", " + Int2Str(Gy) + ", " + Int2Str(Gz) + ", ";
//         result += Int2Str(Mx) + ", " + Int2Str(My) + ", " + Int2Str(Mz);

//         fanuc_raw.publish(result);
//         return;
//     }
// }


// void dataReceiver::sensor_receiver_filt(const imu_network::filtered_imu_network::ConstPtr& msg)
// {
//     int total = msg->filtered_imu_network[0].total_number;
//     string result;
//     for(int i=0; i<total; i++)
//     {
//         lax = msg->filtered_imu_network[i].linear_acceleration_x;
//         lay = msg->filtered_imu_network[i].linear_acceleration_y;
//         laz = msg->filtered_imu_network[i].linear_acceleration_z;

//         avx = msg->filtered_imu_network[i].angular_velocity_x;
//         avy = msg->filtered_imu_network[i].angular_velocity_y;
//         avz = msg->filtered_imu_network[i].angular_velocity_z;

//         adx = msg->filtered_imu_network[i].angular_displacement_x;
//         ady = msg->filtered_imu_network[i].angular_displacement_y;
//         adz = msg->filtered_imu_network[i].angular_displacement_z;

//         result += Int2Str(lax) + ", " + Int2Str(lay) + ", " + Int2Str(laz) + ", ";
//         result += Int2Str(avx) + ", " + Int2Str(avy) + ", " + Int2Str(avz) + ", ";
//         result += Int2Str(adx) + ", " + Int2Str(ady) + ", " + Int2Str(adz) + ", ";

//         fanuc_filt.publish(result);
//         return;
//     }
// }

// void dataReceiver::points_received(const geometry_msgs::Point& msg)
// {
//     string result;
//     int x, y, z;

//     x = msg.x;
//     y = msg.y;
//     z = msg.z;

//     result = Int2Str(x) + ", " + Int2Str(y) + ", " + Int2Str(z);
//     fanuc_pts.publish(result);
//     return;
// }
