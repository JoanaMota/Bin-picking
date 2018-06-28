#include "../../bin_picking/include/bin_picking/sensorRS232.h"

using namespace std;

char distance_laser[100];
float counter = 0.0;

/**
 * @brief  Reads from port until a specific char. 
 * 
 * This functions reads the values received until it finds a paragraph, to storage the intire distance value.
 *
 * @param  fd - The file descriptor
 * @return none
 */
int ReadPortUntilChar(int fd)
{
    char ch;
    int n;
    do
    {
        n=read( fd, &ch, 1);
        if( n == -1 ) continue;   //perror("Err:");
        sprintf(distance_laser,"%s%c",distance_laser,ch);
    } while( ch != '\n');       //Reads until a paragraph is found
    counter++;
    return 0;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "bin_picking_sensorRS232");

    ros::NodeHandle nh;
    ros::Publisher pub_rs232 = nh.advertise<std_msgs::Float32>("/output_laser_sensor", 1000);
    ros::Rate loop_rate(10);

    int  fd;
    vector <float> readings;
    fd=OpenPort("/dev/ttyACM0", NULL);
    while (ros::ok() & fd == -1) 
    {
        cout << "Error. Could not open port" << endl ; 
        // exit(1); 
    }


    while (ros::ok() && counter < 100)
    {
        std_msgs::Float32 dist; 
        ReadPortUntilChar(fd);                  //Reads the distance given by the Arduino UNO
        dist.data = atof(distance_laser);
        cout << "Distance=" << dist.data << endl; 
        distance_laser[0] = '\0';
        
        readings.push_back(dist.data);
        // pub_rs232.publish(dist_average);
        
        
        // ros::spinOnce();
        loop_rate.sleep();
    }
    
    float average = accumulate( readings.begin(), readings.end(), 0.0)/readings.size();              
    cout << "The average is: " << average << endl; 
    std_msgs::Float32 dist_average; 
    dist_average.data = average;
    pub_rs232.publish(dist_average);

    close(fd);
    return 0;
}