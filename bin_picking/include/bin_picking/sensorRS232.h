#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
#include "std_msgs/Float32.h"
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <vector>                                                               
#include <numeric> 

// #define O_RDONLY         00
// #define O_WRONLY         01
// #define O_RDWR           02

/**
 * @brief  Open Port for connection with Arduino
 *
 * @param  port_name - Port's identification
 * @param  text - Text that will be written to the file
 * @return fd - The file descriptor
 */
int OpenPort( char *port_name, char* text)
{

	int fd;
	fd=open(port_name, O_RDWR| O_NOCTTY ); /*O_RDONLY, O_WRONLY*/
        if (text!=0)
        {
                write( fd, text, strlen(text) );
        }
    return fd;
}