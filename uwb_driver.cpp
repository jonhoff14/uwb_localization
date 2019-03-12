// IndoTraq parsing
// Jonathan Hoff
// 2018-06-07

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <string.h>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>

#include "proto/config.pb.h"
#include "util/utils.h"

using namespace std;

int fd, rd; // file descriptor
const int bsize = 80;
char msg[60]; // message buffer (no newline characters)

// Data format (no anchors):
//      $,10001,0,3000001,3000001,3000001,9984,379,411,12,*34
// Data format (with anchors):
// $,10001,1,0,0,0,0,0,0,0,*31
// $,10002,0,300001,300001,1133,1046,*2C
// $,10003,0,1077,1184,1353,1266,0,0,0,0,*24
// $,10006,0,0,0,600,1603,0,600,1603,1048,600,0,1048,600,0,0,0,0,0,0,0,0,0,0,0,0,*2B
// $,10001,0,743,39,600,9325,-1526,-3138,-927,*19

// Function to parse message
bool parse_message(char *msg,int *pose)
{
    // Message type: true if correct message, false if either wrong message or issues reading
    bool msg_type = false;

    // Data (integers in units of mm)
    int x = 0;
    int y = 0;
    int z = 0;

    // Pointer that starts at beginning of buffer string
    char *pt;

    // Parse buffer by "," delimiter
    pt = strtok(msg,",");

    // Token count number
    int count = 0;

    // Vector of parsed message values
    vector<string> msg_parsed;

    // Iteratively move pointer to next token separated by "," delimiter
    while (pt != NULL)
    {
        // Push strings onto vector
        msg_parsed.push_back(pt);

        // Move to next token
        pt = strtok(NULL,",");

        // Update token count
        count++;
    }
    
    // Check message time and extract xyz coordinates
    if (msg_parsed.size() >= 5)
    {
        // ... TO DO: switch between different tags
        // Tag 0
        if (msg_parsed.at(0) == "$" && msg_parsed.at(1) == "10001" && msg_parsed.at(2) == "0" && msg_parsed.size() == 11)
        {
            x = atoi(msg_parsed.at(3).c_str());
            y = atoi(msg_parsed.at(4).c_str());
            z = atoi(msg_parsed.at(5).c_str());
            //printf("X is %i Y is %i Z is %i\n",x,y,z);
            msg_type = true; // return correct message
        }
    }
    pose[0] = x;
    pose[1] = y;
    pose[2] = z;
    return msg_type;
}

// from GNSS driver serial_stream.cpp
speed_t get_serial_baudrate(uint32_t rate) {
  switch (rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return 0;
  }
}

int open_port(string fname, int rate)
{
    // Open serial port
    int fd = open(fname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1)
    {
        ROS_ERROR("Unable to open serial port device %s. Run "
        "$ sudo chmod a+rw <device> to allow admin access, or "
        "check the device name.\n\n",fname.c_str());
        return fd;
    }
    else
    {
        ROS_INFO("Successfully opened serial port device %s.\n",fname.c_str());
    }
    
    // termios structures for serial port configuration
    struct termios oldtio, tio;
    
    // Get current serial port configuration
    tcgetattr(fd, &oldtio);

    // Get baudrate
    speed_t baudrate = get_serial_baudrate(rate);

    // Set flags
	tio.c_cflag 	= baudrate | CS8 | CLOCAL | CREAD;
	tio.c_iflag 	= 0;
	tio.c_oflag 	= 0;
	tio.c_lflag 	= 0;
    
	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &tio);
    return fd;
}

int main(int argc, char **argv)
{
    // Message to publish
    //geometry_msgs::Point pose_msg;
    geometry_msgs::PointStamped pose_msg;
    pose_msg.point.x = 0;
    pose_msg.point.y = 0;
    pose_msg.point.z = 0;

    // Initialization
    ros::init(argc, argv, "uwb_driver");

    // Node for communication with ROS environment
    ros::NodeHandle nh;

    // Publisher
    //ros::Publisher uwb_driver_pub = nh.advertise<geometry_msgs::Point>("pose", 1000);
    ros::Publisher uwb_driver_pub = nh.advertise<geometry_msgs::PointStamped>("/apollo/sensor/uwb/raw_pose", 1000);
    ros::Rate loop_rate(1000);

    // Get parameters from configuration file
    std::string cfg_file;
    nh.param("uwb_conf", cfg_file, std::string("./conf/uwb_conf.txt"));
    apollo::drivers::uwb::config::Config config;
    if (!apollo::drivers::uwb::parse_config_text(cfg_file, &config))
    {
        ROS_FATAL_STREAM("Failed to load config file: " << cfg_file);
        return 0;
    }
    
    string serialdevice;
    int baudrate;

    //nh.getParam("/uwb_driver/serialdevice",serialdevice); // from launch file
    //nh.getParam("/uwb_driver/baudrate",baudrate); // from launch file

    serialdevice = config.serialdevice();
    baudrate = config.baudrate();

    ROS_INFO("Selected serial device is: %s\n",serialdevice.c_str());
    ROS_INFO("Selected baudrate is: %i\n",baudrate);
        
    // Open serial port
    fd = open_port(serialdevice,baudrate);
    if (fd == -1)
    {
        printf("Ending program.\n");
        return 0;
    }

    while (ros::ok())
    {
        std_msgs::String ros_msg;
        std::stringstream ss;

        int msg_idx = 0; // index of message array for reading in new values from buffer
        bool msg_end = false; // true when detecting end of message

        char buff[bsize];
        // Read serial port into buffer
        rd = read(fd,buff,sizeof(buff));

        // Parse message only if more than 0 bytes read
        if (rd > 0)
        {
            int i = 0;
            while (i < rd)
            {
                // Read until "\r\n" found
                if (buff[i] == '\r') // end message when detecting carriage return
                {
                    //printf("\r\n");
                    msg_end = true;
                    i += 2;
                }
                else if (buff[i] == '\n') // redundancy (in case miss the '\r' read)
                {
                    //printf("\r\n");
                    msg_end = true;
                    i += 1;
                }
                else
                {
                    //printf("%c",buff[i]);
                    msg[msg_idx]= buff[i];
                    i++;
                    msg_idx++;
                }
                if (msg_end)
                {
                    msg_idx = 0;
                    msg_end = false;
                    //printf("%s\r\n",msg);

                    // Call parsing function
                    int pose[3];
                    bool msg_type = parse_message(msg,pose);

                    // Send message if it is complete
                    if (msg_type)
                    {
                        pose_msg.header.stamp = ros::Time::now();
                        pose_msg.header.frame_id = "indotraq";
                        pose_msg.point.x = (double)pose[0];
                        pose_msg.point.y = (double)pose[1];
                        pose_msg.point.z = (double)pose[2];
                        uwb_driver_pub.publish(pose_msg);
                        //printf("X is %i Y is %i Z is %i\n",pose[0],pose[1],pose[2]);
                    }

                    // Clear message in order to store new message
                    memset(&msg,0,sizeof(msg));
                }
            }
        }
        else if (rd == 0)
        {
            //printf("Zero bytes read\t");
        }
        else if (rd == -1)
        {
            printf("Error reading serial port\t");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(fd);

    return 0;
}
