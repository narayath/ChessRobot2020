#include <ros/ros.h> //ALWAYS need to include this
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>
#include <chessRobot/arm_cmd.h>
#include <vector>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif
#include <chessRobot/gripper_msg.h>
#include "include/ik_solver.h"

using namespace std;

//global variables
double right_grab_pos = 0.0;
double left_grab_pos = 0.0;
int fd = 0;

//converting factor for maestro controller for specific servo
double servo_MG995 = 37.74;

//sets servo by sending message to maestro
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
	unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
	if (write(fd, command, sizeof(command)) == -1)
	{
	perror("error writing");
	return -1;
	}
	return 0;
}

//callback which communicates with server over gripper_msg
bool posCmdCB(chessRobot::gripper_msgRequest& req, chessRobot::gripper_msgResponse& res) 
{
    ROS_INFO("received value of gripper_action is: %d", req.grip_action);
    
    //Calculating the gripper position based on request recieved
    if(req.grip_action == 0)
    {
    }
    
    else if(req.grip_action == 1)
    {
       right_grab_pos = 0.0;
       left_grab_pos = 0.0; 
    }
  
    else if(req.grip_action == 2)
    {
       right_grab_pos = -pi/28;
       left_grab_pos = pi/28; 
    }
    right_grab_pos = right_grab_pos * (180/pi);
    right_grab_pos = 6000 + right_grab_pos * servo_MG995;
    
    left_grab_pos = left_grab_pos * (180/pi);
    left_grab_pos = 6000 + left_grab_pos * servo_MG995;
    
    res.pos_ok = true;
    return true;
}


int main(int argc, char **argv) {
    //initializations:
    ros::init(argc, argv, "servoGripper");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("servoGripper", posCmdCB);
    
    cout << "Started servoGripper service \n";
    
    //setting the port and file descriptor
    const char * device = "//dev//ttyACM0";
    fd = open(device, O_RDWR | O_NOCTTY);
	if (fd == -1)
	{
		perror(device);
		return 1;
	}

    
    //here is the main controller loop:
    while (ros::ok()) 
    { 
        //setting the servo value
        maestroSetTarget(fd, 7, right_grab_pos);
        
        ros::spinOnce();
    }
} 
