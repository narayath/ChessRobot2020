/*
 * This node recieves angles from the ik_srv and send to the pololu
 * @Nikhil Arayath
 * Last Modified 2/28/21
 */

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
#include <chessRobot/angles_msg.h>
#include "include/ik_solver.h"


using namespace std;

//initializing angle variables
double swivel_pos = 0.0;
double shoulder_pos = 0.0;
double shoulder_pos_r = 0.0;
double elbow_pos = 0.0;
double elbow_pos_r = 0.0;
double wrist_pos = 0.0;

//converting factor for maestro controller for specific servo
double servo_MG995 = 35.0;
double servo_SAVOX = 36.9;
double servo_goBilda = 26.7;

int fd = 0;

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

int maestroGetPosition(int fd, unsigned char channel)
{
  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
 
  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }
 
  return response[0] + 256*response[1];
}

//Callback method which recieves the request from the ik_srv and converts it to an angle to be read by pololu
bool posCmdCB(chessRobot::angles_msgRequest& req, chessRobot::angles_msgResponse& res) 
{
    //713 = -90  1500 = 0    2287 = 90
    //recieves from service
    swivel_pos = req.swivel_ang;
    //converts from radians to degrees
    swivel_pos = swivel_pos * (180/pi);
    //6000 is servo middle position * 4 and the angle in degrees is multiplied by conversion to convert to pulses
    swivel_pos = 6000 + swivel_pos * servo_MG995;
    
    shoulder_pos_r = ((pi/2) - req.shoulder_ang);
    shoulder_pos_r = shoulder_pos_r * (180/pi);
    ROS_INFO("shoulder angle in degrees: %f", shoulder_pos_r);
    
    shoulder_pos = 6000 - (shoulder_pos_r * servo_SAVOX);
    shoulder_pos_r = 6000 + (shoulder_pos_r * servo_SAVOX);
    
    //pi - because it starts from the vertical position and ik_solver calculates inner angle
    elbow_pos_r = pi- req.elbow_ang;
    elbow_pos_r = elbow_pos_r * (180/pi);
    ROS_INFO("elbow angle in degrees: %f", elbow_pos_r);
    
    elbow_pos = 6000 - (elbow_pos_r * servo_goBilda);
    elbow_pos_r = 6000 + (elbow_pos_r * servo_goBilda);

    
    wrist_pos = (pi - req.wrist_ang);
    wrist_pos = wrist_pos * (180/pi);
    wrist_pos = 6000 + wrist_pos * servo_MG995;
    
   // ROS_INFO("received value of swivel_cmd is: %f", swivel_pos);
    ROS_INFO("received value of shoulder_cmd is: %f", shoulder_pos);
    ROS_INFO("received value of elbow_cmd is: %f", elbow_pos);
   // ROS_INFO("received value of wrist_cmd is: %f", wrist_pos);
    
    //Sends angles to the pololu with the apropriate ports and angles
    //maestroSetTarget(fd, 0, swivel_pos);
    maestroSetTarget(fd, 1, shoulder_pos_r);
    maestroSetTarget(fd, 2, shoulder_pos);
    maestroSetTarget(fd, 3, elbow_pos);
    maestroSetTarget(fd, 4, elbow_pos_r);
    maestroSetTarget(fd, 6, wrist_pos);
    
    res.pos_ok = true;
    return true;
}


int main(int argc, char **argv) {
    //initializations:
    ros::init(argc, argv, "servoRobot");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("servoRobot", posCmdCB);
        
    cout << "Started servoRobot service \n";

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
        ros::spinOnce();
    }
} 
