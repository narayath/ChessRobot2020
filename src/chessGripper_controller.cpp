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
#include <unistd.h>
#include <chessRobot/gripper_msg.h>
#include "include/ik_solver.h"

using namespace std;
//some "magic number" global params:
const double Kp = 10.0; //controller gains
const double Kv = 3;
const double dt = 0.01;

//a simple saturation function; provide saturation threshold, sat_val, and arg to be saturated, val

double sat(double val, double sat_val) {
    if (val > sat_val)
        return (sat_val);
    if (val< -sat_val)
        return (-sat_val);
    return val;

}

double min_periodicity(double theta_val) {
    double periodic_val = theta_val;
    while (periodic_val > M_PI) {
        periodic_val -= 2 * M_PI;
    }
    while (periodic_val< -M_PI) {
        periodic_val += 2 * M_PI;
    }
    return periodic_val;
}

double right_grab_pos = 0.0;
double left_grab_pos = 0.0;

bool posCmdCB(chessRobot::gripper_msgRequest& req, chessRobot::gripper_msgResponse& res) 
{
    ROS_INFO("received value of gripper_action is: %d", req.grip_action);
    
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
    
    res.pos_ok = true;
    return true;
}

bool test_services() {
    bool service_ready = false;
    if (!ros::service::exists("/gazebo/apply_joint_effort", true)) {
        ROS_WARN("waiting for apply_joint_effort service");
        return false;
    }
    if (!ros::service::exists("/gazebo/get_joint_properties", true)) {
        ROS_WARN("waiting for /gazebo/get_joint_properties service");
        return false;
    }
    ROS_INFO("services are ready");
    return true;
}

int main(int argc, char **argv) {
    //initializations:
    ros::init(argc, argv, "chessGripper_controller");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);

    // make sure services are available before attempting to proceed, else node will crash
    while (!test_services()) {
        ros::spinOnce();
        half_sec.sleep();
    }

    ros::ServiceServer service = nh.advertiseService("chessGripper_controller", posCmdCB);
    
    ros::ServiceClient set_trq_client =
            nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    ros::ServiceClient get_jnt_state_client =
            nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
    
    gazebo_msgs::ApplyJointEffort right_grab_effort_cmd;
    gazebo_msgs::GetJointProperties right_grab_joint_state;
    
    gazebo_msgs::ApplyJointEffort left_grab_effort_cmd;
    gazebo_msgs::GetJointProperties left_grab_joint_state;
    
    ros::Publisher trq_publisher = nh.advertise<std_msgs::Float64>("jnt_trq", 1);
    ros::Publisher vel_publisher = nh.advertise<std_msgs::Float64>("jnt_vel", 1);
    ros::Publisher pos_publisher = nh.advertise<std_msgs::Float64>("jnt_pos", 1);
    ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    

    std_msgs::Float64 trq_msg, q1_msg, q1dot_msg;
    double q1, q1dot, q1_err, trq_cmd;
    sensor_msgs::JointState right_grab_joint_state_msg;
    sensor_msgs::JointState left_grab_joint_state_msg;
    ros::Duration duration(dt);
    ros::Rate rate_timer(1 / dt);
    
    right_grab_effort_cmd.request.joint_name = "right_grab";
    right_grab_effort_cmd.request.effort = 0.0;
    right_grab_effort_cmd.request.duration = duration;
    right_grab_joint_state.request.joint_name = "right_grab";
    
    left_grab_effort_cmd.request.joint_name = "left_grab";
    left_grab_effort_cmd.request.effort = 0.0;
    left_grab_effort_cmd.request.duration = duration;
    left_grab_joint_state.request.joint_name = "left_grab";
    

    right_grab_joint_state_msg.header.stamp = ros::Time::now();
    right_grab_joint_state_msg.name.push_back("right_grab");
    right_grab_joint_state_msg.position.push_back(0.0);
    right_grab_joint_state_msg.velocity.push_back(0.0);
    
    left_grab_joint_state_msg.header.stamp = ros::Time::now();
    left_grab_joint_state_msg.name.push_back("left_grab");
    left_grab_joint_state_msg.position.push_back(0.0);
    left_grab_joint_state_msg.velocity.push_back(0.0);
    
    //here is the main controller loop:
    while (ros::ok()) { 

        //setting right grab pos
        get_jnt_state_client.call(right_grab_joint_state);
        q1 = right_grab_joint_state.response.position[0];
        q1_msg.data = q1;
        pos_publisher.publish(q1_msg); //republish his val on topic jnt_pos

        q1dot = right_grab_joint_state.response.rate[0];
        q1dot_msg.data = q1dot;
        vel_publisher.publish(q1dot_msg);

        right_grab_joint_state_msg.header.stamp = ros::Time::now();
        right_grab_joint_state_msg.position[0] = q1;
        right_grab_joint_state_msg.velocity[0] = q1dot;
        joint_state_publisher.publish(right_grab_joint_state_msg);

        q1_err = min_periodicity(right_grab_pos - q1); //jnt angle err; watch for periodicity

        trq_cmd = Kp * (q1_err) - Kv*q1dot;
        trq_msg.data = trq_cmd;
        trq_publisher.publish(trq_msg);

        right_grab_effort_cmd.request.effort = trq_cmd; // send torque command to Gazebo
        set_trq_client.call(right_grab_effort_cmd);
        //make sure service call was successful
        bool result = right_grab_effort_cmd.response.success;
        if (!result)
        {
            ROS_WARN("service call to right_grab_effort_cmd failed!");
        }
     
        //setting left grab pos
        get_jnt_state_client.call(left_grab_joint_state);
        q1 = left_grab_joint_state.response.position[0];
        q1_msg.data = q1;
        pos_publisher.publish(q1_msg); //republish his val on topic jnt_pos

        q1dot = left_grab_joint_state.response.rate[0];
        q1dot_msg.data = q1dot;
        vel_publisher.publish(q1dot_msg);

        left_grab_joint_state_msg.header.stamp = ros::Time::now();
        left_grab_joint_state_msg.position[0] = q1;
        left_grab_joint_state_msg.velocity[0] = q1dot;
        joint_state_publisher.publish(left_grab_joint_state_msg);

        q1_err = min_periodicity(left_grab_pos - q1); //jnt angle err; watch for periodicity

        trq_cmd = Kp * (q1_err) - Kv*q1dot;
        trq_msg.data = trq_cmd;
        trq_publisher.publish(trq_msg);

        left_grab_effort_cmd.request.effort = trq_cmd; // send torque command to Gazebo
        set_trq_client.call(left_grab_effort_cmd);
        //make sure service call was successful
        result = left_grab_effort_cmd.response.success;
        if (!result)
        {
            ROS_WARN("service call to left_grab_effort_cmd failed!");
        }

        
        ros::spinOnce();
        rate_timer.sleep();
    }
} 
 
