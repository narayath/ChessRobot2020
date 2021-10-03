#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

#include <first_pkg/Num.h>
#include <first_pkg/test1_srv.h>

const double forearm_length = 0.3;
const double upper_arm_length = 0.3;
const double pi = 3.141593;
const double half_base = 0.1;

struct Angles 
{
    double swivel_angle;
    double shoulder_angle;
    double elbow_angle;
    double wrist_angle;
};

class IkSolver
{
public:
    std::vector<Angles> ik_solve(double x, double y, double z);
    IkSolver();
    
private:
    double calc_swivel(double x, double y);
    double calc_swiv_x(double x, double y);
    double calc_dist_from_base(double x, double z);
    bool check_triangle(double x, double z, double dist_from_base);
    double calc_elbow(double dist_from_base);
    double calc_shoulder(double x, double z, double dist_from_base);
    double calc_wrist(double shld, double elb);
};


#endif
