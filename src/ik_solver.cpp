#include "include/ik_solver.h"
#include <math.h>

using namespace std;

IkSolver::IkSolver()
{
}

double IkSolver::calc_swivel(double x, double y)
{
    double angle = 0;
    
    if(y != 0)
    {
        angle = atan(y/x);
    }
    
    return angle;
}

double IkSolver::calc_swiv_x(double x, double y)
{
    //when swivelling the base, the x and z are on a new plane, which is the hypotenuse of a triangle created with the x and y
    return (sqrt((x*x)+(y*y)));
}

double IkSolver::calc_dist_from_base(double x, double z)
{
    z = z - half_base;
    return (sqrt((x * x) + (z * z)));
}

bool IkSolver::check_triangle(double x, double z, double dist_from_base)
{
    double tri_array[3] = {x, z, dist_from_base};
    double temp;
    
    if(tri_array[0] >= tri_array[1])
    {
        temp = tri_array[0];
        tri_array[0] = tri_array[1];
        tri_array[1] = temp;
        
        if(tri_array[1] >= tri_array[2])
        {
            temp = tri_array[1];
            tri_array[1] = tri_array[2];
            tri_array[2] = temp;
        }
    }
    
    else
    {
        if(tri_array[1] >= tri_array[2])
        {
            temp = tri_array[1];
            tri_array[1] = tri_array[2];
            tri_array[2] = temp;
        }
    }
    
    if(tri_array[0] + tri_array[1] > tri_array[2])
    {
        return true;
    }
    
    else
    {
        return false;
    }
}

double IkSolver::calc_elbow(double dist_from_base)
{
    double cos_val = (pow(dist_from_base, 2) - pow(forearm_length, 2) - pow(upper_arm_length, 2));
    cos_val = cos_val/(-2 * forearm_length * upper_arm_length);
    
    double angle = acos(cos_val);
    
    return angle;
}

double IkSolver::calc_shoulder(double x, double z, double dist_from_base)
{
    z = z - half_base;
    double cos_val = (pow(forearm_length, 2) - pow(upper_arm_length, 2) - pow(dist_from_base, 2));
    cos_val = cos_val/(-2 * dist_from_base * upper_arm_length);
    
    double top_angle = acos(cos_val);
    
    cos_val = (pow(z, 2) - pow(x, 2) - pow(dist_from_base, 2));
    cos_val = cos_val/(-2 * dist_from_base * x);
    
    double bot_angle = acos(cos_val);
    
    return (top_angle + bot_angle);
}

double IkSolver::calc_wrist(double shld, double elb)
{
    double wrist_angle = ((3*pi/2) - shld - elb);
    return wrist_angle;
}

std::vector<Angles> IkSolver::ik_solve(double x, double y, double z)
{
    cout << "Entered ik_solve method \n";
    std::vector<Angles> ik_solutions;
    
    double swivel_angle = calc_swivel(x, y);
    double elbow_angle;
    double shoulder_angle;
    double wrist_angle;
    
    
    x = calc_swiv_x(x, y);
    double dist_from_base = calc_dist_from_base(x, z);
    
    if(check_triangle(upper_arm_length, forearm_length, dist_from_base))
    {
        elbow_angle = calc_elbow(dist_from_base);
        shoulder_angle = calc_shoulder(x, z, dist_from_base);
        
        double droop_elb = (elbow_angle + (elbow_angle)*(0.02));
        double droop_shld = (shoulder_angle) + (pi - (shoulder_angle))*(0.03);
        
        wrist_angle = calc_wrist(shoulder_angle, elbow_angle);

        Angles angles = {swivel_angle, droop_shld, droop_elb, wrist_angle};
         
        
        auto it = ik_solutions.insert(ik_solutions.begin() , angles);
    }
    
    for(int i = 0; i<ik_solutions.size(); i++)
    {
        cout << "Angle " << i << " is " << ik_solutions.at(i).swivel_angle << "\n";
        cout << "Angle " << i << " is " << ik_solutions.at(i).shoulder_angle << "\n";
        cout << "Angle " << i << " is " << ik_solutions.at(i).elbow_angle << "\n";
        cout << "Angle " << i << " is " << ik_solutions.at(i).wrist_angle << "\n";
    }
    
    return ik_solutions;
}



