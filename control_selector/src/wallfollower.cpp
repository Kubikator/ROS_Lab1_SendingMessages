#include"wallfollower.h"

void Wallfollower::setLaserData(const std::vector<float> &data)
{
    obstacle = false;
    double closest_range = data[0];
    for (size_t i = 0; i<data.size()/2; ++i)
    {
        if (data[i] < closest_range){
            closest_range = data[i];
        }
        if ( data[i] < min_range )
        {
            obstacle = true;
            ROS_WARN_STREAM("OBSTACLE!!!");
            break;
        }
    }
    old_error = error;
    error = target_range - closest_range;
}

void Wallfollower::setRobotPose(double x, double y, double theta)
{
}

void Wallfollower::getControl(double &v, double &w)
{
    if (obstacle){
        v = 0;
        w = max_omega;
    } else {
        v = max_vel;
        int_error += error;
        diff_error = error -old_error;
        
        w = 2*error + 0.001*int_error + 5*diff_error;
    }
}

Wallfollower::Wallfollower(double range, double maxv, double maxw) : min_range(range),
                                                                                       max_vel(maxv),
                                                                                       max_omega(maxw),
                                                                                       int_error(0.),
                                                                                       old_error(0.),
                                                                                       diff_error(0.)
{
    ROS_DEBUG_STREAM("VoyagerControl constructor");
}

