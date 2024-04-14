#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "DataSrc.hpp"
#include "DataCheck.hpp"

using namespace std;

class VelMagCheck : public DataCheck<nav_msgs::Odometry>
{
public:
    VelMagCheck(DataSrc<nav_msgs::Odometry> &data_src_, double check_freq_, double max_vel_)
        : DataCheck<nav_msgs::Odometry>(check_freq_, data_src_), max_vel(max_vel_)
    {
        auto &[odom_src] = data_src;
        ROS_INFO_STREAM("\033[32m[Odom Fusion] Performing velocity magnitude check on " << odom_src.src_topic << " with max velocity = " << max_vel << " m/s @_@");
    }

    bool processData() override
    {
        auto &[odom_src] = data_src;
        lock_guard<mutex> lock(odom_src.src_mutex);

        if (odom_src.data_buf.back().twist.twist.linear.x > max_vel)
        {
            ROS_ERROR_STREAM("[Odom Fusion] Velocity from odometry " << odom_src.src_topic << " is too high #^#");
            return false;
        }

        return true;
    }

    double max_vel;
};