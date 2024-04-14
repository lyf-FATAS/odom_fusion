#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "DataSrc.hpp"
#include "DataCheck.hpp"

using namespace std;

class NegZCheck : public DataCheck<nav_msgs::Odometry>
{
public:
    NegZCheck(DataSrc<nav_msgs::Odometry> &data_src_, double check_freq_, double min_z_)
        : DataCheck<nav_msgs::Odometry>(check_freq_, data_src_), min_z(min_z_)
    {
        auto &[odom_src] = data_src;
        ROS_INFO_STREAM("\033[32m[Odom Fusion] Performing negative position.z check on " << odom_src.src_topic << " with min z = " << min_z << " m @_@");
    }

    bool processData() override
    {
        auto &[odom_src] = data_src;
        lock_guard<mutex> lock(odom_src.src_mutex);

        if (odom_src.data_buf.back().pose.pose.position.z < min_z)
        {
            ROS_ERROR_STREAM("[Odom Fusion] Position.z from odometry " << odom_src.src_topic << " has dropped to irrational level (below " << min_z << " m) #^#");
            return false;
        }

        return true;
    }

    double min_z;
};