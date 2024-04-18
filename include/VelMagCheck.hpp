#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigen>

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
        ROS_INFO_STREAM("[Odom Fusion] \033[32mPerforming velocity magnitude check on " << odom_src.src_topic << " with max velocity = " << max_vel << " m/s @_@");
    }

    inline bool processData() override
    {
        auto &[odom_src] = data_src;
        nav_msgs::Odometry latest_odom = odom_src.getLatestDataCopy();

        Eigen::Vector3d v(latest_odom.twist.twist.linear.x,
                          latest_odom.twist.twist.linear.y,
                          latest_odom.twist.twist.linear.z);

        if (v.norm() > max_vel)
        {
            if (check_pass == true)
                ROS_ERROR_STREAM("[Odom Fusion] Velocity from odometry " << odom_src.src_topic << " is too high (" << v.norm() << "m/s > " << max_vel << "m/s) #^#");
            return false;
        }
        else
            return true;
    }

    double max_vel;
};