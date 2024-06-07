#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "DataSrc.hpp"
#include "DataCheck.hpp"

using namespace std;

class PosMagCheck : public DataCheck<nav_msgs::Odometry>
{
public:
    PosMagCheck(DataSrc<nav_msgs::Odometry> &data_src_, double check_freq_, double min_x_, double max_x_, double min_y_, double max_y_, double min_z_, double max_z_, double init_x_, double init_y_, double init_z_)
        : DataCheck<nav_msgs::Odometry>(check_freq_, data_src_), min_x(min_x_), max_x(max_x_), min_y(min_y_), max_y(max_y_), min_z(min_z_), max_z(max_z_), init_x(init_x_), init_y(init_y_), init_z(init_z_)
    {
        auto &[odom_src] = data_src;
        ROS_INFO_STREAM("[Odom Fusion] \033[32mPerforming position magnitude check on " << odom_src.src_topic << " with space boundary x in [" << min_x << "m, " << max_x << "m], y in [" << min_y << "m, " << max_y << "m], z in [" << min_z << "m, " << max_z << "m] @_@");
    }

    inline bool processData() override
    {
        auto &[odom_src] = data_src;
        nav_msgs::Odometry latest_odom = odom_src.getLatestDataCopy();

        if (latest_odom.pose.pose.position.x - init_x < min_x)
        {
            if (check_pass == true)
                ROS_ERROR_STREAM("[Odom Fusion] Position.x from odometry " << odom_src.src_topic << " is beyond border (" << latest_odom.pose.pose.position.x - init_x << "m < " << min_x << "m) #^#");
            return false;
        }
        else if (latest_odom.pose.pose.position.x - init_x > max_x)
        {
            if (check_pass == true)
                ROS_ERROR_STREAM("[Odom Fusion] Position.x from odometry " << odom_src.src_topic << " is beyond border (" << latest_odom.pose.pose.position.x - init_x << "m > " << max_x << "m) #^#");
            return false;
        }
        else if (latest_odom.pose.pose.position.y - init_y < min_y)
        {
            if (check_pass == true)
                ROS_ERROR_STREAM("[Odom Fusion] Position.y from odometry " << odom_src.src_topic << " is beyond border (" << latest_odom.pose.pose.position.y - init_y << "m < " << min_y << "m) #^#");
            return false;
        }
        else if (latest_odom.pose.pose.position.y - init_y > max_y)
        {
            if (check_pass == true)
                ROS_ERROR_STREAM("[Odom Fusion] Position.y from odometry " << odom_src.src_topic << " is beyond border (" << latest_odom.pose.pose.position.y - init_y << "m > " << max_y << "m) #^#");
            return false;
        }
        else if (latest_odom.pose.pose.position.z - init_z < min_z)
        {
            if (check_pass == true)
                ROS_ERROR_STREAM("[Odom Fusion] Position.z from odometry " << odom_src.src_topic << " is beyond border (" << latest_odom.pose.pose.position.z - init_z << "m < " << min_z << "m) #^#");
            return false;
        }
        else if (latest_odom.pose.pose.position.z - init_z > max_z)
        {
            if (check_pass == true)
                ROS_ERROR_STREAM("[Odom Fusion] Position.z from odometry " << odom_src.src_topic << " is beyond border (" << latest_odom.pose.pose.position.z - init_z << "m > " << max_z << "m) #^#");
            return false;
        }
        else
            return true;
    }

    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
    double init_x, init_y, init_z;
};