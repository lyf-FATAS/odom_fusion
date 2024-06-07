#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigen>

#include "DataSrc.hpp"
#include "DataCheck.hpp"

using namespace std;
using namespace Eigen;

class PosContinuityCheck : public DataCheck<nav_msgs::Odometry>
{
public:
    PosContinuityCheck(DataSrc<nav_msgs::Odometry> &data_src_, double check_freq_, double max_jump_, double cooling_time_)
        : DataCheck<nav_msgs::Odometry>(check_freq_, data_src_), max_jump(max_jump_), cooling_time(cooling_time_), unstable(false)
    {
        auto &[odom_src] = data_src;
        ROS_INFO_STREAM("[Odom Fusion] \033[32mPerforming position continuity check on " << odom_src.src_topic << " with max jump threshold = " << max_jump << "m @_@");
    }

    inline bool processData() override
    {
        auto &[odom_src] = data_src;
        nav_msgs::Odometry latest_odom = odom_src.getLatestDataCopy();
        if (odom_src.getBufSize() < 2)
            return true;
        nav_msgs::Odometry second_latest_odom = odom_src.getPenultimateCopy(1);

        const Vector3d latest_pos(latest_odom.pose.pose.position.x,
                                  latest_odom.pose.pose.position.y,
                                  latest_odom.pose.pose.position.z);
        const Vector3d second_latest_pos(second_latest_odom.pose.pose.position.x,
                                         second_latest_odom.pose.pose.position.y,
                                         second_latest_odom.pose.pose.position.z);

        double jump = (latest_pos - second_latest_pos).norm();

        if (jump > max_jump)
        {
            ROS_ERROR_STREAM("[Odom Fusion] Position from odometry " << odom_src.src_topic << " is discontinuous (jump = " << jump << "m) #^#");
            unstable = true;
            unstable_start_time = ros::Time::now();
        }

        if (unstable && (ros::Time::now() - unstable_start_time).toSec() > cooling_time)
        {
            unstable = false;
        }

        return !unstable;
    }

    double max_jump;
    double cooling_time;
    bool unstable;
    ros::Time unstable_start_time;
};