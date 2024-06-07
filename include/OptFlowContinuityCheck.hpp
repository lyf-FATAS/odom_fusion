#pragma once

#include <ros/ros.h>
#include <mavros_msgs/OpticalFlowRad.h>

#include "DataSrc.hpp"
#include "DataCheck.hpp"

using namespace std;

class OptFlowContinuityCheck : public DataCheck<mavros_msgs::OpticalFlowRad>
{
public:
    OptFlowContinuityCheck(DataSrc<mavros_msgs::OpticalFlowRad> &data_src_, double check_freq_, double max_jump_, double cooling_time_)
        : DataCheck<mavros_msgs::OpticalFlowRad>(check_freq_, data_src_), max_jump(max_jump_), cooling_time(cooling_time_), unstable(false)
    {
        auto &[of_src] = data_src;
        ROS_INFO_STREAM("[Odom Fusion] \033[32mPerforming distance continuity check on " << of_src.src_topic << " with max jump threshold = " << max_jump << "m @_@");
    }

    inline bool processData() override
    {
        auto &[of_src] = data_src;
        mavros_msgs::OpticalFlowRad latest_of = of_src.getLatestDataCopy();
        if (of_src.getBufSize() < 2)
            return true;
        mavros_msgs::OpticalFlowRad second_latest_of = of_src.getPenultimateCopy(1);

        double jump = abs(latest_of.distance - second_latest_of.distance);

        if (jump > max_jump)
        {
            ROS_ERROR_STREAM("[Odom Fusion] Range measurement from " << of_src.src_topic << " is discontinuous (jump = " << jump << "m) #^#");
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