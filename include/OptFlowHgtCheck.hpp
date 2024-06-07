#pragma once

#include <ros/ros.h>
#include <mavros_msgs/OpticalFlowRad.h>

#include "DataSrc.hpp"
#include "DataCheck.hpp"

using namespace std;

class OptFlowHgtCheck : public DataCheck<mavros_msgs::OpticalFlowRad>
{
public:
    OptFlowHgtCheck(DataSrc<mavros_msgs::OpticalFlowRad> &data_src_, double check_freq_, double max_hgt_)
        : DataCheck<mavros_msgs::OpticalFlowRad>(check_freq_, data_src_), max_hgt(max_hgt_)
    {
        auto &[of_src] = data_src;
        ROS_INFO_STREAM("[Odom Fusion] \033[32mPerforming height check on " << of_src.src_topic << " with max height = " << max_hgt << "m @_@");
    }

    inline bool processData() override
    {
        auto &[of_src] = data_src;
        mavros_msgs::OpticalFlowRad latest_of = of_src.getLatestDataCopy();

        if (latest_of.distance > max_hgt || latest_of.distance < 0)
        {
            if (check_pass == true)
                ROS_ERROR_STREAM("[Odom Fusion] Height from optflow " << of_src.src_topic << " exceeds safety threshold (" << ((latest_of.distance > max_hgt) ? (to_string(latest_of.distance) + "m > " + to_string(max_hgt)) : (to_string(latest_of.distance) + "m < 0")) << "m) #^#");
            return false;
        }
        else
            return true;
    }

    double max_hgt;
};