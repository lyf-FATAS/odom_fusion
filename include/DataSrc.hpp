#pragma once

#include <mutex>
#include <queue>
#include <string>
#include <ros/ros.h>

using namespace std;

template <typename T>
class DataSrc
{
public:
    DataSrc(ros::NodeHandle &nh, const string &topic, double freq)
        : src_topic(topic), src_freq(freq), src_timeout(-1.0), stable_stream(false), recv_first_msg(false), recv_msg(false), inform_no_data(true), inform_no_data_count(0), inform_unstable(true)
    {
        data_sub = nh.subscribe(src_topic, 100, &DataSrc::recvDataCallback, this, ros::TransportHints().tcpNoDelay());
        buf_size = 5 * src_freq; // 5 seconds buffer

        check_stream_stability_timer = nh.createTimer(ros::Duration(5 / src_freq), &DataSrc::checkStreamStabilityCallback, this); // Timeout after 5 periods
    }

    DataSrc(ros::NodeHandle &nh, const string &topic, double freq, double timeout)
        : src_topic(topic), src_freq(freq), src_timeout(timeout), stable_stream(false), recv_first_msg(false), recv_msg(false), inform_no_data(true), inform_no_data_count(0), inform_unstable(true)
    {
        data_sub = nh.subscribe(src_topic, 100, &DataSrc::recvDataCallback, this, ros::TransportHints().tcpNoDelay());
        buf_size = 5 * src_freq; // 5 seconds buffer

        check_stream_stability_timer = nh.createTimer(ros::Duration(src_timeout), &DataSrc::checkStreamStabilityCallback, this);
    }

    void recvDataCallback(const T &msg)
    {
        lock_guard<mutex> lock(src_mutex);

        stable_stream = recv_first_msg = recv_msg = inform_unstable = true;
        data_buf.push(msg);
        if (data_buf.size() > buf_size)
            data_buf.pop();
    }

    void checkStreamStabilityCallback(const ros::TimerEvent &)
    {
        lock_guard<mutex> lock(src_mutex);

        if (recv_first_msg)
        {
            if (!recv_msg)
            {
                stable_stream = false;
                if (inform_unstable)
                    ROS_ERROR_STREAM("[Odom Fusion] Unstable data stream !!! Data not received from " << src_topic << " for " << ((src_timeout < 0.0) ? "5 periods #^#" : to_string(src_timeout) + " seconds #^#"));
                inform_unstable = false;
            }
            recv_msg = false;
        }
        else
        {
            if (inform_no_data)
            {
                if (inform_no_data_count > 5.0 / ((src_timeout < 0.0) ? 5 / src_freq : src_timeout)) // Inform no data after 5 seconds
                {
                    ROS_ERROR_STREAM("[Odom Fusion] No data from " << src_topic << " #^#");
                    inform_no_data = false;
                }
                inform_no_data_count++;
            }
        }
    }

    string src_topic;
    double src_freq;
    double src_timeout;
    mutex src_mutex;
    queue<T> data_buf;
    bool stable_stream;

    ros::Subscriber data_sub;
    ros::Timer check_stream_stability_timer;

    size_t buf_size;
    bool recv_first_msg;
    bool recv_msg;
    bool inform_no_data;
    size_t inform_no_data_count;
    bool inform_unstable;
};