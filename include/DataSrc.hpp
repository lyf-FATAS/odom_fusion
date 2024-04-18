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
        : src_available(false), src_topic(topic), src_freq(freq), src_timeout(-1.0), src_started(false), stable_stream(true), recv_first_msg(false), recv_msg(false), inform_no_data_counter(0), inform_unstable(true), inform_restart_deadlock_counter(0)
    {
        data_sub = nh.subscribe(src_topic, 100, &DataSrc::recvDataCallback, this, ros::TransportHints().tcpNoDelay());
        buf_size = 5 * src_freq; // 5 seconds buffer

        check_stream_stability_timer = nh.createTimer(ros::Duration(5 / src_freq), &DataSrc::checkStreamStabilityCallback, this); // Timeout after 5 periods
    }

    DataSrc(ros::NodeHandle &nh, const string &topic, double freq, double timeout)
        : src_available(false), src_topic(topic), src_freq(freq), src_timeout(timeout), src_started(false), stable_stream(true), recv_first_msg(false), recv_msg(false), inform_no_data_counter(0), inform_unstable(true), inform_restart_deadlock_counter(0)
    {
        data_sub = nh.subscribe(src_topic, 100, &DataSrc::recvDataCallback, this, ros::TransportHints().tcpNoDelay());
        buf_size = 5 * src_freq; // 5 seconds buffer

        check_stream_stability_timer = nh.createTimer(ros::Duration(src_timeout), &DataSrc::checkStreamStabilityCallback, this);
    }

    inline void recvDataCallback(const T &msg)
    {
        lock_guard<mutex> lock(src_mutex);

        data_buf.push(msg);
        if (data_buf.size() > buf_size)
            data_buf.pop();

        src_started = stable_stream = recv_first_msg = recv_msg = inform_unstable = true;
    }

    inline void checkStreamStabilityCallback(const ros::TimerEvent &)
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
            if (inform_no_data_counter > 23.0 / ((src_timeout < 0.0) ? 5 / src_freq : src_timeout)) // Inform no data every 23 seconds
            {
                ROS_ERROR_STREAM("[Odom Fusion] No data from " << src_topic << " #^#");
                inform_no_data_counter = 0;
            }
            inform_no_data_counter++;
        }
    }

    inline void restart()
    {
        lock_guard<mutex> lock(src_mutex);

        src_started = false; // Equivalent to recv_first_msg currently @_@

        this_thread::sleep_for(chrono::milliseconds(500)); // Wait for data checks to finish. TODO: is there another way ? #^#

        // Msckf still sends stable data on reboot and generates a jump the moment the reboot is complete
        this_thread::sleep_for(chrono::milliseconds(500)); // Ensuring reboot completion by delaying
        // while (stable_stream) // The restart function assumes that the data stream has already stopped
        // {
        //     this_thread::sleep_for(chrono::milliseconds(1));

        //     if (inform_restart_deadlock_counter > 3.0 / 0.001) // Inform restart deadlock every 3.0 seconds
        //     {
        //         ROS_ERROR_STREAM("[Odom Fusion] Sustained data from " << src_topic << ", restart stucked #^#");
        //         inform_restart_deadlock_counter = 0;
        //     }
        //     inform_restart_deadlock_counter++;
        // }
        // inform_restart_deadlock_counter = 0;

        // The stream is defined to be stable if it is not started
        stable_stream = true;

        recv_first_msg = recv_msg = false;
        inform_unstable = true;
        inform_no_data_counter = 0;
        queue<T> empty;
        swap(data_buf, empty);
    }

    inline void setAvailable() { src_available = true; }
    inline void setUnavailable() { src_available = false; }
    inline bool isAvailable() const { return src_available; }
    inline bool isStarted() const { return src_started; }
    inline bool isStable() const { return stable_stream; }

    inline T getLatestDataCopy() // TODO: + const and return ref
    {
        if (!recv_first_msg)
            ROS_ERROR("[Odom Fusion] Unable to retrieve data, buffer is empty and the program should be dead ... Please contact the author: 13322809634, this is a class design issue #^#");
        lock_guard<mutex> lock(src_mutex);
        return data_buf.back();
    }

    bool src_available;

    string src_topic;
    double src_freq;
    double src_timeout;
    mutex src_mutex;
    queue<T> data_buf;
    bool src_started;
    bool stable_stream;

    ros::Subscriber data_sub;
    ros::Timer check_stream_stability_timer;

    size_t buf_size;
    bool recv_first_msg;
    bool recv_msg;
    size_t inform_no_data_counter;
    bool inform_unstable;
    size_t inform_restart_deadlock_counter;
};