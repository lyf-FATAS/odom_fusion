#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <ros/ros.h>

using namespace std;

template <typename T>
class DataSrc
{
public:
    DataSrc(ros::NodeHandle &nh, const string &topic, double freq)
        : DataSrc(nh, topic, freq, -1.0) {}

    DataSrc(ros::NodeHandle &nh, const string &topic, double freq, double timeout)
        : src_topic(topic), src_freq(freq), src_timeout(timeout), src_started(false), stable_stream(true), recv_msg(false)
    {
        data_sub = nh.subscribe(src_topic, 100, &DataSrc::recvDataCallback, this, ros::TransportHints().tcpNoDelay());
        buf_size = 5 * src_freq; // 5 seconds buffer
        check_stream_stability_timer = nh.createTimer(ros::Duration(getStreamTimeout()), &DataSrc::checkStreamStabilityCallback, this);
    }

    inline void recvDataCallback(const T &msg)
    {
        lock_guard<mutex> lock(src_mutex);

        data_buf.push_back(msg);
        if (data_buf.size() > buf_size)
            data_buf.pop_front();

        src_started = true;
        stable_stream = true;
        recv_msg = true;
    }

    inline void checkStreamStabilityCallback(const ros::TimerEvent &)
    {
        lock_guard<mutex> lock(src_mutex);

        if (src_started)
        {
            if (!recv_msg)
            {
                stable_stream = false;
                ROS_ERROR_STREAM_THROTTLE(10.0, "[Odom Fusion] Unstable data stream !!! Data not received from " << src_topic << " for " << ((src_timeout < 0.0) ? "5 periods #^#" : to_string(src_timeout) + " seconds #^#"));
            }
            recv_msg = false;
        }
        else
        {
            ROS_ERROR_STREAM_THROTTLE(10.0, "[Odom Fusion] No data from " << src_topic << " #^#");
        }
    }

    inline void restart()
    {
        lock_guard<mutex> lock(src_mutex);

        src_started = false;

        this_thread::sleep_for(chrono::milliseconds(500)); // Wait for data checks to finish. TODO: is there another way ? #^#

        // The stream is defined to be stable if it is not started
        stable_stream = true;

        recv_msg = false;
        deque<T> empty;
        swap(data_buf, empty);
    }

    inline bool isStarted() const { return src_started; }
    inline bool isStable() const { return stable_stream; }

    inline T getLatestDataCopy() // TODO: + const and return ref
    {
        lock_guard<mutex> lock(src_mutex);
        if (!src_started)
            ROS_ERROR("[Odom Fusion] Unable to retrieve data, buffer is empty and the program should be dead ... This is a class design issue #^# Currently, you have to make sure that there is already data in the buffer before accessing it 🤧🤧🤧");
        return data_buf.back();
    }

    inline T getPenultimateCopy(int i)
    {
        lock_guard<mutex> lock(src_mutex);
        if (!src_started)
            ROS_ERROR("[Odom Fusion] Unable to retrieve data, buffer is empty and the program should be dead ... This is a class design issue #^# Currently, you have to make sure that there is already data in the buffer before accessing it 🤧🤧🤧");
        return data_buf[data_buf.size() - 1 - i];
    }

    inline int getBufSize()
    {
        lock_guard<mutex> lock(src_mutex);
        return data_buf.size();
    }

    inline double getStreamTimeout() const
    {
        return (src_timeout < 0.0) ? 5.0 / src_freq : src_timeout;
    }

    string src_topic;
    double src_freq;
    double src_timeout;
    mutex src_mutex;
    deque<T> data_buf;
    atomic<bool> src_started;
    atomic<bool> stable_stream;

    ros::Subscriber data_sub;
    ros::Timer check_stream_stability_timer;

    size_t buf_size;
    bool recv_msg;
};
