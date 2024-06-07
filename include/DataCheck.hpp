#pragma once

#include <thread>
#include <tuple>
#include <ros/ros.h>

using namespace std;

template <typename... Ts>
class DataCheck
{
public:
    DataCheck(double check_freq_, DataSrc<Ts> &...data_src_)
        : enable_check(true), check_pass(true), check_freq(check_freq_), data_src(data_src_...)
    {
        check_loop = thread(&DataCheck::processDataLoop, this);
    }

    void processDataLoop()
    {
        ros::Rate check_rate(check_freq);
        while (ros::ok())
        {
            if (apply([](const auto &...src)
                      { return (src.isStarted() && ...); },
                      data_src) &&
                enable_check)
            {
                if (apply([](const auto &...src)
                          { return (src.stable_stream && ...); },
                          data_src))
                {
                    check_pass = processData();
                }
                else
                {
                    check_pass = false;
                }
            }
            else
            {
                // The check is defined to be passed if it is not enabled or the source is not started
                check_pass = true;
            }
            check_rate.sleep();
        }
    }

    virtual bool processData() = 0;

    inline bool isPassed() { return check_pass; }

    inline void startCheck() { enable_check = true; }
    inline void stopCheck() { enable_check = false; }

    bool enable_check;
    bool check_pass;
    double check_freq;
    tuple<DataSrc<Ts> &...> data_src;
    thread check_loop;
};