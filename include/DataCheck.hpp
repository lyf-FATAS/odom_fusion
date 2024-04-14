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
        : check_pass(false), check_freq(check_freq_), data_src(data_src_...)
    {
        check_loop = thread(&DataCheck::processDataLoop, this);
    }

    void processDataLoop()
    {
        ros::Rate check_rate(check_freq);
        while (ros::ok())
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
            check_rate.sleep();
        }
    }

    virtual bool processData() = 0;

    bool check_pass;
    double check_freq;
    tuple<DataSrc<Ts> &...> data_src;
    thread check_loop;
};