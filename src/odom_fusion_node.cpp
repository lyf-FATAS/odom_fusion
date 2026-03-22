#include <algorithm>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Eigen>

#include "DataSrc.hpp"
#include "PosMagCheck.hpp"
#include "VelMagCheck.hpp"
#include "PosContinuityCheck.hpp"
#include "ToFContinuityCheck.hpp"

#include "quadrotor_msgs/TakeoffLand.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "LowPassFilter.hpp"

using namespace std;
using namespace Eigen;
using namespace pcl;

enum FsmState
{
    WAIT_FOR_ODOM,
    TAKEOFF,
    FLY_WITH_ODOM
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_fusion_node");
    ros::NodeHandle nh("~");

    string hyperparam_path = argv[1];
    cv::FileStorage param(hyperparam_path, cv::FileStorage::READ);
    string odom_topic;
    double odom_freq, odom_timeout;
    nh.param("odom_topic", odom_topic, string("/fusion_odometry/current_point_odom"));
    nh.param("odom_freq", odom_freq, 200.0);
    nh.param("odom_timeout", odom_timeout, 0.1);

    int ros_logger_level = param["ros_logger_level"];
    switch (ros_logger_level)
    {
    case 0:
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        break;
    case 1:
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
        break;
    case 2:
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
        break;
    case 3:
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
        break;
    default:
        break;
    }

    //************* Data sources *************//
    DataSrc<nav_msgs::Odometry> odom_src(nh, odom_topic, odom_freq, odom_timeout);
    ROS_INFO_STREAM("[Odom Fusion] Input odom: topic=" << odom_topic << ", freq=" << odom_freq << " Hz, timeout=" << odom_timeout << " s");

    double init_x, init_y, init_z;
    if (getenv("ipx") != nullptr && getenv("ipy") != nullptr && getenv("ipz") != nullptr)
    {
        init_x = atof(getenv("ipx"));
        init_y = atof(getenv("ipy"));
        init_z = atof(getenv("ipz"));
    }
    else
        init_x = init_y = init_z = 0.0;

    DataSrc<mavros_msgs::OpticalFlowRad> tof_src(nh, param["tof_topic"], param["tof_freq"], param["tof_timeout"]);

    ros::AsyncSpinner spinner(10);
    spinner.start();

    //************* Data checks *************//
    PosMagCheck odom_takeoff_pos_mag_check(odom_src, param["pos_mag_check_freq"],
                                           param["min_x_takeoff"], param["max_x_takeoff"],
                                           param["min_y_takeoff"], param["max_y_takeoff"],
                                           param["min_z_takeoff"], param["max_z_takeoff"],
                                           init_x, init_y, init_z);
    PosMagCheck odom_pos_mag_check(odom_src, param["pos_mag_check_freq"],
                                   param["min_x"], param["max_x"],
                                   param["min_y"], param["max_y"],
                                   param["min_z"], param["max_z"],
                                   init_x, init_y, init_z);
    VelMagCheck odom_vel_mag_check(odom_src, param["vel_mag_check_freq"], param["max_vel"]);
    PosContinuityCheck odom_pos_continuity_check(odom_src, param["continuity_check_freq"], param["pos_max_jump"], param["pos_jump_cooling_time"]);
    ToFContinuityCheck tof_continuity_check(tof_src, param["continuity_check_freq"], param["tof_max_jump"], param["tof_jump_cooling_time"]);
    tof_continuity_check.stopCheck();

    auto isOdomCheckPassed = [&]()
    {
        return odom_takeoff_pos_mag_check.isPassed() &&
               odom_pos_mag_check.isPassed() &&
               odom_vel_mag_check.isPassed() &&
               odom_pos_continuity_check.isPassed();
    };

    //************* Main loop *************//
    atomic<FsmState> state(FsmState::WAIT_FOR_ODOM);
    ros::Rate data_check_rate(10.0);

    ros::Subscriber takeoff_signal_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>(param["takeoff_signal_topic"], 10,
                                                  [&](const quadrotor_msgs::TakeoffLand::ConstPtr &signal)
                                                  {
                                                      switch (state.load())
                                                      {
                                                      case FsmState::WAIT_FOR_ODOM:
                                                      {
                                                          ROS_ERROR("[Odom Fusion] Input odom is not ready. No odometry output and takeoff should fail #^#");
                                                          break;
                                                      }
                                                      case FsmState::TAKEOFF:
                                                      {
                                                          break;
                                                      }
                                                      case FsmState::FLY_WITH_ODOM:
                                                      {
                                                          if (signal->takeoff_land_cmd == 1)
                                                              ROS_WARN("[Odom Fusion] Takeoff signal received while already flying with odom :|");
                                                          break;
                                                      }
                                                      default:
                                                      {
                                                          ROS_ERROR("[Odom Fusion] \033[36mThe program should never reach here ^^");
                                                          break;
                                                      }
                                                      }
                                                  });

    ros::Subscriber hover_signal_sub =
        nh.subscribe<geometry_msgs::PoseStamped>(param["hover_signal_topic"], 10,
                                                 [&](const geometry_msgs::PoseStamped::ConstPtr &)
                                                 {
                                                     switch (state.load())
                                                     {
                                                     case FsmState::WAIT_FOR_ODOM:
                                                     {
                                                         ROS_ERROR("[Odom Fusion] \033[36mThe program should never reach here ^^");
                                                         break;
                                                     }
                                                     case FsmState::TAKEOFF:
                                                     {
                                                         odom_takeoff_pos_mag_check.stopCheck();
                                                         tof_continuity_check.startCheck();
                                                         state.store(FsmState::FLY_WITH_ODOM);
                                                         ROS_INFO("[Odom Fusion] \033[32mTakeoff done ^v^ \033[43;30mTAKEOFF\033[0m --> \033[43;30mFLY_WITH_ODOM\033[0m :)");
                                                         break;
                                                     }
                                                     case FsmState::FLY_WITH_ODOM:
                                                     {
                                                         ROS_WARN("[Odom Fusion] Hover signal received while already flying with odom :|");
                                                         break;
                                                     }
                                                     default:
                                                     {
                                                         ROS_ERROR("[Odom Fusion] \033[36mThe program should never reach here ^^");
                                                         break;
                                                     }
                                                     }
                                                 });

    thread odom_output_thread;

    //************* Z Correction With ToF or Lidar Map *************//
    int Z_correction_mode = param["Z_correction_mode"];
    double est_z_rate = param["est_z_rate"];
    LowPassFilter<double> z_lpf(param["cutoff_frequency_z"], param["max_delta_z"]);
    atomic<double> z_odom_latest(-23333.33);
    bool publish_debug_topic = (int)param["publish_debug_topic"];
    ros::Publisher debug_z_est_pub = nh.advertise<std_msgs::Float32>("z_est", 10);

    // Z_correction_mode == 1: use z from ToF
    thread est_z_from_tof_thread;
    atomic<double> delta_z_from_tof(0.0);

    mutex latest_q_mtx;
    Quaterniond latest_q;
    auto getToFHeightInWorldZ = [&](const Quaterniond &q, bool clamp_negative_to_zero, double &z_tof) -> bool
    {
        double tof_dist = tof_src.getLatestDataCopy().distance;
        if (tof_dist < 0.0)
        {
            if (!clamp_negative_to_zero)
                return false;
            tof_dist = 0.0;
        }

        Vector3d tof_vec_body(0.0, 0.0, -tof_dist);
        Vector3d tof_vec_world = q * tof_vec_body;
        z_tof = abs(tof_vec_world.z());
        return true;
    };
    auto useZFromToF = [&](nav_msgs::Odometry &odom_output)
    {
        const double z_odom = odom_output.pose.pose.position.z;
        z_odom_latest = z_odom;
        Quaterniond odom_q(odom_output.pose.pose.orientation.w,
                           odom_output.pose.pose.orientation.x,
                           odom_output.pose.pose.orientation.y,
                           odom_output.pose.pose.orientation.z);
        {
            lock_guard<mutex> lock(latest_q_mtx);
            latest_q = odom_q;
        }

        if (state.load() == FsmState::TAKEOFF)
        {
            if (tof_src.isStable())
            {
                double z_tof = 0.0;
                getToFHeightInWorldZ(odom_q, true, z_tof);
                delta_z_from_tof = z_tof - z_odom;
                odom_output.pose.pose.position.z = z_tof;
            }
            else
            {
                ROS_ERROR_THROTTLE(1.0, "[Odom Fusion] ToF stream is unstable during TAKEOFF. Fall back to odom z and reset delta_z_from_tof #^#");
                delta_z_from_tof = 0.0;
            }
            return;
        }

        // When enabling /use_sim_time and playing rosbag with --clock, ros::Time::now() is only updated at 100Hz 😤😤😤
        double now_in_seconds = chrono::duration_cast<chrono::duration<double>>(chrono::high_resolution_clock::now().time_since_epoch()).count();
        z_lpf.input(z_odom + delta_z_from_tof.load(), now_in_seconds);
        odom_output.pose.pose.position.z = z_lpf.output();
    };

    // Z_correction_mode == 2: estimate z from lidar map
    bool recv_revised_odom = false;
    mutex revised_odom_mtx;
    Vector3d latest_revised_p;
    Quaterniond latest_revised_q;
    ros::Time prev_revised_odom_time;
    auto revisedOdomCallback = [&](const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (msg->header.stamp - prev_revised_odom_time < ros::Duration(0.5 / est_z_rate))
            return;

        lock_guard<mutex> lock(revised_odom_mtx);
        latest_revised_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        latest_revised_q = Quaterniond(msg->pose.pose.orientation.w,
                                       msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y,
                                       msg->pose.pose.orientation.z);
        prev_revised_odom_time = msg->header.stamp;

        recv_revised_odom = true;
    };
    ros::Subscriber revised_odom_sub;

    thread est_z_from_lidar_map_thread;
    atomic<double> z_est(0.0);
    atomic<bool> z_est_updated(false);

    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    KdTreeFLANN<PointXYZ> kdtree;
    double search_radius = param["kdtree_search_radius"];
    double min_2nd_singular_value = param["min_2nd_singular_value"];
    double max_3rd_singular_value = param["max_3rd_singular_value"];
    double lidar2tof_z_offset = param["lidar2tof_z_offset"];

    ros::Publisher debug_map_pub = nh.advertise<sensor_msgs::PointCloud2>("map", 10);
    ros::Publisher debug_ground_search_point_pub = nh.advertise<geometry_msgs::PointStamped>("ground_search_point", 10);
    ros::Publisher debug_local_ground_pub = nh.advertise<sensor_msgs::PointCloud2>("local_ground", 10);
    ros::Publisher debug_singular_value_pub = nh.advertise<geometry_msgs::Vector3>("singular_values", 10);
    if (Z_correction_mode == 2)
    {
        revised_odom_sub =
            nh.subscribe<nav_msgs::Odometry>(param["revised_odom_topic"], 1000,
                                             revisedOdomCallback,
                                             ros::VoidConstPtr(),
                                             ros::TransportHints().tcpNoDelay());

        string map_path = argv[2];
        if (pcl::io::loadPCDFile<PointXYZ>(map_path, *cloud) == -1)
        {
            ROS_FATAL_STREAM("[Odom Fusion] Failed to load lidar map from " << (string)param["map_path"]);
            ros::shutdown();
            return -1;
        }
        if (publish_debug_topic)
        {
            sensor_msgs::PointCloud2 map_msg;
            pcl::toROSMsg(*cloud, map_msg);
            map_msg.header.frame_id = "world";
            for (size_t i = 0; i < 13; ++i)
            {
                debug_map_pub.publish(map_msg);
                this_thread::sleep_for(chrono::milliseconds(200));
            }
        }
        kdtree.setInputCloud(cloud);
    }

    double delta_z_from_est = 0.0;
    auto estZFromLidarMap = [&](nav_msgs::Odometry &odom_output)
    {
        z_odom_latest = odom_output.pose.pose.position.z;

        bool ground_smooth = tof_continuity_check.isPassed();
        if (ground_smooth && z_est_updated)
        {
            delta_z_from_est = z_est - odom_output.pose.pose.position.z;
            z_est_updated = false;
        }

        // When enabling /use_sim_time and playing rosbag with --clock, ros::Time::now() is only updated at 100Hz 😤😤😤
        double now_in_seconds = chrono::duration_cast<chrono::duration<double>>(chrono::high_resolution_clock::now().time_since_epoch()).count();
        z_lpf.input(odom_output.pose.pose.position.z + delta_z_from_est, now_in_seconds);
        odom_output.pose.pose.position.z = z_lpf.output();
    };

    int self_id = 250;
    if (getenv("DRONE_ID") != nullptr)
        self_id = atoi(getenv("DRONE_ID"));
    else
        ROS_WARN("[Odom Fusion] DRONE_ID is not found in env var #^#");
    auto changeHeader = [&](nav_msgs::Odometry &odom_output)
    {
        odom_output.child_frame_id = "drone_" + to_string(self_id);
        odom_output.header.frame_id = "world";
    };
    ros::Rate odom_output_rate(odom_src.src_freq);
    ros::Publisher odom_output_pub = nh.advertise<nav_msgs::Odometry>(param["odom_output_topic"], 100);
    bool odom_check_alarm_active = false;
    auto formatDouble = [](double value)
    {
        ostringstream oss;
        oss << fixed << setprecision(3) << value;
        return oss.str();
    };
    auto getOdomCheckFailureDetail = [&]() -> string
    {
        vector<string> issues;
        const double stream_timeout = odom_src.src_timeout < 0.0 ? 5.0 / odom_src.src_freq : odom_src.src_timeout;
        if (!odom_src.isStable())
            issues.emplace_back("data_stream_unstable: no odom received for more than " + formatDouble(stream_timeout) + "s");

        nav_msgs::Odometry latest_odom = odom_src.getLatestDataCopy();
        const double x = latest_odom.pose.pose.position.x - init_x;
        const double y = latest_odom.pose.pose.position.y - init_y;
        const double z = latest_odom.pose.pose.position.z - init_z;
        const double speed = Vector3d(latest_odom.twist.twist.linear.x,
                                      latest_odom.twist.twist.linear.y,
                                      latest_odom.twist.twist.linear.z)
                                 .norm();

        auto appendPosMagIssue = [&](PosMagCheck &check, const string &label)
        {
            if (!check.enable_check.load() || check.isPassed())
                return;

            if (x < check.min_x)
                issues.emplace_back(label + ".x=" + formatDouble(x) + " < min=" + formatDouble(check.min_x));
            else if (x > check.max_x)
                issues.emplace_back(label + ".x=" + formatDouble(x) + " > max=" + formatDouble(check.max_x));
            else if (y < check.min_y)
                issues.emplace_back(label + ".y=" + formatDouble(y) + " < min=" + formatDouble(check.min_y));
            else if (y > check.max_y)
                issues.emplace_back(label + ".y=" + formatDouble(y) + " > max=" + formatDouble(check.max_y));
            else if (z < check.min_z)
                issues.emplace_back(label + ".z=" + formatDouble(z) + " < min=" + formatDouble(check.min_z));
            else if (z > check.max_z)
                issues.emplace_back(label + ".z=" + formatDouble(z) + " > max=" + formatDouble(check.max_z));
            else if (odom_src.isStable())
                issues.emplace_back(label + " failed but current sample is within bounds");
        };

        appendPosMagIssue(odom_takeoff_pos_mag_check, "takeoff_position_check");
        appendPosMagIssue(odom_pos_mag_check, "position_check");

        if (odom_vel_mag_check.enable_check.load() && !odom_vel_mag_check.isPassed())
        {
            if (speed > odom_vel_mag_check.max_vel)
                issues.emplace_back("velocity_check: speed=" + formatDouble(speed) + " > max=" + formatDouble(odom_vel_mag_check.max_vel));
            else if (odom_src.isStable())
                issues.emplace_back("velocity_check failed but current speed=" + formatDouble(speed));
        }

        if (odom_pos_continuity_check.enable_check.load() && !odom_pos_continuity_check.isPassed())
        {
            if (odom_src.getBufSize() >= 2)
            {
                nav_msgs::Odometry previous_odom = odom_src.getPenultimateCopy(1);
                const Vector3d latest_pos(latest_odom.pose.pose.position.x,
                                          latest_odom.pose.pose.position.y,
                                          latest_odom.pose.pose.position.z);
                const Vector3d previous_pos(previous_odom.pose.pose.position.x,
                                            previous_odom.pose.pose.position.y,
                                            previous_odom.pose.pose.position.z);
                const double jump = (latest_pos - previous_pos).norm();
                if (jump > odom_pos_continuity_check.max_jump)
                    issues.emplace_back("position_continuity_check: jump=" + formatDouble(jump) + " > max=" + formatDouble(odom_pos_continuity_check.max_jump));
                else if (odom_pos_continuity_check.unstable)
                {
                    const double remaining_cooling = max(0.0, odom_pos_continuity_check.cooling_time - (ros::Time::now() - odom_pos_continuity_check.unstable_start_time).toSec());
                    issues.emplace_back("position_continuity_check: in cooling window, remaining=" + formatDouble(remaining_cooling) + "s after previous jump");
                }
                else
                    issues.emplace_back("position_continuity_check failed");
            }
            else
                issues.emplace_back("position_continuity_check failed with insufficient buffered samples");
        }

        if (issues.empty())
            issues.emplace_back("unknown odom check failure");

        ostringstream oss;
        for (size_t i = 0; i < issues.size(); ++i)
        {
            if (i != 0)
                oss << "; ";
            oss << issues[i];
        }
        return oss.str();
    };
    auto reportOdomCheckFailure = [&]()
    {
        if (!isOdomCheckPassed())
        {
            ROS_ERROR_STREAM_THROTTLE(1.0, "[Odom Fusion] Input odometry check failed: " << getOdomCheckFailureDetail());
            odom_check_alarm_active = true;
        }
        else if (odom_check_alarm_active)
        {
            ROS_INFO("[Odom Fusion] Input odometry check recovered :)");
            odom_check_alarm_active = false;
        }
    };

    while (ros::ok())
    {
        switch (state.load())
        {
        case FsmState::WAIT_FOR_ODOM:
        {
            bool inform_waiting_for_odom = true;
            while ((!odom_src.isStarted() || (Z_correction_mode == 1 && !tof_src.isStarted())) && ros::ok())
            {
                if (inform_waiting_for_odom)
                {
                    ROS_INFO_STREAM("[Odom Fusion] Waiting for odom" << ((Z_correction_mode == 1) ? " and tof" : "") << " ...");
                    inform_waiting_for_odom = false;
                }
                this_thread::sleep_for(chrono::milliseconds(500));
            }
            ROS_INFO_STREAM("[Odom Fusion] \033[43;30mWAIT_FOR_ODOM\033[0m --> \033[43;30mTAKEOFF\033[0m :)");
            state.store(FsmState::TAKEOFF);
            break;
        }

        case FsmState::TAKEOFF:
        {
            if (!odom_output_thread.joinable())
                odom_output_thread = thread(
                    [&]()
                    {
                        while (ros::ok())
                        {
                            if (state.load() == FsmState::WAIT_FOR_ODOM)
                            {
                                odom_output_rate.sleep();
                                continue;
                            }

                            nav_msgs::Odometry latest_odom = odom_src.getLatestDataCopy();

                            switch (Z_correction_mode)
                            {
                            case 0:
                                break;
                            case 1:
                                useZFromToF(latest_odom);
                                break;
                            case 2:
                                estZFromLidarMap(latest_odom);
                                break;
                            }

                            changeHeader(latest_odom);
                            odom_output_pub.publish(latest_odom);

                            odom_output_rate.sleep();
                        }
                    });

            reportOdomCheckFailure();
            break;
        }

        case FsmState::FLY_WITH_ODOM:
        {
            if (!est_z_from_tof_thread.joinable() && Z_correction_mode == 1 && z_odom_latest > -23333.3)
                est_z_from_tof_thread = thread(
                    [&]()
                    {
                        ros::Rate r(est_z_rate);
                        bool first_z_tof_ = true;
                        double first_z_tof, first_z_odom, prev_z_tof, prev_z_odom;
                        double prev_delta_z = 0;
                        auto resetToFTracking = [&]()
                        {
                            first_z_tof_ = true;
                            prev_delta_z = delta_z_from_tof.load();
                        };
                        while (ros::ok())
                        {
                            if (!tof_continuity_check.isPassed())
                            {
                                ROS_ERROR_THROTTLE(1.0, "[Odom Fusion] ToF continuity check failed !!! Skip this frame of ToF z correction #^#");
                                resetToFTracking();
                                r.sleep();
                                continue;
                            }

                            Quaterniond current_q;
                            {
                                lock_guard<mutex> lock(latest_q_mtx);
                                current_q = latest_q;
                            }

                            double body_z_alignment = std::clamp((current_q * Vector3d::UnitZ()).dot(Vector3d::UnitZ()), -1.0, 1.0);
                            if (acos(body_z_alignment) > (M_PI / 4))
                            {
                                ROS_ERROR("[Odom Fusion] Excessive drone attitude !!! Skip this frame of ToF z correction #^#");
                                resetToFTracking();
                                r.sleep();
                                continue;
                            }

                            double z_tof = 0.0;
                            if (!getToFHeightInWorldZ(current_q, false, z_tof))
                            {
                                ROS_ERROR_THROTTLE(1.0, "[Odom Fusion] Invalid ToF sample (distance < 0) !!! Skip this frame of ToF z correction #^#");
                                resetToFTracking();
                                r.sleep();
                                continue;
                            }

                            double z_odom = z_odom_latest;
                            if (first_z_tof_)
                            {
                                prev_delta_z = delta_z_from_tof.load();
                                first_z_tof = prev_z_tof = z_tof;
                                first_z_odom = prev_z_odom = z_odom;
                                first_z_tof_ = false;
                            }
                            else
                            {
                                double z_tof_diff = z_tof - prev_z_tof;
                                double z_odom_diff = z_odom - prev_z_odom;
                                if (abs(z_tof_diff - z_odom_diff) > 0.5)
                                {
                                    ROS_WARN_STREAM("[Odom Fusion] Exception in delta_z from ToF (diff of delta_z from odom = " << abs(z_tof_diff - z_odom_diff) << "m) #^#");
                                    resetToFTracking();
                                }
                                else
                                {
                                    delta_z_from_tof = (z_tof - first_z_tof) - (z_odom - first_z_odom) + prev_delta_z;

                                    prev_z_tof = z_tof;
                                    prev_z_odom = z_odom;

                                    if (publish_debug_topic)
                                    {
                                        std_msgs::Float32 z_est_msg;
                                        z_est_msg.data = z_tof;
                                        debug_z_est_pub.publish(z_est_msg);
                                    }
                                }
                            }

                            r.sleep();
                        }
                    });

            if (!est_z_from_lidar_map_thread.joinable() && Z_correction_mode == 2 && z_odom_latest > -23333.3)
                est_z_from_lidar_map_thread = thread(
                    [&]()
                    {
                        ros::Rate r(est_z_rate);
                        bool first_z_est = true;
                        double prev_z_est, prev_z_odom;
                        bool inform_insufficient_pts = true;
                        bool inform_bad_ground_shape = true;
                        while (ros::ok())
                        {
                            bool ground_smooth = tof_continuity_check.isPassed();
                            if (recv_revised_odom && ground_smooth)
                            {
                                //************* 1. Compute z with respect to the ground hit point of ToF sensor *************//
                                double tof_dist = tof_src.getLatestDataCopy().distance;
                                Vector3d tof_vec_body(0.0, 0.0, -tof_dist);
                                Vector3d tof_vec_world;
                                Vector3d ground_hit_pt;
                                {
                                    lock_guard<mutex> lock(revised_odom_mtx);
                                    double revised_body_z_alignment = std::clamp((latest_revised_q * Vector3d::UnitZ()).dot(Vector3d::UnitZ()), -1.0, 1.0);
                                    if (acos(revised_body_z_alignment) > (M_PI / 6))
                                    {
                                        ROS_ERROR("[Odom Fusion] Excessive drone attitude !!! Terminate map-based z estimation #^#");
                                        goto end_z_est;
                                    }
                                    tof_vec_world = latest_revised_q * tof_vec_body;
                                    ground_hit_pt = latest_revised_q * tof_vec_body + latest_revised_p + Vector3d(0.0, 0.0, z_lpf.output() - latest_revised_p.z());
                                }
                                double z_to_ground = abs(tof_vec_world.z());

                                if (publish_debug_topic)
                                {
                                    geometry_msgs::PointStamped gsp_msg;
                                    gsp_msg.header.frame_id = "world";
                                    gsp_msg.point.x = ground_hit_pt.x();
                                    gsp_msg.point.y = ground_hit_pt.y();
                                    gsp_msg.point.z = ground_hit_pt.z();
                                    debug_ground_search_point_pub.publish(gsp_msg);
                                }

                                //************* 2. Compute ground height *************//
                                vector<Vector3d> ground_pts;
                                vector<int> ids;
                                vector<float> sqr_dists;
                                PointCloud<PointXYZ>::Ptr local_ground(new PointCloud<PointXYZ>);
                                kdtree.radiusSearch(PointXYZ(ground_hit_pt.x(),
                                                             ground_hit_pt.y(),
                                                             ground_hit_pt.z()),
                                                    search_radius, ids, sqr_dists);
                                for (size_t i = 0; i < ids.size(); ++i)
                                {
                                    PointXYZ pt = (*cloud)[ids[i]];
                                    ground_pts.emplace_back(pt.x, pt.y, pt.z);

                                    if (publish_debug_topic)
                                        local_ground->points.push_back(pt);
                                }

                                if (publish_debug_topic)
                                {
                                    local_ground->width = ground_pts.size();
                                    local_ground->height = 1;
                                    local_ground->is_dense = true;

                                    sensor_msgs::PointCloud2 local_ground_msg;
                                    pcl::toROSMsg(*local_ground, local_ground_msg);
                                    local_ground_msg.header.frame_id = "world";
                                    debug_local_ground_pub.publish(local_ground_msg);
                                }

                                if (ground_pts.size() < 8)
                                {
                                    if (inform_insufficient_pts)
                                    {
                                        ROS_WARN_STREAM("[Odom Fusion] Only " << ids.size() << " (< 8) ground points found around hit point of the ToF sensor. Terminate map-based z estimation #^#");
                                        inform_insufficient_pts = false;
                                    }
                                    goto end_z_est;
                                }
                                else
                                    inform_insufficient_pts = true;

                                // 🥳🥳🥳 PCA 🥳🥳🥳
                                Vector3d centroid(0.0, 0.0, 0.0);
                                for (const auto &pt : ground_pts)
                                    centroid += pt;
                                centroid /= ground_pts.size();

                                MatrixXd centered_pts(3, ground_pts.size());
                                for (size_t i = 0; i < ground_pts.size(); ++i)
                                    centered_pts.col(i) = ground_pts[i] - centroid;

                                JacobiSVD<MatrixXd> svd(centered_pts, ComputeThinU | ComputeThinV);
                                Vector3d singular_values = svd.singularValues();
                                ROS_DEBUG_STREAM("[Odom Fusion] Singular values of the ground (" << ids.size() << " points) around the ToF hit point: " << singular_values.transpose() << "  Normal: " << svd.matrixU().col(2).transpose());
                                if (publish_debug_topic)
                                {
                                    geometry_msgs::Vector3 singular_values_msg;
                                    singular_values_msg.x = singular_values(0);
                                    singular_values_msg.y = singular_values(1);
                                    singular_values_msg.z = singular_values(2);
                                    debug_singular_value_pub.publish(singular_values_msg);
                                }

                                if ((singular_values(1) < min_2nd_singular_value ||
                                     singular_values(2) > max_3rd_singular_value) &&
                                    (singular_values(1) / singular_values(2)) < 4)
                                {
                                    if (inform_bad_ground_shape)
                                    {
                                        ROS_WARN("[Odom Fusion] Ground around the ToF hit point is not like a plane. Terminate map-based z estimation #^#");
                                        inform_bad_ground_shape = false;
                                    }
                                    goto end_z_est;
                                }
                                else
                                    inform_bad_ground_shape = true;

                                Vector3d normal = svd.matrixU().col(2);
                                double normal_tilt_angle = acos(std::clamp(normal.dot(Vector3d::UnitZ()) / normal.norm(), -1.0, 1.0));
                                if (normal_tilt_angle > M_PI / 2)
                                    normal_tilt_angle = M_PI - normal_tilt_angle;
                                if (normal_tilt_angle > M_PI / 4)
                                {
                                    ROS_WARN_STREAM("[Odom Fusion] Ground plane around the ToF hit point is tilted too much (" << normal_tilt_angle * 180.0 / M_PI << " > 45 deg). Terminate map-based z estimation #^#");
                                    goto end_z_est;
                                }

                                double z_ground = centroid.z() + lidar2tof_z_offset;

                                //************* 3. Estimate z *************//
                                z_est = z_to_ground + z_ground;

                                if (first_z_est)
                                {
                                    prev_z_est = z_est;
                                    prev_z_odom = z_odom_latest;
                                    first_z_est = false;
                                }
                                else
                                {
                                    double z_est_diff = z_est - prev_z_est;
                                    double z_odom_diff = z_odom_latest - prev_z_odom;
                                    if (abs(z_est_diff - z_odom_diff) > 0.23)
                                    {
                                        ROS_WARN_STREAM("[Odom Fusion] Exception in z estimation (diff of speed.z = " << abs(z_est_diff - z_odom_diff) << "m) #^#");
                                        z_est_updated = false;
                                        first_z_est = true;
                                    }
                                    else
                                    {
                                        z_est_updated = true;
                                        prev_z_est = z_est;
                                        prev_z_odom = z_odom_latest;

                                        if (publish_debug_topic)
                                        {
                                            std_msgs::Float32 z_est_msg;
                                            z_est_msg.data = z_est;
                                            debug_z_est_pub.publish(z_est_msg);
                                        }
                                    }
                                }
                            }
                        end_z_est:
                            r.sleep();
                        }
                    });

            reportOdomCheckFailure();
            break;
        }

        default:
            break;
        }
        data_check_rate.sleep();
    }
    return 0;
}
