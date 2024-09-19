#include <atomic>
#include <string>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Eigen>

#include "DataSrc.hpp"
#include "PosMagCheck.hpp"
#include "VelMagCheck.hpp"
#include "ToFHgtCheck.hpp"
#include "PosContinuityCheck.hpp"
#include "ToFContinuityCheck.hpp"

#include "traj_utils/take_off.h"
#include "quadrotor_msgs/TakeoffLand.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include "LowPassFilter.hpp"

using namespace std;
using namespace Eigen;
using namespace pcl;

enum FsmState
{
    WAIT_FOR_FCU_ODOM,
    CALIB_FCU_ODOM,
    TAKEOFF,
    FLY_WITH_VINS,
    FLY_WITH_MSCKF,
    FLY_WITH_FCU_ODOM
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_fusion_node");
    ros::NodeHandle nh("~");

    string hyperparam_path = argv[1];
    cv::FileStorage param(hyperparam_path, cv::FileStorage::READ);

    //************* Data sources *************//
    DataSrc<nav_msgs::Odometry> vins_odom_src(nh, param["vins_odom_topic"], param["vins_odom_freq"], param["vins_odom_timeout"]);
    DataSrc<nav_msgs::Odometry> msckf_odom_src(nh, param["msckf_odom_topic"], param["msckf_odom_freq"], param["msckf_odom_timeout"]);
    DataSrc<nav_msgs::Odometry> fcu_odom_src(nh, param["fcu_odom_topic"], param["fcu_odom_freq"], param["fcu_odom_timeout"]);

    double init_x, init_y, init_z;
    if (getenv("ipx") != nullptr && getenv("ipy") != nullptr && getenv("ipz") != nullptr)
    {
        init_x = atof(getenv("ipx"));
        init_y = atof(getenv("ipy"));
        init_z = atof(getenv("ipz"));
    }
    else
        init_x = init_y = init_z = 0.0;

    DataSrc<sensor_msgs::Imu> imu_src(nh, param["imu_topic"], param["imu_freq"]);
    DataSrc<mavros_msgs::OpticalFlowRad> tof_src(nh, param["tof_topic"], param["tof_freq"], param["tof_timeout"]);

    ros::AsyncSpinner spinner(10);
    spinner.start();

    //************* Data checks *************//
    PosMagCheck vins_takeoff_pos_mag_check(vins_odom_src, param["pos_mag_check_freq"],
                                           param["min_x_takeoff"], param["max_x_takeoff"],
                                           param["min_y_takeoff"], param["max_y_takeoff"],
                                           param["min_z_takeoff"], param["max_z_takeoff"],
                                           init_x, init_y, init_z);
    PosMagCheck msckf_takeoff_pos_mag_check(msckf_odom_src, param["pos_mag_check_freq"],
                                            param["min_x_takeoff"], param["max_x_takeoff"],
                                            param["min_y_takeoff"], param["max_y_takeoff"],
                                            param["min_z_takeoff"], param["max_z_takeoff"],
                                            init_x, init_y, init_z);
    PosMagCheck fcu_odom_takeoff_pos_mag_check(fcu_odom_src, param["pos_mag_check_freq"],
                                               param["min_x_takeoff"], param["max_x_takeoff"],
                                               param["min_y_takeoff"], param["max_y_takeoff"],
                                               param["min_z_takeoff"], param["max_z_takeoff"],
                                               0.0, 0.0, 0.0);

    PosMagCheck vins_pos_mag_check(vins_odom_src, param["pos_mag_check_freq"],
                                   param["min_x"], param["max_x"],
                                   param["min_y"], param["max_y"],
                                   param["min_z"], param["max_z"],
                                   init_x, init_y, init_z);
    PosMagCheck msckf_pos_mag_check(msckf_odom_src, param["pos_mag_check_freq"],
                                    param["min_x"], param["max_x"],
                                    param["min_y"], param["max_y"],
                                    param["min_z"], param["max_z"],
                                    init_x, init_y, init_z);
    PosMagCheck fcu_odom_pos_mag_check(fcu_odom_src, param["pos_mag_check_freq"],
                                       param["min_x"], param["max_x"],
                                       param["min_y"], param["max_y"],
                                       param["min_z"], param["max_z_fcu_odom"],
                                       0.0, 0.0, 0.0);

    ToFHgtCheck tof_conservative_hgt_check(tof_src, param["tof_conservative_hgt_check_freq"], param["max_conservative_hgt"]);
    ToFHgtCheck tof_hgt_check(tof_src, param["tof_hgt_check_freq"], param["max_hgt"]);

    VelMagCheck vins_vel_mag_check(vins_odom_src, param["vel_mag_check_freq"], param["max_vel"]);
    VelMagCheck msckf_vel_mag_check(msckf_odom_src, param["vel_mag_check_freq"], param["max_vel"]);
    VelMagCheck fcu_odom_vel_mag_check(fcu_odom_src, param["vel_mag_check_freq"], param["max_vel"]);

    PosContinuityCheck vins_pos_continuity_check(vins_odom_src, param["continuity_check_freq"], param["pos_max_jump"], param["pos_jump_cooling_time"]);
    PosContinuityCheck msckf_pos_continuity_check(msckf_odom_src, param["continuity_check_freq"], param["pos_max_jump"], param["pos_jump_cooling_time"]);
    PosContinuityCheck fcu_odom_pos_continuity_check(fcu_odom_src, param["continuity_check_freq"], param["pos_max_jump"], param["pos_jump_cooling_time"]);
    ToFContinuityCheck tof_continuity_check(tof_src, param["continuity_check_freq"], param["tof_max_jump"], param["tof_jump_cooling_time"]);

    // Fcu odometer is unreliable until the propeller is spinned
    fcu_odom_takeoff_pos_mag_check.stopCheck();
    fcu_odom_pos_mag_check.stopCheck();
    fcu_odom_vel_mag_check.stopCheck();
    fcu_odom_pos_continuity_check.stopCheck();

    auto isVinsCheckPassed = [&]()
    {
        return vins_takeoff_pos_mag_check.isPassed() &&
               vins_pos_mag_check.isPassed() &&
               vins_vel_mag_check.isPassed() &&
               vins_pos_continuity_check.isPassed();
    };
    auto isMsckfCheckPassed = [&]()
    {
        return msckf_takeoff_pos_mag_check.isPassed() &&
               msckf_pos_mag_check.isPassed() &&
               msckf_vel_mag_check.isPassed();
    };
    auto isFcuOdomCheckPassed = [&]()
    {
        return fcu_odom_takeoff_pos_mag_check.isPassed() &&
               fcu_odom_pos_mag_check.isPassed() &&
               fcu_odom_vel_mag_check.isPassed() &&
               fcu_odom_pos_continuity_check.isPassed() &&
               tof_hgt_check.isPassed() &&
               tof_continuity_check.isPassed();
    };

    //************* Main loop *************//
    FsmState state(FsmState::WAIT_FOR_FCU_ODOM);
    const double data_check_freq = max({10.0,
                                        (double)param["pos_mag_check_freq"],
                                        (double)param["tof_conservative_hgt_check_freq"],
                                        (double)param["tof_hgt_check_freq"],
                                        (double)param["vel_mag_check_freq"],
                                        (double)param["continuity_check_freq"],
                                        (double)param["imu_amp_check_freq"]});
    ros::Rate data_check_rate(data_check_freq);

    // Calibrate initial position and yaw bias of the fcu odometry for takeoff
    // Note that these static biases may only be used during takeoff and can become dynamic during flight
    thread calib_fcu_odom_thread;
    Vector3d p_fcu_bias;
    Quaterniond q_fcu_bias;
    auto debiasFcuOdom = [&p_fcu_bias, &q_fcu_bias, init_x, init_y, init_z](const nav_msgs::Odometry &fcu_odom, nav_msgs::Odometry &fcu_odom_debiased)
    {
        Vector3d p(fcu_odom.pose.pose.position.x,
                   fcu_odom.pose.pose.position.y,
                   fcu_odom.pose.pose.position.z);
        Quaterniond q(fcu_odom.pose.pose.orientation.w,
                      fcu_odom.pose.pose.orientation.x,
                      fcu_odom.pose.pose.orientation.y,
                      fcu_odom.pose.pose.orientation.z);
        Vector3d v(fcu_odom.twist.twist.linear.x,
                   fcu_odom.twist.twist.linear.y,
                   fcu_odom.twist.twist.linear.z);

        Vector3d p_debiased = q_fcu_bias.inverse() * (p - p_fcu_bias);
        Quaterniond q_debiased = q_fcu_bias.inverse() * q;
        Vector3d v_world = q_debiased * v;

        fcu_odom_debiased = fcu_odom; // FIXME: necessary copy ?
        fcu_odom_debiased.pose.pose.position.x = p_debiased.x() + init_x;
        fcu_odom_debiased.pose.pose.position.y = p_debiased.y() + init_y;
        fcu_odom_debiased.pose.pose.position.z = p_debiased.z() + init_z;
        fcu_odom_debiased.twist.twist.linear.x = v_world.x();
        fcu_odom_debiased.twist.twist.linear.y = v_world.y();
        fcu_odom_debiased.twist.twist.linear.z = v_world.z();
        fcu_odom_debiased.pose.pose.orientation.w = q_debiased.w();
        fcu_odom_debiased.pose.pose.orientation.x = q_debiased.x();
        fcu_odom_debiased.pose.pose.orientation.y = q_debiased.y();
        fcu_odom_debiased.pose.pose.orientation.z = q_debiased.z();
    };

    ros::Subscriber takeoff_signal_sub_0 =
        nh.subscribe<traj_utils::take_off>(param["takeoff_signal_topic_0"], 10,
                                           [&](const traj_utils::take_off::ConstPtr &signal)
                                           {
                                               switch (state)
                                               {
                                               case FsmState::WAIT_FOR_FCU_ODOM:
                                               case FsmState::CALIB_FCU_ODOM:
                                               {
                                                   ROS_ERROR("[Odom Fusion] Fcu odometry is not ready. No odometry output and takeoff should fail #^#");
                                                   break;
                                               }
                                               case FsmState::TAKEOFF:
                                               {
                                                   this_thread::sleep_for(chrono::seconds(2)); // Wait for the propeller to start spinning
                                                   fcu_odom_takeoff_pos_mag_check.startCheck();
                                                   fcu_odom_pos_mag_check.startCheck();
                                                   fcu_odom_vel_mag_check.startCheck();
                                                   fcu_odom_pos_continuity_check.startCheck();
                                                   break;
                                               }
                                               case FsmState::FLY_WITH_VINS:
                                               case FsmState::FLY_WITH_MSCKF:
                                               case FsmState::FLY_WITH_FCU_ODOM:
                                               {
                                                   ROS_INFO("[Odom Fusion] Landing signal received :|");
                                                   break;
                                               }
                                               default:
                                               {
                                                   ROS_ERROR("[Odom Fusion] \033[36mThe program should never reach here ^^");
                                                   break;
                                               }
                                               }
                                           });

    ros::Subscriber takeoff_signal_sub_1 =
        nh.subscribe<quadrotor_msgs::TakeoffLand>(param["takeoff_signal_topic_1"], 10,
                                                  [&](const quadrotor_msgs::TakeoffLand::ConstPtr &signal)
                                                  {
                                                      switch (state)
                                                      {
                                                      case FsmState::WAIT_FOR_FCU_ODOM:
                                                      case FsmState::CALIB_FCU_ODOM:
                                                      {
                                                          ROS_ERROR("[Odom Fusion] Fcu odometry is not ready. No odometry output and takeoff should fail #^#");
                                                          break;
                                                      }
                                                      case FsmState::TAKEOFF:
                                                      {
                                                          this_thread::sleep_for(chrono::seconds(2)); // Wait for the propeller to start spinning
                                                          fcu_odom_takeoff_pos_mag_check.startCheck();
                                                          fcu_odom_pos_mag_check.startCheck();
                                                          fcu_odom_vel_mag_check.startCheck();
                                                          fcu_odom_pos_continuity_check.startCheck();
                                                          break;
                                                      }
                                                      case FsmState::FLY_WITH_VINS:
                                                      case FsmState::FLY_WITH_MSCKF:
                                                      case FsmState::FLY_WITH_FCU_ODOM:
                                                      {
                                                          if (signal->takeoff_land_cmd == 1)
                                                              ROS_WARN("[Odom Fusion] Takeoff signal received but odometry has been switched. Check fcu odometry please :|");
                                                          break;
                                                      }
                                                      default:
                                                      {
                                                          ROS_ERROR("[Odom Fusion] \033[36mThe program should never reach here ^^");
                                                          break;
                                                      }
                                                      }
                                                  });

    // Odometry restart services
    ros::ServiceClient vins_restart_srv = nh.serviceClient<std_srvs::Trigger>((string)param["vins_restart_service"]);
    ros::ServiceClient msckf_restart_srv = nh.serviceClient<std_srvs::Trigger>((string)param["msckf_restart_service"]);
    auto restartOdom = [](ros::ServiceClient &odom_restart_srv, DataSrc<nav_msgs::Odometry> &odom_src)
    {
        std_srvs::Trigger restart;
        odom_restart_srv.call(restart);
        ROS_INFO("[Odom Fusion] Odom restart service called :|");

        odom_src.restart();
    };

    ros::Subscriber hover_signal_sub =
        nh.subscribe<geometry_msgs::PoseStamped>(param["hover_signal_topic"], 10,
                                                 [&](const geometry_msgs::PoseStamped::ConstPtr &signal)
                                                 {
                                                     vins_takeoff_pos_mag_check.stopCheck();
                                                     msckf_takeoff_pos_mag_check.stopCheck();
                                                     fcu_odom_takeoff_pos_mag_check.stopCheck();

                                                     switch (state)
                                                     {
                                                     case FsmState::WAIT_FOR_FCU_ODOM:
                                                     case FsmState::CALIB_FCU_ODOM:
                                                     {
                                                         ROS_ERROR("[Odom Fusion] \033[36mThe program should never reach here ^^");
                                                         break;
                                                     }
                                                     case FsmState::TAKEOFF:
                                                     {
                                                         state = FsmState::FLY_WITH_VINS;
                                                         ROS_INFO("[Odom Fusion] \033[32mTakeoff done ^v^");
                                                         break;
                                                     }
                                                     case FsmState::FLY_WITH_VINS:
                                                     case FsmState::FLY_WITH_MSCKF:
                                                     case FsmState::FLY_WITH_FCU_ODOM:
                                                     {
                                                         ROS_WARN("[Odom Fusion] Hover signal received but odometry has been switched during takeoff :|");
                                                         break;
                                                     }
                                                     default:
                                                     {
                                                         ROS_ERROR("[Odom Fusion] \033[36mThe program should never reach here ^^");
                                                         break;
                                                     }
                                                     }
                                                 });

    // Ensure currently unused odometry sources are available as backups
    ros::Rate logistic_rate(data_check_freq);
    thread logistic_thread = thread(
        [&]()
        {
            while (ros::ok())
            {
                if (state != FsmState::FLY_WITH_VINS && state != FsmState::TAKEOFF &&
                    vins_odom_src.isStarted() && // If the data source never starts at program startup, or midway through after sending the restart signal, the program will be helpless 😥😥😥
                    !isVinsCheckPassed() && vins_odom_src.isAvailable())
                {
                    ROS_ERROR("[Odom Fusion] Exceptions in vins odometry #^# Restarting ...");
                    vins_odom_src.setUnavailable();
                    restartOdom(vins_restart_srv, vins_odom_src);
                }
                else if (vins_odom_src.isStarted() && isVinsCheckPassed() && !vins_odom_src.isAvailable())
                {
                    ROS_INFO_STREAM("[Odom Fusion] \033[32mVins odometry becomes available :)");
                    vins_odom_src.setAvailable();
                }

                if (state != FsmState::FLY_WITH_MSCKF &&
                    msckf_odom_src.isStarted() &&
                    !isMsckfCheckPassed() && msckf_odom_src.isAvailable())
                {
                    ROS_ERROR("[Odom Fusion] Exceptions in msckf odometry #^# Restarting ...");
                    msckf_odom_src.setUnavailable();
                    restartOdom(msckf_restart_srv, msckf_odom_src);
                }
                else if (msckf_odom_src.isStarted() && isMsckfCheckPassed() && !msckf_odom_src.isAvailable())
                {
                    ROS_INFO_STREAM("[Odom Fusion] \033[32mMsckf odometry becomes available :)");
                    msckf_odom_src.setAvailable();
                }

                if (state != FsmState::CALIB_FCU_ODOM && state != FsmState::FLY_WITH_FCU_ODOM &&
                    fcu_odom_src.isStarted() &&
                    !isFcuOdomCheckPassed() && fcu_odom_src.isAvailable())
                {
                    fcu_odom_src.setUnavailable();
                    ROS_ERROR("[Odom Fusion] Exceptions in fcu odometry #^# Fcu odometry restart functionality is not supported yet !!! Pretend it's been restarted #_#");
                }
                else if (fcu_odom_src.isStarted() && isFcuOdomCheckPassed() && !fcu_odom_src.isAvailable())
                {
                    ROS_INFO_STREAM("[Odom Fusion] \033[32mFcu odometry becomes available :)");
                    fcu_odom_src.setAvailable();
                }

                logistic_rate.sleep();
            }
        });

    thread odom_output_thread;
    Vector3d p_smoother_p(0.0, 0.0, 0.0);
    Quaterniond p_smoother_q(1.0, 0.0, 0.0, 0.0);
    Quaterniond q_smoother(1.0, 0.0, 0.0, 0.0);
    double smoother_decay_const_p = param["smoother_decay_const_p"];
    double smoother_decay_const_yaw = param["smoother_decay_const_yaw"];
    int smoother_decay_timestep = 0;
    auto applySmoother = [&](nav_msgs::Odometry &odom_output)
    {
        Vector3d p(odom_output.pose.pose.position.x,
                   odom_output.pose.pose.position.y,
                   odom_output.pose.pose.position.z);
        Quaterniond q(odom_output.pose.pose.orientation.w,
                      odom_output.pose.pose.orientation.x,
                      odom_output.pose.pose.orientation.y,
                      odom_output.pose.pose.orientation.z);

        Vector3d p_smoothed = p_smoother_q.slerp(1 - exp(-smoother_decay_const_p * smoother_decay_timestep), Quaterniond::Identity()) * p +
                              exp(-smoother_decay_const_p * smoother_decay_timestep) * p_smoother_p;
        Quaterniond q_smoothed = q_smoother.slerp(1 - exp(-smoother_decay_const_yaw * smoother_decay_timestep), Quaterniond::Identity()) * q;
        smoother_decay_timestep++;

        odom_output.pose.pose.position.x = p_smoothed.x();
        odom_output.pose.pose.position.y = p_smoothed.y();
        odom_output.pose.pose.position.z = p_smoothed.z();
        odom_output.pose.pose.orientation.w = q_smoothed.w();
        odom_output.pose.pose.orientation.x = q_smoothed.x();
        odom_output.pose.pose.orientation.y = q_smoothed.y();
        odom_output.pose.pose.orientation.z = q_smoothed.z();
    };
    auto generateSmoother = [&](nav_msgs::Odometry &odom_prv, nav_msgs::Odometry &odom_fut)
    {
        applySmoother(odom_prv);

        const Vector3d p_prv(odom_prv.pose.pose.position.x,
                             odom_prv.pose.pose.position.y,
                             odom_prv.pose.pose.position.z);
        const Vector3d p_fut(odom_fut.pose.pose.position.x,
                             odom_fut.pose.pose.position.y,
                             odom_fut.pose.pose.position.z);
        const Quaterniond q_prv(odom_prv.pose.pose.orientation.w,
                                odom_prv.pose.pose.orientation.x,
                                odom_prv.pose.pose.orientation.y,
                                odom_prv.pose.pose.orientation.z);
        const Quaterniond q_fut(odom_fut.pose.pose.orientation.w,
                                odom_fut.pose.pose.orientation.x,
                                odom_fut.pose.pose.orientation.y,
                                odom_fut.pose.pose.orientation.z);

        Matrix3d R_diff = (q_prv * q_fut.inverse()).toRotationMatrix();
        double yaw_diff = atan2(R_diff(1, 0), R_diff(0, 0));
        p_smoother_q = AngleAxisd(yaw_diff, Vector3d::UnitZ());

        p_smoother_p = p_prv - p_smoother_q * p_fut;
        q_smoother = p_smoother_q;
        smoother_decay_timestep = 0;
    };
    auto toWorldVel = [&](nav_msgs::Odometry &odom)
    {
        Quaterniond q_b2w(odom.pose.pose.orientation.w,
                          odom.pose.pose.orientation.x,
                          odom.pose.pose.orientation.y,
                          odom.pose.pose.orientation.z);

        Vector3d v_body(odom.twist.twist.linear.x,
                        odom.twist.twist.linear.y,
                        odom.twist.twist.linear.z);
        Vector3d v_world = q_b2w * v_body;

        odom.twist.twist.linear.x = v_world.x();
        odom.twist.twist.linear.y = v_world.y();
        odom.twist.twist.linear.z = v_world.z();
    };

    int Z_mode = param["Z_mode"];

    // Z_mode == 1: use z from ToF
    bool first_use_tof_z = true;
    bool prev_tof_valid;
    double prev_odom_z;
    double z_smoother = 0.0;
    auto useZFromToF = [&](nav_msgs::Odometry &odom_output)
    {
        bool tof_valid = fcu_odom_src.isStarted() && isFcuOdomCheckPassed();
        if (tof_valid)
        {
            nav_msgs::Odometry latest_fcu_odom = fcu_odom_src.getLatestDataCopy();
            nav_msgs::Odometry latest_fcu_odom_debiased;
            debiasFcuOdom(latest_fcu_odom, latest_fcu_odom_debiased);
            odom_output.pose.pose.position.z = latest_fcu_odom_debiased.pose.pose.position.z;
        }

        // Z smoother
        if (first_use_tof_z)
            first_use_tof_z = false;
        else if (tof_valid != prev_tof_valid)
            z_smoother = prev_odom_z - odom_output.pose.pose.position.z;
        odom_output.pose.pose.position.z += z_smoother;
        prev_tof_valid = tof_valid;
        prev_odom_z = odom_output.pose.pose.position.z;
    };

    // Z_mode == 2: estimate z from lidar map
    double est_z_rate = param["est_z_rate"];
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
    atomic<bool> z_est_available(false);
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    KdTreeFLANN<PointXYZ> kdtree;
    double search_radius = param["kdtree_search_radius"];
    if (Z_mode == 2)
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
        kdtree.setInputCloud(cloud);
    }
    double delta_z = 0.0;
    LowPassFilter<double> delta_z_lpf(param["cutoff_frequency_z"], param["max_delta_z"]);
    auto estZFromLidarMap = [&](nav_msgs::Odometry &odom_output)
    {
        bool ground_smooth = tof_src.isStarted() && tof_continuity_check.isPassed();
        if (ground_smooth && z_est_available)
            delta_z = z_est - odom_output.pose.pose.position.z;

        delta_z_lpf.input(delta_z, ros::Time::now().toSec());

        odom_output.pose.pose.position.z += delta_z_lpf.output();
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
    ros::Rate odom_output_rate(vins_odom_src.src_freq); // Vins is used first during takeoff
    ros::Publisher odom_output_pub = nh.advertise<nav_msgs::Odometry>(param["odom_output_topic"], 100);

    thread commu_with_px4ctrl_thread;
    ros::Rate commu_with_px4ctrl_rate(data_check_freq);
    bool allow_high_altitude_flight;
    param["allow_high_altitude_flight"] >> allow_high_altitude_flight;
    bool allow_px4ctrl_cmd_ctrl = true;
    ros::ServiceClient set_cmd_ctrl_permission_srv = nh.serviceClient<std_srvs::SetBool>(param["set_px4ctrl_cmd_ctrl_permission_service"]);

    while (ros::ok())
    {
        bool inform_bad_odom = true;
        switch (state)
        {
        case FsmState::WAIT_FOR_FCU_ODOM:
        {
            if (fcu_odom_src.isStarted() && tof_src.isStarted())
            {
                state = FsmState::CALIB_FCU_ODOM;
                // FIXME: use GLOG please 😣😣😣
                ROS_INFO_STREAM("[Odom Fusion] \033[43;30mWAIT_FOR_FCU_ODOM\033[0m --> \033[43;30mCALIB_FCU_ODOM\033[0m :)");
            }
            break;
        }

        case FsmState::CALIB_FCU_ODOM:
        {
            if (!calib_fcu_odom_thread.joinable())
                calib_fcu_odom_thread = thread(
                    [&]()
                    {
                        double x_avg = 0.0, y_avg = 0.0, z_avg = 0.0, yaw_avg = 0.0;
                        ros::Rate calib_proc_rate(fcu_odom_src.src_freq);
                        double first_yaw = 0.0;
                        for (size_t i = 1; ros::ok(); i++)
                        {
                            // TODO: retrieve const ref not value
                            nav_msgs::Odometry latest_fcu_odom = fcu_odom_src.getLatestDataCopy();
                            double x = latest_fcu_odom.pose.pose.position.x;
                            double y = latest_fcu_odom.pose.pose.position.y;
                            double z = latest_fcu_odom.pose.pose.position.z;
                            Matrix3d R = Quaterniond(latest_fcu_odom.pose.pose.orientation.w,
                                                     latest_fcu_odom.pose.pose.orientation.x,
                                                     latest_fcu_odom.pose.pose.orientation.y,
                                                     latest_fcu_odom.pose.pose.orientation.z)
                                             .toRotationMatrix();
                            double yaw = atan2(R(1, 0), R(0, 0));

                            if (i != 1)
                            {
                                if (abs(x - x_avg) > 0.013 || abs(y - y_avg) > 0.013 || abs(z - z_avg) > 0.013 || abs(yaw - yaw_avg) > 0.013)
                                {
                                    state = FsmState::WAIT_FOR_FCU_ODOM;
                                    ROS_ERROR("[Odom Fusion] \033[43;30mCALIB_FCU_ODOM\033[0m --> \033[43;30mWAIT_FOR_FCU_ODOM\033[0m Unstable initial z or yaw bias of fcu odometry, calibration restarted #^#");
                                    continue;
                                }

                                if (abs(yaw) > 3.1015926)
                                {
                                    if (abs(first_yaw) < 1e-5)
                                        first_yaw = yaw;

                                    if (yaw * first_yaw > 0)
                                        ;
                                    else
                                        yaw *= -1;

                                    ROS_WARN("[Odom Fusion] Initial yaw bias of fcu odometry is close to PI and -PI @_@");
                                }
                            }

                            x_avg += (x - x_avg) / i;
                            y_avg += (y - y_avg) / i;
                            z_avg += (z - z_avg) / i;
                            yaw_avg += (yaw - yaw_avg) / i;

                            if (i > 77) // or 250 or 520 or 1314
                            {
                                p_fcu_bias << x_avg, y_avg, z_avg;
                                q_fcu_bias = Quaterniond(AngleAxisd(yaw_avg, Vector3d::UnitZ()));
                                ROS_INFO_STREAM("[Odom Fusion] \033[32mCalibration finished \\^_^/ bias x = " << x_avg << "m, bias y = " << y_avg << "m, bias z = " << z_avg << "m, yaw = " << yaw_avg << "rad");

                                bool inform_waiting_for_vins = true;
                                while (!vins_odom_src.isAvailable() && ros::ok())
                                {
                                    if (inform_waiting_for_vins)
                                    {
                                        ROS_INFO_STREAM("[Odom Fusion] Waiting for vins ...");
                                        inform_waiting_for_vins = false;
                                    }
                                    this_thread::sleep_for(chrono::milliseconds(500));
                                }
                                state = FsmState::TAKEOFF;
                                ROS_INFO_STREAM("[Odom Fusion] \033[43;30mCALIB_FCU_ODOM\033[0m --> \033[43;30mTAKEOFF\033[0m :)");
                                break;
                            }

                            if (!isFcuOdomCheckPassed())
                            {
                                state = FsmState::WAIT_FOR_FCU_ODOM;
                                ROS_ERROR("[Odom Fusion] \033[43;30mCALIB_FCU_ODOM\033[0m --> \033[43;30mWAIT_FOR_FCU_ODOM\033[0m Exceptions in fcu odometry (probably no optical flow), calicration terminated #^#");
                                ROS_ERROR("[Odom Fusion] Fcu odometry restart functionality is not supported yet !!! Pretend it's been restarted #^#");
                                break;
                            }

                            calib_proc_rate.sleep();
                        }
                    });
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
                            switch (state)
                            {
                            case FsmState::TAKEOFF:
                            case FsmState::FLY_WITH_VINS:
                            {
                                nav_msgs::Odometry latest_vins_odom = vins_odom_src.getLatestDataCopy();

                                applySmoother(latest_vins_odom);

                                switch (Z_mode)
                                {
                                case 0:
                                    break;
                                case 1:
                                    useZFromToF(latest_vins_odom);
                                    break;
                                case 2:
                                    estZFromLidarMap(latest_vins_odom);
                                    break;
                                }

                                changeHeader(latest_vins_odom);
                                odom_output_pub.publish(latest_vins_odom);
                                break;
                            }

                            case FsmState::FLY_WITH_MSCKF:
                            {
                                nav_msgs::Odometry latest_msckf_odom = msckf_odom_src.getLatestDataCopy();

                                applySmoother(latest_msckf_odom);

                                switch (Z_mode)
                                {
                                case 0:
                                    break;
                                case 1:
                                    useZFromToF(latest_msckf_odom);
                                    break;
                                case 2:
                                    estZFromLidarMap(latest_msckf_odom);
                                    break;
                                }

                                changeHeader(latest_msckf_odom);
                                odom_output_pub.publish(latest_msckf_odom);
                                break;
                            }

                            case FsmState::FLY_WITH_FCU_ODOM:
                            {
                                nav_msgs::Odometry latest_fcu_odom = fcu_odom_src.getLatestDataCopy();
                                nav_msgs::Odometry latest_fcu_odom_debiased;
                                debiasFcuOdom(latest_fcu_odom, latest_fcu_odom_debiased);

                                applySmoother(latest_fcu_odom_debiased);
                                toWorldVel(latest_fcu_odom_debiased);

                                switch (Z_mode)
                                {
                                case 0:
                                    break;
                                case 1:
                                    useZFromToF(latest_fcu_odom_debiased);
                                    break;
                                case 2:
                                    estZFromLidarMap(latest_fcu_odom_debiased);
                                    break;
                                }

                                changeHeader(latest_fcu_odom_debiased);
                                odom_output_pub.publish(latest_fcu_odom_debiased);
                                break;
                            }

                            default:
                            {
                                ROS_ERROR("[Odom Fusion] \033[36mThe program should never reach here ^^ No odom output *_*");
                                break;
                            }
                            }

                            odom_output_rate.sleep();
                        }
                    });

            if (!commu_with_px4ctrl_thread.joinable())
                commu_with_px4ctrl_thread = thread(
                    [&]()
                    {
                        while (ros::ok())
                        {
                            bool safe_to_enable_cmd_ctrl =
                                (allow_high_altitude_flight ? true : (tof_src.isStarted() && tof_conservative_hgt_check.isPassed())) &&
                                ((vins_odom_src.isStarted() && isVinsCheckPassed()) ||
                                 (msckf_odom_src.isStarted() && isMsckfCheckPassed()));

                            if (safe_to_enable_cmd_ctrl && !allow_px4ctrl_cmd_ctrl)
                            {
                                std_srvs::SetBool allow;
                                allow.request.data = true;
                                set_cmd_ctrl_permission_srv.call(allow);
                                allow_px4ctrl_cmd_ctrl = true;
                                ROS_INFO("[Odom Fusion] \033[32mVIO ready and flight altitude safe !!! Allow command control ^_^");
                            }
                            else if (!safe_to_enable_cmd_ctrl && allow_px4ctrl_cmd_ctrl)
                            {
                                std_srvs::SetBool disallow;
                                disallow.request.data = false;
                                set_cmd_ctrl_permission_srv.call(disallow);
                                allow_px4ctrl_cmd_ctrl = false;
                                ROS_ERROR("[Odom Fusion] Exceptions in VIOs or flight altitude exceeding maximum range !!! Prohibit command control #^#");
                            }

                            commu_with_px4ctrl_rate.sleep();
                        }
                    });

            // Try to switch to another odometer when there are exceptions in the current odometer
            if (!isVinsCheckPassed())
            {
                if (msckf_odom_src.isAvailable())
                {
                    nav_msgs::Odometry latest_vins_odom = vins_odom_src.getLatestDataCopy();

                    nav_msgs::Odometry latest_msckf_odom = msckf_odom_src.getLatestDataCopy();

                    generateSmoother(latest_vins_odom, latest_msckf_odom);

                    odom_output_rate = ros::Rate(msckf_odom_src.src_freq);
                    state = FsmState::FLY_WITH_MSCKF;
                    ROS_ERROR("[Odom Fusion] Exceptions in vins odometry !!! Fly with msckf instead #^#");
                    inform_bad_odom = true;
                }
                else if (fcu_odom_src.isAvailable())
                {
                    nav_msgs::Odometry latest_vins_odom = vins_odom_src.getLatestDataCopy();

                    nav_msgs::Odometry latest_fcu_odom = fcu_odom_src.getLatestDataCopy();
                    nav_msgs::Odometry latest_fcu_odom_debiased;
                    debiasFcuOdom(latest_fcu_odom, latest_fcu_odom_debiased);

                    generateSmoother(latest_vins_odom, latest_fcu_odom_debiased);

                    odom_output_rate = ros::Rate(fcu_odom_src.src_freq);
                    state = FsmState::FLY_WITH_FCU_ODOM;
                    ROS_ERROR("[Odom Fusion] Exceptions in vins odometry but unavailble msckf !!! Fly with fcu odometry instead #^#");
                    inform_bad_odom = true;
                }
                else
                {
                    if (inform_bad_odom)
                    {
                        ROS_ERROR("[Odom Fusion] Exceptions in vins odometry but unavailble msckf and fcu odometry !!! Continue flying with vins #~# Watch out for crash !!!");
                        inform_bad_odom = false;
                    }
                }
            }
            break;
        }

        case FsmState::FLY_WITH_VINS:
        {
            if (!est_z_from_lidar_map_thread.joinable() && Z_mode == 2)
                est_z_from_lidar_map_thread = thread(
                    [&]()
                    {
                        ros::Rate r(est_z_rate);
                        bool first_z_est = true;
                        double prev_z_est;
                        while (ros::ok())
                        {
                            bool ground_smooth = tof_src.isStarted() && tof_continuity_check.isPassed();
                            if (recv_revised_odom && ground_smooth)
                            {
                                //************* 1. Compute z with respect to the ground hit point of ToF sensor *************//
                                double tof_dist = tof_src.getLatestDataCopy().distance;
                                Vector3d tof_vec_body(0.0, 0.0, -tof_dist);
                                Vector3d tof_vec_world;
                                Vector3d ground_hit_pt;
                                {
                                    lock_guard<mutex> lock(revised_odom_mtx);
                                    if (acos((latest_revised_q * Vector3d::UnitZ()).dot(Vector3d::UnitZ())) > (M_PI / 6))
                                    {
                                        ROS_ERROR("[Odom Fusion] Excessive drone attitude #^# Terminate map-based z estimation 🤧🤧🤧");
                                        goto end_z_est;
                                    }
                                    tof_vec_world = latest_revised_q * tof_vec_body;
                                    ground_hit_pt = latest_revised_q * tof_vec_body + latest_revised_p;
                                }
                                double z_to_ground = abs(tof_vec_world.z());

                                //************* 2. Compute ground height *************//
                                vector<Vector3d> ground_pts;
                                vector<int> ids;
                                vector<float> sqr_dists;
                                if (kdtree.radiusSearch(PointXYZ(ground_hit_pt.x(),
                                                                 ground_hit_pt.y(),
                                                                 ground_hit_pt.z()),
                                                        search_radius, ids, sqr_dists) > 9)
                                {
                                    for (size_t i = 0; i < ids.size(); ++i)
                                    {
                                        PointXYZ pt = (*cloud)[ids[i]];
                                        ground_pts.emplace_back(pt.x, pt.y, pt.z);
                                    }
                                }
                                else
                                {
                                    ROS_WARN_STREAM("[Odom Fusion] Only " << ids.size() << " (< 10) ground points found around hit point of the ToF sensor. Terminate map-based z estimation 🤧🤧🤧");
                                    goto end_z_est;
                                }

                                // 🥳🥳🥳 PCA 🥳🥳🥳
                                Vector3d centroid;
                                for (const auto &pt : ground_pts)
                                    centroid += pt;
                                centroid /= ground_pts.size();

                                MatrixXd centered_pts(3, ground_pts.size());
                                for (size_t i = 0; i < ground_pts.size(); ++i)
                                    centered_pts.col(i) = ground_pts[i] - centroid;

                                JacobiSVD<MatrixXd> svd(centered_pts, ComputeThinU | ComputeThinV);
                                Vector3d singular_values = svd.singularValues();
                                ROS_INFO_STREAM("[Odom Fusion] Singular values of the ground (" << ids.size() << " points) around the ToF hit point: " << singular_values.transpose());

                                if ((singular_values(1) / singular_values(0)) < 0.8 ||
                                    (singular_values(2) / singular_values(0)) > 0.2)
                                {
                                    ROS_WARN("[Odom Fusion] Ground around the ToF hit point is not like a plane. Terminate map-based z estimation 🤧🤧🤧");
                                    goto end_z_est;
                                }

                                Vector3d normal = svd.matrixU().col(2);
                                double normal_tilt_angle = acos(std::clamp(normal.dot(Vector3d::UnitZ()) / normal.norm(), -1.0, 1.0));
                                if (normal_tilt_angle > M_PI / 2)
                                    normal_tilt_angle = M_PI - normal_tilt_angle;
                                if (normal_tilt_angle > M_PI / 4)
                                {
                                    ROS_WARN_STREAM("[Odom Fusion] Ground plane around the ToF hit point is tilted too much (" << normal_tilt_angle * 180.0 / M_PI << " > 45 deg). Terminate map-based z estimation 🤧🤧🤧");
                                    goto end_z_est;
                                }

                                double z_ground = centroid.z();

                                //************* 3. Estimate z *************//
                                z_est = z_to_ground + z_ground;

                                if (first_z_est)
                                {
                                    prev_z_est = z_est;
                                    first_z_est = false;
                                }
                                else
                                {
                                    if (abs(z_est - prev_z_est) > 0.250)
                                    {
                                        ROS_WARN_STREAM("[Odom Fusion] Exception in z estimation (z jump = " << abs(z_est - prev_z_est) << "m) 🤧🤧🤧");
                                        z_est_available = false;
                                        first_z_est = true;
                                    }
                                    else
                                    {
                                        z_est_available = true;
                                        prev_z_est = z_est;
                                    }
                                }
                            }
                        end_z_est:
                            r.sleep();
                        }
                    });

            // Try to switch to another odometer when there are exceptions in the current odometer
            if (!isVinsCheckPassed())
            {
                if (msckf_odom_src.isAvailable())
                {
                    nav_msgs::Odometry latest_vins_odom = vins_odom_src.getLatestDataCopy();

                    nav_msgs::Odometry latest_msckf_odom = msckf_odom_src.getLatestDataCopy();

                    generateSmoother(latest_vins_odom, latest_msckf_odom);

                    odom_output_rate = ros::Rate(msckf_odom_src.src_freq);
                    state = FsmState::FLY_WITH_MSCKF;
                    ROS_ERROR("[Odom Fusion] Exceptions in vins odometry !!! Fly with msckf instead #^#");
                    inform_bad_odom = true;
                }
                else if (fcu_odom_src.isAvailable())
                {
                    nav_msgs::Odometry latest_vins_odom = vins_odom_src.getLatestDataCopy();

                    nav_msgs::Odometry latest_fcu_odom = fcu_odom_src.getLatestDataCopy();
                    nav_msgs::Odometry latest_fcu_odom_debiased;
                    debiasFcuOdom(latest_fcu_odom, latest_fcu_odom_debiased);

                    generateSmoother(latest_vins_odom, latest_fcu_odom_debiased);

                    odom_output_rate = ros::Rate(fcu_odom_src.src_freq);
                    state = FsmState::FLY_WITH_FCU_ODOM;
                    ROS_ERROR("[Odom Fusion] Exceptions in vins odometry but unavailble msckf !!! Fly with fcu odometry instead #^#");
                    inform_bad_odom = true;
                }
                else
                {
                    if (inform_bad_odom)
                    {
                        ROS_ERROR("[Odom Fusion] Exceptions in vins odometry but unavailble msckf and fcu odometry !!! Continue flying with vins #~# Watch out for crash !!!");
                        inform_bad_odom = false;
                    }
                }
            }
            break;
        }

        case FsmState::FLY_WITH_MSCKF:
        {
            ROS_ERROR("[Odom Fusion] Msckf is currently unsupported !!! The program should never reach here @_@");

            // Tendency to use higher priority odometers
            if (vins_odom_src.isAvailable())
            {
                nav_msgs::Odometry latest_msckf_odom = msckf_odom_src.getLatestDataCopy();

                nav_msgs::Odometry latest_vins_odom = vins_odom_src.getLatestDataCopy();

                generateSmoother(latest_msckf_odom, latest_vins_odom);

                odom_output_rate = ros::Rate(vins_odom_src.src_freq);
                state = FsmState::FLY_WITH_VINS;
                ROS_INFO_STREAM("[Odom Fusion] Vins is availble while flying with msckf \033[43;30mFLY_WITH_MSCKF\033[0m --> \033[43;30mFLY_WITH_VINS\033[0m :)");
                inform_bad_odom = true;
            }

            // Try to switch to another odometer when there are exceptions in the current odometer
            if (!isMsckfCheckPassed())
            {
                if (fcu_odom_src.isAvailable())
                {
                    nav_msgs::Odometry latest_msckf_odom = msckf_odom_src.getLatestDataCopy();

                    nav_msgs::Odometry latest_fcu_odom = fcu_odom_src.getLatestDataCopy();
                    nav_msgs::Odometry latest_fcu_odom_debiased;
                    debiasFcuOdom(latest_fcu_odom, latest_fcu_odom_debiased);

                    generateSmoother(latest_msckf_odom, latest_fcu_odom_debiased);

                    odom_output_rate = ros::Rate(fcu_odom_src.src_freq);
                    state = FsmState::FLY_WITH_FCU_ODOM;
                    ROS_ERROR("[Odom Fusion] Exceptions in msckf odometry but unavailble vins !!! Fly with fcu odometry instead #^#");
                    inform_bad_odom = true;
                }
                else
                {
                    if (inform_bad_odom)
                    {
                        ROS_ERROR("[Odom Fusion] Exceptions in msckf odometry but unavailble vins and fcu odometry !!! Continue flying with msckf #~# Watch out for crash !!!");
                        inform_bad_odom = false;
                    }
                }
            }
            break;
        }

        case FsmState::FLY_WITH_FCU_ODOM:
        {
            // Tendency to use higher priority odometers
            if (vins_odom_src.isAvailable())
            {
                nav_msgs::Odometry latest_fcu_odom = fcu_odom_src.getLatestDataCopy();
                nav_msgs::Odometry latest_fcu_odom_debiased;
                debiasFcuOdom(latest_fcu_odom, latest_fcu_odom_debiased);

                nav_msgs::Odometry latest_vins_odom = vins_odom_src.getLatestDataCopy();

                generateSmoother(latest_fcu_odom_debiased, latest_vins_odom);

                odom_output_rate = ros::Rate(vins_odom_src.src_freq);
                state = FsmState::FLY_WITH_VINS;
                ROS_INFO_STREAM("[Odom Fusion] Vins is availble while flying with fcu odometry \033[43;30mFLY_WITH_FCU_ODOM\033[0m --> \033[43;30mFLY_WITH_VINS\033[0m :)");
                inform_bad_odom = true;
            }
            else if (msckf_odom_src.isAvailable())
            {
                nav_msgs::Odometry latest_fcu_odom = fcu_odom_src.getLatestDataCopy();
                nav_msgs::Odometry latest_fcu_odom_debiased;
                debiasFcuOdom(latest_fcu_odom, latest_fcu_odom_debiased);

                nav_msgs::Odometry latest_msckf_odom = msckf_odom_src.getLatestDataCopy();

                generateSmoother(latest_fcu_odom_debiased, latest_msckf_odom);

                odom_output_rate = ros::Rate(msckf_odom_src.src_freq);
                state = FsmState::FLY_WITH_MSCKF;
                ROS_INFO_STREAM("[Odom Fusion] Msckf is availble but vins is not while flying with fcu odometry \033[43;30mFLY_WITH_FCU_ODOM\033[0m --> \033[43;30mFLY_WITH_MSCKF\033[0m :|");
                inform_bad_odom = true;
            }
            else
            {
                if (inform_bad_odom)
                {
                    ROS_ERROR("[Odom Fusion] Unavailble vins and msckf :( Continue flying with fcu odometry #~# Watch out for crash !!!");
                    inform_bad_odom = false;
                }
            }
            break;
        }

        default:
            break;
        }
        data_check_rate.sleep();
    }
    return 0;
}