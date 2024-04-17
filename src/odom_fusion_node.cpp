#include <queue>
#include <string>
#include <thread>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Eigen>

#include "DataSrc.hpp"
#include "VelMagCheck.hpp"

#include "traj_utils/take_off.h"
#include "quadrotor_msgs/TakeoffLand.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/OpticalFlowRad.h>

using namespace std;
using namespace Eigen;

enum FsmState
{
    WAIT_FOR_FCU_ODOM,
    CALIB_FCU_ODOM,
    TAKEOFF,
    FLY_WITH_VINS,
    FLY_WITH_MSCKF,
    FLY_WITH_FCU_ODOM,
    UNKNOWN
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_fusion_node");
    ros::NodeHandle nh("~");

    string hyperparam_path = argv[1];
    cv::FileStorage param(hyperparam_path, cv::FileStorage::READ);

    //************* Data sources *************//
    DataSrc<nav_msgs::Odometry> vins_odom_src(nh, param["vins_odom_topic"], param["vins_odom_freq"]);
    DataSrc<nav_msgs::Odometry> msckf_odom_src(nh, param["msckf_odom_topic"], param["msckf_odom_freq"]);
    DataSrc<nav_msgs::Odometry> fcu_odom_src(nh, param["fcu_odom_topic"], param["fcu_odom_freq"]);
    DataSrc<sensor_msgs::Imu> imu_src(nh, param["imu_topic"], param["imu_freq"]);
    DataSrc<mavros_msgs::OpticalFlowRad> optical_flow_src(nh, param["optical_flow_topic"], param["optical_flow_freq"], param["optical_flow_timeout"]);

    ros::AsyncSpinner spinner(10); // There are 2 callbacks per data source
    spinner.start();

    //************* Data checks *************//
    // Velocity magnitude check
    VelMagCheck vins_vel_mag_check(vins_odom_src, param["vel_mag_check_freq"], param["max_vel"]);
    VelMagCheck msckf_vel_mag_check(msckf_odom_src, param["vel_mag_check_freq"], param["max_vel"]);
    VelMagCheck fcu_vel_mag_check(fcu_odom_src, param["vel_mag_check_freq"], param["max_vel"]);

    //************* Main loop *************//
    FsmState state = FsmState::WAIT_FOR_FCU_ODOM;
    ros::Rate main_loop_rate(max({10.0, (double)param["vel_mag_check_freq"], (double)param["imu_amp_check_freq"]}));

    // Calibrate initial z and yaw bias of the fcu odometry for takeoff
    // Note that these static biases can only be used during takeoff and may become dynamic during flight
    thread calib_fcu_odom_thread;
    Vector3d p_fcu_bias;
    Quaterniond q_fcu_bias;
    auto debias_fcu_odom = [&p_fcu_bias, &q_fcu_bias](const nav_msgs::Odometry &fcu_odom, nav_msgs::Odometry &fcu_odom_debiased)
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

        fcu_odom_debiased.header = fcu_odom.header;
        fcu_odom_debiased.pose.pose.position.x = p_debiased.x();
        fcu_odom_debiased.pose.pose.position.y = p_debiased.y();
        fcu_odom_debiased.pose.pose.position.z = p_debiased.z();
        fcu_odom_debiased.twist.twist.linear.x = v_world.x();
        fcu_odom_debiased.twist.twist.linear.y = v_world.y();
        fcu_odom_debiased.twist.twist.linear.z = v_world.z();
        fcu_odom_debiased.pose.pose.orientation.w = q_debiased.w();
        fcu_odom_debiased.pose.pose.orientation.x = q_debiased.x();
        fcu_odom_debiased.pose.pose.orientation.y = q_debiased.y();
        fcu_odom_debiased.pose.pose.orientation.z = q_debiased.z();
    };

    ros::Subscriber takeoff_signal_sub_0 =
        nh.subscribe<traj_utils::take_off>((string)param["takeoff_signal_topic_0"], 10,
                                           [&state](const traj_utils::take_off::ConstPtr &signal)
                                           {
                                               switch (state)
                                               {
                                               case FsmState::WAIT_FOR_FCU_ODOM:
                                               case FsmState::CALIB_FCU_ODOM:
                                                   ROS_ERROR("[Odom Fusion] Fcu odometry not ready #^#");
                                                   break;
                                               case FsmState::TAKEOFF:
                                                   break;
                                               default:
                                                   ROS_WARN("[Odom Fusion] Takeoff signal received more than once ??? Ignoring current takeoff signal @^@");
                                                   break;
                                               }
                                           });
    ros::Subscriber takeoff_signal_sub_1 =
        nh.subscribe<quadrotor_msgs::TakeoffLand>((string)param["takeoff_signal_topic_1"], 10,
                                                  [&state](const quadrotor_msgs::TakeoffLand::ConstPtr &signal)
                                                  {
                                                      switch (state)
                                                      {
                                                      case FsmState::WAIT_FOR_FCU_ODOM:
                                                      case FsmState::CALIB_FCU_ODOM:
                                                          ROS_ERROR("[Odom Fusion] Fcu odometry not ready #^#");
                                                          break;
                                                      case FsmState::TAKEOFF:
                                                          break;
                                                      default:
                                                          ROS_ERROR("[Odom Fusion] Takeoff signal received more than once ??? Ignoring current takeoff signal @^@");
                                                          break;
                                                      }
                                                  });

    thread odom_output_thread;
    Vector3d p_smoother(0.0, 0.0, 0.0);
    Quaterniond q_smoother(1.0, 0.0, 0.0, 0.0);
    Vector3d v_smoother(0.0, 0.0, 0.0);
    auto generate_smoother = [&p_smoother, &q_smoother, &v_smoother](const nav_msgs::Odometry &odom_prv, const nav_msgs::Odometry &odom_fut)
    {
        p_smoother += Vector3d(odom_prv.pose.pose.position.x - odom_fut.pose.pose.position.x,
                               odom_prv.pose.pose.position.y - odom_fut.pose.pose.position.y,
                               odom_prv.pose.pose.position.z - odom_fut.pose.pose.position.z);

        v_smoother += Vector3d(odom_prv.twist.twist.linear.x - odom_fut.twist.twist.linear.x,
                               odom_prv.twist.twist.linear.y - odom_fut.twist.twist.linear.y,
                               odom_prv.twist.twist.linear.z - odom_fut.twist.twist.linear.z);

        const Quaterniond q_prv(odom_prv.pose.pose.orientation.w,
                                odom_prv.pose.pose.orientation.x,
                                odom_prv.pose.pose.orientation.y,
                                odom_prv.pose.pose.orientation.z);
        const Quaterniond q_fut(odom_fut.pose.pose.orientation.w,
                                odom_fut.pose.pose.orientation.x,
                                odom_fut.pose.pose.orientation.y,
                                odom_fut.pose.pose.orientation.z);
        q_smoother *= q_fut.inverse() * q_prv;
    };
    auto apply_smoother = [&p_smoother, &q_smoother, &v_smoother](nav_msgs::Odometry &odom_output)
    {
        Quaterniond q(odom_output.pose.pose.orientation.w,
                      odom_output.pose.pose.orientation.x,
                      odom_output.pose.pose.orientation.y,
                      odom_output.pose.pose.orientation.z);
        Quaterniond q_smooth = q * q_smoother;

        odom_output.pose.pose.position.x += p_smoother.x();
        odom_output.pose.pose.position.y += p_smoother.y();
        odom_output.pose.pose.position.z += p_smoother.z();
        odom_output.twist.twist.linear.x += v_smoother.x();
        odom_output.twist.twist.linear.y += v_smoother.y();
        odom_output.twist.twist.linear.z += v_smoother.z();
        odom_output.pose.pose.orientation.w = q_smooth.w();
        odom_output.pose.pose.orientation.x = q_smooth.x();
        odom_output.pose.pose.orientation.y = q_smooth.y();
        odom_output.pose.pose.orientation.z = q_smooth.z();
    };
    ros::Rate odom_output_rate(fcu_odom_src.src_freq); // Fcu odometry is used during takeoff
    ros::Publisher odom_output_pub = nh.advertise<nav_msgs::Odometry>((string)param["odom_output_topic"], 100);

    ros::Subscriber hover_signal_sub =
        nh.subscribe<geometry_msgs::PoseStamped>((string)param["hover_signal_topic"], 10,
                                                 [&](const geometry_msgs::PoseStamped::ConstPtr &signal)
                                                 {
                                                     switch (state)
                                                     {
                                                     case FsmState::WAIT_FOR_FCU_ODOM:
                                                     case FsmState::CALIB_FCU_ODOM:
                                                         ROS_INFO("\033[36m[Odom Fusion] The program should never reach here ^^");
                                                         break;
                                                     case FsmState::TAKEOFF:
                                                     {
                                                         state = FsmState::FLY_WITH_VINS;
                                                         ROS_INFO_STREAM("[Odom Fusion] \033[43;30mTAKEOFF\033[0m --> \033[43;30mFLY_WITH_VINS\033[0m :)");

                                                         odom_output_rate = ros::Rate(vins_odom_src.src_freq);

                                                         const nav_msgs::Odometry &latest_fcu_odom = fcu_odom_src.data_buf.back();
                                                         const nav_msgs::Odometry &latest_vins_odom = vins_odom_src.data_buf.back();
                                                         nav_msgs::Odometry latest_fcu_odom_debiased;
                                                         debias_fcu_odom(latest_fcu_odom, latest_fcu_odom_debiased);
                                                         generate_smoother(latest_fcu_odom_debiased, latest_vins_odom);
                                                         break;
                                                     }
                                                     default:
                                                         ROS_ERROR("[Odom Fusion] Hover signal received but not during takeoff ??? Ignoring current hover signal @^@");
                                                         break;
                                                     }
                                                 });

    while (ros::ok())
    {
        switch (state)
        {
        case FsmState::WAIT_FOR_FCU_ODOM:
        {
            if (fcu_odom_src.stable_stream && optical_flow_src.stable_stream)
            {
                state = FsmState::CALIB_FCU_ODOM;
                // TODO: use GLOG please 😣😣😣
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
                        double z_avg = 0.0, yaw_avg = 0.0;
                        ros::Rate calib_proc_rate(fcu_odom_src.src_freq);
                        for (size_t i = 1; ros::ok(); i++)
                        {
                            const nav_msgs::Odometry &latest_fcu_odom = fcu_odom_src.data_buf.back();
                            double z = latest_fcu_odom.pose.pose.position.z;
                            Matrix3d R = Quaterniond(latest_fcu_odom.pose.pose.orientation.w,
                                                     latest_fcu_odom.pose.pose.orientation.x,
                                                     latest_fcu_odom.pose.pose.orientation.y,
                                                     latest_fcu_odom.pose.pose.orientation.z)
                                             .toRotationMatrix();
                            double yaw = atan2(R(1, 0), R(0, 0));

                            if (i != 1)
                            {
                                if (abs(z - z_avg) > 0.013 || abs(yaw - yaw_avg) > 0.013)
                                {
                                    z_avg = yaw_avg = i = 0;
                                    ROS_ERROR("[Odom Fusion] Unstable initial z or yaw bias of fcu odometry, calibration restarted #^#");
                                    continue;
                                }

                                if (yaw < -3.0415926)
                                {
                                    yaw = yaw_avg;
                                    ROS_WARN("[Odom Fusion] Initial yaw bias of fcu odometry is close to PI and -PI. yaw ≈ -PI is abandoned which may cause insufficient calibration data #~#");
                                }
                            }

                            z_avg += (z - z_avg) / i;
                            yaw_avg += (yaw - yaw_avg) / i;

                            if (i > 250) // or 520 or 1314 or 7758
                            {
                                state = FsmState::TAKEOFF;
                                p_fcu_bias << 0.0, 0.0, z_avg;
                                q_fcu_bias = Quaterniond(AngleAxisd(yaw_avg, Vector3d::UnitZ()));
                                ROS_INFO_STREAM("\033[32m[Odom Fusion] Calibration finished \\^_^/ bias z = " << z_avg << " m, yaw = " << yaw_avg << " rad");
                                ROS_INFO_STREAM("[Odom Fusion] \033[43;30mCALIB_FCU_ODOM\033[0m --> \033[43;30mTAKEOFF\033[0m :)");
                                break;
                            }

                            if (!fcu_odom_src.stable_stream || !optical_flow_src.stable_stream)
                            {
                                state = FsmState::WAIT_FOR_FCU_ODOM;
                                ROS_WARN_STREAM("[Odom Fusion] \033[43;30mCALIB_FCU_ODOM\033[0m --> \033[43;30mWAIT_FOR_FCU_ODOM\033[0m Unstable" << ((!fcu_odom_src.stable_stream) ? "fcu odometry from " + fcu_odom_src.src_topic + ", calicration terminated #^#" : "optical flow from " + optical_flow_src.src_topic + ", calicration terminated #^#"));
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
                            {
                                const nav_msgs::Odometry &latest_fcu_odom = fcu_odom_src.data_buf.back();
                                nav_msgs::Odometry odom_output;
                                debias_fcu_odom(latest_fcu_odom, odom_output);
                                odom_output_pub.publish(odom_output);
                                break;
                            }

                            case FsmState::FLY_WITH_VINS:
                            {
                                const nav_msgs::Odometry &latest_vins_odom = vins_odom_src.data_buf.back();
                                nav_msgs::Odometry odom_output(latest_vins_odom);
                                apply_smoother(odom_output);
                                odom_output_pub.publish(odom_output);
                                break;
                            }

                            case FsmState::FLY_WITH_MSCKF:
                            {
                                // const nav_msgs::Odometry &latest_msckf_odom = msckf_odom_src.data_buf.back();
                                // odom_output_pub.publish(latest_msckf_odom);
                                break;
                            }

                            case FsmState::FLY_WITH_FCU_ODOM:
                            {
                                break;
                            }

                            default:
                            {
                                break;
                            }
                            }

                            odom_output_rate.sleep();
                        }
                    });

            if (!fcu_odom_src.stable_stream || !optical_flow_src.stable_stream)
            {
                if (vins_odom_src.stable_stream)
                {
                    state = FsmState::FLY_WITH_VINS;
                    ROS_ERROR_STREAM("[Odom Fusion] Unstable fcu odometry or optical flow !!! Takeoff with vins instead #^#");

                    odom_output_rate = ros::Rate(vins_odom_src.src_freq);

                    const nav_msgs::Odometry &latest_fcu_odom = fcu_odom_src.data_buf.back();
                    const nav_msgs::Odometry &latest_vins_odom = vins_odom_src.data_buf.back();
                    nav_msgs::Odometry latest_fcu_odom_debiased;
                    debias_fcu_odom(latest_fcu_odom, latest_fcu_odom_debiased);
                    generate_smoother(latest_fcu_odom_debiased, latest_vins_odom);
                }
                else if (msckf_odom_src.stable_stream)
                {
                    state = FsmState::FLY_WITH_MSCKF;
                    ROS_ERROR_STREAM("[Odom Fusion] Unstable fcu odometry or optical flow and vins !!! Takeoff with msckf instead #^#");

                    odom_output_rate = ros::Rate(msckf_odom_src.src_freq);

                    const nav_msgs::Odometry &latest_fcu_odom = fcu_odom_src.data_buf.back();
                    const nav_msgs::Odometry &latest_msckf_odom = msckf_odom_src.data_buf.back();
                    nav_msgs::Odometry latest_fcu_odom_debiased;
                    debias_fcu_odom(latest_fcu_odom, latest_fcu_odom_debiased);
                    generate_smoother(latest_fcu_odom_debiased, latest_msckf_odom);
                }
                else
                {
                    ROS_ERROR_STREAM("[Odom Fusion] Unstable fcu odometry or optical flow, vins and msckf !!! Continue with fcu odometry #~# Watch out for crash !!!");
                }
            }
        }

        default:
            break;
        }
        main_loop_rate.sleep();
    }

    ros::waitForShutdown();
}