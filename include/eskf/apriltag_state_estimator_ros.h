#ifndef _STATE_ESTIMATOR_H_
#define _STATE_ESTIMATOR_H_

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <ros/ros.h>

// Subscribing messages
#include <sensor_msgs/Imu.h> // Imu (acc & gyro)
#include <sensor_msgs/MagneticField.h> // Imu magnetometer
#include <geometry_msgs/PoseWithCovarianceStamped.h> // Apriltag
#include <apriltag_ros/AprilTagDetection.h> // Apriltag
#include <std_msgs/Time.h>

// Publishing messages
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// Error state kalman filter.
#include "eskf/error_state_kalman_filter.h"
#include "eskf/timer.h"

using namespace Eigen;

#define FilterType ESKF

class ApriltagStateEstimatorRos{
// Callback functions
private:
    void callbackApriltag(const apriltag_ros::AprilTagDetectionConstPtr& msg);
    void callbackIMU(const sensor_msgs::ImuConstPtr& msg);

    void getParameters();

    void run();

public:
    ApriltagStateEstimatorRos(ros::NodeHandle& nh);
    ~ApriltagStateEstimatorRos();

private:
    ros::NodeHandle nh_;

    // subscribers
    std::string topicname_imu_;
    std::string topicname_Apriltag_;

    ros::Subscriber sub_imu_;
    ros::Subscriber sub_mag_;
    ros::Subscriber sub_Apriltag_;

    sensor_msgs::Imu           imu_current_;
    geometry_msgs::PoseWithCovarianceStamped T_ct_current_;

    // publishers
    std::string topicname_nav_raw_;
    std::string topicname_nav_filtered_;
    std::string topicname_nav_filtered_lpf_;

    ros::Publisher pub_nav_raw_;
    ros::Publisher pub_nav_filtered_;
    ros::Publisher pub_nav_filtered_lpf_;

    nav_msgs::Odometry nav_raw_current_;
    nav_msgs::Odometry nav_filtered_current_;
    nav_msgs::Odometry nav_filtered_current_lpf_;

    std::string topicname_imu_dt_;
    std::string topicname_Apriltag_dt_;

    ros::Publisher pub_imu_dt_;
    ros::Publisher pub_apriltag_dt_;

    std_msgs::Time msg_imu_dt_;
    std_msgs::Time msg_Apriltag_dt_;

// Filter
private:
    std::unique_ptr<FilterType> filter_;

    bool get_bias_from_param_;
    std::vector<double> acc_bias_;
    std::vector<double> gyro_bias_;

    double noise_std_acc_;
    double noise_std_gyro_;

    // does apriltag msg properly include covariance?
    double noise_Apriltag_position_;
    double noise_Apriltag_orientation_;

    std::vector<double> T_CI_vec_; // p_CI (3), q_CI (4)
    std::vector<double> T_WT_vec_; // p_WT (3), q_WT (4)
    geometry::Tf T_WT_;

    bool verbose_all_estimation_;

    double timestamp_last_apriltag_;
    double timestamp_last_imu_;

};
#endif