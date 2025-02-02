#ifndef _STATE_ESTIMATOR_H_
#define _STATE_ESTIMATOR_H_

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <ros/ros.h>

// Subscribing messages
#include <sensor_msgs/Imu.h> // Imu (acc & gyro)
#include <sensor_msgs/MagneticField.h> // Imu magnetometer
#include <geometry_msgs/PoseStamped.h> // optitrack
#include <std_msgs/Time.h>

// Publishing messages
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// Error state kalman filter.
#include "eskf/error_state_kalman_filter.h"
#include "eskf/timer.h"

using namespace Eigen;

#define FilterType ESKF

class OptitrackStateEstimatorRos{
// Callback functions
private:
	void callbackOptitrack(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
	void callbackMag(const sensor_msgs::MagneticFieldConstPtr& msg);

    void getParameters();

    void run();

public:
    OptitrackStateEstimatorRos(ros::NodeHandle& nh);
    ~OptitrackStateEstimatorRos();
    
private:
    ros::NodeHandle nh_;

	// subscribers
    std::string topicname_imu_;
    std::string topicname_mag_;
    std::string topicname_optitrack_;

	ros::Subscriber sub_imu_;
	ros::Subscriber sub_mag_;
	ros::Subscriber sub_optitrack_;

    sensor_msgs::Imu           imu_current_;
    sensor_msgs::MagneticField mag_current_;
    geometry_msgs::PoseStamped optitrack_current_;

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
    std::string topicname_optitrack_dt_;

    ros::Publisher pub_imu_dt_;
    ros::Publisher pub_optitrack_dt_;

    std_msgs::Time msg_imu_dt_;
    std_msgs::Time msg_optitrack_dt_;

// Filter
private:   
    std::unique_ptr<FilterType> filter_;

    bool get_bias_from_param_;
    std::vector<double> acc_bias_;
    std::vector<double> gyro_bias_;
    std::vector<double> mag_bias_;

    double noise_std_acc_;
    double noise_std_gyro_;
    double noise_std_mag_;

    double noise_optitrack_position_;
    double noise_optitrack_orientation_;

    std::vector<double> q_BI_vec_;

    bool verbose_all_estimation_;


    double timestamp_last_optitrack_;
    double timestamp_last_imu_;

    //Added by KGC for init_buffer_
    int N_init, N_acc;
    double w_err, w_rot, stdv_samples;
    
};
#endif