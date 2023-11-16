//
// Created by skykim0609 on 23. 11. 15.
//
#include "eskf/apriltag_state_estimator_ros.h"

ApriltagStateEstimatorRos::ApriltagStateEstimatorRos(ros::NodeHandle &nh): nh_(nh), verbose_all_estimation_(false) {
    ROS_INFO_STREAM("ApriltagStateEstimatorRos - starts");

    // get ros parameters
    this->getParameters();

    ROS_INFO_STREAM("ApriltagStateEstimatorRos - ROS parameters are successfully got.\n");

    // Subscribing
    sub_imu_       = nh_.subscribe<sensor_msgs::Imu>
            (topicname_imu_, 1, &ApriltagStateEstimatorRos::callbackIMU, this);
    sub_Apriltag_ = nh_.subscribe<apriltag_ros::AprilTagDetection>
            (topicname_Apriltag_, 1, &ApriltagStateEstimatorRos::callbackApriltag, this);

    // Publishing 
    pub_nav_raw_      = nh_.advertise<nav_msgs::Odometry>
            (topicname_nav_raw_, 1);
    pub_nav_filtered_ = nh_.advertise<nav_msgs::Odometry>
            (topicname_nav_filtered_, 1);
    pub_nav_filtered_lpf_ = nh_.advertise<nav_msgs::Odometry>
            (topicname_nav_filtered_lpf_, 1);


    topicname_imu_dt_ = "/imu_dt";
    topicname_Apriltag_dt_ = "/apriltag_dt";
    pub_imu_dt_ = nh_.advertise<std_msgs::Time>
            (topicname_imu_dt_, 1);
    pub_apriltag_dt_ = nh_.advertise<std_msgs::Time>
            (topicname_Apriltag_dt_, 1);

    // Filter generation
    filter_ = std::make_unique<FilterType>();

    // Set parameters
    if(get_bias_from_param_) filter_->setBias(acc_bias_[0], acc_bias_[1], acc_bias_[2], gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
    filter_->setIMUNoise(noise_std_acc_, noise_std_gyro_, 0.0);
    filter_->setObservationNoise(noise_Apriltag_position_, noise_Apriltag_orientation_);

    Vec3 p_CI;
    Vec4 q_CI;
    for(int i = 0; i < 3; ++i) p_CI(i) = T_CI_vec_.at(i);
    for(int i = 3; i < 7; ++i) q_CI(i) = T_CI_vec_.at(i);
    geometry::Tf T_CI(q_CI, p_CI);
    filter_->setTransformFromCameraToIMU(T_CI);

    Vec3 p_WT;
    Vec4 q_WT;
    for(int i = 0; i < 3; ++i) p_WT(i) = T_WT_vec_.at(i);
    for(int i = 3; i < 7; ++i) q_WT(i) = T_WT_vec_.at(i);
    T_WT_ = {p_WT, q_WT};

    timestamp_last_apriltag_ = 0.0f;
    timestamp_last_imu_ = 0.0f;

    // run
    this->run();
}

void ApriltagStateEstimatorRos::getParameters() {
// get parameters
    // sub param names
    if(!ros::param::has("~topic_imu")) throw std::runtime_error("ApriltagStateEstimatorRos - no 'topic_imu' is set. terminate program.\n");
    ros::param::get("~topic_imu", topicname_imu_);

    if(!ros::param::has("~topic_apriltag")) throw std::runtime_error("ApriltagStateEstimatorRos - no 'topic_Apriltag' is set. terminate program.\n");
    ros::param::get("~topic_Apriltag", topicname_Apriltag_);

    // pub param names
    if(!ros::param::has("~topic_nav_filtered")) throw std::runtime_error("ApriltagStateEstimatorRos - no 'topic_nav_filtered' is set. terminate program.\n");
    ros::param::get("~topic_nav_filtered", topicname_nav_filtered_);
    topicname_nav_filtered_lpf_ = topicname_nav_filtered_ + "/lpf";

    if(!ros::param::has("~topic_nav_raw")) throw std::runtime_error("ApriltagStateEstimatorRos - no 'topic_nav_raw' is set. terminate program.\n");
    ros::param::get("~topic_nav_raw", topicname_nav_raw_);

    if(!ros::param::has("~verbose_all_estimation")) throw std::runtime_error("ApriltagStateEstimatorRos - no 'verbose_all_estimation_' is set. terminate program.\n");
    ros::param::get("~verbose_all_estimation", verbose_all_estimation_);

    ROS_INFO_STREAM("Verbose all estimation: " << (verbose_all_estimation_ ? "true" : "false") );

    if(!ros::param::has("~noise_accel"))
        throw std::runtime_error("there is no 'noise_accel'. ");
    if(!ros::param::has("~noise_gyro"))
        throw std::runtime_error("there is no 'noise_gyro'. ");

    if(!ros::param::has("~noise_Apriltag_position"))
        throw std::runtime_error("there is no 'noise_Apriltag_position'.");
    if(!ros::param::has("~noise_Apriltag_orientation"))
        throw std::runtime_error("there is no 'noise_Apriltag_orientation'.");

    if(!ros::param::has("~q_BI"))
        throw std::runtime_error("there is no 'q_BI'. ");

    nh_.param("get_bias_from_params", get_bias_from_param_, false);
    if(get_bias_from_param_) {
        ROS_INFO("Get Bias From Param = True. Loading values from rosparam server");
        if(!ros::param::has("~acc_bias"))
            throw std::runtime_error("there is no 'acc_bias'. in param server");
        if(!ros::param::has("~gyro_bias"))
            throw std::runtime_error("there is no 'gyro_bias'. in param server");
        nh_.param("acc_bias", acc_bias_, std::vector<double>());
        nh_.param("gyro_bias", gyro_bias_, std::vector<double>());
    }
    else{
        ROS_INFO("Get Bias From Param = False. Bias will be estimated assuming that the platform is standing still for few seconds");
    }

    if(acc_bias_.size() != 3)
        throw std::runtime_error("'acc_bias_.size() != 3. acc_bias_.size() should be 3.");
    if(gyro_bias_.size() != 3)
        throw std::runtime_error("'gyro_bias_.size() != 3. gyro_bias_.size() should be 3.");

    nh_.param("T_CI", T_CI_vec_, std::vector<double>());
    if(T_CI_vec_.size() != 7)
        throw std::runtime_error("'T_CI_vec_.size() != 7. T_CI_vec_.size() should be 7 (p, q).");

    nh_.param("T_WT", T_WT_vec_, std::vector<double>());
    if(T_WT_vec_.size() != 7)
        throw std::runtime_error("'T_WT_vec_.size() != 7. T_WT_vec_.size() should be 7 (p, q).");

    ros::param::get("~noise_accel", noise_std_acc_);
    ros::param::get("~noise_gyro",  noise_std_gyro_);

    ros::param::get("~noise_Apriltag_position",    noise_Apriltag_position_);
    ros::param::get("~noise_Apriltag_orientation", noise_Apriltag_orientation_);
}

void ApriltagStateEstimatorRos::run(){
    int node_rate = 1000;
    ROS_INFO_STREAM("ApriltagStateEstimatorRos - runs at [" << node_rate <<"] Hz.");
    ros::Rate rate(node_rate);

    ros::Time t_prev = ros::Time::now();
    ros::Time t_curr;

    double period_show = 1.0; // sec.
    while(ros::ok()){

        t_curr = ros::Time::now();

        if( verbose_all_estimation_ && (t_curr-t_prev).toSec() >= period_show ) {
            filter_->showFilterStates();
            t_prev = t_curr;
        }

        ros::spinOnce();
        rate.sleep();
    }
};

void ApriltagStateEstimatorRos::callbackApriltag(const apriltag_ros::AprilTagDetectionConstPtr &msg) {
    T_ct_current_ = msg->pose;
    double t_now = T_ct_current_.header.stamp.toSec();

    std_msgs::Time msg_time;
    msg_time.data = ros::Time(t_now-timestamp_last_apriltag_);
    pub_apriltag_dt_.publish(msg_time);

    if(t_now-timestamp_last_apriltag_ < 0.005) {
        timestamp_last_apriltag_ = t_now;
        return;
    }
    timestamp_last_apriltag_ = t_now;

    Vec3 p_ct;
    Vec4 q_ct;
    p_ct << T_ct_current_.pose.pose.position.x,
            T_ct_current_.pose.pose.position.y,
            T_ct_current_.pose.pose.position.z;

    q_ct << T_ct_current_.pose.pose.orientation.w,
            T_ct_current_.pose.pose.orientation.x,
            T_ct_current_.pose.pose.orientation.y,
            T_ct_current_.pose.pose.orientation.z;
    geometry::Tf T_ct(q_ct, p_ct);
    geometry::Tf T_tc = T_ct.Inverse();

    RMat6 Cov_ct;
    const boost::array<double, 36>& cov_vec = T_ct_current_.pose.covariance;
    int idx = 0;
    for(int i = 0; i< 6; ++i){
        for(int j = 0; j < 6; ++j){
            Cov_ct(i, j) = cov_vec.at(idx);
            ++idx;
        }
    }

    //update filter
    filter_->updateAprilTag(T_tc, Cov_ct, t_now);

    // Fill the nav message
    nav_raw_current_.header.frame_id = "map";
    ++nav_raw_current_.header.seq;
    nav_raw_current_.header.stamp = ros::Time::now();

    //Position and orientation
    auto X_nom_w = filter_->getWorldFrameState(T_WT_, "apriltag");

    nav_raw_current_.pose.pose.position.x = X_nom_w.p(0);
    nav_raw_current_.pose.pose.position.y = X_nom_w.p(1);
    nav_raw_current_.pose.pose.position.z = X_nom_w.p(2);

    nav_raw_current_.pose.pose.orientation.w = X_nom_w.q(0);
    nav_raw_current_.pose.pose.orientation.x = X_nom_w.q(1);
    nav_raw_current_.pose.pose.orientation.y = X_nom_w.q(2);
    nav_raw_current_.pose.pose.orientation.z = X_nom_w.q(3);

    // Velocities
    nav_raw_current_.twist.twist.linear.x = X_nom_w.v(0);
    nav_raw_current_.twist.twist.linear.y = X_nom_w.v(1);
    nav_raw_current_.twist.twist.linear.z = X_nom_w.v(2);

    nav_raw_current_.twist.twist.angular.x = imu_current_.angular_velocity.x;
    nav_raw_current_.twist.twist.angular.y = imu_current_.angular_velocity.y;
    nav_raw_current_.twist.twist.angular.z = imu_current_.angular_velocity.z;

    pub_nav_raw_.publish(nav_raw_current_);
}

void ApriltagStateEstimatorRos::callbackIMU(const sensor_msgs::ImuConstPtr &msg) {
    imu_current_ = *msg;
    // double t_now = imu_current_.header.stamp.toSec();
    double t_now = msg->header.stamp.toSec();

    std_msgs::Time msg_time;
    msg_time.data = ros::Time(t_now-timestamp_last_imu_);
    pub_imu_dt_.publish(msg_time);
    timestamp_last_imu_ = t_now;

    Vec3 am;
    Vec3 wm;
    am << imu_current_.linear_acceleration.x, imu_current_.linear_acceleration.y, imu_current_.linear_acceleration.z;
    wm << imu_current_.angular_velocity.x, imu_current_.angular_velocity.y, imu_current_.angular_velocity.z;

    if(!filter_->isBiasInitialized()){
        // need bias initialization
        timer::tic();
        bool bias_init_success = filter_->tryInitializeBias(am, wm, t_now);
        timer::toc(0);
        if(bias_init_success){
            ROS_INFO("Bias Initialization successful");
        }
    }
    else if( filter_->isInitialized() ){ // is Initialized
        // we got Apriltag estimation
        filter_->updateGravity(am,t_now);

        // Predict filter
        timer::tic();
        filter_->predict(am, wm, t_now);
        timer::toc(0);

        // publish the filtered data
        //ESKF::NominalState X_nom;
        //filter_->getFilteredStates(X_nom);

        // Fill the nav message
        nav_filtered_current_.header.frame_id = "map";
        ++nav_filtered_current_.header.seq;
        nav_filtered_current_.header.stamp = ros::Time::now();

        // Position and orientation. In {w} frame
        ESKF::NominalState X_nom_w = filter_->getWorldFrameState(T_WT_, "apriltag");

        //X_nom.p = filter_->getFixedParameters().R_BI*X_nom.p;
        nav_filtered_current_.pose.pose.position.x = X_nom_w.p(0); // global
        nav_filtered_current_.pose.pose.position.y = X_nom_w.p(1);
        nav_filtered_current_.pose.pose.position.z = X_nom_w.p(2);

        //X_nom.q = geometry::q1_mult_q2(geometry::q1_mult_q2(filter_->getFixedParameters().q_IB, X_nom.q), filter_->getFixedParameters().q_BI);

        nav_filtered_current_.pose.pose.orientation.w = X_nom_w.q(0); // global
        nav_filtered_current_.pose.pose.orientation.x = X_nom_w.q(1);
        nav_filtered_current_.pose.pose.orientation.y = X_nom_w.q(2);
        nav_filtered_current_.pose.pose.orientation.z = X_nom_w.q(3);

        // Velocities
        //X_nom.v = filter_->getFixedParameters().R_BI*X_nom.v;
        nav_filtered_current_.twist.twist.linear.x = X_nom_w.v(0); // global
        nav_filtered_current_.twist.twist.linear.y = X_nom_w.v(1);
        nav_filtered_current_.twist.twist.linear.z = X_nom_w.v(2);

        Vec3 wm;
        wm << imu_current_.angular_velocity.x, imu_current_.angular_velocity.y, imu_current_.angular_velocity.z;
        wm = wm - X_nom_w.bg;
        wm = filter_->getFixedParameters().R_BI*wm; // body
        nav_filtered_current_.twist.twist.angular.x = wm(0);
        nav_filtered_current_.twist.twist.angular.y = wm(1);
        nav_filtered_current_.twist.twist.angular.z = wm(2);

        // Publish filtered states
        pub_nav_filtered_.publish(nav_filtered_current_);

        // filtered data with LPF acc. and gyro.
        Vec3 w_lpf;
        nav_filtered_current_lpf_ = nav_filtered_current_;
        filter_->getGyroLowPassFiltered(w_lpf);
        w_lpf = w_lpf - X_nom_w.bg;
        w_lpf = filter_->getFixedParameters().R_BI*w_lpf; // body
        nav_filtered_current_lpf_.twist.twist.angular.x = w_lpf(0);
        nav_filtered_current_lpf_.twist.twist.angular.y = w_lpf(1);
        nav_filtered_current_lpf_.twist.twist.angular.z = w_lpf(2);

        pub_nav_filtered_lpf_.publish(nav_filtered_current_lpf_);
    }

}