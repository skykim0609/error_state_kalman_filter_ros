//
// Created by skykim0609 on 23. 11. 15.
//
#include "eskf/apriltag_state_estimator_ros.h"
#include "eskf/ros_utils.h"

ApriltagStateEstimatorRos::ApriltagStateEstimatorRos(ros::NodeHandle &nh): nh_(nh), verbose_all_estimation_(false) {
    ROS_INFO_STREAM("ApriltagStateEstimatorRos - starts");

    this->getParameters();

    // get Apriltag info
    this->getAprilTagInfo(apriltag_setting_filename);

    ROS_INFO_STREAM("ApriltagStateEstimatorRos - ROS parameters are successfully got.\n");

    // Subscribing
    sub_imu_       = nh_.subscribe<sensor_msgs::Imu>
            (topicname_imu_, 1, &ApriltagStateEstimatorRos::callbackIMU, this);
    sub_Apriltag_ = nh_.subscribe<apriltag_ros::AprilTagDetectionArray>
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
    filter_->setIMUNoise(noise_std_acc_, noise_std_gyro_);
    //filter_->setObservationNoise(noise_Apriltag_position_, noise_Apriltag_orientation_); // Actually does nothing
    Eigen::DiagonalMatrix<double, 6> D_ct;

    Vec3 np2, no2;
    for(int i=0; i<3; ++i){
        np2(i) = POW2(noise_Apriltag_position_[i]);
        no2(i) = POW2(noise_Apriltag_orientation_[i]);
    }
    D_ct.diagonal() << np2, no2;
    Cov_ct = D_ct.toDenseMatrix();
    filter_->setApriltagWorldPoses(apriltag_world_poses);
    filter_->setRefTagId(ref_tag_id_);

    Mat33 S_init;
    Eigen::DiagonalMatrix<double, 3> D_init;
    double var_samples = POW2(stdv_samples);
    D_init.diagonal() << var_samples, var_samples, var_samples;
    S_init = D_init.toDenseMatrix();
    filter_->setInitBuffer(N_init, w_err, w_rot, N_acc, S_init);

    Vec3 p_CI;
    Vec4 q_CI;
    for(int i = 0; i < 3; ++i) p_CI(i) = T_CI_vec_.at(i);
    for(int i = 3; i < 7; ++i) q_CI(i-3) = T_CI_vec_.at(i);
    geometry::Tf T_CI(q_CI, p_CI);
    filter_->setTransformFromCameraToIMU(T_CI);

    timestamp_last_apriltag_ = 0.0f;
    timestamp_last_imu_ = 0.0f;

    // run
    this->run();
}

void ApriltagStateEstimatorRos::getAprilTagInfo(const std::string &setting_filename) {
    YAML::Node config = YAML::LoadFile(setting_filename);
    for(const YAML::detail::iterator_value& item : config["tags"]){
        auto p_arr = item["position"].as<std::array<double, 3>>();
        auto q_arr = item["orientation"].as<std::array<double, 4>>();
        int id = item["id"].as<int>();
        geometry::Tf Twt(q_arr, p_arr);
        apriltag_world_poses.push_back(id, Twt);
        std::cout<<"Got Apriltag Information "<<id<<", pose : "<<Twt<<"\n";
    }
}

void ApriltagStateEstimatorRos::getParameters() {
// get parameters
    // sub param names
    ROSUtils::checkAndLoad<std::string>("topic_imu", topicname_imu_);
    ROSUtils::checkAndLoad<std::string>("topic_apriltag", topicname_Apriltag_);

    // pub param names
    ROSUtils::checkAndLoad<std::string>("topic_nav_filtered", topicname_nav_filtered_);
    topicname_nav_filtered_lpf_ = topicname_nav_filtered_ + "/lpf";
    ROSUtils::checkAndLoad<std::string>("topic_nav_raw", topicname_nav_raw_);
    ROSUtils::checkAndLoad<std::string>("topic_nav_filtered", topicname_nav_filtered_);

    //verbosity
    ROSUtils::checkAndLoad<bool>("verbose_all_estimation", verbose_all_estimation_);
    ROS_INFO_STREAM("Verbose all estimation: " << (verbose_all_estimation_ ? "true" : "false") );

    //noise params
    ROSUtils::checkAndLoad<double>("noise_accel", noise_std_acc_);
    ROSUtils::checkAndLoad<double>("noise_gyro", noise_std_gyro_);
    ROSUtils::checkVectorSizeAndLoad<double>("noise_apriltag_position", noise_Apriltag_position_, 3);
    ROSUtils::checkVectorSizeAndLoad<double>("noise_apriltag_orientation", noise_Apriltag_orientation_, 3);

    // Added by KGC
    ROSUtils::checkVectorSizeAndLoad<double>("T_CI", T_CI_vec_, 7);
    ROSUtils::checkAndLoad<std::string>("apriltag_setting_filename", apriltag_setting_filename);
    ROSUtils::checkAndLoad<int>("ref_tag_id", ref_tag_id_);

    nh_.param("get_bias_from_params", get_bias_from_param_, false);
    if(get_bias_from_param_) {
        ROS_INFO("Get Bias From Param = True. Loading values from rosparam server");

        ROSUtils::checkVectorSizeAndLoad<double>("acc_bias", acc_bias_, 3);
        ROSUtils::checkVectorSizeAndLoad<double>("gyro_bias", gyro_bias_, 3);
    }
    else{
        ROS_INFO("Get Bias From Param = False. Bias will be estimated assuming that the platform is standing still for few seconds");
    }
    // Added by KGC. Init Buffer initialization
    ROSUtils::checkAndLoad<int>("initialization/N_init_samples", N_init);
    ROSUtils::checkAndLoad<int>("initialization/N_acc_samples", N_acc);
    ROSUtils::checkAndLoad<double>("initialization/w_err", w_err);
    ROSUtils::checkAndLoad<double>("initialization/w_rot", w_rot);
    ROSUtils::checkAndLoad<double>("initialization/stdv_samples", stdv_samples);
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

        if(t_curr < t_prev ) {
            std::cerr<< std::fixed;
            std::cerr<<"Time reversal happened? t_curr : "<<t_curr.toNSec()<<", t_prev : "<<t_prev.toNSec()<<"\n";
        }
        if( verbose_all_estimation_ && (t_curr-t_prev).toSec() >= period_show ) {
            filter_->showFilterStates();
            t_prev = t_curr;
        }
        

        ros::spinOnce();
        rate.sleep();
    }
};

void ApriltagStateEstimatorRos::callbackApriltag(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg) {
    if(msg->detections.empty()) {
        ROS_WARN_STREAM("callbackApriltag called, but empty detection message at time "<< msg->header.stamp);
        return;
    }
    double t_now = msg->header.stamp.toSec();
    T_ct_current_.clear();
    ApriltagInfoArr apriltag_detections;
    for(const auto& det : msg->detections){
        const auto& pose = det.pose.pose.pose;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = pose;
        int id = det.id[0];
        T_ct_current_.emplace(id, pose_stamped);
        const auto& q_r = pose.orientation;
        const auto& p_r = pose.position;
        Vec4 q_ct(q_r.w, q_r.x, q_r.y, q_r.z);
        Vec3 p_ct(p_r.x, p_r.y, p_r.z);
        apriltag_detections.push_back(id, geometry::Tf(q_ct, p_ct), Cov_ct);
    }

    std_msgs::Time msg_time;
    msg_time.data = ros::Time(t_now-timestamp_last_apriltag_);
    pub_apriltag_dt_.publish(msg_time);

    if(t_now-timestamp_last_apriltag_ < 0.005) {
        timestamp_last_apriltag_ = t_now;
        return;
    }
    timestamp_last_apriltag_ = t_now;

    //update filter
    filter_->updateAprilTag(apriltag_detections, t_now);

    // Fill the nav message
    nav_raw_current_.header.frame_id = "map";
    ++nav_raw_current_.header.seq;
    nav_raw_current_.header.stamp = ros::Time::now();

    //Position and orientation
    ESKF::NominalState X_nom_w = filter_->getWorldFrameState();
    Mat1515 P;
    filter_->getCovariance(P);

    nav_raw_current_.pose.pose.position.x = X_nom_w.p(0);
    nav_raw_current_.pose.pose.position.y = X_nom_w.p(1);
    nav_raw_current_.pose.pose.position.z = X_nom_w.p(2);

    nav_raw_current_.pose.pose.orientation.w = X_nom_w.q(0);
    nav_raw_current_.pose.pose.orientation.x = X_nom_w.q(1);
    nav_raw_current_.pose.pose.orientation.y = X_nom_w.q(2);
    nav_raw_current_.pose.pose.orientation.z = X_nom_w.q(3);

    Mat33 P_pp = P.block<3, 3>(0, 0);
    Mat33 P_qq = P.block<3, 3>(6, 6);
    Mat33 P_pq = P.block<3, 3>(0, 6);
    Mat66 P_pose = Mat66::Identity();
    P_pose << P_pp, P_pq, P_pq.transpose(), P_qq;
    int k = 0;
    for(int i = 0; i < 6; ++i){
        for(int j = 0; j < 6; ++j){
            nav_raw_current_.pose.covariance[k] = P_pose(i, j);
            ++k;
        }
    }

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

        // Position and orientation. In world frame
        ESKF::NominalState X_nom_w = filter_->getWorldFrameState();

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