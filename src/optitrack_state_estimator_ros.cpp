#include "eskf/optitrack_state_estimator_ros.h"
#include "eskf/ros_utils.h"

OptitrackStateEstimatorRos::OptitrackStateEstimatorRos(ros::NodeHandle& nh)
: nh_(nh), verbose_all_estimation_(false)
{
    ROS_INFO_STREAM("OptitrackStateEstimatorRos - starts");

    // get ros parameters
    this->getParameters();

    ROS_INFO_STREAM("OptitrackStateEstimatorRos - ROS parameters are successfully got.\n");

    // Subscribing
    sub_imu_       = nh_.subscribe<sensor_msgs::Imu>
        (topicname_imu_, 1, &OptitrackStateEstimatorRos::callbackIMU, this);
    sub_mag_       = nh_.subscribe<sensor_msgs::MagneticField>
        (topicname_mag_, 1, &OptitrackStateEstimatorRos::callbackMag, this);
    sub_optitrack_ = nh_.subscribe<geometry_msgs::PoseStamped>
        (topicname_optitrack_, 1, &OptitrackStateEstimatorRos::callbackOptitrack, this);

    // Publishing 
    pub_nav_raw_      = nh_.advertise<nav_msgs::Odometry>
        (topicname_nav_raw_, 1);
    pub_nav_filtered_ = nh_.advertise<nav_msgs::Odometry>
        (topicname_nav_filtered_, 1);
    pub_nav_filtered_lpf_ = nh_.advertise<nav_msgs::Odometry>
        (topicname_nav_filtered_lpf_, 1);


    topicname_imu_dt_ = "/imu_dt";
    topicname_optitrack_dt_ = "/optitrack_dt";
    pub_imu_dt_ = nh_.advertise<std_msgs::Time>
        (topicname_imu_dt_, 1);
    pub_optitrack_dt_ = nh_.advertise<std_msgs::Time>
        (topicname_optitrack_dt_, 1);

    // Filter generation
    filter_ = std::make_unique<FilterType>();

    // Set parameters
    if(get_bias_from_param_) filter_->setBias(acc_bias_[0], acc_bias_[1], acc_bias_[2], gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
    filter_->setIMUNoise(noise_std_acc_, noise_std_gyro_);
    filter_->setObservationNoise(noise_optitrack_position_, noise_optitrack_orientation_);
    filter_->setRefTagId(0);

    Mat33 S_init;
    Eigen::DiagonalMatrix<double, 3> D_init;
    double var_samples = POW2(stdv_samples);
    D_init.diagonal() << var_samples, var_samples, var_samples;
    S_init = D_init.toDenseMatrix();
    filter_->setInitBuffer(N_init, w_err, w_rot, N_acc, S_init);
    
    Vec4 q_BI;
    q_BI << q_BI_vec_[0],q_BI_vec_[1], q_BI_vec_[2], q_BI_vec_[3];

    filter_->setRotationFromBodyToIMU(q_BI);

    timestamp_last_optitrack_ = 0.0f;
    timestamp_last_imu_ = 0.0f;

    // run
    this->run();
};

OptitrackStateEstimatorRos::~OptitrackStateEstimatorRos(){
    ROS_INFO_STREAM("OptitrackStateEstimatorRos - terminated");
};

void OptitrackStateEstimatorRos::run(){
    int node_rate = 5000;
    ROS_INFO_STREAM("OptitrackStateEstimatorRos - runs at [" << node_rate <<"] Hz.");
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


void OptitrackStateEstimatorRos::callbackIMU(const sensor_msgs::ImuConstPtr& msg){

    static uint32_t n_imu_data = 0;

    // ++n_imu_data;
    // if(n_imu_data < 8 ) {
    //     return;
    // }
    // else{
    //     n_imu_data = 0;
    // }

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
        if(bias_init_success){
            ROS_INFO("Bias Initialization successful");
        }
        timer::toc(0);
    }
    else if( filter_->isInitialized() ){
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
};


void OptitrackStateEstimatorRos::callbackMag(const sensor_msgs::MagneticFieldConstPtr& msg){
    mag_current_ = *msg;
    // std::cout << mag_current_.header.seq << ", Mag Gets: " << mag_current_.header.stamp.toNSec() << std::endl;
    
    // filter_->updateMagnetometer();
};

void OptitrackStateEstimatorRos::callbackOptitrack(const geometry_msgs::PoseStampedConstPtr& msg){
    static uint32_t n_optitrack_data = 0;
    
    // ++n_optitrack_data;
    // if(n_optitrack_data < 10 ) {
    //     return;
    // }
    // else{
    //     n_optitrack_data = 0;
    // }

    optitrack_current_ = *msg;
    double t_now = msg->header.stamp.toSec();
    
    std_msgs::Time msg_time;
    msg_time.data = ros::Time(t_now-timestamp_last_optitrack_);
    pub_optitrack_dt_.publish(msg_time);

    if(t_now-timestamp_last_optitrack_ < 0.005) {
        timestamp_last_optitrack_ = t_now;
        return;
    }
    timestamp_last_optitrack_ = t_now;

    Vec3 p_observe;
    Vec4 q_observe;
    p_observe << optitrack_current_.pose.position.x, 
                 optitrack_current_.pose.position.y, 
                 optitrack_current_.pose.position.z;
    
    q_observe << optitrack_current_.pose.orientation.w, 
                 optitrack_current_.pose.orientation.x, 
                 optitrack_current_.pose.orientation.y, 
                 optitrack_current_.pose.orientation.z;

    // Update filter
    filter_->updateOptitrack(p_observe, q_observe, t_now);
    
    // Fill the nav message
    nav_raw_current_.header.frame_id = "map";
    ++nav_raw_current_.header.seq;
    nav_raw_current_.header.stamp = ros::Time::now();
    
    // Position and orientation
    nav_raw_current_.pose.pose.position.x = optitrack_current_.pose.position.x;
    nav_raw_current_.pose.pose.position.y = optitrack_current_.pose.position.y;
    nav_raw_current_.pose.pose.position.z = optitrack_current_.pose.position.z;

    nav_raw_current_.pose.pose.orientation.w = optitrack_current_.pose.orientation.w;
    nav_raw_current_.pose.pose.orientation.x = optitrack_current_.pose.orientation.x;
    nav_raw_current_.pose.pose.orientation.y = optitrack_current_.pose.orientation.y;
    nav_raw_current_.pose.pose.orientation.z = optitrack_current_.pose.orientation.z;

    // Velocities
    nav_raw_current_.twist.twist.linear.x = 0;
    nav_raw_current_.twist.twist.linear.y = 0;
    nav_raw_current_.twist.twist.linear.z = 0;

    nav_raw_current_.twist.twist.angular.x = imu_current_.angular_velocity.x;
    nav_raw_current_.twist.twist.angular.y = imu_current_.angular_velocity.y;
    nav_raw_current_.twist.twist.angular.z = imu_current_.angular_velocity.z;

    pub_nav_raw_.publish(nav_raw_current_);
};

void OptitrackStateEstimatorRos::getParameters(){
    // get parameters
    //sub topic names
    ROSUtils::checkAndLoad<std::string>("topic_imu", topicname_imu_);
    ROSUtils::checkAndLoad<std::string>("topic_mag", topicname_mag_);
    ROSUtils::checkAndLoad<std::string>("topic_optitrack", topicname_optitrack_);

    //pub topic names
    ROSUtils::checkAndLoad<std::string>("topic_nav_filtered", topicname_nav_filtered_);
    topicname_nav_filtered_lpf_ = topicname_nav_filtered_ + "/lpf";
    ROSUtils::checkAndLoad<std::string>("topic_nav_raw", topicname_nav_raw_);
    ROSUtils::checkAndLoad<std::string>("topic_nav_filtered", topicname_nav_filtered_);
    ROSUtils::checkAndLoad<double>("noise_accel", noise_std_acc_);
    ROSUtils::checkAndLoad<double>("noise_gyro", noise_std_gyro_);
    ROSUtils::checkAndLoad<double>("noise_mag", noise_std_mag_);
    ROSUtils::checkAndLoad<bool>("verbose_all_estimation", verbose_all_estimation_);
    ROSUtils::checkAndLoad<double>("noise_optitrack_position", noise_optitrack_position_);
    ROSUtils::checkAndLoad<double>("noise_optitrack_orientation", noise_optitrack_orientation_);

    ROS_INFO_STREAM("Verbose all estimation: " << (verbose_all_estimation_ ? "true" : "false") );


    if(!ros::param::has("~q_BI"))
        throw std::runtime_error("there is no 'q_BI'. ");

    nh_.param("get_bias_from_params", get_bias_from_param_, false);
    if(get_bias_from_param_) {
        ROS_INFO("Get Bias From Param = True. Loading values from rosparam server");

        ROSUtils::checkVectorSizeAndLoad<double>("acc_bias", acc_bias_, 3);
        ROSUtils::checkVectorSizeAndLoad<double>("gyro_bias", gyro_bias_, 3);
        ROSUtils::checkVectorSizeAndLoad<double>("mag_bias", mag_bias_, 3);
    }

    else{
        ROS_INFO("Get Bias From Param = False. Bias will be estimated assuming that the platform is standing still for few seconds");
    }

    ROSUtils::checkVectorSizeAndLoad<double>("q_BI", q_BI_vec_, 4);


    // Added by KGC. Init Buffer initialization
    ROSUtils::checkAndLoad<int>("initialization/N_init_samples", N_init);
    ROSUtils::checkAndLoad<int>("initialization/N_acc_samples", N_acc);
    ROSUtils::checkAndLoad<double>("initialization/w_err", w_err);
    ROSUtils::checkAndLoad<double>("initialization/w_rot", w_rot);
    ROSUtils::checkAndLoad<double>("initialization/stdv_samples", stdv_samples);
};