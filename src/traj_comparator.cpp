//
// Created by skykim0609 on 23. 12. 7.
//
#include <fstream>
#include <filesystem>

#include "eskf/traj_comparator.h"

#define RISC(thr, X) ROS_INFO_STREAM_COND(VERBOSITY >= thr, X)

using namespace geometry;
namespace fs = std::filesystem;
using fs_path = fs::path;

bool TrajComparator::InitBuffer::tryInitialize(const Vec4 &q_ib_nom, geometry::Tf &T_ib, geometry::Tf &T_wm) {
    if(N_samples < N_init) return false;
    T_wm.setRot(Vec4(1.0, 0.0, 0.0, 0.0));
    T_wm.setTrans(estimatePwm());

    T_ib.setRot(estimateQib(q_ib_nom));
    T_ib.setTrans(Vec3::Zero());

    return true;
}

void TrajComparator::InitBuffer::addSamples(const geometry::Tf &T_mi, const geometry::Tf &T_wb) {
    T_mi_samples.emplace_back(T_mi);
    T_wb_samples.emplace_back(T_wb);
    ++N_samples;
}

Vec3 TrajComparator::InitBuffer::estimatePwm() const {
    Vec3 t_wm_sum = Vec3::Zero();
    for(int i =0; i < N_samples; ++i){
        t_wm_sum += T_wb_samples[i].trans() - T_mi_samples[i].trans(); // t_mw = t_wb - t_mb
    }
    Vec3 t_wm_avg = t_wm_sum / N_samples;
    RISC(0, "estimatePwm result : "<<t_wm_avg.transpose());
    return t_wm_avg;
}

Vec4 TrajComparator::InitBuffer::estimateQib(const Vec4& q_ib_nom) const {
    int iter = 0;
    Vec4 q_ib_curr = q_ib_nom;
    Vec3 w_avg;
    double err = 1e10;
    while(iter < N_max){
        RISC(2, "estimate Qib : iteration "<<++iter);
        Vec3 w_sum = Vec3::Zero();
        for(int i =0; i < N_samples; ++i){
            Vec4 q_mi = T_mi_samples[i].rot();
            Vec4 q_wb = T_wb_samples[i].rot();
            Vec4 q_ib = q1_mult_q2(q_conj(q_mi), q_wb); // R_im * R_mw(I) * R_wb
            Vec3 w_i = q2rotvec(q1_mult_q2(q_conj(q_ib_curr), q_ib));
            w_sum += w_i;
            RISC(3, "estimate Qib : Sample "<<i <<" : rotation error current : "<<w_i.transpose());
        }
        w_avg = w_sum / N_samples;
        RISC(2, "estimate Qib : w_avg : "<<w_avg.transpose());
        err = w_avg.norm();
        if(err < w_thr){
            RISC(0, "estimate Qib : Reached w_error threshold. Error : "<<err<< " Threshold : "<<w_thr);
            RISC(0, "estimate Qib : q_ib : "<<q_ib_curr.transpose());
            return q_ib_curr;
        }
        q_ib_curr = q1_mult_q2(q_ib_curr, rotvec2q(w_avg));
    }
    RISC(0, "estimate Qib : Max iter reached but does not match error threshold. Error : "<<err);
    RISC(0, "estimate Qib : q_ib : "<<q_ib_curr.transpose());
    return q_ib_curr;
}

geometry::Tf TrajComparator::GtEstPair::getTwgEst(const geometry::Tf &T_wm, const geometry::Tf &T_eg) const {
    return T_wm.mult(T_me).mult(T_eg);
}

geometry::Tf TrajComparator::GtEstPair::computeGTToEst(const geometry::Tf &T_wm, const geometry::Tf &T_eg) const{
    return T_wg.Inverse().mult(getTwgEst(T_wm, T_eg));
}

TrajComparator::PoseError TrajComparator::GtEstPair::computePoseError(const geometry::Tf &T_wm,
                                                                      const geometry::Tf &T_eg) const {
    auto T_err = computeGTToEst(T_wm, T_eg);
    Vec3 rot_err = geometry::q2rotvec(T_err.rot());
    Vec3 trans_err = T_err.trans();
    return PoseError(rot_err, trans_err);
}

std::ostream& operator<<(std::ostream& os, const TrajComparator::PoseError& P){
    os <<"Trans Error : "<<P.trans_err.transpose()<<", size : "<<P.dt_norm<<"\n";
    os <<"Rot Error : "<<P.trans_err.transpose()<<", size : "<<P.dr_norm<<"\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, const TrajComparator::GtEstPair& P){
    os <<"At time "<<P.time<<"\n";
    os <<"T_wg(gt) : "<<P.T_wg<<"\n";
    os <<"T_me(est) : "<<P.T_me<<"\n";
    return os;
}

TrajComparator::TrajComparator(const ros::NodeHandle& nh):nh_(nh), got_gt_pose(false), is_initialized(false), got_first_time(false){
    ROSUtils::checkAndLoad<double>("print_every", print_every_);
    ROSUtils::checkAndLoad<int>("N_max_iter", N_max_iter_);
    ROSUtils::checkAndLoad<int>("N_init_samples", N_init_samples_);
    ROSUtils::checkAndLoad<double>("w_thresh", w_thresh_);
    ROSUtils::checkAndLoad<std::string>("output_dir", output_dir);
    ROSUtils::checkAndLoad<std::string>("session_name", session_name);
    ROSUtils::checkAndLoad<std::string>("gt_pose_topic", gt_pose_topic);
    ROSUtils::checkAndLoad<std::string>("est_nav_topic", est_nav_topic);

    std::vector<double> q_ib_nom_vec;
    ROSUtils::checkVectorSizeAndLoad<double>("q_ib_nom", q_ib_nom_vec, 4);
    q_ib_nom << q_ib_nom_vec[0],q_ib_nom_vec[1],q_ib_nom_vec[2],q_ib_nom_vec[3];

    sub_gt_ = nh_.subscribe<geometry_msgs::PoseStamped>(gt_pose_topic, 5, &TrajComparator::callbackGT, this);
    sub_est_ = nh_.subscribe<nav_msgs::Odometry>(est_nav_topic, 5, &TrajComparator::callbackEst, this);

    init_buffer_ = std::make_unique<InitBuffer>(N_init_samples_, N_max_iter_, w_thresh_);
    this->run();
}

void TrajComparator::callbackGT(const geometry_msgs::PoseStampedConstPtr& msg){
    pose_T_wb_last = *msg;
    got_gt_pose = true;
}

void TrajComparator::callbackEst(const nav_msgs::OdometryConstPtr &msg) {
    odom_T_mi_last = *msg;
    if(!got_gt_pose){
        RISC(1, "callbackEst : Got odometry, but no GT pose received");
        return;
    }
    if(!got_first_time){
        RISC(1, "callbackEst : Got the first estimated estimation msg");
        got_first_time = true;
        t_start = msg->header.stamp.toSec();
    }
    geometry::Tf T_mi = ROSUtils::posemsgToTf(odom_T_mi_last.pose.pose);
    geometry::Tf T_wb = ROSUtils::posemsgToTf(pose_T_wb_last.pose);
    if(!is_initialized){
        init_buffer_->addSamples(T_mi, T_wb);
        RISC(1, "Try initializing Rel transform");
        if(init_buffer_->tryInitialize(q_ib_nom, T_ib_, T_wm_)){
            RISC(0, "Relative Transform Initialized.");
            is_initialized = true;
        }
        return;
    }
    double t = odom_T_mi_last.header.stamp.toSec() - t_start;
    GtEstPair pair(T_wb, T_mi, t);
    gt_est_history.emplace_back(pair);
    pose_error_history.emplace_back(pair.computePoseError(T_wm_, T_ib_));
}

void TrajComparator::print() const{
    std::cout<<"trajcomparator\n";
    size_t n_pose = pose_error_history.size();
    std::cout<<"# of pose comparisons : "<<n_pose<<"\n";
    double dw_sum = 0.0;
    double dt_sum = 0.0;
    for(const PoseError& pose_error : pose_error_history){
        dw_sum += pose_error.dr_norm;
        dt_sum += pose_error.dt_norm;
    }
    std::cout<<"Position Error Average : "<<dt_sum / n_pose<<"\n";
    std::cout<<"Rotation Error average : "<<dw_sum / n_pose<<"\n";
}

void TrajComparator::run(){
    int node_rate = 1000;
    ros::Rate rate(node_rate);

    ros::Time t_prev = ros::Time::now();
    ros::Time t_curr;

    while(ros::ok()){
        t_curr = ros::Time::now();
        if((t_curr - t_prev).toSec() > print_every_) {
            this->print();
            t_prev = t_curr;
        }
        ros::spinOnce();
        rate.sleep();
    }
    this->output();
}

void TrajComparator::output() const{
    RISC(1, "Exporting Result to "<< output_dir);
    fs_path output_dir_fs(output_dir);
    if(!fs::exists(fs_path(output_dir_fs))){
        RISC(0, "Output directory "<<output_dir<<" does not exists. Try creating");
        try{
            if(fs::create_directory(output_dir)) {
                RISC(0, "Creation successful");
            }
        }
        catch(const fs::filesystem_error& e){
            ROS_ERROR_STREAM(e.what());
            return;
        }
    }
    fs_path log_fs = output_dir_fs / fs_path("log.txt");
    printLog(log_fs.string());

    fs_path pose_error_fs = output_dir_fs / fs_path("pose_error.csv");
    fs_path pose_fs = output_dir_fs / fs_path("gt_est_aligned.csv");
    printCsvFiles(pose_error_fs.string(), pose_fs.string());
}

void TrajComparator::printLog(const std::string &logfile_name) const {
    RISC(2, "Writing log file to "<<logfile_name);
    std::ofstream ofs_log;
    ofs_log.open(logfile_name);
    ofs_log << "Session " << session_name << "\n";
    ofs_log << "=======================Relative coordinate frame calibration : \n";
    ofs_log << "T_wm(odometry map to world frame) : " << T_wm_ << "\n";
    ofs_log << "T_ib(body to IMU frame) : " << T_ib_ << "\n";
    ofs_log << "================================================\n";
    ofs_log << "======================GT pose and Estimated pose & Pose Error history\n";
    size_t n_hist = gt_est_history.size();
    if(n_hist != pose_error_history.size())
        throw std::runtime_error("Pose Error history size != gt_est_history size");
    for(size_t i = 0; i  < n_hist;  ++i){
        ofs_log << gt_est_history[i] << pose_error_history[i] << "\n";
    }
    // print averager
    ofs_log <<" ====================== Summary ======================\n";
    size_t n_pose = pose_error_history.size();
    ofs_log<<"# of pose comparisons : "<<n_pose<<"\n";
    double dw_sum = 0.0;
    double dt_sum = 0.0;
    for(const PoseError& pose_error : pose_error_history){
        dw_sum += pose_error.dr_norm;
        dt_sum += pose_error.dt_norm;
    }
    ofs_log<<"Position Error Average : "<<dt_sum / n_pose<<"\n";
    ofs_log<<"Rotation Error average : "<<dw_sum / n_pose<<"\n";
    ofs_log.close();
    RISC(2, "Done");
}

void TrajComparator::printCsvFiles(const std::string &pose_error_csv, const std::string &pose_csv) const {
    RISC(2, "Writing pose history / pose error history file to "<<pose_csv<<", "<<pose_error_csv);
    std::ofstream ofs_pe, ofs_pose;
    ofs_pe.open(pose_error_csv);
    ofs_pose.open(pose_csv);
    ofs_pe << "dpx,dpy,dpz,dp_norm,drx,dry,drz,dr_norm\n";
    ofs_pose <<"p_wg_x,p_wg_y,p_wg_z,q_wg_w,q_wg_x,q_wg_y,q_wg_z,"
            <<"p_we_x,p_we_y,p_we_z,q_we_w,q_we_x,q_we_y,q_we_z\n";
    size_t n_hist = gt_est_history.size();
    if(n_hist != pose_error_history.size())
        throw std::runtime_error("Pose Error history size != gt_est_history size");
    for(size_t i = 0; i  < n_hist;  ++i){
        auto Twg_est = gt_est_history[i].getTwgEst(T_wm_, T_ib_);
        const auto& Twg = gt_est_history[i].T_wg;
        const PoseError& pe = pose_error_history[i];
        // pose error
        for(int k = 0; k < 3; ++k){
            ofs_pe << pe.trans_err(k)<<",";
        }
        ofs_pe <<pe.dt_norm<<",";
        for(int k = 0; k < 3; ++k){
            ofs_pe << pe.rot_err(k)<<",";
        }
        ofs_pe <<pe.dr_norm<<"\n";

        // gt pose
        Vec3 twg_gt = Twg.trans();
        Vec4 qwg_gt = Twg.rot();
        for(int k = 0; k < 3; ++k){
            ofs_pose << twg_gt(k) <<",";
        }
        for(int k = 0; k < 4; ++k){
            ofs_pose << qwg_gt(k) <<",";
        }

        // est pose
        Vec3 twg_est = Twg_est.trans();
        Vec4 qwg_est = Twg_est.rot();
        for(int k = 0; k < 3; ++k){
            ofs_pose << twg_est(k) <<",";
        }
        for(int k = 0; k < 3; ++k){
            ofs_pose << qwg_est(k) <<",";
        }
        ofs_pose <<qwg_est(3) <<"\n";
    }
    ofs_pose.close();
    ofs_pe.close();
    RISC(2, "Done");
}
