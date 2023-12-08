//
// Created by skykim0609 on 23. 12. 7.
//

#ifndef ERROR_STATE_KALMAN_FILTER_ROS_TRAJ_COMPARATOR_H
#define ERROR_STATE_KALMAN_FILTER_ROS_TRAJ_COMPARATOR_H

#endif //ERROR_STATE_KALMAN_FILTER_ROS_TRAJ_COMPARATOR_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "eskf/ros_utils.h"
#include "eskf/geometry_library.h"

#define VERBOSITY 3 // very  verbose

class TrajComparator{
public:
    struct InitBuffer{
    public:
        InitBuffer(int N_init_samples, int N_max_iter, double w_thresh):
        N_init(N_init_samples), N_max(N_max_iter), w_thr(w_thresh), N_samples(0){}
        void addSamples(const geometry::Tf& T_mi, const geometry::Tf& T_wb);
        bool tryInitialize(const Vec4 &q_ib_nom, geometry::Tf &T_ib, geometry::Tf &T_wm);
    private:
        int N_init, N_max, N_samples;
        double w_thr;
        std::vector<geometry::Tf> T_mi_samples; // odometry samples
        std::vector<geometry::Tf> T_wb_samples; // gt(mocap) samples

        Vec3 estimatePwm() const;
        Vec4 estimateQib(const Vec4& q_ib_nom) const;
    };

    struct PoseError{
        PoseError(Vec3 dw, Vec3 dt):rot_err(std::move(dw)), trans_err(std::move(dt)){
            dr_norm = rot_err.norm();
            dt_norm = trans_err.norm();
        }
        Vec3 rot_err, trans_err;
        double dr_norm, dt_norm;

        friend std::ostream& operator<<(std::ostream& os, const PoseError& P);
    };

    struct GtEstPair{
        GtEstPair(const geometry::Tf& T_wg_in, const geometry::Tf& T_me_in, double t):
        T_wg(T_wg_in), T_me(T_me_in), time(t){}
        geometry::Tf getTwgEst(const geometry::Tf& T_wm, const geometry::Tf& T_eg) const;
        geometry::Tf computeGTToEst(const geometry::Tf& T_wm, const geometry::Tf& T_eg) const;
        PoseError computePoseError(const geometry::Tf& T_wm, const geometry::Tf& T_eg) const;
        geometry::Tf T_wg; // world -> GT pose
        geometry::Tf T_me; // map -> est pose
        double time;

        friend std::ostream& operator<<(std::ostream& os, const GtEstPair& P);
    };

public:
    TrajComparator(const ros::NodeHandle& nh);
    void run();
    void print() const;
    void output() const;
private:
    /*ros related*/
    ros::NodeHandle nh_;
    std::string gt_pose_topic;
    std::string est_nav_topic;

    ros::Subscriber sub_gt_;
    ros::Subscriber sub_est_;

    void callbackGT(const geometry_msgs::PoseStampedConstPtr& msg);
    void callbackEst(const nav_msgs::OdometryConstPtr& msg);
    /* end ros related*/
    /*loaded from rosparam*/
    //session params
    double print_every_;
    std::string output_dir;
    std::string session_name;

    //init related
    int N_max_iter_;
    int N_init_samples_;
    double w_thresh_;

    // nominal transforms
    Vec4 q_ib_nom; // won't
    /*end loaded from rosparam*/

    // Estimated transforms
    geometry::Tf T_wm_;
    geometry::Tf T_ib_;

    // buffers
    std::unique_ptr<InitBuffer> init_buffer_;
    geometry_msgs::PoseStamped pose_T_wb_last;
    nav_msgs::Odometry odom_T_mi_last;
    bool got_gt_pose;
    bool is_initialized;

    std::vector<GtEstPair> gt_est_history;
    std::vector<PoseError> pose_error_history;
    double t_start;
    bool got_first_time;

    //logging
    void printLog(const std::string& logfile_name) const;
    void printCsvFiles(const std::string& pose_error_csv, const std::string& pose_csv) const;
};
