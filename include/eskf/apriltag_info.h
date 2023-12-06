//
// Created by skykim0609 on 23. 12. 5.
//

#ifndef ERROR_STATE_KALMAN_FILTER_ROS_APRILTAG_INFO_H
#define ERROR_STATE_KALMAN_FILTER_ROS_APRILTAG_INFO_H

#include <Eigen/Dense>
#include <eskf/geometry_library.h>

struct ApriltagInfo{
    int id;
    geometry::Tf T_xt; // tag coordinate in world frame
    Mat66 S; // covariance of coordinate
    ApriltagInfo():id(0), T_xt(){
        S.setZero();
    }
    ApriltagInfo(int i, geometry::Tf T):id(i), T_xt(std::move(T)){
        S.setZero();
    }
    ApriltagInfo(int i, geometry::Tf T, Mat66 S_in):id(i), T_xt(std::move(T)), S(std::move(S_in)){}
};

struct ApriltagInfoArr{
    std::vector<ApriltagInfo> tag_infos;

    ApriltagInfoArr()=default;
    void push_back(const ApriltagInfo& tag){
        tag_infos.emplace_back(tag);
    }

    void push_back(int i, const geometry::Tf& T){
        tag_infos.emplace_back(i, T);
    }

    void push_back(int i, const geometry::Tf& T, const Mat66& R){
        tag_infos.emplace_back(i, T, R);
    }

    bool find(int id, ApriltagInfo& tag) const{
        for(const ApriltagInfo& tag_info : tag_infos) {
            if (tag_info.id == id) {
                tag = tag_info;
                return true;
            }
        }
        return false;
    }

    bool getTxt(int id, geometry::Tf& Txt) const{
        for(const ApriltagInfo& tag_info : tag_infos){
            if(tag_info.id == id) {
                Txt = tag_info.T_xt;
                return true;
            }
        }
        return false;
    }

//    bool getTxtAndCovariance(int id, geometry::Tf& Txt, Mat66& R) const{
//        for(const ApriltagInfo& tag_info : tag_infos){
//            if(tag_info.id == id) {
//                Txt = tag_info.T_xt;
//                R = tag_info.S;
//                return true;
//            }
//        }
//        return false;
//    }
};


#endif //ERROR_STATE_KALMAN_FILTER_ROS_APRILTAG_INFO_H
