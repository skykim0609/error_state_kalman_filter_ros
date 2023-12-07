//
// Created by skykim0609 on 23. 12. 6.
//

#ifndef ERROR_STATE_KALMAN_FILTER_ROS_ROS_UTILS_H
#define ERROR_STATE_KALMAN_FILTER_ROS_ROS_UTILS_H

#include <string>
#include <vector>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include "eskf/geometry_library.h"

namespace ROSUtils{
template <typename T> void checkAndLoad(const std::string& param_name, T& placeholder, bool is_local=true){
    std::string local_param_name;
    if(is_local) local_param_name = "~"+param_name;
    else local_param_name = param_name;
    if(!ros::param::has(local_param_name)) {
        std::string err_string = std::string("There is no '")+param_name+std::string("' in params");
        throw std::runtime_error(err_string);
    }
    ros::param::get(local_param_name, placeholder);
}
template <typename T> void checkVectorSizeAndLoad(const std::string& param_name, std::vector<T>& vec, size_t size, bool is_local=true){
    checkAndLoad<std::vector<T>>(param_name, vec, is_local);
    if(vec.size() != size){
        std::string err_string = std::string("Size of ") + param_name + std::string(" != ") + std::to_string(size)
                                 + std::string(" ( Got vector size : ") + std::to_string(vec.size())+std::string(").");
        throw std::runtime_error(err_string);
    }
}

[[nodiscard]] inline geometry::Tf posemsgToTf(const geometry_msgs::Pose& msg){
    geometry::Tf T;
    auto p = msg.position;
    auto q = msg.orientation;
    T.setTrans(Vec3(p.x, p.y, p.z));
    T.setRot(Vec4(q.w, q.x, q.y, q.z));
    return T;
}
}//namespace ROSUtils

#endif //ERROR_STATE_KALMAN_FILTER_ROS_ROS_UTILS_H
