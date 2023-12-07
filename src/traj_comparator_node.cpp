//
// Created by skykim0609 on 23. 12. 7.
//
#include <ros/ros.h>
#include "eskf/traj_comparator.h"
#include "eskf/signal_handler_linux.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "traj_comparator", ros::init_options::NoSigintHandler);
    SignalHandle::initSignalHandler();

    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("Traj Comparator node starts");

    try{
        if(ros::ok()){
            std::unique_ptr<TrajComparator> traj_comp = std::make_unique<TrajComparator>(nh);
        }
    }
    catch(std::exception& e){
        ROS_ERROR_STREAM(e.what());
    }
    ROS_INFO_STREAM("Traj comparator node terminated");
    return 1;
}