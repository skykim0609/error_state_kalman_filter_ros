#include <iostream>
#include <ros/ros.h>

#include "eskf/optitrack_state_estimator_ros.h"

#include "eskf/signal_handler_linux.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "optitrack_state_estimator_node", ros::init_options::NoSigintHandler);
    SignalHandle::initSignalHandler();
    // ros::init(argc, argv, "state_estimator_node");

    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("Optitrack state_estimator_node - STARTS.");

	try{
		if(ros::ok()){
			std::unique_ptr<OptitrackStateEstimatorRos> st_est;
			st_est = std::make_unique<OptitrackStateEstimatorRos>(nh);
		}
		else throw std::runtime_error("ROS not ok");
	}
	catch (std::exception& e){
        ROS_ERROR_STREAM(e.what());
	}

    ROS_INFO_STREAM("state_estimator_node - TERMINATED.");
	return 0;
}