#ifndef INITIALPOSE_CONFIG_H
#define INITIALPOSE_CONFIG_H

#include <initialpose_include.hpp>

struct InitialPose_Config {
	void GetParam(ros::NodeHandle* n_) {
		n_->param<std::string>("laser_topic_name", laser_topic_name, "scan");
		n_->param<std::string>("base_frame_id", base_frame_name, "base_link");
		
		n_->param<double>("boundary_distance", boundary_distance, 2.1);
		n_->param<double>("prob_threshold", prob_threshold, 0.88);
		n_->param<int>("corner_type", corner_type, -1);
		
		n_->param<std::string>("init_pose_topic_name", initialpose_pub_name, "initialpose");
		n_->param<bool>("use_in_game", use_in_game, true);
	}
	std::string laser_topic_name;
	std::string base_frame_name;
	
	double boundary_distance;
	double prob_threshold;
	int corner_type;
	
	std::string initialpose_pub_name;
	bool use_in_game;
};

#endif