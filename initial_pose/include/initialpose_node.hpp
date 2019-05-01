#ifndef INITIALPOSE_NODE_H
#define INITIALPOSE_NODE_H

#include <initialpose_include.hpp>

#include <initialpose_config.hpp>

#include <initialpose_estimation.hpp>

class initialpose_node {
public:
	initialpose_node(std::string node_name);
	bool setTransform(std::string laser_topic_name);
	bool GetStaticMap();
	
	void LaserScanCallback(const sensor_msgs::LaserScanConstPtr msg);
	void GameStatusCallback(const roborts_msgs::GameStatusPtr msg);
	
public:
	std::unique_ptr<initialpose_estimation> estimator_ptr_;
	bool CheckSide_WithLaser_Valid_ = false;
	
	double x_lb_;
	double y_lb_;
	double alpha_lb_;
	double cos_alpha_lb_;
	double sin_alpha_lb_;
	
private:
	ros::NodeHandle n_;
	ros::Subscriber LaserScan_sub_;
	ros::Subscriber GameStatus_sub_;
	ros::ServiceClient static_map_srv_;
	
	std::unique_ptr<tf::TransformListener> tf_listener_ptr_;
	
	std::string base_frame_;
	
	ros::Publisher initialpose_pub_;

	bool is_status_valid_;
};

#endif