#ifndef INITIALPOSE_ESTIMATION_H
#define INITIALPOSE_ESTIMATION_H

#include <initialpose_include.hpp>

#include <initialpose_g2o.hpp>

#include <likelihood_match.hpp>

enum corner_type {
	UNKNOWN = -1,
	LEFT = 0,
	RIGHT = 1
};

class initialpose_estimation {
public:
	initialpose_estimation(float boundary_threshold, corner_type type, double side_check_threshold):
		boundary_threshold_(boundary_threshold), type_(type), side_check_threshold_(side_check_threshold){}
		
	bool boundary_extract(const std::vector<float>& ranges_raw,
																	const std::vector<int>& number_raw);
	void corner_extract();
	
	bool min_range_search();
		
	Eigen::Vector3d pose_estimate(corner_type type);
	
	void HandleMapMessage(const nav_msgs::OccupancyGrid &map_msg);
	bool IsValidForSideCheck(const double& x, const double& y);
	corner_type side_check();
	
	void variance_calculate();
private:
	void angle_standardize(double& angle);
	double smoothness_square(int number); 
	
	Eigen::Vector3d left_rough_pose_estimate();
	Eigen::Vector3d right_rough_pose_estimate();
	void left_precise_pose_estimate(Eigen::Vector3d& initial_pose);
	void right_precise_pose_estimate(Eigen::Vector3d& initial_pose);
	
public:
	float angle_min_;
	float angle_increment_;
	std::vector<float> ranges_raw_;
	std::vector<int> increment_raw_;
	
	Eigen::Vector3d initial_pose_;
	std::vector<float> ranges_boundary_;
	std::vector<float> angles_boundary_;
	int corner_;
	
	std::shared_ptr<likelihood_match> side_checker_ptr_;
	corner_type type_;
	
	double variance_xx_ = 0;
	double variance_yy_ = 0;
	double variance_aa_ = 0;
	double covariance_xa_ = 0;
	double covariance_ya_ = 0;

private:
	float min_range1_ = 100;
	float min_range2_ = 100;
	
	float boundary_threshold_;
	
	double side_check_threshold_;
};

#endif