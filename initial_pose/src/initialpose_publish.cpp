/**********************
规定 type:
type -1: UNKNOWN
type 0: LEFT
type 1: RIGHT
**********************/

#include <initialpose_node.hpp>

//ranges_boundary和angles_boundary是按逆时针顺序排列的边缘点信息
//std::vector<float> ranges_boundary; 
//std::vector<float> angles_boundary;

int corner = 0;

Eigen::Matrix4d T_lb;

initialpose_node::initialpose_node(std::string node_name) {
	//std::cout << "Node : " << name << " initializing..." << std::endl;
	
	InitialPose_Config initialpose_config;
	initialpose_config.GetParam(&n_);

	std::string laser_topic_name = std::move(initialpose_config.laser_topic_name);
	float boundary_threshold = initialpose_config.boundary_distance;
	corner_type initial_type = (corner_type)initialpose_config.corner_type;
	double side_check_threshold = initialpose_config.prob_threshold;
	std::string initialpose_topic_name = std::move(initialpose_config.initialpose_pub_name);
	is_status_valid_ = !(initialpose_config.use_in_game);
	
	std::cout << "laser_topic_name : " << laser_topic_name << std::endl
						<< "boundary_threshold : " << boundary_threshold << std::endl
						<< "initial_type : " << initial_type << std::endl
						<< "side_check_threshold : " << side_check_threshold << std::endl;

	base_frame_ = std::move(initialpose_config.base_frame_name);
	
	tf_listener_ptr_ = std::make_unique<tf::TransformListener>();
	
	estimator_ptr_ = std::make_unique<initialpose_estimation>(boundary_threshold, initial_type, side_check_threshold);
	CheckSide_WithLaser_Valid_ = GetStaticMap();
	
	if (!setTransform(laser_topic_name)) {
		x_lb_ = 0;
		y_lb_ = 0;
		alpha_lb_ = 0;
		sin_alpha_lb_ = 0;
		cos_alpha_lb_ = 1;
	}
#ifdef CHECK_VALUE
	std::cout << "x_lb_ = " << x_lb_ << "\ty_lb_ = " << y_lb_ << "\talpha_lb_ = " << alpha_lb_ << std::endl;
#endif

	LaserScan_sub_ = n_.subscribe(laser_topic_name, 2, &initialpose_node::LaserScanCallback, this);
	GameStatus_sub_ = n_.subscribe("game_status", 1, &initialpose_node::GameStatusCallback, this);
	initialpose_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(initialpose_topic_name, 50);
}

bool initialpose_node::setTransform(std::string laser_topic_name) {
	auto laser_scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_topic_name);
	
	tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                            tf::Vector3(0, 0, 0)),
																						ros::Time(),
																						laser_scan_msg->header.frame_id);
  tf::Stamped<tf::Pose> pose_stamp;
  try {
    this->tf_listener_ptr_->transformPose(base_frame_,
                                          ident,
                                          pose_stamp);
  } catch (tf::TransformException &e) {
		std::cout << "Couldn't transform from " << laser_scan_msg->header.frame_id << " to " << base_frame_ << std::endl;
    return false;
  }
	double yaw, pitch, roll;
	pose_stamp.getBasis().getEulerYPR(yaw, pitch, roll);
	alpha_lb_ = -yaw;
	cos_alpha_lb_ = cos(alpha_lb_);
	sin_alpha_lb_ = sin(alpha_lb_);
	
	double x_bl = pose_stamp.getOrigin().x();
	double y_bl = pose_stamp.getOrigin().y();
	x_lb_ = -x_bl * cos_alpha_lb_ + y_bl * sin_alpha_lb_;
	y_lb_ = -x_bl * sin_alpha_lb_ - y_bl * cos_alpha_lb_;
	return true;
}

bool initialpose_node::GetStaticMap() {
	static_map_srv_ = n_.serviceClient<nav_msgs::GetMap>("static_map");
  ros::service::waitForService("static_map", -1);
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;
  if(static_map_srv_.call(req,res)) {
    std::cout << "Received Static Map" << std::endl;
    estimator_ptr_->HandleMapMessage(res.map);
    return true;
  } else{
    std::cout << "Get static map failed!" << std::endl;
		std::cout << "We cannot check side with Laser Scan!" << std::endl;
    return false;
  }
}

void initialpose_node::LaserScanCallback(const sensor_msgs::LaserScanConstPtr msg) {
	if (!is_status_valid_)
		return;
#ifdef CHECK_VALUE
	boost::timer process_timer;
#endif
	
	float angle_max = msg->angle_max;
	float angle_min = msg->angle_min;
	float angle_increment = msg->angle_increment;
	int angle_number = (int)((angle_max - angle_min)/angle_increment) + 1;
	
	estimator_ptr_->ranges_raw_.clear();
	estimator_ptr_->increment_raw_.clear();
	//去除遮挡点，保留有效雷达点信息，ranges_raw是range信息，increment是有效点顺序信息（用于后续生成角度信息）
	for (int i = 0; i < angle_number; i++) {
		if (msg->ranges[i] != std::numeric_limits<float>::infinity()) {
			estimator_ptr_->ranges_raw_.push_back(msg->ranges[i]);
			estimator_ptr_->increment_raw_.push_back(i);
		}
	}

	//estimator(angle_min, angle_increment, start_region_boundary_distance, type);
	//如果要判断机器人在左边还是在右边，在判断程序前给estimator_ptr_->ranges_赋值
	estimator_ptr_->angle_min_ = angle_min;
	estimator_ptr_->angle_increment_ = angle_increment;

	bool IsBoundaryValid = estimator_ptr_->boundary_extract(estimator_ptr_->ranges_raw_, estimator_ptr_->increment_raw_);
	//std::cout << "boundary_size: " << ranges_boundary.size() << std::endl;
	estimator_ptr_->corner_extract();
	bool IsMinRangeValid = estimator_ptr_->min_range_search();
	
	if ((!IsBoundaryValid) || (!IsMinRangeValid)) {
		std::cout << "We cannot continue the calculation with current scan!" << std::endl;
		return;
	}
	
	if (estimator_ptr_->type_ == UNKNOWN)
		estimator_ptr_->type_ = estimator_ptr_->side_check();
	
	else {
		estimator_ptr_->initial_pose_ = estimator_ptr_->pose_estimate(estimator_ptr_->type_);

		estimator_ptr_->variance_calculate();
	
		estimator_ptr_->initial_pose_(0) += 0.075;
		estimator_ptr_->initial_pose_(1) += 0.075;

		geometry_msgs::PoseWithCovarianceStamped initialpose_with_covariance;
		initialpose_with_covariance.header.stamp = msg->header.stamp;
		initialpose_with_covariance.header.frame_id = "map";
		
		double delta_xb_alpha = - x_lb_ * sin(estimator_ptr_->initial_pose_(2)) -  y_lb_ * cos(estimator_ptr_->initial_pose_(2)), 
					delta_yb_alpha = x_lb_ * cos(estimator_ptr_->initial_pose_(2)) + y_lb_ * sin(estimator_ptr_->initial_pose_(2));
		
		initialpose_with_covariance.pose.pose.orientation = tf::createQuaternionMsgFromYaw(estimator_ptr_->initial_pose_(2) + alpha_lb_);
		initialpose_with_covariance.pose.pose.position.x = estimator_ptr_->initial_pose_(0)
																			+ x_lb_ * cos(estimator_ptr_->initial_pose_(2))
																			- y_lb_ * sin(estimator_ptr_->initial_pose_(2));
		
		initialpose_with_covariance.pose.pose.position.y = estimator_ptr_->initial_pose_(1) - delta_xb_alpha;
		
		double variance_xx_base = estimator_ptr_->variance_xx_ + 2 * delta_xb_alpha * estimator_ptr_->covariance_xa_ + delta_xb_alpha * delta_xb_alpha * estimator_ptr_->variance_aa_,
			variance_yy_base = estimator_ptr_->variance_yy_ + 2 * delta_yb_alpha * estimator_ptr_->covariance_ya_ + delta_yb_alpha * delta_yb_alpha * estimator_ptr_->variance_aa_;
	
#ifdef CHECK_VALUE
		std::cout << "precise_pose: " << estimator_ptr_->initial_pose_.transpose() << std::endl;
		std::cout << "base_link_pose: " <<  initialpose_with_covariance.pose.pose.position.x << "\t" << initialpose_with_covariance.pose.pose.position.y << std::endl;
		std::cout << "variance_xx = " << estimator_ptr_->variance_xx_
							<< "\tvariance_yy = " << estimator_ptr_->variance_yy_
							<< "\tvariance_aa = " << estimator_ptr_->variance_aa_ 
							<< "\tcovariance_xa = " << estimator_ptr_->covariance_xa_
							<< "\tcovariance_ya = " << estimator_ptr_->covariance_ya_ << std::endl;
		std::cout << "variance_xx_base = " << variance_xx_base << "\tvariance_yy_base = " << variance_yy_base << std::endl;
#endif
	
		boost::array<double, 36> covariance = {
			variance_xx_base * 10, 0, 0, 0, 0, 0,
			0, variance_yy_base * 10, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, estimator_ptr_->variance_aa_ * 10
		};
		initialpose_with_covariance.pose.covariance = covariance;
		
		initialpose_pub_.publish(initialpose_with_covariance);
	}
	
#ifdef CHECK_VALUE
		double process_time = process_timer.elapsed();
		std::cout << "Use time : " << process_time << "s." << std::endl << std::endl;
#endif
}

void initialpose_node::GameStatusCallback(const roborts_msgs::GameStatusPtr msg) {
	if (msg->game_status == 3 && msg->remaining_time > 1)
		is_status_valid_ = true;
	else
		is_status_valid_ = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initialpose");
	//ros::Time::init();
	initialpose_node node("initialpose");

	ros::Rate loop_rate(10);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
  return 0;
}