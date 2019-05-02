#include <initialpose_estimation.hpp>

bool initialpose_estimation::boundary_extract(const std::vector<float>& ranges_raw,
																							const std::vector<int>& number_raw) {
	if (ranges_raw.size() != number_raw.size()) {
		std::cout << "The raw data of ranges and angles do not equal!" << std::endl;
		std::cout << "Fatal error! The program is shutting down..." << std::endl;
		exit(0);
	}
	
	int available_number = ranges_raw.size();
	std::vector<bool> distance(available_number, false);
	
	//找到第一个不符合距离要求的点
	int first_negative = -1;
	for (int i = 0; i < available_number; i++) {
		if (ranges_raw[i] < boundary_threshold_)
			distance[i] = true;
		else if (first_negative == -1)
			first_negative = i;
	}
	
	int continuous_max_last_location = 0;
	int continuous_max_number = 0;
	int continuous_max_first_location = 0;
	int continuous_first_location_temp = 0;
	int continuous_number_now = 0;
	//找到满足距离要求的最大连续点集,以判断两个直角边位置,从第一个不满足距离要求的点开始，循环一圈，防止重复遗漏，以及vector起始终止处连续点集判断错误的情况
	for (int i = first_negative + 1; i < available_number; i++) {
		if (distance[i]) {
			//如果上一个点远，这一个点近，则取当前点作为当前连续点集起始边界，由于是从first_negative+1开始，到available结束的，所以不会判断初始集外的位置
			if (!distance[i - 1])
				continuous_first_location_temp = i;
			
			continuous_number_now++;
			if (continuous_number_now > continuous_max_number) {
				continuous_max_number = continuous_number_now;
				continuous_max_last_location = i;
				continuous_max_first_location = continuous_first_location_temp;
			}
		} else 
			continuous_number_now = 0;
	}
	if (distance[0]) {
		if (!distance[available_number - 1]) 
			continuous_first_location_temp = 0;
		
		continuous_number_now++;
		if (continuous_number_now > continuous_max_number) {
			continuous_max_number = continuous_number_now;
			continuous_max_last_location = 0;
			continuous_max_first_location = continuous_first_location_temp;
		}
	} else 
		continuous_number_now = 0;
	for (int i = 1; i < first_negative + 1; i++) {
		if (distance[i]) {
			//如果上一个点远，这一个点近，则取当前点作为当前连续点集起始边界，由于是从first_negative+1开始，到available结束的，所以不会判断初始集外的位置
			if (!distance[i - 1])
				continuous_first_location_temp = i;
			
			continuous_number_now++;
			if (continuous_number_now > continuous_max_number) {
				continuous_max_number = continuous_number_now;
				continuous_max_last_location = i;
				continuous_max_first_location = continuous_first_location_temp;
			}
		} else 
			continuous_number_now = 0;
	}
	
	ranges_boundary_.clear();
	angles_boundary_.clear();
	
	if (continuous_max_first_location < continuous_max_last_location) {
		for (int i = continuous_max_first_location; i < continuous_max_last_location + 1; i++) {
			ranges_boundary_.push_back(ranges_raw[i]);
			angles_boundary_.push_back(angle_min_ + angle_increment_ * number_raw[i]);
		}
	} else {
		for (int i = continuous_max_first_location; i < available_number; i++) {
			ranges_boundary_.push_back(ranges_raw[i]);
			angles_boundary_.push_back(angle_min_ + angle_increment_ * number_raw[i]);
		}
		for (int i = 0; i <= continuous_max_last_location; i++) {
			ranges_boundary_.push_back(ranges_raw[i]);
			angles_boundary_.push_back(angle_min_ + angle_increment_ * number_raw[i]);
		}
	}

	if (ranges_boundary_.size() < 22) {
		std::cout << "The size of boundary points is too small!" << std::endl;
		return false;
	} else
		return true;
}

void initialpose_estimation::corner_extract() {
	double smoothness_max = 0;
	for (int i = 11; i < ranges_boundary_.size() - 11; i++) {
		double smoothness = smoothness_square(i);
		if (smoothness > smoothness_max) {
			smoothness_max = smoothness;
			corner_ = i;
		}
	}
	
#ifdef CHECK_VALUE
	std::cout << "Boundary_size : " << ranges_boundary_.size() 
						<< "\tCorner: " << corner_
						<< "\tSmoothness_max1: " << smoothness_max << std::endl;
#endif
}

Eigen::Vector3d initialpose_estimation::pose_estimate(corner_type type) {
	Eigen::Vector3d initial_pose;
	switch(type) {
		case LEFT:
			initial_pose = left_rough_pose_estimate();
			left_precise_pose_estimate(initial_pose);
			break;
		case RIGHT:
			initial_pose = right_rough_pose_estimate();
			right_precise_pose_estimate(initial_pose);
			break;
	}
	return initial_pose;
}

bool initialpose_estimation::min_range_search() {
	min_range1_ = 100, min_range2_ = 100;
	for (int i = 0; i < corner_ - 9; i++) {
		if (ranges_boundary_[i] < min_range1_)
			min_range1_ = ranges_boundary_[i];
	}
	for (int i = corner_ + 10; i < ranges_boundary_.size(); i++) {
		if (ranges_boundary_[i] < min_range2_) {
			min_range2_ = ranges_boundary_[i];
		}
	}
	
	if (min_range1_ > ranges_boundary_[corner_] && min_range2_ > ranges_boundary_[corner_]) {
		std::cout << "Ranges of all boundary points are bigger than range of corner points!" << std::endl;
		return false;
	} else
		return true;
}

Eigen::Vector3d initialpose_estimation::left_rough_pose_estimate() {
	double x, y, alpha, theta;
	alpha = angles_boundary_[corner_];
	x = min_range1_;
	y = min_range2_;
	if (min_range1_ < min_range2_)
		theta = acos(min_range1_ / ranges_boundary_[corner_]);
	else
		theta = asin(min_range2_ / ranges_boundary_[corner_]);
	alpha = M_PI - alpha + theta;

	angle_standardize(alpha);
	return Eigen::Vector3d(x, y, alpha);
}

Eigen::Vector3d initialpose_estimation::right_rough_pose_estimate() {
	double x, y, alpha, theta;
	alpha = angles_boundary_[corner_];
	x = 8 - min_range2_;
	y = min_range1_;
	if (min_range1_ > min_range2_)
		theta = acos(min_range2_ / ranges_boundary_[corner_]);
	else
		theta = asin(min_range1_ / ranges_boundary_[corner_]);
	alpha = - alpha - theta;
	
	angle_standardize(alpha);
	return Eigen::Vector3d(x, y, alpha);
}


void initialpose_estimation::left_precise_pose_estimate(Eigen::Vector3d& initial_pose) {
	// pose 维度为 3, landmark 维度为 1
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<3, 1> > Block;
	// 线性方程求解器
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
	// 矩阵块求解器
	Block* solver_ptr = new Block ( linearSolver );
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(solver);

  // vertex
	BoundaryPoseVertex* pose_vertex = new BoundaryPoseVertex(); // camera pose
	pose_vertex->setEstimate(initial_pose);
	pose_vertex->setId(0);
	optimizer.addVertex(pose_vertex);
	
	int index = 1;
		
	Eigen::Matrix2d Information_Matrix = Eigen::Matrix2d::Identity() * 1e-4;


	for (int i = 0; i < corner_ - 9; i++) {
		Eigen::Vector2d point(angles_boundary_[i], ranges_boundary_[i]);
		BoundaryEdge_Left* edge = new BoundaryEdge_Left(angles_boundary_[i], ranges_boundary_[i]);
		edge->setId(index);
		edge->setVertex(0, pose_vertex);
		// 设置连接的顶点
		edge->setMeasurement(point);
		// 观测数值
		edge->setInformation(Information_Matrix);
		// 信息矩阵：协方差矩阵之逆
		optimizer.addEdge(edge);
		index++;
	}
	for (int i = corner_ + 10; i < ranges_boundary_.size(); i++) {
			Eigen::Vector2d point(angles_boundary_[i], ranges_boundary_[i]);
			BoundaryEdge_Bottom* edge = new BoundaryEdge_Bottom(angles_boundary_[i], ranges_boundary_[i]);
			edge->setId(index);
			edge->setVertex(0, pose_vertex);
			// 设置连接的顶点
			edge->setMeasurement(point);
			// 观测数值
			edge->setInformation(Information_Matrix);
			// 信息矩阵：协方差矩阵之逆
			optimizer.addEdge(edge);
			index++;
	}
	
	optimizer.initializeOptimization();
	optimizer.optimize(200);
		
	initial_pose = pose_vertex->estimate();
}

void initialpose_estimation::right_precise_pose_estimate(Eigen::Vector3d& initial_pose) {
		// pose 维度为 3, landmark 维度为 1
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<3, 1> > Block;
	// 线性方程求解器
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
	// 矩阵块求解器
	Block* solver_ptr = new Block ( linearSolver );
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(solver);

  // vertex
	BoundaryPoseVertex* pose_vertex = new BoundaryPoseVertex(); // camera pose
	pose_vertex->setEstimate(initial_pose);
	pose_vertex->setId(0);
	optimizer.addVertex(pose_vertex);
	
	int index = 1;
		
	Eigen::Matrix2d Information_Matrix = Eigen::Matrix2d::Identity() * 1e-4;
	
	for (int i = 0; i < corner_ - 9; i++) {
		Eigen::Vector2d point(angles_boundary_[i], ranges_boundary_[i]);
		BoundaryEdge_Bottom* edge = new BoundaryEdge_Bottom(angles_boundary_[i], ranges_boundary_[i]);
		edge->setId(index);
		edge->setVertex(0, pose_vertex);
		// 设置连接的顶点
		edge->setMeasurement(point);
		// 观测数值
		edge->setInformation(Information_Matrix);
		// 信息矩阵：协方差矩阵之逆
		optimizer.addEdge(edge);
		index++;
	}
	for (int i = corner_ + 10; i < ranges_boundary_.size(); i++) {
		Eigen::Vector2d point(angles_boundary_[i], ranges_boundary_[i]);
		BoundaryEdge_Right* edge = new BoundaryEdge_Right(angles_boundary_[i], ranges_boundary_[i]);
		edge->setId(index);
		edge->setVertex(0, pose_vertex);
		// 设置连接的顶点
		edge->setMeasurement(point);
		// 观测数值
		edge->setInformation(Information_Matrix);
		// 信息矩阵：协方差矩阵之逆
		optimizer.addEdge(edge);
		index++;
	}
		
	optimizer.initializeOptimization();
	optimizer.optimize(200);
		
	initial_pose = pose_vertex->estimate();
}

void initialpose_estimation::HandleMapMessage(const nav_msgs::OccupancyGrid &map_msg) {
	side_checker_ptr_ = std::make_shared<likelihood_match>(map_msg, 5.0);
}

bool initialpose_estimation::IsValidForSideCheck(const double& x, const double& y) {
	if (x < 0.05 || x > 7.95)
		return false;
	else if (y < 0.05 || y > 4.95)
		return false;
	else if ((x <= 1.0 || x >= 7.0) && y >= 4.0)
		return false;
	else
		return true;
}

corner_type initialpose_estimation::side_check() {
	Eigen::Vector3d pose_left, pose_right;
	
	pose_left = pose_estimate(LEFT);
	pose_right = pose_estimate(RIGHT);
	
	std::vector<double> points_x;
	std::vector<double> points_y;
	
	double left_prob, right_prob;
	
	for (int i = 0; i < ranges_raw_.size(); i++) {
		double point_angle = pose_left(2) + (double)(angle_min_ + angle_increment_ * increment_raw_[i]);
		double x = pose_left(0) + ((double)ranges_raw_[i]) * cos(point_angle);
		double y = pose_left(1) + ((double)ranges_raw_[i]) * sin(point_angle);
		if (IsValidForSideCheck(x, y)) {
			points_x.push_back(x);
			points_y.push_back(y);
		}
	}
	left_prob = side_checker_ptr_->ProbMatch(points_x, points_y);
	
	points_x.clear();
	points_y.clear();
	
	for (int i = 0; i <ranges_raw_.size(); i++) {
		double point_angle = pose_right(2) + (double)(angle_min_ + angle_increment_ * increment_raw_[i]);
		double x = pose_right(0) + ((double)ranges_raw_[i]) * cos(point_angle);
		double y = pose_right(1) + ((double)ranges_raw_[i]) * sin(point_angle);
		if (IsValidForSideCheck(x, y)) {
			points_x.push_back(x);
			points_y.push_back(y);
		}
	}
	right_prob = side_checker_ptr_->ProbMatch(points_x, points_y);
	double total_prob = left_prob + right_prob;
	
	std::cout << "Total prob:" << total_prob << std::endl;
	
	if (total_prob < 0.1)
		return UNKNOWN;
	
 	left_prob /= total_prob;
	right_prob /= total_prob;
	
	std::cout << "\tLeft_prob:" << left_prob << "\tRight_prob:" << right_prob << std::endl;
	
	if (left_prob > side_check_threshold_)
		return LEFT;
	else if (right_prob > side_check_threshold_)
		return RIGHT;
	else
		return UNKNOWN;
}

void initialpose_estimation::variance_calculate() {
	variance_xx_ = 0;
	variance_yy_ = 0;
	variance_aa_ = 0;
	covariance_xa_ = 0;
	covariance_ya_ = 0;

	if (type_ == LEFT) {
		for (int i = 0; i < corner_ - 9; i++) {
			double delta_x = -(initial_pose_(0) + ranges_boundary_[i] * cos(angles_boundary_[i] + initial_pose_(2)));
			variance_xx_ += delta_x * delta_x;
										
			double delta_alpha1;
			if (initial_pose_(0) > ranges_boundary_[i])
				delta_alpha1 = M_PI;
			else if (initial_pose_(0) < -ranges_boundary_[i])
				delta_alpha1 = 0;
			else 
				delta_alpha1 = acos(-initial_pose_(0) / ranges_boundary_[i]);
			double delta_alpha2 = -delta_alpha1;
			delta_alpha1 -= angles_boundary_[i] + initial_pose_(2);
			delta_alpha2 -= angles_boundary_[i] + initial_pose_(2);
			
			angle_standardize(delta_alpha1);
			angle_standardize(delta_alpha2);
			
			if (fabs(delta_alpha1) < fabs(delta_alpha2)) {
				variance_aa_ += delta_alpha1 * delta_alpha1;
				covariance_xa_ += delta_x * delta_alpha1;
			} else {
				variance_aa_ += delta_alpha2 * delta_alpha2;
				covariance_xa_ += delta_x * delta_alpha2;
			}
		}
		for (int i = corner_ + 10; i < ranges_boundary_.size(); i++) {
			double delta_y = -(initial_pose_(1) + ranges_boundary_[i] * sin(angles_boundary_[i] + initial_pose_(2)));
			variance_yy_ += delta_y * delta_y;
			
			double delta_alpha1;
			if (initial_pose_(1) > ranges_boundary_[i])
				delta_alpha1 = -M_PI/2;
			else if (initial_pose_(1) < -ranges_boundary_[i]) 
				delta_alpha1 = M_PI/2;
			else
				delta_alpha1 = asin(-initial_pose_(1) / ranges_boundary_[i]);
			double delta_alpha2 = M_PI - delta_alpha1;
			delta_alpha1 -= angles_boundary_[i] + initial_pose_(2);
			delta_alpha2 -= angles_boundary_[i] + initial_pose_(2);
			
			angle_standardize(delta_alpha1);
			angle_standardize(delta_alpha2);
			if (fabs(delta_alpha1) < fabs(delta_alpha2)) {
				variance_aa_ += delta_alpha1 * delta_alpha1;
				covariance_ya_ += delta_y * delta_alpha1;
			} else {
				variance_aa_ += delta_alpha2 * delta_alpha2;
				covariance_ya_ += delta_y * delta_alpha2;
			}
		}
		variance_xx_ /= (corner_ - 10);
		variance_yy_ /= (ranges_boundary_.size() - corner_ - 11);
		variance_aa_ /= (ranges_boundary_.size() - 20);
		covariance_xa_ /= (corner_ - 10);
		covariance_ya_ /= (ranges_boundary_.size() - corner_ - 11);
	} else if (type_ == RIGHT){
		for (int i = 0; i < corner_ - 9; i++) {
			double delta_y = -(initial_pose_(1) + ranges_boundary_[i] * sin(angles_boundary_[i] + initial_pose_(2)));
			variance_yy_ += delta_y * delta_y;
			
			double delta_alpha1;
			if (initial_pose_(1) > ranges_boundary_[i])
				delta_alpha1 = -M_PI/2;
			else if (initial_pose_(1) < -ranges_boundary_[i])
				delta_alpha1 = M_PI/2;
			else 
				delta_alpha1 = asin(-initial_pose_(1) / ranges_boundary_[i]);
			double delta_alpha2 = M_PI - delta_alpha1;			
			delta_alpha1 -= angles_boundary_[i] + initial_pose_(2);
			delta_alpha2 -= angles_boundary_[i] + initial_pose_(2);
			
			angle_standardize(delta_alpha1);
			angle_standardize(delta_alpha2);
			if (fabs(delta_alpha1) < fabs(delta_alpha2)) {
				variance_aa_ += delta_alpha1 * delta_alpha1;
				covariance_ya_ += delta_y * delta_alpha1;
			} else {
				variance_aa_ += delta_alpha2 * delta_alpha2;
				covariance_ya_ += delta_y * delta_alpha2;
			}
		}
		for (int i = corner_ + 10; i < ranges_boundary_.size(); i++) {
			double delta_x = 8 - initial_pose_(0) - ranges_boundary_[i] * cos(angles_boundary_[i] + initial_pose_(2));
			variance_xx_ += delta_x * delta_x;
			
			double delta_alpha1;
			if ((8 - initial_pose_(0)) > ranges_boundary_[i])
				delta_alpha1 = 0;
			else if ((8 - initial_pose_(0)) < -ranges_boundary_[i])
				delta_alpha1 = M_PI;
			else
				delta_alpha1 = acos((8 - initial_pose_(0)) / ranges_boundary_[i]);
			double delta_alpha2 = -delta_alpha1;
			delta_alpha1 -= angles_boundary_[i] + initial_pose_(2);
			delta_alpha2 -= angles_boundary_[i] + initial_pose_(2);
			
			angle_standardize(delta_alpha1);
			angle_standardize(delta_alpha2);
			if (fabs(delta_alpha1) < fabs(delta_alpha2)) {
				variance_aa_ += delta_alpha1 * delta_alpha1;
				covariance_xa_ += delta_x * delta_alpha1;
			} else {
				variance_aa_ += delta_alpha2 * delta_alpha2;
				covariance_xa_ += delta_x * delta_alpha2;
			}
		}
		variance_xx_ /= (ranges_boundary_.size() - corner_ - 11);
		variance_yy_ /= (corner_ - 10);
		variance_aa_ /= (ranges_boundary_.size() - 20);
		covariance_xa_ /= (ranges_boundary_.size() - corner_ - 11);
		covariance_ya_ /= (corner_ - 10);
	} else {
		std::cout << "You should set the side of robot!" << std::endl;
		std::cout << "Fatal error! The program is shutting down ..." << std::endl;
		exit(0);
	}
}

void initialpose_estimation::angle_standardize(double& angle) {
	while (angle > M_PI)
		angle -= 2 * M_PI;
	while (angle < - M_PI)
		angle += 2 * M_PI;
}

double initialpose_estimation::smoothness_square(int number) {
	double delta_x = 0, delta_y = 0;
	
	for (int i = -9 ; i < 10; i++) {
			if (i == 0)
				continue;
			double delta_angle = angles_boundary_[number + i] - angles_boundary_[number];
			delta_x += ranges_boundary_[number] - ranges_boundary_[number + i] * cos(delta_angle);
			delta_y += ranges_boundary_[number + i] * sin(delta_angle);
	}
	return (delta_x * delta_x + delta_y * delta_y) / (ranges_boundary_[number] * ranges_boundary_[number]);
}