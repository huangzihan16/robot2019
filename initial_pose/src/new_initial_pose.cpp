/**********************
规定 type:
type 0: left bottom
type 1: right bottom
type 2: left top
type 3: right top
**********************/

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <vector>
#include <string.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include </usr/local/include/g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <cmath>
#include <chrono>

#define start_region_boundary_distance 2.1

std::vector<int> laser_blacklist;

double angle_max;
double angle_min;
double angle_increment;

int corner_type = 0;
//ranges_boundary和angles_boundary是按逆时针顺序排列的边缘点信息
std::vector<double> ranges_boundary; 
std::vector<double> angles_boundary;

int corner = 0;

//g2o图优化顶点：位姿
class BoundaryPoseVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 重置
    virtual void setToOriginImpl() 
    {
        _estimate << 0,0,0;
    }
     // 更新
    virtual void oplusImpl( const double* update )
    {
        _estimate += Eigen::Vector3d(update);
    }
    // 存盘和读盘：留空
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}
};
// 误差模型 模板参数：观测值维度，类型，连接顶点类型
//_measurement是角度和距离的二维向量
//场地下边的点所构成的图优化边
class BoundaryEdge_Bottom: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, BoundaryPoseVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BoundaryEdge_Bottom(double point_theta, double point_range): 
    BaseUnaryEdge(), _point_theta(point_theta), _point_range(point_range) {}
    // 计算曲线模型误差
    void computeError()
    {
        const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
        const Eigen::Vector3d pose_estimation = pose->estimate();
        _error(0,0) = pose_estimation(1) + _measurement(1) * sin( _measurement(0) + pose_estimation(2) ) ;
    }
    virtual void linearizeOplus()
    {
        const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
        const Eigen::Vector3d pose_estimation = pose->estimate();

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = 1;
        _jacobianOplusXi(0,2) = _point_range * cos(_point_theta + pose_estimation(2));
    }
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}  
    
public:
	  double _point_range;
		double _point_theta;
		//measurement(0)其实和point_theta是一样的，measurement(1)其实和point_range是一样的
};
//场地左边的点所构成的图优化边
class BoundaryEdge_Left: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, BoundaryPoseVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BoundaryEdge_Left(double point_theta, double point_range): 
    BaseUnaryEdge(), _point_theta(point_theta), _point_range(point_range) {}
    // 计算曲线模型误差
    void computeError()
    {
        const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
        const Eigen::Vector3d pose_estimation = pose->estimate();
        _error(0,0) = pose_estimation(0) + _measurement(1) * cos( _measurement(0) + pose_estimation(2) ) ;
    }
    virtual void linearizeOplus()
    {
        const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
        const Eigen::Vector3d pose_estimation = pose->estimate();

        _jacobianOplusXi(0,0) = 1;
        _jacobianOplusXi(0,1) = 0;
        _jacobianOplusXi(0,2) = -_point_range * sin(_point_theta + pose_estimation(2));
    }
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}  
    
public:
	  double _point_range;
		double _point_theta;
		//measurement(0)其实和point_theta是一样的，measurement(1)其实和point_range是一样的
};
class BoundaryEdge_Top: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, BoundaryPoseVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BoundaryEdge_Top(double point_theta, double point_range): 
    BaseUnaryEdge(), _point_theta(point_theta), _point_range(point_range) {}
    // 计算曲线模型误差
    void computeError()
    {
        const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
        const Eigen::Vector3d pose_estimation = pose->estimate();
        _error(0,0) = 5.0 - pose_estimation(1) - _measurement(1) * sin( _measurement(0) + pose_estimation(2) ) ;
    }
    virtual void linearizeOplus()
    {
        const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
        const Eigen::Vector3d pose_estimation = pose->estimate();

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -1;
        _jacobianOplusXi(0,2) = -_point_range * cos(_point_theta + pose_estimation(2));
    }
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}  
    
public:
	  double _point_range;
		double _point_theta;
		//measurement(0)其实和point_theta是一样的，measurement(1)其实和point_range是一样的
};
class BoundaryEdge_Right: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, BoundaryPoseVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BoundaryEdge_Right(double point_theta, double point_range): 
    BaseUnaryEdge(), _point_theta(point_theta), _point_range(point_range) {}
    // 计算曲线模型误差
    void computeError()
    {
        const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
        const Eigen::Vector3d pose_estimation = pose->estimate();
        _error(0,0) = 8.0 - pose_estimation(0) - _measurement(1) * cos( _measurement(0) + pose_estimation(2) ) ;
    }
    virtual void linearizeOplus()
    {
        const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
        const Eigen::Vector3d pose_estimation = pose->estimate();

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -1;
        _jacobianOplusXi(0,2) = _point_range * sin(_point_theta + pose_estimation(2));
    }
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}  
    
public:
	  double _point_range;
		double _point_theta;
		//measurement(0)其实和point_theta是一样的，measurement(1)其实和point_range是一样的
};

//计算提取出来的边界点的曲率的平方的25倍，由于是为了比较大小，不在开方以及乘以倍数，注意不能提取最靠两边的四个点，否则会出现错误
double curve_smoothness_square(int number) {
	double delta_x = 0, delta_y = 0;
	
	for (int i = -9 ; i < 10; i++) {
			if (i == 0)
				continue;
			double delta_angle = angles_boundary[number + i] - angles_boundary[number];
			delta_x += ranges_boundary[number] - ranges_boundary[number + i] * cos(delta_angle);
			delta_y += ranges_boundary[number + i] * sin(delta_angle);
	}
	double smoothness_square = (delta_x * delta_x + delta_y * delta_y) / (ranges_boundary[number] * ranges_boundary[number]);
	return smoothness_square;
}
//将角度限制在-π到π之间
void angle_standard(double& angle) {
	while (angle > M_PI)
		angle -= 2 * M_PI;
	while (angle < - M_PI)
		angle += 2 * M_PI;
}

//提取边缘点信息到ranges_boundary和angles_boundary
void extract_boundary_information(const std::vector<double> &ranges_raw, const std::vector<int> &increment_raw) {
	int available_number = ranges_raw.size();
	std::vector<bool> distance(available_number, false);
	
	//找到第一个不符合距离要求的点
	int first_negative = -1;
	for (int i = 0; i < available_number; i++) {
		if (ranges_raw[i] < start_region_boundary_distance)
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
	
	ranges_boundary.clear();
	angles_boundary.clear();
	
	
	if (continuous_max_first_location < continuous_max_last_location) {
		for (int i = continuous_max_first_location; i < continuous_max_last_location + 1; i++) {
			ranges_boundary.push_back(ranges_raw[i]);
			angles_boundary.push_back(angle_min + angle_increment * increment_raw[i]);
		}
	} else {
		for (int i = continuous_max_first_location; i < available_number; i++) {
			ranges_boundary.push_back(ranges_raw[i]);
			angles_boundary.push_back(angle_min + angle_increment * increment_raw[i]);
		}
		for (int i = 0; i <= continuous_max_last_location; i++) {
			ranges_boundary.push_back(ranges_raw[i]);
			angles_boundary.push_back(angle_min + angle_increment * increment_raw[i]);
		}
	}
}

//根据距离算坐标，根据曲率算角点，corner_number是曲率最大点
void find_roughpose_and_corner(Eigen::Vector3d &pose, int &corner_number, const int type) {

	double smoothness_max = 0;
	corner_number = 0;
	for (int i = 9; i < ranges_boundary.size() - 9; i++) {
		double smoothness = curve_smoothness_square(i);
		if (smoothness > smoothness_max) {
			smoothness_max = smoothness;
			corner_number = i;
		}
	}
	std::cout << "Corner: " << corner_number << "\tSmoothness_max1: " << smoothness_max << std::endl;
	
	double min_range1 = 100, min_range2 = 100;
	
	for (int i = 0; i < corner_number - 9; i++) {
		if (ranges_boundary[i] < min_range1)
			min_range1 = ranges_boundary[i];
	}
	for (int i = corner_number + 9; i < ranges_boundary.size(); i++) {
		if (ranges_boundary[i] < min_range2) {
			min_range2 = ranges_boundary[i];
		}
	}
	
	//距离边缘最近的距离分别是lidar位置x,y，alpha是x=0线到lidar的angle_min的基准线的角度 theta是中间变量
	double x, y, alpha, theta;
	
	if (min_range1 > ranges_boundary[corner_number] && min_range2 > ranges_boundary[corner_number]) {
		std::cout << "Ranges of all boundary points are bigger than range of corner points!" << std::endl
							<< "Fatal error happened! Program is shuting down..." << std::endl;
		
		exit(0);
	}
	
	switch(type) {
		case 0:
			alpha = angles_boundary[corner_number];
			x = min_range1;
			y = min_range2;
			if (min_range1 < min_range2)
				theta = acos(min_range1 / ranges_boundary[corner_number]);
			else
				theta = asin(min_range2 / ranges_boundary[corner_number]);
			alpha = M_PI - alpha + theta;
			break;
		case 1:
			alpha = angles_boundary[corner_number];
			x = 8 - min_range2;
			y = min_range1;
			if (min_range1 > min_range2)
				theta = acos(min_range2 / ranges_boundary[corner_number]);
			else
				theta = asin(min_range1 / ranges_boundary[corner_number]);
			alpha = - alpha - theta;
			break;
		case 2:
			alpha = angles_boundary[corner_number];
			x = min_range2;
			y = 5 - min_range1;
			if (min_range1 > min_range2)
				theta = acos(min_range2 / ranges_boundary[corner_number]);
			else
				theta = asin(min_range1 / ranges_boundary[corner_number]);
			alpha = M_PI - theta - alpha;
			break;
		case 3:
			alpha = angles_boundary[corner_number];
			x = 8 - min_range1;
			y = 5 - min_range2;
			if (min_range1 < min_range2)
				theta = acos(min_range1 / ranges_boundary[corner_number]);
			else
				theta = asin(min_range2 / ranges_boundary[corner_number]);
			alpha = -alpha + theta;
			break;
	}
	
	angle_standard(alpha);
	pose = Eigen::Vector3d(x, y, alpha);
}

Eigen::Vector3d find_pose_precise(const Eigen::Vector3d &initial_pose, const int& corner, const int type) {
	Eigen::Vector3d pose;
	// pose 维度为 3, landmark 维度为 1
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<3, 1> > Block;
	// 线性方程求解器
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
	// 矩阵块求解器
	Block* solver_ptr = new Block ( linearSolver );
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm ( solver );

  // vertex
	BoundaryPoseVertex* pose_vertex = new BoundaryPoseVertex(); // camera pose
	pose_vertex->setEstimate(initial_pose);
	pose_vertex->setId(0);
	optimizer.addVertex(pose_vertex);
	
	int index = 1;
		
	Eigen::Matrix2d Information_Matrix = Eigen::Matrix2d::Identity() * 1e-4;
	
	switch(type) {
		case 0:
			for (int i = 0; i < corner - 9; i++) {
				Eigen::Vector2d point(angles_boundary[i], ranges_boundary[i]);
				BoundaryEdge_Left* edge = new BoundaryEdge_Left(angles_boundary[i], ranges_boundary[i]);
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
			for (int i = corner + 9; i < ranges_boundary.size(); i++) {
					Eigen::Vector2d point(angles_boundary[i], ranges_boundary[i]);
					BoundaryEdge_Bottom* edge = new BoundaryEdge_Bottom(angles_boundary[i], ranges_boundary[i]);
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
			break;
		case 1:
			for (int i = 0; i < corner - 9; i++) {
				Eigen::Vector2d point(angles_boundary[i], ranges_boundary[i]);
				BoundaryEdge_Bottom* edge = new BoundaryEdge_Bottom(angles_boundary[i], ranges_boundary[i]);
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
			for (int i = corner + 9; i < ranges_boundary.size(); i++) {
				Eigen::Vector2d point(angles_boundary[i], ranges_boundary[i]);
				BoundaryEdge_Right* edge = new BoundaryEdge_Right(angles_boundary[i], ranges_boundary[i]);
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
			break;
		case 2:
			for (int i = 0; i < corner - 9; i++) {
				Eigen::Vector2d point(angles_boundary[i], ranges_boundary[i]);
				BoundaryEdge_Top* edge = new BoundaryEdge_Top(angles_boundary[i], ranges_boundary[i]);
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
			for (int i = corner + 9; i < ranges_boundary.size(); i++) {
				Eigen::Vector2d point(angles_boundary[i], ranges_boundary[i]);
				BoundaryEdge_Left* edge = new BoundaryEdge_Left(angles_boundary[i], ranges_boundary[i]);
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
			break;
		case 3:
			for (int i = 0; i < corner - 9; i++) {
				Eigen::Vector2d point(angles_boundary[i], ranges_boundary[i]);
				BoundaryEdge_Right* edge = new BoundaryEdge_Right(angles_boundary[i], ranges_boundary[i]);
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
			for (int i = corner + 9; i < ranges_boundary.size(); i++) {
				Eigen::Vector2d point(angles_boundary[i], ranges_boundary[i]);
				BoundaryEdge_Top* edge = new BoundaryEdge_Top(angles_boundary[i], ranges_boundary[i]);
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
			break;
	}
	std::cout << "start optimization" << std::endl;
	optimizer.initializeOptimization();
	optimizer.optimize(100);
	
	return pose_vertex->estimate();
}



void scanCallback(const sensor_msgs::LaserScanConstPtr msg) {
	angle_max = msg->angle_max;
	angle_min = msg->angle_min;
	angle_increment = msg->angle_increment;
	int angle_number = (int)( (angle_max - angle_min)/angle_increment ) + 1;
	
	std::vector<double> ranges_raw;
	std::vector<int> increment_raw;

	//去除遮挡点，保留有效雷达点信息，ranges_raw是range信息，increment是有效点顺序信息（用于后续生成角度信息）
	for (int i = 0; i < angle_number; i++) {
		if (msg->ranges[i] != std::numeric_limits<float>::infinity()) {
			ranges_raw.push_back(msg->ranges[i]);
			increment_raw.push_back(i);
		}
	}

	extract_boundary_information(ranges_raw, increment_raw);
	//std::cout << "boundary_size: " << ranges_boundary.size() << std::endl;
	Eigen::Vector3d rough_pose;
	
	find_roughpose_and_corner(rough_pose, corner, corner_type);
	
	std::cout << "rough_pose: " << rough_pose.transpose() << std::endl;
	
	Eigen::Vector3d pose;
	pose = find_pose_precise(rough_pose, corner, corner_type);
	angle_standard(pose(2));
	
	std::cout << "precise_pose: " << pose.transpose() << std::endl << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initial_pose");
  ros::NodeHandle n;
	
  ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
	sensor_msgs::LaserScan scan;
	
	ros::Publisher boundary_pub = n.advertise<sensor_msgs::PointCloud>("boundary", 50);
	ros::Publisher corner_pub = n.advertise<sensor_msgs::PointCloud>("corner", 50);
	
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		unsigned int num_points = ranges_boundary.size();
		sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "laser";

    cloud.points.resize(num_points);

    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);

    //generate some fake data for our point cloud
    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = ranges_boundary[i] * cos(angles_boundary[i]);
      cloud.points[i].y = ranges_boundary[i] * sin(angles_boundary[i]);
      cloud.points[i].z = 0;
      cloud.channels[0].values[i] = 100;
    }

    sensor_msgs::PointCloud corner_cloud;
		corner_cloud.header.stamp = ros::Time::now();
		corner_cloud.header.frame_id = "laser";
		corner_cloud.points.resize(2);
		corner_cloud.channels.resize(1);
		corner_cloud.channels[0].name = "intensities";
		corner_cloud.channels[0].values.resize(2);
		if (corner != 0) {
			corner_cloud.points[0].x = (ranges_boundary[corner] + 1) * cos(angles_boundary[corner]);
			corner_cloud.points[0].y = (ranges_boundary[corner] + 1) * sin(angles_boundary[corner]);
			corner_cloud.points[0].z = 0;
			corner_cloud.channels[0].values[0] = 200;
		}
		corner_cloud.points[1].x = 1;
		corner_cloud.points[1].y = 0;
		corner_cloud.points[1].z = 0;
		corner_cloud.channels[0].values[1] = 10;
    boundary_pub.publish(cloud);
		corner_pub.publish(corner_cloud);
		ros::spinOnce();
		loop_rate.sleep();
	}
  return 0;
}