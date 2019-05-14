#include "apriltag_detector.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

//#include <apriltags_ros/AprilTagDetection.h>
//#include <apriltags_ros/AprilTagDetectionArray.h>

#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include<math.h>

#include "proto/armor_detection.pb.h"
#include "constraint_set/proto/constraint_set.pb.h"

#include <ros/package.h>
#include "io/io.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


#define PI (3.1415926535897932346f)  

namespace apriltags_ros{

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh): it_(nh){
  //XmlRpc::XmlRpcValue april_tag_descriptions;
  //descriptions_ = parse_tag_descriptions(april_tag_descriptions);
  // if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
  //   ROS_WARN("No april tags specified");
  // }
  // else{
  //   try{
  //     descriptions_ = parse_tag_descriptions(april_tag_descriptions);
  //   } catch(XmlRpc::XmlRpcException e){
  //     ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
  //   }
  // }

  // if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)){
  //   sensor_frame_id_ = "";
  // }

  // std::string tag_family;
  // pnh.param<std::string>("tag_family", tag_family, "36h11");

  // pnh.param<bool>("projected_optics", projected_optics_, false);

  // const AprilTags::TagCodes* tag_codes;
  // if(tag_family == "16h5"){
  //   tag_codes = &AprilTags::tagCodes16h5;
  // }
  // else if(tag_family == "25h7"){
  //   tag_codes = &AprilTags::tagCodes25h7;
  // }
  // else if(tag_family == "25h9"){
  //   tag_codes = &AprilTags::tagCodes25h9;
  // }
  // else if(tag_family == "36h9"){
  //   tag_codes = &AprilTags::tagCodes36h9;
  // }
  // else if(tag_family == "36h11"){
  //   tag_codes = &AprilTags::tagCodes36h11;
  // }
  // else{
  //   ROS_WARN("Invalid tag family specified; defaulting to 36h11");
  //   tag_codes = &AprilTags::tagCodes36h11;
  // }

   //-----------------去除pnh.getParam
   sensor_frame_id_ = "";
   std::string tag_family = "36h11";
   bool projected_optics_ = false;
   const AprilTags::TagCodes* tag_codes;
   tag_codes = &AprilTags::tagCodes36h11;

  std::string frame_name0 ="tag_0";
  std::string frame_name1 ="tag_1";
  AprilTagDescription description0(0, 0.068, frame_name0);
  AprilTagDescription description1(1, 0.068, frame_name1);
  descriptions_.insert(std::make_pair(0, description0));
  descriptions_.insert(std::make_pair(1, description1));

//-----------------读取相机偏移信息
  // roborts_detection::ArmorDetectionAlgorithms offset_param;
  // std::string file_name = ros::package::getPath("back_camera") + "/backcamera/config/armor_detection.prototxt";
  // bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &offset_param);
  // delta_x = offset_param.supply_offset().delta_x();
  // delta_y = offset_param.supply_offset().delta_y();
  // delta_theta = offset_param.supply_offset().delta_theta();

//----------------读取敌方机器人颜色
  roborts_detection::ConstraintSetConfig constraint_set_config_;
  std::string file_name = ros::package::getPath("back_camera") + \
      "/backcamera/constraint_set/config/constraint_set.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &constraint_set_config_);
  ROS_ASSERT_MSG(read_state, "Cannot open %s", file_name.c_str());
  enemy_color_ = constraint_set_config_.enemy_color();

  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
  image_sub_ = it_.subscribeCamera("/back_camera/image_raw", 1, &AprilTagDetector::imageCb, this);   
  image_pub_ = it_.advertise("tag_detections_image", 1);
  initialpose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 50);
}

AprilTagDetector::~AprilTagDetector(){
  image_sub_.shutdown();
}

void AprilTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info){
 
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

  std::vector<AprilTags::TagDetection>	detections = tag_detector_->extractTags(gray);
  ROS_DEBUG("%d tag detected", (int)detections.size());

  tag_detect_amount_ = (int)detections.size();
  if (tag_detect_amount_ == 0){
    return;
  }
 // std::cout << "tag_detect_amount = " << tag_detect_amount_ << std::endl;

/*相机数据读不进来，手动初始化相机内参，还需修改TagDetection.cc文件的畸变参数distParam*/ 
   double fx = cam_info->K[0];
   double fy = cam_info->K[4];
   double px = cam_info->K[2];
   double py = cam_info->K[5];
   double k1 = cam_info->D[0];
   double k2 = cam_info->D[1];
   double k3 = cam_info->D[3];
   double k4 = cam_info->D[4];
  //std::cout << "fx=" << fx << std::endl;
 
  if(!sensor_frame_id_.empty())
  cv_ptr->header.frame_id = sensor_frame_id_;
  //AprilTagDetectionArray tag_detection_array;      //apriltags标签坐标
  geometry_msgs::PoseArray tag_pose_array;
  tag_pose_array.header = cv_ptr->header; 
  geometry_msgs::PoseWithCovarianceStamped initialpose_with_covariance;

  BOOST_FOREACH(AprilTags::TagDetection detection, detections)
  {
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end()){
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }

    detection.draw(cv_ptr->image);

//---------------------判断是否停错补给区
    tag_id = detection.id;
    if(tag_id == enemy_color_){
      return;
    }

    std::cout << "error supply station，tag_id = " << detection.id << std::endl;   
    tf::StampedTransform transform;
    try{
    listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
    } catch (tf::TransformException &ex) { 
    ROS_ERROR("%s",ex.what()); 
    return; 
    } 

    double pitch, roll, amcl_yaw, amcl_x, amcl_y, z;
    transform.getBasis().getEulerYPR(amcl_yaw,pitch,roll);
    amcl_x = transform.getOrigin().x();
    amcl_y = transform.getOrigin().y();
    std::cout << "tf form /base_link to /map is : "  << std::endl
              << "x = : " << amcl_x << std::endl
              << "y = : " << amcl_y<< std::endl
              << "z = : " << amcl_yaw << std::endl;
  
    Eigen::Vector3d robot_pose;  //  x,y,yaw
    robot_pose(0) = 8 - amcl_y;
    robot_pose(1) = 5 + amcl_x;
    robot_pose(2) = amcl_yaw + PI;
    

/*-----------------------计算pnp,并计算机器人位姿态 
    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py,k1,k2,k3,k4);  //计算标签坐标
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);           //旋转矩阵R
    Eigen::Vector3d p_oc;
    Eigen::Vector3d T = transform.col(3).head(3);
    Eigen::Matrix3d rot_inverse = rot.inverse();
    
    //场地长边为x,短边为y
    //p_oc(0)为x,p_oc(2)为y

    p_oc = -rot_inverse*T;     //二维码坐标系y朝下
    //std::cout << "camera center = " << p_oc << std::endl;

    // double theta_x = atan2(rot_inverse(2, 1), rot_inverse(2, 2));    
    // double theta_y = atan2(-rot_inverse(2, 0),
    // sqrt(rot_inverse(2, 1)*rot_inverse(2, 1) + rot_inverse(2, 2)*rot_inverse(2, 2)));
    // double theta_z = atan2(rot_inverse(1, 0), rot_inverse(0, 0));

    // std::cout << "theta_x = " << theta_x/PI*180 << std::endl
    //           << "theta_y = " <<补激a_y/PI*180 << std::endl
    //           << "theta_z = " << theta_z/PI*180 << std::endl;

    float l = 0.15;  //相机中心到机器人中心距离
    Eigen::Vector3d robot_pose;
    // double alfa = theta_y;           //从相机旋转轴转到机器人中心
    // double beta = atan2(p_oc(0),p_oc(2));
    // robot_pose(0) = 4 + p_oc(0) + l*sin(alfa+beta);
    // robot_pose(1) = 5 - p_oc(2) + l*cos(alfa+beta);
    // robot_pose(2) = -PI/2 + alfa + beta; 

    std::cout << "tf form /base_link to /map is : " << amcl_yaw << std::endl;

    double theta = amcl_yaw - PI;    //yaw角从amcl读取
    robot_pose(0) = 4 + p_oc(0) + l*sin(theta);
    robot_pose(1) = 5 - p_oc(2) - l*cos(theta);    
    robot_pose(2) = amcl_yaw;
          
     
     //-----------------发布initial pose
  
    robot_pose(0) = robot_pose(0) + delta_x;
    robot_pose(1) = robot_pose(1) + delta_y;
    robot_pose(2) = robot_pose(2) + delta_theta;
    
    // std::cout << "delta_x = "  << delta_x << std::endl
    //           << "delta_y = "  << delta_y << std::endl;             
*/

     std::cout << "robot_pose = " << std::endl << robot_pose << std::endl;

		 initialpose_with_covariance.header.stamp = msg->header.stamp;
		 initialpose_with_covariance.header.frame_id = "map";
     initialpose_with_covariance.pose.pose.position.x = robot_pose(0) ;
     initialpose_with_covariance.pose.pose.position.y = robot_pose(1) ;
     initialpose_with_covariance.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_pose(2));     
     boost::array<double, 36> covariance = {
			0.0001, 0, 0, 0, 0, 0,
			0, 0.0001, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0.0001
		};
		initialpose_with_covariance.pose.covariance = covariance;
    initialpose_pub_.publish(initialpose_with_covariance);
  }

  //detections_pub_.publish(tag_detection_array);
  //pose_pub_.publish(tag_pose_array);                //发布tag_pose_array
  image_pub_.publish(cv_ptr->toImageMsg());
}


std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);  
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}



}
