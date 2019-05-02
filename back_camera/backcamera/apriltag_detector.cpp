#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include<math.h>

#define PI (3.1415926535897932346f)  

namespace apriltags_ros{

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh): it_(nh){
  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
    ROS_WARN("No april tags specified");
  }
  else{
    try{
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
    }
  }

  if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)){
    sensor_frame_id_ = "";
  }

  std::string tag_family;
  pnh.param<std::string>("tag_family", tag_family, "36h11");

  pnh.param<bool>("projected_optics", projected_optics_, false);

  const AprilTags::TagCodes* tag_codes;
  if(tag_family == "16h5"){
    tag_codes = &AprilTags::tagCodes16h5;
  }
  else if(tag_family == "25h7"){
    tag_codes = &AprilTags::tagCodes25h7;
  }
  else if(tag_family == "25h9"){
    tag_codes = &AprilTags::tagCodes25h9;
  }
  else if(tag_family == "36h9"){
    tag_codes = &AprilTags::tagCodes36h9;
  }
  else if(tag_family == "36h11"){
    tag_codes = &AprilTags::tagCodes36h11;
  }
  else{
    ROS_WARN("Invalid tag family specified; defaulting to 36h11");
    tag_codes = &AprilTags::tagCodes36h11;
  }

    pnh.param<double>("delta_x",delta_x,10);
    pnh.getParam("delta_y",delta_y);
    pnh.getParam("delta_z",delta_theta);
    ROS_INFO_STREAM("delta_x:"<< delta_y << std::endl);

  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
  image_sub_ = it_.subscribeCamera("image_rect", 1, &AprilTagDetector::imageCb, this);   //这句话并没有将相机信息读进来
  image_pub_ = it_.advertise("tag_detections_image", 1);
  //ros::Publisher initialpose_pub_;
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

  /*相机数据读不进来，手动初始化相机内参，还需修改TagDetection.cc文件的畸变参数distParam*/
  
  double fx = 1855.9;
  double fy = 1855.4;
  double px = 1440.4;
  double py = 832.3245;
  
  /*
    fx = cam_info->K[0];
    fy = cam_info->K[4];
    px = cam_info->K[2];
    py = cam_info->K[5];
  */
 
  if(!sensor_frame_id_.empty())
    cv_ptr->header.frame_id = sensor_frame_id_;

  AprilTagDetectionArray tag_detection_array;      //apriltags标签坐标
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
    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

    detection.draw(cv_ptr->image);
   //ROS_INFO_STREAM("tag_size "<< tag_size <<"fx "<< fx <<"fy"<< fy);     //输出相机内参矩阵
    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);  //计算标签坐标
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);           //旋转矩阵R
    Eigen::Vector3d p_oc;
    Eigen::Vector3d T = transform.col(3).head(3);
    p_oc = -rot.inverse()*T;     //二维码坐标系x朝右，y朝外，z朝下

    Eigen::Matrix3d rot_inverse = rot.inverse();
    double theta_x = atan2(rot_inverse(2, 1), rot_inverse(2, 2));    
    double theta_y = atan2(-rot_inverse(2, 0),
    sqrt(rot_inverse(2, 1)*rot_inverse(2, 1) + rot_inverse(2, 2)*rot_inverse(2, 2)));
    double theta_z = atan2(rot_inverse(1, 0), rot_inverse(0, 0));
                
    Eigen::Vector3d robot_pose;  //  x,y,yaw
    robot_pose(0) = 4 - p_oc(0);
    robot_pose(1) = 5 - p_oc(1);
    robot_pose(2) = -PI/2 - theta_z;  
               
    /*计算camera_frame到map的tf*/
   
     
     /****发布initial****/
  
    robot_pose(0) = robot_pose(0) + AprilTagDetector::delta_x;
    robot_pose(1) = robot_pose(1) + AprilTagDetector::delta_y;
    robot_pose(2) = robot_pose(2) + AprilTagDetector::delta_theta;

    ROS_INFO_STREAM("delta_x:"<< AprilTagDetector::delta_x << std::endl);

		 initialpose_with_covariance.header.stamp = msg->header.stamp;
		 initialpose_with_covariance.header.frame_id = "map";
     initialpose_with_covariance.pose.pose.position.x = robot_pose(0) + delta_x;
     initialpose_with_covariance.pose.pose.position.y = robot_pose(1) + delta_y;
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
