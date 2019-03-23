/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <roborts_msgs/GimbalAngle.h>

#include "roborts_msgs/ArmorDetectionAction.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"
#include "std_msgs/Int16.h"

#include "roborts_msgs/PartnerInformation.h"

namespace roborts_decision{

enum class Identity {
	MASTER = 0,
	SERVANT,
	SAME
};

enum class PartnerStatus {
	SUPPLY = 0,
	GAINBUFF,
	BATTLE,
	GUARD,
	PATROL
};

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
			remain_hp_(2000), shooter_heat_(0), shoot_vel_(25),
			supply_number_(0),
			gain_buff_number_(0),
      partner_detect_enemy_(false) {

    start_time_ = ros::Time::now();

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    ros::NodeHandle nh;
		gimbal_angle_sub_= nh.subscribe<roborts_msgs::GimbalAngle>("cmd_gimbal_angle",100 , &Blackboard::GimbalCallback, this);
    
    //partner_bullet_num_sub_= nh.subscribe<std_msgs::Int16>("/partner/bullet_num", 1, &Blackboard::PartnerBulletNumCallback, this);
    //partner_remain_HP_sub_= nh.subscribe<std_msgs::Int16>("/partner/remain_HP", 1, &Blackboard::PartnerRemainHPCallback, this);
		
    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    if (!decision_config.simulate()){

      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    }
		bullet_num_ = decision_config.initial_bullet_num();

    std::string partner_name = decision_config.partner_name();

    std::string partner_topic_sub = "/" + partner_name + "/partner_msg";
		
		partner_sub_ = nh.subscribe<roborts_msgs::PartnerInformation>(partner_topic_sub, 1, &Blackboard::PartnerCallback, this);
		partner_pub_ = nh.advertise<roborts_msgs::PartnerInformation>("partner_msg", 1);
  }

  ~Blackboard() = default;

  // Enemy
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;
      //剔除无效值
      if(camera_pose_msg.pose.position.z < 1.0)
        return;

			enemy_pose_camera_ = camera_pose_msg;

      camera_pose_msg.header.frame_id = "base_link";
      double yaw = GetGimbalYaw();
      double pos_x = camera_pose_msg.pose.position.z/1000;
      double pos_y = -camera_pose_msg.pose.position.x/1000;
      camera_pose_msg.pose.position.x = pos_x * cos(yaw) - pos_y * sin(yaw);
      camera_pose_msg.pose.position.y = pos_x * sin(yaw) + pos_y * cos(yaw);
			camera_pose_msg.pose.position.z = 0;

      // double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
      //     camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      // double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);
      //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
      yaw = 0;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();

      // std::cout <<"camera_pose_msg:" << camera_pose_msg << std::endl;

      poseStampedMsgToTF(camera_pose_msg, tf_pose);
			
      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        //tf_ptr_->transformPose("map", camera_pose_msg, global_pose_msg);
        if(GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
          enemy_pose_ = global_pose_msg;
          
        }
        // std::cout <<"enemy_pose:" << enemy_pose_ << std::endl;
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    } else{
      enemy_detected_ = false;
      ROS_INFO(" nnnnnnnnnnnnnnnnFind Enemy!");

    }
    PublishPartnerInformation();
  }


  bool IsMasterCondition(){
       return false;
  }

  bool IsMasterSupplyCondition(){
      if(GetPartnerStatus()==PartnerStatus::SUPPLY)
         return true;
      else
         return false;

  }
  bool IsMasterGainBuffCondition(){
     if(GetPartnerStatus()==PartnerStatus::GAINBUFF)
         return true;
      else
         return false;
  }

  geometry_msgs::PoseStamped GetEnemy() const {
    return enemy_pose_;
  }
  
  geometry_msgs::PoseStamped GetEnemyInCamera() const {
    return enemy_pose_camera_;
  }

  bool IsEnemyDetected() const{
    
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;

  }

  bool IsBulletLeft() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)bullet_num_);
    //return true;
    if(bullet_num_ > 5){
      return true;
    } else{
      return false;
    }
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }

  void PartnerBulletNumCallback(const std_msgs::Int16::ConstPtr& partner_bullet_num){
    partner_bullet_num_ = *partner_bullet_num;
  }

  void PartnerRemainHPCallback(const std_msgs::Int16::ConstPtr& partner_remain_HP){
    partner_remain_HP_ = *partner_remain_HP;
  }

  void GimbalCallback(const roborts_msgs::GimbalAngle gimbalangle){
    cmd_gimbal_angle_ = gimbalangle;
  }
  roborts_msgs::GimbalAngle GetGimbalAngle() const {
     return cmd_gimbal_angle_;
  }

  double GetGimbalYaw()
  {
    UpdateGimbalPose();
    tf::Quaternion q;
    tf::quaternionMsgToTF(gimbal_base_pose_.pose.orientation, q);
    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  //测试，给固定目标点
  geometry_msgs::PoseStamped GetFixedGoal() {

    geometry_msgs::PoseStamped fix_goal;
    ros::Time current_time = ros::Time::now();
    fix_goal.header.stamp = current_time;
    fix_goal.pose.position.x = 4;
    fix_goal.pose.position.y = 0.5;
    fix_goal.pose.position.z = 0.0;
    fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(90.0/180*3.14);

    return fix_goal;
  }
  geometry_msgs::PoseStamped GetSupplyGoal() {

    geometry_msgs::PoseStamped fix_goal;
    ros::Time current_time = ros::Time::now();
    fix_goal.header.stamp = current_time;
    fix_goal.pose.position.x = 4;
    fix_goal.pose.position.y = 4.5;
    fix_goal.pose.position.z = 0.0;
    fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(-90.0/180*3.14);

    return fix_goal;
  }
  geometry_msgs::PoseStamped GetGuardGoal() {

    geometry_msgs::PoseStamped fix_goal;
    ros::Time current_time = ros::Time::now();
    fix_goal.header.stamp = current_time;
    fix_goal.pose.position.x = 6.3;
    fix_goal.pose.position.y = 1.75;
    fix_goal.pose.position.z = 0.0;
    fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(90.0/180*3.14);

    return fix_goal;
  }
  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const geometry_msgs::PoseStamped GetGimbalBasePose() {
    UpdateGimbalPose();
    return gimbal_base_pose_;
  }


  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }
  
  const int GetBulletNum() {
		return bullet_num_;
	}
	const int GetRemainHP() {
		return remain_hp_;
	}
	const int GetShooterHeat() {
		return shooter_heat_;
	}
	const int GetShootVel() {
		return shoot_vel_;
	}
	
	bool IsSupplyCondition() {
		ros::Duration time_past = ros::Time::now() - start_time_;
		if (time_past.toSec() >= 60 * supply_number_)
			return true;
		else
			return false;
	}
	void AddSupplyNum() {
		supply_number_++;
		bullet_num_ += 50;
	}

  void MinusShootNum(roborts_msgs::ShootCmd shoot_cmd){
    bullet_num_ -= shoot_cmd.request.number;
    ROS_INFO("%d can't open file", bullet_num_);
  }
  
	void AddGainBuffNum() {
		gain_buff_number_++;
	}
	
	bool IsGainBuffCondition() {
		ros::Duration time_past = ros::Time::now() - start_time_;
		if (time_past.toSec() >= 60 * gain_buff_number_)
			return true;
		else
			return false;
	}
	bool ArriveGainBuff() {
		UpdateRobotPose();
		double delta_x = robot_map_pose_.pose.position.x - 6.3;
		double delta_y = robot_map_pose_.pose.position.y - 1.75;
		
		if (delta_x * delta_x + delta_y * delta_y < 0.01) 
			return true;
		else 
			return false;
	}

  void PartnerCallback(const roborts_msgs::PartnerInformationConstPtr& partner_info) {
		partner_status_ = (PartnerStatus)partner_info->status;
		partner_detect_enemy_ = partner_info->enemy_detected;
		partner_enemy_pose_ = partner_info->enemy_pose;
		partner_pose_ = partner_info->partner_pose;
		partner_patrol_count_ = partner_info->patrol_count;
	}

	bool IsPartnerDetectEnemy () {
		return partner_detect_enemy_;
	}

  geometry_msgs::PoseStamped GetPartnerEnemyPose(){ 
    float Yaw;
    UpdateRobotPose();
    Yaw= atan2(partner_enemy_pose_.pose.position.y - robot_map_pose_.pose.position.y , partner_enemy_pose_.pose.position.x - robot_map_pose_.pose.position.x);
    partner_enemy_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(Yaw);
    return partner_enemy_pose_;
  }

  PartnerStatus GetPartnerStatus(){
    return partner_status_;
  }


  void PublishPartnerInformation() {
    partner_msg_pub_.enemy_detected = enemy_detected_;
    partner_msg_pub_.enemy_pose = enemy_pose_;
    UpdateRobotPose();
    partner_msg_pub_.partner_pose = robot_map_pose_;
    partner_msg_pub_.header.stamp = ros::Time::now();
    partner_pub_.publish(partner_msg_pub_);
  }
 private:
  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }

  void UpdateGimbalPose() {
    tf::Stamped<tf::Pose> gimbal_tf_pose;
    gimbal_tf_pose.setIdentity();

    gimbal_tf_pose.frame_id_ = "gimbal";
    gimbal_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped gimbal_pose;
      tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
      tf_ptr_->transformPose("base_link", gimbal_pose, gimbal_base_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }

  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
	geometry_msgs::PoseStamped enemy_pose_camera_;
  geometry_msgs::PoseStamped enemy_pose_;
		
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;
  //云台相对底盘位姿  
  geometry_msgs::PoseStamped  gimbal_base_pose_;

  //! bullet info
  int bullet_num_;
  int remain_hp_;
  int bullet_freq_;
  int shooter_heat_;
  int shoot_vel_;
	
	ros::Subscriber gimbal_angle_sub_;
	roborts_msgs::GimbalAngle cmd_gimbal_angle_;
	//time and supply
	ros::Time start_time_;

    // partner information
  std_msgs::Int16 partner_bullet_num_;
  std_msgs::Int16 partner_remain_HP_;
  ros::Subscriber partner_bullet_num_sub_;
  ros::Subscriber partner_remain_HP_sub_;

  ros::Subscriber partner_sub_;
	ros::Publisher partner_pub_;
	
	Identity self_identity_;
	PartnerStatus partner_status_;
	
	bool partner_detect_enemy_;
	geometry_msgs::PoseStamped partner_enemy_pose_;
	geometry_msgs::PoseStamped partner_pose_;
	int partner_patrol_count_;

public:
	int supply_number_;	
	
	int gain_buff_number_;

  roborts_msgs::PartnerInformation partner_msg_pub_;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
