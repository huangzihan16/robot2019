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
/*************/
#include "roborts_msgs/BonusStatus.h"
#include "roborts_msgs/GameResult.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/GameSurvivor.h"
#include "roborts_msgs/RobotBonus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/ProjectileSupply.h"
/*******************/

namespace roborts_decision{

enum class Identity {
	MASTER = 0,
	SLAVE,
	SAME
};

enum class PartnerStatus {
	SUPPLY = 0,
	GAINBUFF,
	BATTLE,
	GUARD,
	PATROL
};
/**********************************/
enum class GameStatus{
  PRE_MATCH = 0,
  SETUP = 1,
  INIT = 2,
  FIVE_SEC_CD = 3,
  ROUND = 4,
  CALCULATION = 5
};

enum class DamageSource{
  FORWARD = 0,
  BACKWARD = 1,
  LEFT = 2,
  RIGHT = 3,
  NONE = 4
};
enum class DamageType{
  ARMOR = 0,
  OFFLINE = 1,
  EXCEED_HEAT = 2,
  EXCEED_POWER = 3,
};

enum class BonusStatus{
  UNOCCUPIED = 0,
  BEING_OCCUPIED = 1,
  OCCUPIED = 2
};

enum class SupplierStatus{
  CLOSE = 0,
  PREPARING = 1,
  SUPPLYING = 2
};

enum class EnemyStatus{
  NONE = 0,
  FRONT = 1,
  BACK = 2,
  BOTH = 3
};

enum class GameResult{
  DRAW = 0,
  RED_WIN = 1,
  BLUE_WIN = 2
};
/******************************/

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
      /***********************/
      game_status_(GameStatus::ROUND),
      red_bonus_status_(BonusStatus::OCCUPIED),
      blue_bonus_status_(BonusStatus::UNOCCUPIED),
      supplier_status_(SupplierStatus::PREPARING),
      remain_hp_(2000),
      shooter_heat_(0),
      speed_(25),
      last_hp_(2000),
      dmp_(0),
      back_enemy_detected_(false),
      enemy_disappear_count_(0),
      /*******************/
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
			self_identity_(Identity::MASTER),
      partner_detect_enemy_(false),
			supply_number_(0),
      identity_number_(1),
 			gain_buff_number_(0),
      search_count_(0) {

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

    /****************/
    last_get_hp_time_ = ros::Time::now();
    game_status_sub_ = nh.subscribe<roborts_msgs::GameStatus>("game_status",30 , &Blackboard::GameStatusCallback, this);
    game_result_sub_ = nh.subscribe<roborts_msgs::GameResult>("game_result",30 , &Blackboard::GameResultCallback, this);
    game_survival_sub_ = nh.subscribe<roborts_msgs::GameSurvivor>("game_survivor",30 , &Blackboard::GameSurvivorCallback, this);
    bonus_status_sub_ = nh.subscribe<roborts_msgs::BonusStatus>("field_bonus_status",30 , &Blackboard::BonusStatusCallback, this);
    supplier_status_sub_ = nh.subscribe<roborts_msgs::SupplierStatus>("field_supplier_status",30 , &Blackboard::SupplierStatusCallback, this);
    robot_status_sub_ = nh.subscribe<roborts_msgs::RobotStatus>("robot_status",30 , &Blackboard::RobotStatusCallback, this);
    robot_heat_sub_ = nh.subscribe<roborts_msgs::RobotHeat>("robot_heat",30 , &Blackboard::RobotHeatCallback, this);
    robot_bonus_sub_ = nh.subscribe<roborts_msgs::RobotBonus>("robot_bonus",30 , &Blackboard::RobotBonusCallback, this);
    robot_damage_sub_ = nh.subscribe<roborts_msgs::RobotDamage>("robot_damage",30 , &Blackboard::RobotDamageCallback, this);
    robot_shoot_sub_ = nh.subscribe<roborts_msgs::RobotShoot>("robot_shoot",30 , &Blackboard::RobotShootCallback, this);
    projectile_supply_pub_ = nh.advertise<roborts_msgs::ProjectileSupply>("projectile_supply", 1);

    /*******************/

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
      search_count_ = 5;

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;
      //剔除无效值
      if (camera_pose_msg.pose.position.z < 1.0)
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
        if (GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
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
    enemy_disappear_[enemy_disappear_count_++ % 50] = enemy_detected_;
    PublishPartnerInformation();
  }

/***********************************************/
  // Game Status
  void GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr& game_status){
    game_status_ = (GameStatus)game_status->game_status;
    remaining_time_ = game_status->remaining_time;
  }
  // Game Result
  void GameResultCallback(const roborts_msgs::GameResult::ConstPtr& game_result){
    game_result_ = (GameResult)game_result->result;
  } 
  //Game Survior
  void GameSurvivorCallback(const roborts_msgs::GameSurvivor::ConstPtr& game_survival){
    red3_ = game_survival->red3;
    red4_ = game_survival->red4;
    blue3_ = game_survival->blue3;
    blue3_ = game_survival->blue4;
  } 
  //Bonus Status
  void BonusStatusCallback(const roborts_msgs::BonusStatus::ConstPtr& bonus_status){
    red_bonus_status_ = (BonusStatus)bonus_status->red_bonus;
    blue_bonus_status_ = (BonusStatus)bonus_status->blue_bonus;
    
  } 
  //Supplier Status
  void SupplierStatusCallback(const roborts_msgs::SupplierStatus::ConstPtr& supplier_status){
    supplier_status_ = (SupplierStatus)supplier_status->status;
  } 
  //Robot Status
  void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr& robot_status){
    id_ = robot_status->id;
    level_ = robot_status->level;
    remain_hp_ = robot_status->remain_hp;
    max_hp_ = robot_status->max_hp;
    heat_cooling_limit_ = robot_status->heat_cooling_limit;
    heat_cooling_rate_ = robot_status->heat_cooling_rate;
    gimbal_output_ = robot_status->gimbal_output;
    chassis_output_ = robot_status->chassis_output;
    shooter_output_ = robot_status->shooter_output;
  } 
  //Robot Heat
  void RobotHeatCallback(const roborts_msgs::RobotHeat::ConstPtr& robot_heat){
    chassis_volt_ = robot_heat->chassis_volt;
    chassis_current_ = robot_heat->chassis_current;
    chassis_power_ = robot_heat->chassis_power;
    chassis_power_buffer_ = robot_heat->chassis_power_buffer;
    shooter_heat_ = robot_heat->shooter_heat;
  } 
  //Robot Bonus
  void RobotBonusCallback(const roborts_msgs::RobotBonus::ConstPtr& robot_bonus){
    bonus_ = robot_bonus->bonus;
  } 
  //Robot Damage
  void RobotDamageCallback(const roborts_msgs::RobotDamage::ConstPtr& robot_damage){
    last_armor_attacked__time_ = ros::Time::now();
    
    damage_type_ = (DamageType)robot_damage->damage_type;
    armor_attacked_ = (DamageSource)robot_damage->damage_source;
  } 
  //Robot Shoot
  void RobotShootCallback(const roborts_msgs::RobotShoot::ConstPtr& robot_shoot){
    frequency_ = robot_shoot->frequency;
    speed_ = robot_shoot->speed;
  } 
  
  GameStatus GetGameStatus() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)game_status_);
    return game_status_;
  }
  BonusStatus GetBonusStatus() const {// not right
    ROS_INFO("%s: %d", __FUNCTION__, (int)red_bonus_status_);
    return red_bonus_status_;
  }
  BonusStatus GetEnemyBonusStatus() const {// not right
    ROS_INFO("%s: %d", __FUNCTION__, (int)blue_bonus_status_);
    return blue_bonus_status_;
  }

  SupplierStatus GetSupplierStatus(){
    ROS_INFO("%s: %d", __FUNCTION__, (int)supplier_status_);
    return supplier_status_;
  }


  double HurtedPerSecond() {
    if (ros::Time::now()-last_get_hp_time_ > ros::Duration(0.5)) {
      auto reduce_hp = last_hp_ - remain_hp_;
      dmp_ = reduce_hp / (ros::Time::now()-last_get_hp_time_).toSec();
      last_hp_ = remain_hp_;
      last_get_hp_time_ = ros::Time::now();
      return dmp_;
    } else {
      return dmp_;
    }
  }

  DamageSource GetDamageSource() const{
    if (ros::Time::now()-last_armor_attacked__time_>ros::Duration(0.1)){
      ROS_INFO("%s: %s", __FUNCTION__, ": NONE");
      return DamageSource::NONE;
    } else {
      ROS_INFO("%s: %d", __FUNCTION__, (int)armor_attacked_);
      return armor_attacked_;
    }
  }

  EnemyStatus EnemyDetected() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    ROS_INFO("%s: %d", __FUNCTION__, (int)back_enemy_detected_);
    if (enemy_detected_ && back_enemy_detected_)
      return EnemyStatus::BOTH;
    else if (enemy_detected_)
      return EnemyStatus::FRONT;
    else if (back_enemy_detected_)
      return EnemyStatus::BACK;
    else
      return EnemyStatus::NONE;   
  }

  bool EnemyDisappear(){
    for (int i = 0; i < 50; i++) {
      if(enemy_disappear_[i] == 1){
        return true;
      }
    }
    return false;

  }

  void SendSupplyCmd(){
    projectilesupply_.supply = 1;
    projectile_supply_pub_.publish(projectilesupply_);
  }

    ros::Subscriber game_status_sub_;
    ros::Subscriber game_result_sub_;
    ros::Subscriber game_survival_sub_;
    ros::Subscriber bonus_status_sub_;
    ros::Subscriber supplier_status_sub_;
    ros::Subscriber robot_status_sub_;
    ros::Subscriber robot_heat_sub_;
    ros::Subscriber robot_bonus_sub_;
    ros::Subscriber robot_damage_sub_;
    ros::Subscriber robot_shoot_sub_;  
    ros::Publisher projectile_supply_pub_; 
    //! Referee system info
    // Game Status
    GameStatus game_status_;
    unsigned int remaining_time_;
    // Game Result
    GameResult game_result_;
    //Game Survior
    bool red3_;
    bool red4_;
    bool blue3_;
    bool blue4_;
    //Bonus Status
    BonusStatus red_bonus_status_;
    BonusStatus blue_bonus_status_;
    //Supplier Status
    SupplierStatus supplier_status_;
    //Robot Status
    unsigned int id_;
    unsigned int level_;
    unsigned int remain_hp_;
    unsigned int max_hp_;
    unsigned int heat_cooling_limit_;
    unsigned int heat_cooling_rate_;
    bool gimbal_output_;
    bool chassis_output_;
    bool shooter_output_;
    //Robot Heat
    unsigned int chassis_volt_;
    unsigned int chassis_current_;
    float chassis_power_;
    unsigned int chassis_power_buffer_;
    unsigned int shooter_heat_;
    //Robot Bonus
    bool bonus_;
    //Robot Damage
    DamageSource armor_attacked_;
    DamageType damage_type_; 
    //Robot Shoot
    unsigned int frequency_;
    float speed_;
    //projectile supply
    roborts_msgs::ProjectileSupply projectilesupply_;


    unsigned int last_hp_;
    ros::Time last_get_hp_time_;
    double dmp_;
    ros::Time last_armor_attacked__time_;
    bool back_enemy_detected_;
    unsigned int enemy_disappear_[50];
    unsigned int enemy_disappear_count_;




/**********************************************/



  bool IsMasterCondition(){
    ros::Duration time_past = ros::Time::now() - start_time_;
		if (time_past.toSec() >= 60 * identity_number_){
      if (self_identity_ == Identity::MASTER){
        self_identity_ = Identity::SLAVE;
      }
      else{
        self_identity_ = Identity::MASTER;
      }
      identity_number_++;
    }
    if (self_identity_ == Identity::MASTER){
      ROS_INFO(" MASTER MASTER!");
        return true;
    }        
    else{
      ROS_INFO(" SLAVE SLAVE!");
      return false;
    }
        
  }

  bool IsMasterSupplyCondition(){
      if (GetPartnerStatus()==PartnerStatus::SUPPLY)
         return true;
      else
         return false;

  }
  bool IsMasterGainBuffCondition(){
     if (GetPartnerStatus()==PartnerStatus::GAINBUFF)
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
    if (bullet_num_ > 5){
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
    if (new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
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
    fix_goal.pose.position.x = 4 + 0.13;
    fix_goal.pose.position.y = 4.5 - 0.08;
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
		return speed_;
	}
	
	bool IsSupplyCondition() {
		ros::Duration time_past = ros::Time::now() - start_time_;
		if (time_past.toSec() >= 60 * supply_number_){
      return true;
      supplier_status_ = SupplierStatus::PREPARING;
    }
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
		// if (red_bonus_status_ == roborts_decision::BonusStatus::BEING_OCCUPIED)
    //   return true;
    // else 
    //   return false;
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
  int bullet_freq_;
	
	ros::Subscriber gimbal_angle_sub_;
	roborts_msgs::GimbalAngle cmd_gimbal_angle_;
	//time and supply
	ros::Time start_time_;

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
  int identity_number_;
	
	int gain_buff_number_;

  unsigned int search_count_;

  roborts_msgs::PartnerInformation partner_msg_pub_;
};

} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
