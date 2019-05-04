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
#include "roborts_msgs/BackCameraAction.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"
#include "std_msgs/Int16.h"
#include "./blackboard_enum.h"
#include "../example_behavior/line_iterator.h"

#include "roborts_msgs/PartnerInformation.h"
//下面这些是裁判系统信息相关的头文件
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
#include "roborts_msgs/EnemyInfo.h"

#include "roborts_msgs/ShootCmd.h"

namespace roborts_decision{

class CachedMapCell {
public:
	typedef std::shared_ptr<CachedMapCell> Ptr;
	CachedMapCell(unsigned char freespace_threshold, double around_area_radius, double max_lost_radius,
								double best_distance_goal_enemy, roborts_costmap::Costmap2D* costmap2d) :
		freespace_threshold_(freespace_threshold), around_area_radius_(around_area_radius),
		max_lost_radius_(max_lost_radius), best_distance_goal_enemy_(best_distance_goal_enemy), costmap2d_(costmap2d) {
		resolution_ = costmap2d->GetResolution();
		map_x_size_ = costmap2d->GetSizeXCell();
		map_y_size_ = costmap2d->GetSizeYCell();
			
		int bias = around_area_radius / resolution_;
		cached_number_ = 0;
		for (int row = -bias; row <= bias; row++) {
			double dy = row * resolution_;
			for (int col = -bias; col <= bias; col++) {
				if (GetDistanceSquareBetweenCells(row, col, 0, 0) < around_area_radius * around_area_radius && (row != 0 || col != 0)) {
					cached_map_x_bias_.push_back(col);
					cached_map_y_bias_.push_back(row);
					double dx = col * resolution_;
					double distance_square = dx * dx + dy * dy;
					double distance = sqrt(distance_square);
					cached_distance_square_.push_back(distance_square);
					cached_distance_.push_back(distance);
					cached_x_normal_.push_back(dx / distance);
					cached_y_normal_.push_back(dy / distance);
					cached_yaw_.push_back(atan2(-dy, -dx));
					cached_number_++;
				}
			}
		}
	}//the round area around enemy without center
	
	~CachedMapCell() = default;
	
	bool FindChaseGoal(geometry_msgs::PoseStamped enemy_pose, geometry_msgs::PoseStamped self_pose, geometry_msgs::PoseStamped& goal_pose);
	bool FindSupportGoal(geometry_msgs::PoseStamped enemy_pose, geometry_msgs::PoseStamped partner_pose,
                       geometry_msgs::PoseStamped self_pose, geometry_msgs::PoseStamped& goal_pose);
	
	bool IsGoalStillAvailable(geometry_msgs::PoseStamped enemy_pose, geometry_msgs::PoseStamped last_goal);
private:
	double GetDistanceSquareBetweenCells(int x1, int y1, int x2, int y2);
	double GetAngleBetweenNonZeroVectorandCachedVector(double x, double y, int num);
	double GetDistanceBetweenPointandLine(double x, double y, double x1, double y1, double x2, double y2);
	double GetDistanceBetweenPointandLineWithBottomLength(double x, double y, double x1, double y1, double x2, double y2, double bottom_length);
public:
	std::vector<int> cached_map_x_bias_;//from enemy to cell
	std::vector<int> cached_map_y_bias_;//from enemy to cell
	std::vector<double> cached_distance_square_;//distance square from enemy to cell
	std::vector<double> cached_distance_;//distance from enemy to cell
	std::vector<double> cached_x_normal_;//from enemy to cell
	std::vector<double> cached_y_normal_;//from enemy to cell
	std::vector<double> cached_yaw_;//the required yaw if this cell is goal
	int cached_number_;
	//Origin of these vectors is enemy and content is goal.
private:
	unsigned char freespace_threshold_;
	double around_area_radius_;
	double best_distance_goal_enemy_;
	double max_lost_radius_;
		
	double resolution_;
	unsigned int map_x_size_;
	unsigned int map_y_size_;
	int enemy_inflation_;
	roborts_costmap::Costmap2D* costmap2d_;
};

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
      tag_id_(0),
      back_camera_mode_(0),
      /*******************/
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
      back_camera_client_("backcamera_node_action", true),
			self_identity_(Identity::SLAVE),
      partner_detect_enemy_(false),
			supply_number_(0),
      identity_number_(1),
 			gain_buff_number_(0){

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

    last_enemy_disappear_time_ = ros::Time::now() + ros::Duration(5*60);
    /*******************/

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    if (!decision_config.simulate()){

      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");
      back_camera_mode_ = 1;
      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));

      // back_camera_client_.waitForServer();

      // back_camera_goal_.command = 1;
      // back_camera_client_.sendGoal(back_camera_goal_,
      //                              actionlib::SimpleActionClient<roborts_msgs::BackCameraAction>::SimpleDoneCallback(),
      //                              actionlib::SimpleActionClient<roborts_msgs::BackCameraAction>::SimpleActiveCallback(),
      //                              boost::bind(&Blackboard::BackCameraCallback, this, _1));
    }
		bullet_num_ = decision_config.initial_bullet_num();
    std::string partner_name = decision_config.partner_name();
    std::string partner_topic_sub = "/" + partner_name + "/partner_msg";
		partner_sub_ = nh.subscribe<roborts_msgs::PartnerInformation>(partner_topic_sub, 1, &Blackboard::PartnerCallback, this);
		partner_pub_ = nh.advertise<roborts_msgs::PartnerInformation>("partner_msg", 1);
    test_support_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("support_pose", 1);
    test_enemy_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("enemy_pose", 1);
    cachedmapforchaseandsupport_ptr_ =
			std::shared_ptr<CachedMapCell>(new CachedMapCell(100, 2.0, 3.0, 1.0, costmap_2d_));
  }

  ~Blackboard() = default;

  /*******************Enemy Information from roborts_detection*******************/
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback);

  void BackCameraCallback(const roborts_msgs::BackCameraFeedbackConstPtr& feedback);
  void SetBackCameraDetect();
  void SetBackCameraLocalization();
  /*******************Referee System Interaction(Callback and Send Cmd) and Read related Member Variables*******************/
  // Game Status
  void GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr& game_status);
  // Game Result
  void GameResultCallback(const roborts_msgs::GameResult::ConstPtr& game_result);
  //Game Survior
  void GameSurvivorCallback(const roborts_msgs::GameSurvivor::ConstPtr& game_survival);
  //Bonus Status
  void BonusStatusCallback(const roborts_msgs::BonusStatus::ConstPtr& bonus_status);
  //Supplier Status
  void SupplierStatusCallback(const roborts_msgs::SupplierStatus::ConstPtr& supplier_status);
  //Robot Status
  void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr& robot_status);
  //Robot Heat
  void RobotHeatCallback(const roborts_msgs::RobotHeat::ConstPtr& robot_heat);
  //Robot Bonus
  void RobotBonusCallback(const roborts_msgs::RobotBonus::ConstPtr& robot_bonus);
  //Robot Damage
  void RobotDamageCallback(const roborts_msgs::RobotDamage::ConstPtr& robot_damage);
  //Robot Shoot
  void RobotShootCallback(const roborts_msgs::RobotShoot::ConstPtr& robot_shoot);
  //Send Supply Cmd
  void SendSupplyCmd();
  
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

  bool GetRobotBonus(){
    ROS_INFO("%s: %d", __FUNCTION__, (int)bonus_);
    return bonus_;
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

  /*******************Tool Functions of Variables Increase and Decrease Simply*******************/
  void SearchCountM() {
    search_count_ -= 1;
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

  /*******************Localization Information for Supply and Gain Buff*******************/
	bool ArriveGainBuff();
	geometry_msgs::PoseStamped GetSupplyGoal();
  geometry_msgs::PoseStamped GetGuardGoal();

	//测试，给固定目标点
  geometry_msgs::PoseStamped GetFixedGoal();
	
	
  
  /*******************Referee System Information Preliminarily Process(Not Used directly in Behavior Tree)*******************/
  double HurtedPerSecond();

  DamageSource GetDamageSource() const;

  EnemyStatus EnemyDetected() const;

  bool EnemyDisappear();

  /*******************Other(Not Referee System) Output Interface Functions*******************/
  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }
  const geometry_msgs::PoseStamped GetGimbalBasePose() {
    UpdateGimbalPose();
    return gimbal_base_pose_;
  }
  geometry_msgs::PoseStamped GetEnemy() const {
    return enemy_pose_;
  }
  
  geometry_msgs::PoseStamped GetEnemyInCamera() const {
    return enemy_pose_camera_;
  }

  double GetGimbalYaw();
  /*******************Cost Map*******************/
  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }

  /*******************Partner Interaction*******************/
  void PartnerCallback(const roborts_msgs::PartnerInformationConstPtr& partner_info);
  
  void PublishPartnerInformation();

	geometry_msgs::PoseStamped GetPartnerEnemyPose();

  PartnerStatus GetPartnerStatus(){
    return partner_status_;
  }

  bool IsMasterCondition();
  bool IsMasterSupplyCondition();
  bool IsMasterGainBuffCondition();

  /*******************Math Tools*******************/
  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2);

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2);

  /*******************Test Interface for Goal and Gimbal Rotation*******************/
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

  /*******************Functions Used in Behavior Tree*******************/
  bool IsSupplyCondition();
  bool IsGainBuffCondition();
  bool IsEnemyDetected() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }
  bool IsPartnerDetectEnemy () {
		return partner_detect_enemy_;
	}
  bool IsBulletLeft() const;

private:
  /*******************Update Chassis Pose and Gimbal Pose*******************/
  void UpdateRobotPose();

  void UpdateGimbalPose();

public:
  /*******************Referee System Information After Preliminarily Process*******************/
  unsigned int last_hp_;      //上一次检测反馈的HP数，用于计算单位时间hp减少量，以判断是否要逃跑
  ros::Time last_get_hp_time_;//上一次反馈HP的时间
  double dmp_;                //每秒受到的伤害
  ros::Time last_armor_attacked_time_;  //上一次受到伤害的时间
  bool back_enemy_detected_;
  ros::Time last_enemy_disappear_time_; //敌人最后一次出现的时间
  unsigned int search_count_;           //小范围搜索敌人使用，标识用

  /*******************Cost Map*******************/
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  /*******************Variable for Supply Condition and Gain Buff Condition*******************/
  int identity_number_;     //补弹和占Buff所需的身份信息
	int supply_number_;       //补给次数
  int gain_buff_number_;    //占buff次数
  
  /*******************Variable for Chase and Support*******************/
	CachedMapCell::Ptr cachedmapforchaseandsupport_ptr_;

  /****************补弹tag id******************/
  int tag_id_;
  int back_camera_mode_;

  /*******************Referee System Information*******************/
  //Useless
  unsigned int id_;                 //机器人ID，是机器人的所属与种类（无用）
  unsigned int level_;              //机器人等级（无用）
  bool gimbal_output_;  //主控电源gimbal口有电压输出（无用）
  bool chassis_output_; //主控电源chassis口有电压输出（无用）
  bool shooter_output_; //主控电源shooter口有电压输出（无用）
  unsigned int chassis_volt_;         //底盘电压（无用）
  unsigned int chassis_current_;      //底盘电流（无用）
  float chassis_power_;               //底盘功率（无用）
  unsigned int chassis_power_buffer_; //底盘功率缓冲（无用）
  //Game Status
  GameStatus game_status_;      //当前比赛阶段
  unsigned int remaining_time_; //当前阶段剩余时间
  //Game Result
  GameResult game_result_;  //比赛结果
  //Game Survior
  bool red3_;   //红1号是否存活
  bool red4_;   //红2号是否存活
  bool blue3_;  //蓝1号是否存活
  bool blue4_;  //蓝2号是否存活
  //Bonus Status
  BonusStatus red_bonus_status_; //红方防御加成状态，0为未激活，1为触发激活中，2为防御加成已激活
  BonusStatus blue_bonus_status_;//蓝方防御加成状态，0为未激活，1为触发激活中，2为防御加成已激活
  //Supplier Status
  SupplierStatus supplier_status_; //出弹口开闭状态，0为关闭，1为子弹准备中，2为子弹下落
  //Robot Status
  unsigned int remain_hp_;          //剩余血量
  unsigned int max_hp_;             //上限血量
  unsigned int heat_cooling_limit_; //机器人枪口热量上限
  unsigned int heat_cooling_rate_;  //机器人枪口每秒冷却值
  //Robot Heat
  unsigned int shooter_heat_;         //枪口热量
  unsigned int last_shooter_heat_;    //上次记录时的枪口热量
  //Robot Bonus
  bool bonus_;   //机器人是否有防御加成
  //Robot Damage
  DamageSource armor_attacked_;   //受伤装甲
  DamageType damage_type_;        //伤害类型，0装甲伤害扣血，1模块掉线扣血，2超枪口热量扣血，3超底盘功率扣血
  //Robot Shoot
  unsigned int frequency_;        //子弹射频
  float speed_;                   //子弹射速
  //projectile supply
  roborts_msgs::ProjectileSupply projectilesupply_; //补弹指令

private:
  /*******************Enemy Detection Cmd and Result*******************/
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  roborts_msgs::BackCameraGoal back_camera_goal_;
	geometry_msgs::PoseStamped enemy_pose_camera_;
  geometry_msgs::PoseStamped enemy_pose_;
	std::vector<roborts_msgs::EnemyInfo> enemy_info_;
  bool enemy_detected_;

  /*******************Self Chassis Pose and Gimbal Pose*******************/
  geometry_msgs::PoseStamped robot_map_pose_;     //robot map pose
  geometry_msgs::PoseStamped  gimbal_base_pose_;  //gimbal base pose 

  /*******************Bullet Number Left*******************/
  int bullet_num_;   //目前剩余弹数
  
	/*******************Time*******************/
	ros::Time start_time_;   //ros内系统时钟上比赛开始的时间

  /*******************Partner Interaciton Information*******************/
	Identity self_identity_;        //机器人身份
	PartnerStatus partner_status_;  //队友的状态
	
	bool partner_detect_enemy_;     //友方是否检测到敌人
  geometry_msgs::PoseStamped partner_enemy_pose_;
	std::vector<roborts_msgs::EnemyInfo> partner_enemy_info_; //友方检测到的敌人位置
	geometry_msgs::PoseStamped partner_pose_;       //友方的位姿
	int partner_patrol_count_;                      //友方巡逻位置相关

  roborts_msgs::PartnerInformation partner_msg_pub_;  //发送给友方的信息

	/*******************Variables Used in Test Interface*******************/
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;                     //这两个变量是rviz测试目标模式使用的变量
  ros::Publisher test_support_publisher_;
  ros::Publisher test_enemy_publisher_;

	roborts_msgs::GimbalAngle cmd_gimbal_angle_;  //测试云台旋转的topic

  /*******************Referee System Interaction Subscriber and Publisher*******************/
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

  /*******************Other Subscriber and Publisher*******************/
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_; //和detection模块配合的Actionlib接口
  actionlib::SimpleActionClient<roborts_msgs::BackCameraAction> back_camera_client_; //和back camera模块配合的Actionlib接口
  
  ros::Subscriber partner_sub_;  //接收友方信息的订阅器
	ros::Publisher partner_pub_;   //发送给友方信息的发布器

  ros::Subscriber gimbal_angle_sub_;

  std::shared_ptr<tf::TransformListener> tf_ptr_;

  ros::Subscriber enemy_sub_;     //rviz给目标点调试用的
};

} //namespace roborts_decision

#endif //ROBORTS_DECISION_BLACKBOARD_H