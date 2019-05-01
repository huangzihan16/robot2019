#ifndef ROBORTS_DETECTION_SHOOT_EXECUTOR_H
#define ROBORTS_DETECTION_SHOOT_EXECUTOR_H

#include "ros/ros.h"

#include "roborts_msgs/ShootCmd.h"
#include "../../../roborts_base/gimbal/gimbal.h"
#include "../../../roborts_base/chassis/chassis.h"
#include "../../../roborts_base/roborts_base_config.h"
#include "../../../roborts_base/roborts_sdk/sdk.h"
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

namespace roborts_detection{
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
class ShootExecutor{
 public:
  enum class ExcutionMode{
    IDLE_MODE,   ///< Default idle mode with no task
    SHOOT_MODE,  ///< 
  };

  ShootExecutor():excution_mode_(ExcutionMode::IDLE_MODE),
                                is_published_(false) {
  ros::NodeHandle nh;
  cmd_shoot_client_ = nh.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
  ROS_INFO("Shoot server start!");
  zero_shoot_cmd_.request.mode = 0;
	zero_shoot_cmd_.request.number = 0;

  game_status_sub_ = nh.subscribe<roborts_msgs::GameStatus>("game_status",30 , &ShootExecutor::GameStatusCallback, this);
  game_result_sub_ = nh.subscribe<roborts_msgs::GameResult>("game_result",30 , &ShootExecutor::GameResultCallback, this);
    game_survival_sub_ = nh.subscribe<roborts_msgs::GameSurvivor>("game_survivor",30 , &ShootExecutor::GameSurvivorCallback, this);
    bonus_status_sub_ = nh.subscribe<roborts_msgs::BonusStatus>("field_bonus_status",30 , &ShootExecutor::BonusStatusCallback, this);
    supplier_status_sub_ = nh.subscribe<roborts_msgs::SupplierStatus>("field_supplier_status",30 , &ShootExecutor::SupplierStatusCallback, this);
    robot_status_sub_ = nh.subscribe<roborts_msgs::RobotStatus>("robot_status",30 , &ShootExecutor::RobotStatusCallback, this);
    robot_heat_sub_ = nh.subscribe<roborts_msgs::RobotHeat>("robot_heat",30 , &ShootExecutor::RobotHeatCallback, this);
    robot_bonus_sub_ = nh.subscribe<roborts_msgs::RobotBonus>("robot_bonus",30 , &ShootExecutor::RobotBonusCallback, this);
    robot_damage_sub_ = nh.subscribe<roborts_msgs::RobotDamage>("robot_damage",30 , &ShootExecutor::RobotDamageCallback, this);
    robot_shoot_sub_ = nh.subscribe<roborts_msgs::RobotShoot>("robot_shoot",30 , &ShootExecutor::RobotShootCallback, this);
    projectile_supply_pub_ = nh.advertise<roborts_msgs::ProjectileSupply>("projectile_supply", 1);
}
  ~ShootExecutor() = default;

void Execute(){
  excution_mode_ = ExcutionMode::SHOOT_MODE;
  shootdecision();
  if(cmd_shoot_client_.call(shoot_cmd_)) {
    ROS_INFO("yes %d", (int)shoot_cmd_.response.received);
		is_published_ = true;
	}
  else {
    ROS_INFO("no");
		is_published_ = false;
	}
}
  
  bool GetIsPublished(){
    return is_published_;
  }

  void shootdecision() {
    int CD;
    int f_max;
    //update parameters
    if (remain_hp_ >= 400){
        CD = 120;
    }
    else{
        CD = 240;
    }
    // v~d determine v by d

    // //determine max f
    // if((360 - shooter_heat_ - speed_ * GetBulletNum()) >= 0){
    //     f_max = 20;
    // }
    // else{
    //     f_max = CD * GetBulletNum() / (speed_ * GetBulletNum() + shooter_heat_ - 360);
    // }
    
    //mode 0 stop 1 once 2 continuous 
    if (shooter_heat_ >30){
      shoot_cmd_.request.mode = 0;
      shoot_cmd_.request.number = 0;
    } else {
      shoot_cmd_.request.mode = 1;
      shoot_cmd_.request.number = 1;
    }

	}
  const int GetBulletNum() {
		return 40;
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
    damage_type_ = (DamageType)robot_damage->damage_type;
    armor_attacked_ = (DamageSource)robot_damage->damage_source;
  } 
  //Robot Shoot
  void RobotShootCallback(const roborts_msgs::RobotShoot::ConstPtr& robot_shoot){
    frequency_ = robot_shoot->frequency;
    speed_ = robot_shoot->speed;
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

  roborts_msgs::ShootCmd shoot_cmd_;

 private:
 //! execution mode of the executor
  ExcutionMode excution_mode_;
  //! execution state of the executor (same with behavior state)
  //BehaviorState execution_state_;
  roborts_msgs::ShootCmd zero_shoot_cmd_;

  ros::ServiceClient cmd_shoot_client_;

	bool is_published_;
  
};
}


#endif //ROBORTS_DETECTION_SHOOT_EXECUTOR_H