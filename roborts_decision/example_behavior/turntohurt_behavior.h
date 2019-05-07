#ifndef ROBORTS_DECISION_TURNTOHURT_BEHAVIOR_H
#define ROBORTS_DECISION_TURNTOHURT_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"


namespace roborts_decision {

class TurnToHurtBehavior {
 public:
  TurnToHurtBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard) :
		chassis_executor_(chassis_executor), blackboard_(blackboard) {
    rotate_speed_.linear.x = 0;
		rotate_speed_.linear.y = 0;
		rotate_speed_.linear.z = 0;
		rotate_speed_.angular.x = 0;
		rotate_speed_.angular.y = 0;
		rotate_speed_.angular.z = 0;
  }

  void Run() {
    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING)
      initial_damage_source_ = blackboard_->GetDamageSource();
    
    double speed;
    switch (initial_damage_source_){
      case DamageSource::FORWARD:
        rotate_speed_.angular.z = 0;
        break;
      case DamageSource::BACKWARD:
        rotate_speed_.angular.z = 2 * M_PI;
        break;
      case DamageSource::LEFT:
        rotate_speed_.angular.z = 2 * M_PI;
        break;
      case DamageSource::RIGHT:
        rotate_speed_.angular.z = -2 * M_PI;
        break;
      default:
        rotate_speed_.angular.z = 0;
        return;
    }
    double delta_yaw = StandardAngleDiff(blackboard_->GetChassisYaw(), initial_yaw_);
    switch (initial_damage_source_){
      case DamageSource::BACKWARD:
        if (delta_yaw > 170 * M_PI / 180 || delta_yaw < -170 * M_PI / 180) {
          chassis_executor_->Cancel();
          return;
        }
        break;
      case DamageSource::LEFT:
        if (delta_yaw > M_PI / 2) {
          chassis_executor_->Cancel();
          return;
        }
        break;
      case DamageSource::RIGHT:
        if (delta_yaw < -M_PI / 2) {
          chassis_executor_->Cancel();
          return;
        }
        break;
      default:
        return;
    }
    chassis_executor_->Execute(rotate_speed_);
    // ROS_INFO("Executor_state: %d", (int)executor_state);
    /*if (executor_state != BehaviorState::RUNNING){
      double yaw;
      switch (blackboard_->GetDamageSource()){
        case DamageSource::FORWARD:
          break;
        case DamageSource::BACKWARD:
          yaw = M_PI;
          break;
        case DamageSource::LEFT:
          yaw = M_PI/2.;
          break;
        case DamageSource::RIGHT:
          yaw = -M_PI/2.;
          break;
        default:
          return;
      }
      geometry_msgs::PoseStamped hurt_pose;
      auto quaternion = tf::createQuaternionMsgFromYaw(yaw);
      hurt_pose.header.frame_id="base_link";
      hurt_pose.header.stamp=ros::Time::now();
      hurt_pose.pose.orientation=quaternion;
      chassis_executor_->Execute(hurt_pose);
    }*/
  }

  void Cancel() {
		chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~TurnToHurtBehavior() = default;

 private:
  double StandardAngleDiff(double yaw1, double yaw2) {
    double yaw = yaw1 - yaw2;
    while (yaw > M_PI)
      yaw -= 2 * M_PI;
    while (yaw < -M_PI)
      yaw += 2 * M_PI;
    return yaw;
  }

 public:
  //The initial yaw when the behavior is on
  double initial_yaw_;
  DamageSource initial_damage_source_;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  geometry_msgs::Twist rotate_speed_;
};

class TurnBackBehavior {
 public:
  TurnBackBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard) :
		chassis_executor_(chassis_executor), blackboard_(blackboard) {
    rotate_speed_.linear.x = 0;
		rotate_speed_.linear.y = 0;
		rotate_speed_.linear.z = 0;
		rotate_speed_.angular.x = 0;
		rotate_speed_.angular.y = 0;
		rotate_speed_.angular.z = 2 * M_PI;
  }

  void Run() {
    auto executor_state = Update();
    double delta_yaw = StandardAngleDiff(blackboard_->GetChassisYaw(), initial_yaw_);
    if (delta_yaw > 170 * M_PI / 180 || delta_yaw < -170 * M_PI / 180) {
      chassis_executor_->Cancel();
      return;
    }
    chassis_executor_->Execute(rotate_speed_);
    // if (executor_state != BehaviorState::RUNNING){
    //   if(blackboard_->back_enemy_detected_){
    //     double yaw = M_PI;
    //     geometry_msgs::PoseStamped back_pose;
    //     auto quaternion = tf::createQuaternionMsgFromYaw(yaw);
    //     back_pose.header.frame_id="base_link";
    //     back_pose.header.stamp=ros::Time::now();
    //     back_pose.pose.orientation=quaternion;
    //     chassis_executor_->Execute(back_pose);
    //   }else{
    //     return;
    //   }
    // }
  }

  void Cancel() {
		chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~TurnBackBehavior() = default;

private:
  double StandardAngleDiff(double yaw1, double yaw2) {
    double yaw = yaw1 - yaw2;
    while (yaw > M_PI)
      yaw -= 2 * M_PI;
    while (yaw < -M_PI)
      yaw += 2 * M_PI;
    return yaw;
  }

 public:
  //The initial yaw when the behavior is on
  double initial_yaw_;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  geometry_msgs::Twist rotate_speed_;
};




}


#endif //ROBORTS_DECISION_TURNTOHURT_BEHAVIOR_H
