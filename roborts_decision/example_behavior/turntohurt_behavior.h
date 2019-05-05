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

  }

  void Run() {
    auto executor_state = Update();
    // ROS_INFO("Executor_state: %d", (int)executor_state);
    if (executor_state != BehaviorState::RUNNING){
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
    }
  }

  void Cancel() {
		chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~TurnToHurtBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

};

class TurnBackBehavior {
 public:
  TurnBackBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard) :
		chassis_executor_(chassis_executor), blackboard_(blackboard) {

  }

  void Run() {
    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING){
      double yaw = M_PI;
      geometry_msgs::PoseStamped back_pose;
      auto quaternion = tf::createQuaternionMsgFromYaw(yaw);
      back_pose.header.frame_id="base_link";
      back_pose.header.stamp=ros::Time::now();
      back_pose.pose.orientation=quaternion;
      chassis_executor_->Execute(back_pose);
    }
  }

  void Cancel() {
		chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~TurnBackBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

};




}


#endif //ROBORTS_DECISION_TURNTOHURT_BEHAVIOR_H
