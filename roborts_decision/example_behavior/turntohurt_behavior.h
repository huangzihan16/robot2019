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
    last_stop_time_ = ros::Time::now();
    stop_times_ = 0;
  }

  void Run() {
    auto executor_state = Update();
    double delta_yaw = StandardAngleDiff(blackboard_->GetChassisYaw(), initial_yaw_);
    double speed = 0;
    ros::Duration laststop_to_now = ros::Time::now() - last_stop_time_;
    switch (initial_damage_source_){
      case DamageSource::BACKWARD:
        if (delta_yaw > 115 * M_PI / 180 && (stop_times_ == 0 || laststop_to_now.toSec() < 0.3)) {
          speed = 0;
          if (stop_times_ == 0) {         
            last_stop_time_ = ros::Time::now();
            stop_times_++;
          }
        } else if ((delta_yaw > 170 * M_PI / 180 || delta_yaw < 0 * M_PI / 180) &&
                    (stop_times_ == 1 || laststop_to_now.toSec() < 0.3)) {
          speed = 0;
          if (stop_times_ == 1) {
            last_stop_time_ = ros::Time::now();
            stop_times_++;
          }
        } else if (delta_yaw > -125 * M_PI / 180 && (stop_times_ == 2 || laststop_to_now.toSec() < 0.3)) {
          speed = 0;
          if (stop_times_ == 2) {
            last_stop_time_ = ros::Time::now();
            stop_times_++;
          }
        } else if (stop_times_ == 3) {
          chassis_executor_->Cancel();
          return;
        } else 
          speed = initial_speed_;
        break;
      case DamageSource::LEFT:
        if (delta_yaw > 55 * M_PI / 180 && (stop_times_ == 0 || laststop_to_now.toSec() < 0.3)) {
          speed = 0;
          if (stop_times_ == 0) {         
            last_stop_time_ = ros::Time::now();
            stop_times_++;
          }
        } else if (delta_yaw > 115 * M_PI / 180 && (stop_times_ == 1 || laststop_to_now.toSec() < 0.3)) {
          speed = 0;
          if (stop_times_ == 1) {
            last_stop_time_ = ros::Time::now();
            stop_times_++;
          }
        } else if (stop_times_ == 2) {
          chassis_executor_->Cancel();
          return;
        } else 
          speed = initial_speed_;
        break;
      case DamageSource::RIGHT:
        if (delta_yaw < -55 * M_PI / 180 && (stop_times_ == 0 || laststop_to_now.toSec() < 0.3)) {
          speed = 0;
          if (stop_times_ == 0) {         
            last_stop_time_ = ros::Time::now();
            stop_times_++;
          }
        }else if (delta_yaw < -115 * M_PI / 180 && (stop_times_ == 1 || laststop_to_now.toSec() < 0.3)) {
          speed = 0;
          if (stop_times_ == 1) {
            last_stop_time_ = ros::Time::now();
            stop_times_++;
          }
        } else if (stop_times_ == 2) {
          chassis_executor_->Cancel();
          return;
        } else 
          speed = initial_speed_;
        break;
      default:
        return;
    }
    
    rotate_speed_.angular.z = speed;
    chassis_executor_->Execute(rotate_speed_);
  }

  void Cancel() {
		chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  void StopTimesSetToZero() {
    stop_times_ = 0;
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
  double initial_speed_;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  geometry_msgs::Twist rotate_speed_;

  ros::Time last_stop_time_;
  int stop_times_;
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
		rotate_speed_.angular.z = 0;
    initial_speed_ = 2 * M_PI;
  }

  void Run() {
    auto executor_state = Update();
    double delta_yaw = StandardAngleDiff(blackboard_->GetChassisYaw(), initial_yaw_);
    double speed = 0;
    ros::Duration laststop_to_now = ros::Time::now() - last_stop_time_;
    if (delta_yaw > 115 * M_PI / 180 && (stop_times_ == 0 || laststop_to_now.toSec() < 0.3)) {
      speed = 0;
      if (stop_times_ == 0) {         
        last_stop_time_ = ros::Time::now();
        stop_times_++;
      }
    } else if ((delta_yaw > 170 * M_PI / 180 || delta_yaw < 0 * M_PI / 180) &&
               (stop_times_ == 1 || laststop_to_now.toSec() < 0.3)) {
      speed = 0;
      if (stop_times_ == 1) {
        last_stop_time_ = ros::Time::now();
        stop_times_++;
      }
    } else if (delta_yaw > -125 * M_PI / 180 && (stop_times_ == 2 || laststop_to_now.toSec() < 0.3)) {
      speed = 0;
      if (stop_times_ == 2) {
        last_stop_time_ = ros::Time::now();
        stop_times_++;
      }
    } else if (stop_times_ == 3) {
      chassis_executor_->Cancel();
      return;
    } else 
      speed = initial_speed_;

    rotate_speed_.angular.z = speed;
    chassis_executor_->Execute(rotate_speed_);
  }

  void Cancel() {
		chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  void StopTimesSetToZero() {
    stop_times_ = 0;
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
  double initial_speed_;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  geometry_msgs::Twist rotate_speed_;

  ros::Time last_stop_time_;
  int stop_times_;
};




}


#endif //ROBORTS_DECISION_TURNTOHURT_BEHAVIOR_H
