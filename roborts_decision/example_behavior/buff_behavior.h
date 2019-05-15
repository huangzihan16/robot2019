#ifndef ROBORTS_DECISION_BUFF_BEHAVIOR_H
#define ROBORTS_DECISION_BUFF_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
namespace roborts_decision {
class GainBuffBehavior {
 public:
  GainBuffBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard) :
      chassis_executor_(chassis_executor), blackboard_(blackboard), have_execute_(false), i(0), buff_count_(1){ }

  void Run() {
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();
    if(!have_execute_){
      buff_start_time_ = ros::Time::now();
      buff_count_ = 1;
			have_execute_ = true;
    }
    BehaviorState executor_state = Update();
    if (executor_state != BehaviorState::RUNNING && blackboard_->GetBonusStatus() == BonusStatus::UNOCCUPIED)
      chassis_executor_->Execute(GetAddGuardGoal());
    // if (blackboard_->GetBonusStatus() == BonusStatus::BEING_OCCUPIED){
    //       // chassis_executor_->Cancel();
    //   geometry_msgs::PoseStamped fix_goal = blackboard_->GetRobotMapPose();
    //   ros::Time current_time = ros::Time::now();
    //   fix_goal.header.stamp = current_time;
    //   fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(225.0/180*3.14);

    //   chassis_executor_->Execute(fix_goal);
    // }
 }

  void Cancel() {
		have_execute_ = false;
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
		if (blackboard_->GetBonusStatus() == BonusStatus::BEING_OCCUPIED) {
			// chassis_executor_->Cancel();
			have_execute_ = false;
			return BehaviorState::SUCCESS;
		} else {
			BehaviorState state = chassis_executor_->Update();
			return state;
		}
  }

  ~GainBuffBehavior() = default;
  geometry_msgs::PoseStamped GetAddGuardGoal() {
    buff_time_duration_ = ros::Time::now() - buff_start_time_;
    if (/*buff_time_duration_.toSec() / buff_count_ > 1*/ blackboard_->GetBonusStatus() == BonusStatus::UNOCCUPIED){
      buff_count_ += 1;
      i += 1;
    }

    float goal_x_[5] = {6.30, 6.2, 6.40, 6.40, 6.2};
    float goal_y_[5] = {1.85, 1.95, 1.75, 1.95, 1.75};  
    float goal_q_[5] = {180, 135, 225, 180, 135};      
    geometry_msgs::PoseStamped fix_goal;
    ros::Time current_time = ros::Time::now();
    fix_goal.header.stamp = current_time;
    fix_goal.pose.position.x = goal_x_[i % 5];
    fix_goal.pose.position.y = goal_y_[i % 5];
    fix_goal.pose.position.z = 0.0;
    fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_q_[i]/180*3.14);

    return fix_goal;
  }

public:
		bool have_execute_;

    int i;
    double buff_count_;
    ros::Time buff_start_time_;
    ros::Duration buff_time_duration_;
 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
};


}

#endif //ROBORTS_DECISION_BUFF_BEHAVIOR_H