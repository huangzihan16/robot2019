#ifndef ROBORTS_DECISION_GOAL_BEHAVIOR_H
#define ROBORTS_DECISION_GOAL_BEHAVIOR_H


#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"


namespace roborts_decision {
class GoalBehavior {
 public:
  GoalBehavior(ChassisExecutor* &chassis_executor,
               Blackboard* &blackboard) :
      chassis_executor_(chassis_executor),
      blackboard_(blackboard) { }

  void Run() {
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();
    if(blackboard_->IsNewGoal()){
      chassis_executor_->Execute(blackboard_->GetGoal());
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~GoalBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! planning goal
  geometry_msgs::PoseStamped planning_goal_;

};

class SupplyGoalBehavior {
 public:
  SupplyGoalBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard) :
      chassis_executor_(chassis_executor), blackboard_(blackboard), have_execute_(false){ }

  void Run() {
    blackboard_->SuggestGimbalPatrol();
    if(!have_execute_){
      chassis_executor_->Execute(blackboard_->GetSupplyGoal());
			have_execute_ = true;
    }
  }

  void Cancel() {
		have_execute_ = false;
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
		BehaviorState state = chassis_executor_->Update();
		if (state != BehaviorState::RUNNING)
			have_execute_ = false;
    return state;
  }

  ~SupplyGoalBehavior() = default;

public:
		bool have_execute_;
 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
};

class GainBuffGoalBehavior {
 public:
  GainBuffGoalBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard) :
      chassis_executor_(chassis_executor), blackboard_(blackboard), have_execute_(false){ }

  void Run() {
    blackboard_->SuggestGimbalPatrol();
    if(!have_execute_){
            geometry_msgs::PoseStamped fix_goal = blackboard_->GetRobotMapPose();
      ros::Time current_time = ros::Time::now();
      fix_goal.header.stamp = current_time;
      fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(190.0/180*3.14);
      chassis_executor_->Execute(fix_goal);

      // chassis_executor_->Execute(blackboard_->GetGuardGoal());
			have_execute_ = true;
    }
    //     BehaviorState executor_state = Update();
    // if (executor_state != BehaviorState::RUNNING ){
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
		if (blackboard_->ArriveGainBuff()) {
			chassis_executor_->Cancel();
			
			have_execute_ = false;
			return BehaviorState::SUCCESS;
		} else {
			BehaviorState state = chassis_executor_->Update();
			return state;
		}
  }

  ~GainBuffGoalBehavior() = default;

public:
		bool have_execute_;
 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
};

}



#endif //ROBORTS_DECISION_GOAL_BEHAVIOR_H
