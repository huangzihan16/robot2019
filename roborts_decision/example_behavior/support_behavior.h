#ifndef ROBORTS_DECISION_SUPPORT_BEHAVIOR_H
#define ROBORTS_DECISION_SUPPORT_BEHAVIOR_H


#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"


namespace roborts_decision {
class SupportBehavior {
 public:
  SupportBehavior(ChassisExecutor* &chassis_executor, GimbalExecutor* &gimbal_executor,
               Blackboard* &blackboard) :
      chassis_executor_(chassis_executor), gimbal_executor_(gimbal_executor),
      blackboard_(blackboard) { }

  void Run() {
    //gimbal_executor_->WhirlScan();
    geometry_msgs::PoseStamped enemy_pose, self_pose, goal_pose;
    enemy_pose = blackboard_->GetPartnerEnemyPose();
    self_pose = blackboard_->GetRobotMapPose();
    bool find_good_position =
      blackboard_->cachedmapforchaseandsupport_ptr_->FindSupportGoal(enemy_pose, blackboard_->partner_pose_, self_pose, goal_pose);

    if (find_good_position)
      chassis_executor_->Execute(goal_pose);
    else 
      chassis_executor_->Execute(enemy_pose);
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~SupportBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;
	GimbalExecutor* const gimbal_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //geometry_msgs::PoseStamped planning_goal_;

};


}



#endif //ROBORTS_DECISION_GOAL_BEHAVIOR_H
