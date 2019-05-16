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
      blackboard_(blackboard), have_good_goal_(false) { }

  void Run() {
    blackboard_->partner_msg_pub_.status = (char)PartnerStatus::BATTLE;
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();

    geometry_msgs::PoseStamped enemy_pose, self_pose;
    enemy_pose = blackboard_->GetPartnerEnemyPose();
    self_pose = blackboard_->GetRobotMapPose();

    if (have_good_goal_)
      have_good_goal_ = blackboard_->cachedmapforchaseandsupport_ptr_->IsGoalStillAvailable(enemy_pose, goal_pose_);
    if (!have_good_goal_)
      have_good_goal_ =
        blackboard_->cachedmapforchaseandsupport_ptr_->FindSupportGoal(enemy_pose, blackboard_->partner_pose_, self_pose, goal_pose_);

    if (have_good_goal_)
      chassis_executor_->Execute(goal_pose_);
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

 public:
  bool have_good_goal_;
  geometry_msgs::PoseStamped goal_pose_;
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
