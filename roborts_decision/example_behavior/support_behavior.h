#ifndef ROBORTS_DECISION_SUPPORT_BEHAVIOR_H
#define ROBORTS_DECISION_SUPPORT_BEHAVIOR_H


#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"


namespace roborts_decision {
class SupportBehavior {
 public:
  SupportBehavior(ChassisExecutor* &chassis_executor,
               Blackboard* &blackboard) :
      chassis_executor_(chassis_executor),
      blackboard_(blackboard) { }

  void Run() {
      chassis_executor_->Execute(blackboard_->GetPartnerEnemyPose());
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

  //! perception information
  Blackboard* const blackboard_;

  //geometry_msgs::PoseStamped planning_goal_;

};


}



#endif //ROBORTS_DECISION_GOAL_BEHAVIOR_H
