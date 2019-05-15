#ifndef ROBORTS_DECISION_BUFF_CHASE_BEHAVIOR_H
#define ROBORTS_DECISION_BUFF_CHASE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class BuffChaseBehavior {
 public:
  BuffChaseBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


  }

  void Run() {
    blackboard_->ChaseAlertForGimbalControl();
    blackboard_->PublishPartnerInformation();
    auto executor_state = Update();
    
    chassis_executor_->Execute(blackboard_->GetGimbalYaw());
  }


  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  void SetGoal(geometry_msgs::PoseStamped buff_chase_goal) {
    buff_chase_goal_ = buff_chase_goal;
  }

  ~BuffChaseBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! buff_chase goal
  geometry_msgs::PoseStamped buff_chase_goal_;

  //! buff_chase buffer
  std::vector<geometry_msgs::PoseStamped> buff_chase_buffer_;
  unsigned int buff_chase_count_;

  //! cancel flag
  bool cancel_goal_;
};
}

#endif //ROBORTS_DECISION_BUFF_CHASE_BEHAVIOR_H
