#ifndef ROBORTS_DECISION_ROUND_BEHAVIOR_H
#define ROBORTS_DECISION_ROUND_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"


namespace roborts_decision {

class RoundBehavior {
 public:
  RoundBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard, const double yaw_rate) :
		chassis_executor_(chassis_executor), blackboard_(blackboard) {
		round_vel_.linear.x = 0;
		round_vel_.linear.y = 0;
		round_vel_.linear.z = 0;
		round_vel_.angular.x = 0;
		round_vel_.angular.y = 0;
		round_vel_.angular.z = yaw_rate;
  }

  void Run() {
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();
    auto executor_state = Update();
    std::cout << "state: " << (int)(executor_state) << std::endl;
    chassis_executor_->Execute(round_vel_);
  }

  void Cancel() {
		chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~RoundBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;
  geometry_msgs::Twist round_vel_;
  //! perception information
  Blackboard* const blackboard_;

};
}


#endif //ROBORTS_DECISION_SHOOT_BEHAVIOR_H
