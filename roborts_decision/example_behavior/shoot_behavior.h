#ifndef ROBORTS_DECISION_SHOOT_BEHAVIOR_H
#define ROBORTS_DECISION_SHOOT_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/shoot_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "roborts_msgs/ShootCmd.h"

#include "line_iterator.h"

namespace roborts_decision {

class ShootBehavior {
 public:
  ShootBehavior(ShootExecutor* &shoot_executor, Blackboard* &blackboard) :shoot_executor_(shoot_executor), blackboard_(blackboard) {

  }

  void Run() {

    auto executor_state = Update();
    std::cout << "state: " << (int)(executor_state) << std::endl;

      std::cout << "send shoot_cmd" << std::endl;
      shoot_cmd_.request.mode = 1;
      shoot_cmd_.request.number = 1;
      shoot_executor_->Execute(shoot_cmd_);
  }

  void Cancel() {
		shoot_executor_->Cancel();
  }

  BehaviorState Update() {
    return shoot_executor_->Update();
  }


  ~ShootBehavior() = default;

 private:
  //! executor
  ShootExecutor* const shoot_executor_;

  roborts_msgs::ShootCmd shoot_cmd_;

  //! perception information
  Blackboard* const blackboard_;

};
}


#endif //ROBORTS_DECISION_SHOOT_BEHAVIOR_H
