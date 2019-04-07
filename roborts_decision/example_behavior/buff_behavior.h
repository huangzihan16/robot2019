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
      chassis_executor_(chassis_executor), blackboard_(blackboard), have_execute_(false){ }

  void Run() {
    if(!have_execute_){
      chassis_executor_->Execute(blackboard_->GetGuardGoal());
			have_execute_ = true;
    }
  }

  void Cancel() {
		have_execute_ = false;
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
		if (blackboard_->GetBonusStatus() == BonusStatus::OCCUPIED) {
			chassis_executor_->Cancel();
			
			have_execute_ = false;
			return BehaviorState::SUCCESS;
		} else {
			BehaviorState state = chassis_executor_->Update();
			return state;
		}
  }

  ~GainBuffBehavior() = default;

public:
		bool have_execute_;
 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
};


}

#endif //ROBORTS_DECISION_BUFF_BEHAVIOR_H