#ifndef ROBORTS_DECISION_GETOUTFROMSTUCK_BEHAVIOR_H
#define ROBORTS_DECISION_GETOUTFROMSTUCK_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class GetOutFromStuckBehavior {
 public:
  GetOutFromStuckBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {
    front_speed_.linear.x = 1;
    front_speed_.linear.y = 0;
    front_speed_.linear.z = 0;
	front_speed_.angular.x = 0;
	front_speed_.angular.y = 0;
	front_speed_.angular.z = 0;

    back_speed_.linear.x = -1;
    back_speed_.linear.y = 0;
    back_speed_.linear.z = 0;
	back_speed_.angular.x = 0;
	back_speed_.angular.y = 0;
	back_speed_.angular.z = 0;

    left_speed_.linear.x = 0;
    left_speed_.linear.y = 1;
    left_speed_.linear.z = 0;
	left_speed_.angular.x = 0;
	left_speed_.angular.y = 0;
	left_speed_.angular.z = 0;

    right_speed_.linear.x = 0;
    right_speed_.linear.y = -1;
    right_speed_.linear.z = 0;
	right_speed_.angular.x = 0;
	right_speed_.angular.y = 0;
	right_speed_.angular.z = 0;
  }

  void Run() {
    blackboard_->partner_msg_pub_.status = (char)PartnerStatus::STOP;
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();
    int diff_between_leftright = (int)blackboard_->left_cell_cost_ - (int)blackboard_->right_cell_cost_;
    int diff_between_frontback = (int)blackboard_->front_cell_cost_ - (int)blackboard_->back_cell_cost_;
   
    if (blackboard_->back_cell_cost_ == 254)
        chassis_executor_->Execute(front_speed_);
    else if (blackboard_->left_cell_cost_ == 254)
        chassis_executor_->Execute(right_speed_);
    else if (blackboard_->right_cell_cost_ == 254)
        chassis_executor_->Execute(left_speed_);
    else if (blackboard_->front_cell_cost_ == 254)
        chassis_executor_->Execute(back_speed_);
    else {
        if (abs(diff_between_leftright) > abs(diff_between_frontback)) {
            if (blackboard_->left_cell_cost_ < blackboard_->right_cell_cost_)
                chassis_executor_->Execute(left_speed_);
            else
                chassis_executor_->Execute(right_speed_);
        } else {
            if (blackboard_->front_cell_cost_ < blackboard_->back_cell_cost_)
                chassis_executor_->Execute(front_speed_);
            else
                chassis_executor_->Execute(back_speed_);
        }
    }
    
    
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~GetOutFromStuckBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! candidate speed twist
  geometry_msgs::Twist front_speed_;
  geometry_msgs::Twist back_speed_;
  geometry_msgs::Twist left_speed_;
  geometry_msgs::Twist right_speed_;

};
}


#endif //ROBORTS_DECISION_GETOUTFROMSTUCK_BEHAVIOR_H
