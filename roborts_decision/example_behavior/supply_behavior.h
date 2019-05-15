#ifndef ROBORTS_DECISION_SUPPLY_BEHAVIOR_H
#define ROBORTS_DECISION_SUPPLY_BEHAVIOR_H

#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"

namespace roborts_decision {
class SupplyBehavior {
public:
	SupplyBehavior(Blackboard* &blackboard): blackboard_(blackboard), have_applicated_(false), status_(0){}
	
	void Run() {
		blackboard_->SuggestGimbalPatrol();
		blackboard_->PublishPartnerInformation();
		if (!have_applicated_){
			have_applicated_ = true;
			if(blackboard_->GetSupplierStatus() == roborts_decision::SupplierStatus::CLOSE) 
				blackboard_->SendSupplyCmd();//send command
		}
  }

  void Cancel() {
		have_applicated_ = false;
  }

  BehaviorState Update() {
	  if (blackboard_->GetSupplierStatus() == roborts_decision::SupplierStatus::PREPARING)
	  	status_ = 1;
	  if (blackboard_->GetSupplierStatus() == roborts_decision::SupplierStatus::SUPPLYING)
	  	status_ = 2;
		if (!have_applicated_) {
			return BehaviorState::IDLE;
		}

		ros::Duration time_past = ros::Time::now() - application_time_;
		if (time_past.toSec() >= 5 || (status_ == 2 && blackboard_->GetSupplierStatus() == roborts_decision::SupplierStatus::CLOSE)) {
			blackboard_->AddSupplyNum();
			status_ = 0;
			have_applicated_ = false;
			std::cout << "supply_number " << blackboard_->supply_number_ << std::endl;
			return BehaviorState::SUCCESS;
		}
		else 
			return BehaviorState::RUNNING;

        // if (status_ == 2 && blackboard_->GetSupplierStatus() == roborts_decision::SupplierStatus::CLOSE) {
		// 	blackboard_->AddSupplyNum();
		// 	status_ = 0;
		// 	have_applicated_ = false;
		// 	return BehaviorState::SUCCESS;
		// }
		// else
		// 	return BehaviorState::RUNNING;
  }

  ~SupplyBehavior() = default;
	
public:
	ros::Time application_time_;
	int status_;
 private:
  //! perception information
  Blackboard* const blackboard_;
	
	bool have_applicated_;
};
}

#endif