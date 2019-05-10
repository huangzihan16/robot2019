#ifndef ROBORTS_DECISION_SUPPLY_BEHAVIOR_H
#define ROBORTS_DECISION_SUPPLY_BEHAVIOR_H

#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"

namespace roborts_decision {
class SupplyBehavior {
public:
	SupplyBehavior(Blackboard* &blackboard): blackboard_(blackboard), have_applicated_(false){}
	
	void Run() {
		if (!have_applicated_){
			have_applicated_ = true;
			if(blackboard_->GetSupplierStatus() == roborts_decision::SupplierStatus::PREPARING) 
				blackboard_->SendSupply50Cmd();//send command
		}
			
		std::cout << "Supplying..." << std::endl;
  }

  void Cancel() {
		have_applicated_ = false;
  }

  BehaviorState Update() {
		if (!have_applicated_)
			return BehaviorState::IDLE;
		
		ros::Duration time_past = ros::Time::now() - application_time_;
		if (time_past.toSec() >= 2) {
			blackboard_->AddSupplyNum();
			std::cout << "supply_number " << blackboard_->supply_number_ << std::endl;
			return BehaviorState::SUCCESS;
		}
		else 
			return BehaviorState::RUNNING;

        // if (blackboard_->GetSupplierStatus() == roborts_decision::SupplierStatus::CLOSE) {
		// 	blackboard_->AddSupplyNum();
		// 	std::cout << "supply_number " << blackboard_->supply_number_ << std::endl;
		// 	return BehaviorState::SUCCESS;
		// }
		// else
		// 	return BehaviorState::RUNNING;
  }

  ~SupplyBehavior() = default;
	
public:
	ros::Time application_time_;
 private:
  //! perception information
  Blackboard* const blackboard_;
	
	bool have_applicated_;
};
}

#endif