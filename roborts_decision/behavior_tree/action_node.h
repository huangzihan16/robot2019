#ifndef ROBORTS_DECISION_ACTION_BEHAVIOR_H
#define ROBORTS_DECISION_ACTION_BEHAVIOR_H


#include <ros/ros.h>

#include "../executor/chassis_executor.h"

#include "../example_behavior/back_boot_area_behavior.h"
#include "../example_behavior/escape_behavior.h"
#include "../example_behavior/chase_behavior.h"
#include "../example_behavior/buff_chase_behavior.h"

#include "../example_behavior/search_behavior.h"
#include "../example_behavior/patrol_behavior.h"
#include "../example_behavior/goal_behavior.h"

#include "../example_behavior/shoot_behavior.h"
#include "../example_behavior/supply_behavior.h"
#include "../example_behavior/round_behavior.h"
#include "../example_behavior/support_behavior.h"


#include "../behavior_tree/behavior_tree.h"
#include "../behavior_tree/behavior_node.h"
#include "../blackboard/blackboard.h"
/**************************************************/
#include "../example_behavior/turntohurt_behavior.h"
#include "../example_behavior/buff_behavior.h"
/**************************************************/
#include "../example_behavior/getoutfromstuck_behavior.h"

namespace roborts_decision {
class PatrolAction : public ActionNode {
 public:
  PatrolAction(const Blackboard::Ptr &blackboard_ptr, PatrolBehavior &patrol_behavior) :
      ActionNode::ActionNode("patrol_behavior", blackboard_ptr), patrol_behavior_(patrol_behavior){

  }

  virtual ~PatrolAction() = default;

 private:
  virtual void OnInitialize() {
    // //  std::cout << "PatrolAction OnInitialize" << std::endl;
		// patrol_behavior_.have_time_ = true;
		// patrol_behavior_.start_time_ = ros::Time::now();
  };

  virtual BehaviorState Update() {
    patrol_behavior_.Run();
    ROS_INFO(" patrol_behavior_ patrol_behavior_ !");

    return patrol_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				patrol_behavior_.Cancel();
        // //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        // //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  PatrolBehavior patrol_behavior_;

}; // class PatrolAction

class BackBootAreaAction : public ActionNode {
 public:
  BackBootAreaAction(const Blackboard::Ptr &blackboard_ptr, BackBootAreaBehavior &back_boot_area_behavior) :
      ActionNode::ActionNode("back_boot_area_behavior", blackboard_ptr), back_boot_area_behavior_(back_boot_area_behavior){

  }

  virtual ~BackBootAreaAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "BackBootAreaAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    back_boot_area_behavior_.Run();
    ROS_INFO(" back_boot_area_behavior_ back_boot_area_behavior_ !");

    return back_boot_area_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				back_boot_area_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  BackBootAreaBehavior back_boot_area_behavior_;

}; // class BackBootAreaAction

class ChaseAction : public ActionNode {
 public:
  ChaseAction(const Blackboard::Ptr &blackboard_ptr, ChaseBehavior &chase_behavior) :
      ActionNode::ActionNode("chase_behavior", blackboard_ptr), chase_behavior_(chase_behavior){

  }

  virtual ~ChaseAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "ChaseAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    chase_behavior_.Run();
    ROS_INFO(" chase_behavior_ chase_behavior_ !");

    return chase_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				chase_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  ChaseBehavior chase_behavior_;

}; // class ChaseAction

class BuffChaseAction : public ActionNode {
 public:
  BuffChaseAction(const Blackboard::Ptr &blackboard_ptr, BuffChaseBehavior &buff_chase_behavior) :
      ActionNode::ActionNode("buff_chase_behavior", blackboard_ptr), buff_chase_behavior_(buff_chase_behavior){

  }

  virtual ~BuffChaseAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "BuffChaseAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    buff_chase_behavior_.Run();
    ROS_INFO(" buff_chase_behavior_ buff_chase_behavior_ !");

    return buff_chase_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				buff_chase_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  BuffChaseBehavior buff_chase_behavior_;

}; // class BuffChaseAction

class SearchAction : public ActionNode {
 public:
  SearchAction(const Blackboard::Ptr &blackboard_ptr, SearchBehavior &search_behavior) :
      ActionNode::ActionNode("chase_behavior", blackboard_ptr), search_behavior_(search_behavior){

  }

  virtual ~SearchAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "SearchAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    search_behavior_.Run();
    ROS_INFO(" search_behavior_ search_behavior_ !");
    return search_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				search_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  SearchBehavior search_behavior_;

}; // class SearchAction


class EscapeAction : public ActionNode {
 public:
  EscapeAction(const Blackboard::Ptr &blackboard_ptr, EscapeBehavior &escape_behavior) :
      ActionNode::ActionNode("escape_behavior", blackboard_ptr), escape_behavior_(escape_behavior){

  }

  virtual ~EscapeAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "EscapeAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    escape_behavior_.Run();
    ROS_INFO(" escape_behavior_ escape_behavior_ !");
    return escape_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				escape_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  EscapeBehavior &escape_behavior_;

}; // class EscapeAction


class GoalAction : public ActionNode {
 public:
  GoalAction(const Blackboard::Ptr &blackboard_ptr, GoalBehavior &goal_behavior) :
      ActionNode::ActionNode("goal_behavior", blackboard_ptr), goal_behavior_(goal_behavior){

  }

  virtual ~GoalAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "GoalAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    goal_behavior_.Run();
    ROS_INFO(" goal_behavior_ goal_behavior_ !");
    return goal_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				goal_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  GoalBehavior goal_behavior_;

}; // class GoalAction

class SupplyGoalAction : public ActionNode {
 public:
  SupplyGoalAction(const Blackboard::Ptr &blackboard_ptr, SupplyGoalBehavior &supply_goal_behavior) :
      ActionNode::ActionNode("supply_goal_behavior", blackboard_ptr), supply_goal_behavior_(supply_goal_behavior){

  }

  virtual ~SupplyGoalAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "SupplyGoalAction OnInitialize" << std::endl;
		supply_goal_behavior_.have_execute_ = false;
  };

  virtual BehaviorState Update() {
    supply_goal_behavior_.Run();
    ROS_INFO(" supply_goal_behavior_ supply_goal_behavior_ !");
    return supply_goal_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				supply_goal_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  SupplyGoalBehavior supply_goal_behavior_;
};

class GainBuffGoalAction : public ActionNode {
 public:
  GainBuffGoalAction(const Blackboard::Ptr &blackboard_ptr, GainBuffGoalBehavior &buff_goal_behavior) :
      ActionNode::ActionNode("gain_buff_goal_behavior", blackboard_ptr), buff_goal_behavior_(buff_goal_behavior){
  }

  virtual ~GainBuffGoalAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "GainBuffGoalAction OnInitialize" << std::endl;
		buff_goal_behavior_.have_execute_ = false;
  };

  virtual BehaviorState Update() {
    buff_goal_behavior_.Run();
    ROS_INFO(" buff_goal_behavior_ buff_goal_behavior_ !");
    return buff_goal_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				buff_goal_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  GainBuffGoalBehavior buff_goal_behavior_;
};

class GainBuffAction : public ActionNode {
 public:
  GainBuffAction(const Blackboard::Ptr &blackboard_ptr, GainBuffBehavior &buff_behavior) :
      ActionNode::ActionNode("gain_buff_behavior", blackboard_ptr), buff_behavior_(buff_behavior){
  }

  virtual ~GainBuffAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "GainBuffAction OnInitialize" << std::endl;
		buff_behavior_.have_execute_ = false;
  };

  virtual BehaviorState Update() {
    buff_behavior_.Run();
    ROS_INFO(" buff_behavior_ buff_behavior_ !");
    return buff_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				buff_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  GainBuffBehavior buff_behavior_;
};

class ShootAction : public ActionNode {
 public:
  ShootAction(const Blackboard::Ptr &blackboard_ptr, ShootBehavior &shoot_behavior) :
      ActionNode::ActionNode("shoot_behavior", blackboard_ptr), shoot_behavior_(shoot_behavior){

  }

  virtual ~ShootAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "ShootAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    shoot_behavior_.Run();

    return shoot_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				shoot_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  ShootBehavior shoot_behavior_;

};// class ShootAction

class SupplyApplicateNode : public ActionNode {
 public:
  SupplyApplicateNode(const Blackboard::Ptr &blackboard_ptr, SupplyBehavior &supply_application_behavior) :
      ActionNode::ActionNode("supply_application_behavior", blackboard_ptr), supply_application_behavior_(supply_application_behavior){

  }

  virtual ~SupplyApplicateNode() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "SupplyApplicateNode OnInitialize" << std::endl;
		supply_application_behavior_.application_time_ = ros::Time::now();
  };

  virtual BehaviorState Update() {
    supply_application_behavior_.Run();
    ROS_INFO(" supply_application_behavior_ supply_application_behavior_ !");
    return supply_application_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				supply_application_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  SupplyBehavior supply_application_behavior_;

};

class GainBuffGuardAction : public ActionNode {
 public:
  GainBuffGuardAction(const Blackboard::Ptr &blackboard_ptr, RoundBehavior &round_behavior) :
      ActionNode::ActionNode("gain_buff_guard_behavior", blackboard_ptr), round_behavior_(round_behavior){
  }

  virtual ~GainBuffGuardAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "GainBuffGuardAction OnInitialize" << std::endl;
		start_time_ = ros::Time::now();

  };

  virtual BehaviorState Update() {
		round_behavior_.Run();
    ROS_INFO(" round_behavior_ round_behavior_ !");
		//  std::cout << "Buff Gainning" << std::endl;
		ros::Duration time_past = ros::Time::now() - start_time_;
		if (time_past.toSec() >= 2) {
			blackboard_ptr_->AddGainBuffNum();
			round_behavior_.Cancel();
			return BehaviorState::SUCCESS;
		}
		else
			return BehaviorState::RUNNING;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				round_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  RoundBehavior round_behavior_;
	ros::Time start_time_;
};

class GuardAction : public ActionNode {
 public:
  GuardAction(const Blackboard::Ptr &blackboard_ptr, RoundBehavior &round_behavior) :
      ActionNode::ActionNode("guard_behavior", blackboard_ptr), round_behavior_(round_behavior){
  }

  virtual ~GuardAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "GuardAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
			round_behavior_.Run();
    ROS_INFO(" round_behavior_ round_behavior_ !");
			return round_behavior_.Update();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				round_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  RoundBehavior round_behavior_;
};

class SupportAction : public ActionNode {
 public:
  SupportAction(const Blackboard::Ptr &blackboard_ptr, SupportBehavior &support_behavior) :
      ActionNode::ActionNode("support_behavior", blackboard_ptr), support_behavior_(support_behavior){

      }

  virtual ~SupportAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "SupportAction OnInitialize" << std::endl;
    support_behavior_.have_good_goal_ = false;
  };

  virtual BehaviorState Update() {
			support_behavior_.Run();
         ROS_INFO(" support_behavior_ support_behavior_ !"); 
			return support_behavior_.Update();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				support_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  SupportBehavior support_behavior_;
};

/**********************************************************/
class TurnToHurtAction : public ActionNode {
 public:
  TurnToHurtAction(const Blackboard::Ptr &blackboard_ptr, TurnToHurtBehavior &turn_to_hurt_behavior) :
      ActionNode::ActionNode("turn_to_hurt_behavior", blackboard_ptr), turn_to_hurt_behavior_(turn_to_hurt_behavior){

      }

  virtual ~TurnToHurtAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "TurnToHurtAction OnInitialize" << std::endl;
    turn_to_hurt_behavior_.initial_yaw_ = blackboard_ptr_->GetChassisYaw();
    turn_to_hurt_behavior_.initial_damage_source_ = blackboard_ptr_->GetDamageSource();
    
    switch (turn_to_hurt_behavior_.initial_damage_source_){
      case DamageSource::BACKWARD:
        turn_to_hurt_behavior_.initial_speed_ = 2 * M_PI;
        break;
      case DamageSource::LEFT:
        turn_to_hurt_behavior_.initial_speed_ = 2 * M_PI;
        break;
      case DamageSource::RIGHT:
        turn_to_hurt_behavior_.initial_speed_ = -2 * M_PI;
        break;
      default:
        turn_to_hurt_behavior_.initial_speed_ = 0;
    }
    turn_to_hurt_behavior_.StopTimesSetToZero();
  };

  virtual BehaviorState Update() {
			turn_to_hurt_behavior_.Run();
      ROS_INFO(" turn_to_hurt_behavior_ turn_to_hurt_behavior_ !"); 
			return turn_to_hurt_behavior_.Update();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				turn_to_hurt_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  TurnToHurtBehavior turn_to_hurt_behavior_;
};

class TurnBackAction : public ActionNode {
 public:
  TurnBackAction(const Blackboard::Ptr &blackboard_ptr, TurnBackBehavior &turn_back_behavior) :
      ActionNode::ActionNode("turn_back_behavior", blackboard_ptr), turn_back_behavior_(turn_back_behavior){

      }

  virtual ~TurnBackAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "TurnBackAction OnInitialize" << std::endl;
    turn_back_behavior_.initial_yaw_ = blackboard_ptr_->GetChassisYaw();
    turn_back_behavior_.StopTimesSetToZero();
  };

  virtual BehaviorState Update() {
			turn_back_behavior_.Run();
      ROS_INFO(" turn_back_behavior_ turn_back_behavior_ !"); 
			return turn_back_behavior_.Update();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				turn_back_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  TurnBackBehavior turn_back_behavior_;
};

/**********************************************************/
class GetOutFromStuckAction : public ActionNode {
 public:
  GetOutFromStuckAction(const Blackboard::Ptr &blackboard_ptr, GetOutFromStuckBehavior &getoutfromstuck_behavior) :
      ActionNode::ActionNode("getoutfromstuck_behavior", blackboard_ptr), getoutfromstuck_behavior_(getoutfromstuck_behavior){

      }

  virtual ~GetOutFromStuckAction() = default;

 private:
  virtual void OnInitialize() {
    //  std::cout << "GetOutFromStuckAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
			getoutfromstuck_behavior_.Run();
      ROS_INFO(" getoutfromstuck_behavior_ getoutfromstuck_behavior_ !"); 
			return getoutfromstuck_behavior_.Update();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				getoutfromstuck_behavior_.Cancel();
        //  std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        //  std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        //  std::cout << "FAILURE" << std::endl;
        break;
      default:
        //  std::cout << "ERROR" << std::endl;
        return;
    }
  }

  GetOutFromStuckBehavior getoutfromstuck_behavior_;
};



}//namespace roborts_decision
#endif //ROBORTS_DECISION_ACTION_BEHAVIOR_H
