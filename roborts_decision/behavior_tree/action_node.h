#ifndef ROBORTS_DECISION_ACTION_BEHAVIOR_H
#define ROBORTS_DECISION_ACTION_BEHAVIOR_H


#include <ros/ros.h>

#include "../executor/chassis_executor.h"

#include "../example_behavior/back_boot_area_behavior.h"
#include "../example_behavior/escape_behavior.h"
#include "../example_behavior/chase_behavior.h"
#include "../example_behavior/search_behavior.h"
#include "../example_behavior/patrol_behavior.h"
#include "../example_behavior/goal_behavior.h"

#include "../example_behavior/shoot_behavior.h"

#include "../behavior_tree/behavior_tree.h"
#include "../behavior_tree/behavior_node.h"
#include "../blackboard/blackboard.h"


namespace roborts_decision {
class PatrolAction : public ActionNode {
 public:
  PatrolAction(const Blackboard::Ptr &blackboard_ptr, PatrolBehavior &patrol_behavior) :
      ActionNode::ActionNode("patrol_behavior", blackboard_ptr), patrol_behavior_(patrol_behavior){

  }

  virtual ~PatrolAction() = default;

 private:
  virtual void OnInitialize() {
    std::cout << "PatrolAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    patrol_behavior_.Run();
	
    return patrol_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				patrol_behavior_.Cancel();
        std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        std::cout << "FAILURE" << std::endl;
        break;
      default:
        std::cout << "ERROR" << std::endl;
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
    std::cout << "BackBootAreaAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    back_boot_area_behavior_.Run();

    return back_boot_area_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				back_boot_area_behavior_.Cancel();
        std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        std::cout << "FAILURE" << std::endl;
        break;
      default:
        std::cout << "ERROR" << std::endl;
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
    std::cout << "ChaseAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    chase_behavior_.Run();

    return chase_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				chase_behavior_.Cancel();
        std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        std::cout << "FAILURE" << std::endl;
        break;
      default:
        std::cout << "ERROR" << std::endl;
        return;
    }
  }

  ChaseBehavior chase_behavior_;

}; // class ChaseAction


class SearchAction : public ActionNode {
 public:
  SearchAction(const Blackboard::Ptr &blackboard_ptr, SearchBehavior &search_behavior) :
      ActionNode::ActionNode("chase_behavior", blackboard_ptr), search_behavior_(search_behavior){

  }

  virtual ~SearchAction() = default;

 private:
  virtual void OnInitialize() {
    std::cout << "SearchAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    search_behavior_.Run();

    return search_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				search_behavior_.Cancel();
        std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        std::cout << "FAILURE" << std::endl;
        break;
      default:
        std::cout << "ERROR" << std::endl;
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
    std::cout << "EscapeAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    escape_behavior_.Run();

    return escape_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				escape_behavior_.Cancel();
        std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        std::cout << "FAILURE" << std::endl;
        break;
      default:
        std::cout << "ERROR" << std::endl;
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
    std::cout << "GoalAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    goal_behavior_.Run();

    return goal_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				goal_behavior_.Cancel();
        std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        std::cout << "FAILURE" << std::endl;
        break;
      default:
        std::cout << "ERROR" << std::endl;
        return;
    }
  }

  GoalBehavior goal_behavior_;

}; // class GoalAction

class ShootAction : public ActionNode {
 public:
  ShootAction(const Blackboard::Ptr &blackboard_ptr, ShootBehavior &shoot_behavior) :
      ActionNode::ActionNode("shoot_behavior", blackboard_ptr), shoot_behavior_(shoot_behavior){

  }

  virtual ~ShootAction() = default;

 private:
  virtual void OnInitialize() {
    std::cout << "ShootAction OnInitialize" << std::endl;
  };

  virtual BehaviorState Update() {
    shoot_behavior_.Run();

    return shoot_behavior_.Update();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
				shoot_behavior_.Cancel();
        std::cout << "IDLE" << std::endl;
        break;
      case BehaviorState::SUCCESS:
        std::cout << "SUCCESS" << std::endl;
        break;
      case BehaviorState::FAILURE:
        std::cout << "FAILURE" << std::endl;
        break;
      default:
        std::cout << "ERROR" << std::endl;
        return;
    }
  }

  ShootBehavior shoot_behavior_;

};// class ShootAction



}//namespace roborts_decision
#endif //ROBORTS_DECISION_ACTION_BEHAVIOR_H