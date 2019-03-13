#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "executor/gimbal_executor.h"
#include "executor/shoot_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/shoot_behavior.h"
#include "behavior_tree/behavior_tree.h"
#include "behavior_tree/behavior_node.h"
#include "blackboard/blackboard.h"
#include "behavior_tree//action_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto gimbal_executor = new roborts_decision::GimbalExecutor;
  auto shoot_executor = new roborts_decision::ShootExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);
  //auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(full_path);
 // auto root_node = new roborts_decision::SequenceNode("seq1", blackboard_ptr_);
  std::shared_ptr<roborts_decision::Blackboard> blackboard_ptr_(new roborts_decision::Blackboard(full_path));

  

 // behavior
  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, gimbal_executor, blackboard, full_path);
  roborts_decision::GoalBehavior         goal_behavior(chassis_executor, blackboard);
  roborts_decision::ShootBehavior        shoot_behavior(shoot_executor, blackboard);
 
 //action
  auto patrol_action_ = std::make_shared<roborts_decision::PatrolAction>(blackboard_ptr_, patrol_behavior);
  auto back_boot_area_action_ = std::make_shared<roborts_decision::BackBootAreaAction>(blackboard_ptr_, back_boot_area_behavior);
  auto chase_action_ = std::make_shared<roborts_decision::ChaseAction>(blackboard_ptr_, chase_behavior);
  auto search_action_ = std::make_shared<roborts_decision::SearchAction>(blackboard_ptr_, search_behavior);
  auto escape_action_ = std::make_shared<roborts_decision::EscapeAction>(blackboard_ptr_, escape_behavior);
  auto goal_action_ = std::make_shared<roborts_decision::GoalAction>(blackboard_ptr_, goal_behavior);
  auto shoot_action_ = std::make_shared<roborts_decision::ShootAction>(blackboard_ptr_, shoot_behavior);

  //
  
  std::shared_ptr<roborts_decision::SelectorNode> root_node(new roborts_decision::SelectorNode("root_selector", blackboard_ptr_));

  std::shared_ptr<roborts_decision::PreconditionNode> patrol_condition_(new roborts_decision::PreconditionNode("patrol condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsEnemyDetected()) {
                                                                                                 return false;
                                                                                               } else {
                                                                                                 return true;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  
  std::shared_ptr<roborts_decision::PreconditionNode> shoot_selector_condition_(new roborts_decision::PreconditionNode("shoot_selector_condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsEnemyDetected()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));

  std::shared_ptr<roborts_decision::SelectorNode> shoot_selector(new roborts_decision::SelectorNode("shoot_selector", blackboard_ptr_));
  std::shared_ptr<roborts_decision::PreconditionNode> escape_condition_(new roborts_decision::PreconditionNode("escape_condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsBulletLeft()) {
                                                                                                 return false;
                                                                                               } else {
                                                                                                 return true;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  
  std::shared_ptr<roborts_decision::PreconditionNode> shoot_condition_(new roborts_decision::PreconditionNode("patrol_condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsBulletLeft()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));

  roborts_decision::BehaviorTree behaviortree(root_node,30);
  root_node->AddChildren(shoot_selector_condition_);
	root_node->AddChildren(patrol_condition_);

  shoot_selector_condition_->SetChild(shoot_selector);
  shoot_selector->AddChildren(escape_condition_);
  shoot_selector->AddChildren(shoot_condition_);


  escape_condition_->SetChild(goal_action_);
  shoot_condition_->SetChild(shoot_action_);
	
	patrol_condition_->SetChild(patrol_action_);
  ros::Rate rate(10);

  while(ros::ok()){
    ros::spinOnce();
    behaviortree.Run();
    rate.sleep();
  }

  return 0;
}