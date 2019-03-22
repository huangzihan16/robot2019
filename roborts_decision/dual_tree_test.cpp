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
#include "example_behavior/round_behavior.h"
#include "example_behavior/support_behavior.h"

#include "behavior_tree/behavior_tree.h"
#include "behavior_tree/behavior_node.h"
#include "blackboard/blackboard.h"
#include "behavior_tree/action_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "dual_behavior_tree_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto gimbal_executor = new roborts_decision::GimbalExecutor;
  auto shoot_executor = new roborts_decision::ShootExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);
	
  std::shared_ptr<roborts_decision::Blackboard> blackboard_ptr_(blackboard);


 // behavior
  roborts_decision::PatrolBehavior        patrol_behavior(chassis_executor, gimbal_executor, blackboard, full_path);
  roborts_decision::SupportBehavior       support_behavior(chassis_executor, blackboard);
  roborts_decision::ShootBehavior         shoot_behavior(shoot_executor, chassis_executor, blackboard, full_path);
  roborts_decision::SupplyGoalBehavior    supply_goal_behavior(chassis_executor, blackboard);
  roborts_decision::SupplyBehavior        supply_application_behavior(blackboard);
	roborts_decision::GainBuffGoalBehavior  gain_buff_goal_behavior(chassis_executor, blackboard);
	roborts_decision::RoundBehavior         gain_buff_round_behavior(chassis_executor, blackboard, 2);
  roborts_decision::RoundBehavior         guard_behavior(chassis_executor, blackboard, 10);
	

 //action
  auto patrol_action_ = std::make_shared<roborts_decision::PatrolAction>(blackboard_ptr_, patrol_behavior);
  auto support_action_ = std::make_shared<roborts_decision::SupportAction>(blackboard_ptr_, support_behavior);
  auto shoot_action_ = std::make_shared<roborts_decision::ShootAction>(blackboard_ptr_, shoot_behavior);
	auto supply_goal_action_ = std::make_shared<roborts_decision::SupplyGoalAction>(blackboard_ptr_, supply_goal_behavior);
  auto supply_application_action_ = std::make_shared<roborts_decision::SupplyApplicateNode>(blackboard_ptr_, supply_application_behavior);
  auto gain_buff_goal_action_ = std::make_shared<roborts_decision::GainBuffGoalAction>(blackboard_ptr_, gain_buff_goal_behavior);
  auto gain_buff_guard_action_ = std::make_shared<roborts_decision::GainBuffGuardAction>(blackboard_ptr_, gain_buff_round_behavior);
  auto guard_action_ = std::make_shared<roborts_decision::GuardAction>(blackboard_ptr_, guard_behavior);

  //
  
	
  std::shared_ptr<roborts_decision::SelectorNode> root_node(new roborts_decision::SelectorNode("root_selector", blackboard_ptr_));
  std::shared_ptr<roborts_decision::PreconditionNode> master_condition_(new roborts_decision::PreconditionNode("master_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->IsMasterCondition()) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::SelectorNode> master_node(new roborts_decision::SelectorNode("master_selector", blackboard_ptr_));                                                                                            
  root_node->AddChildren(master_condition_);
  master_condition_->SetChild(master_node);
  std::shared_ptr<roborts_decision::PreconditionNode> slave_condition_(new roborts_decision::PreconditionNode("slave_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->IsMasterCondition()) {
																																																	return false;
																																																} else {
																																																	return true;
																																																}
																																															} , roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::SelectorNode> slave_node(new roborts_decision::SelectorNode("slave_selector", blackboard_ptr_));                                                                                            
  root_node->AddChildren(slave_condition_);
  slave_condition_->SetChild(slave_node);


  //master robot
  //master robot supply
  std::shared_ptr<roborts_decision::PreconditionNode> master_supply_condition_(new roborts_decision::PreconditionNode("master_supply_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->IsSupplyCondition()) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::BOTH));
	std::shared_ptr<roborts_decision::SequenceNode> master_supply_sequence(new roborts_decision::SequenceNode("master_supply", blackboard_ptr_));
  master_node->AddChildren(master_supply_condition_);
  master_supply_condition_->SetChild(master_supply_sequence);
	master_supply_sequence->AddChildren(supply_goal_action_);
	master_supply_sequence->AddChildren(supply_application_action_);
  //master gain buff
  std::shared_ptr<roborts_decision::PreconditionNode> master_gain_buff_condition_(new roborts_decision::PreconditionNode("master_gain_buff_condition_",blackboard_ptr_,
																																							[&]() {
																																								if (!blackboard_ptr_->IsSupplyCondition() && blackboard_ptr_->IsGainBuffCondition()) {
																																									return true;
																																								} else {
																																									return false;
																																								}
																																							} , roborts_decision::AbortType::BOTH));

	master_node->AddChildren(master_gain_buff_condition_);
  master_gain_buff_condition_->SetChild(patrol_action_);

  //master robot patrol
  std::shared_ptr<roborts_decision::PreconditionNode> master_patrol_condition_(new roborts_decision::PreconditionNode("master_patrol condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsEnemyDetected() || blackboard_ptr_->IsPartnerDetectEnemy()) {
                                                                                                 return false;
                                                                                               } else {
                                                                                                 return true;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  master_node->AddChildren(master_patrol_condition_);
  master_patrol_condition_->SetChild(patrol_action_);
  //master robot shoot and guard
  std::shared_ptr<roborts_decision::PreconditionNode> master_shoot_selector_condition_(new roborts_decision::PreconditionNode("master_shoot_selector_condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsEnemyDetected() || blackboard_ptr_->IsPartnerDetectEnemy ()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::SelectorNode> master_enemy_selector(new roborts_decision::SelectorNode("master_enemy_selector", blackboard_ptr_));                                                                                           
  master_node->AddChildren(master_shoot_selector_condition_);
  master_shoot_selector_condition_->SetChild(master_enemy_selector);
  
  //master robot enemy_detected_condition
  std::shared_ptr<roborts_decision::PreconditionNode> master_enemy_detected_condition_(new roborts_decision::PreconditionNode("master_enemy_detected_condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsEnemyDetected()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::SelectorNode> master_shoot_selector(new roborts_decision::SelectorNode("master_shoot_selector", blackboard_ptr_));
  std::shared_ptr<roborts_decision::PreconditionNode> master_shoot_condition_(new roborts_decision::PreconditionNode("master_shoot condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsBulletLeft()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::PreconditionNode> master_guard_condition_(new roborts_decision::PreconditionNode("master_guard condition", blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->IsBulletLeft()) {
																																																	return false;
																																																} else {
																																																	return true;
																																																}
																																															}, roborts_decision::AbortType::BOTH));        
  master_enemy_selector->AddChildren(master_enemy_detected_condition_);
  master_enemy_detected_condition_->SetChild(master_shoot_selector);
  master_shoot_selector->AddChildren(master_shoot_condition_);
  master_shoot_selector->AddChildren(master_guard_condition_);
	master_shoot_condition_->SetChild(shoot_action_);
  master_guard_condition_->SetChild(guard_action_);


  std::shared_ptr<roborts_decision::PreconditionNode> master_slave_enemy_detected_condition_(new roborts_decision::PreconditionNode("master_slave_enemy_detected_condition_",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsPartnerDetectEnemy()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  
  
  master_enemy_selector->AddChildren(master_slave_enemy_detected_condition_);
  master_slave_enemy_detected_condition_->SetChild(support_action_);	  


  //slave robot
  //slave robot supply
  std::shared_ptr<roborts_decision::PreconditionNode> slave_supply_condition_(new roborts_decision::PreconditionNode("slave_supply_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->IsMasterSupplyCondition()) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::BOTH));
  slave_node->AddChildren(slave_supply_condition_);
  slave_supply_condition_->SetChild(patrol_action_);

  //slave robot gain buff
  std::shared_ptr<roborts_decision::PreconditionNode> slave_gain_buff_condition_(new roborts_decision::PreconditionNode("slave_gain_buff_condition_",blackboard_ptr_,
																																							[&]() {
																																								if (!blackboard_ptr_->IsMasterSupplyCondition() && blackboard_ptr_->IsMasterGainBuffCondition()) {
																																									return true;
																																								} else {
																																									return false;
																																								}
																																							} , roborts_decision::AbortType::BOTH));
	std::shared_ptr<roborts_decision::SequenceNode> slave_gain_buff_sequence(new roborts_decision::SequenceNode("slave_gain_buff", blackboard_ptr_));
	
  slave_node->AddChildren(slave_gain_buff_condition_);
  slave_gain_buff_condition_->SetChild(slave_gain_buff_sequence);
	slave_gain_buff_sequence->AddChildren(gain_buff_goal_action_);
	slave_gain_buff_sequence->AddChildren(gain_buff_guard_action_);

  //slave robot patrol
  std::shared_ptr<roborts_decision::PreconditionNode> slave_patrol_condition_(new roborts_decision::PreconditionNode("slave_patrol condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsEnemyDetected() || blackboard_ptr_->IsPartnerDetectEnemy()) {
                                                                                                 return false;
                                                                                               } else {
                                                                                                 return true;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  slave_node->AddChildren(slave_patrol_condition_);
  slave_patrol_condition_->SetChild(patrol_action_);
  
  //slave robot shoot and guard
//slave robot shoot and guard
  std::shared_ptr<roborts_decision::PreconditionNode> slave_shoot_selector_condition_(new roborts_decision::PreconditionNode("slave_shoot_selector_condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsEnemyDetected() || blackboard_ptr_->IsPartnerDetectEnemy()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::SelectorNode> slave_enemy_selector(new roborts_decision::SelectorNode("slave_enemy_selector", blackboard_ptr_));                                                                                           
  slave_node->AddChildren(slave_shoot_selector_condition_);
  slave_shoot_selector_condition_->SetChild(slave_enemy_selector);
  
  //slave robot enemy_detected_condition
  std::shared_ptr<roborts_decision::PreconditionNode> slave_enemy_detected_condition_(new roborts_decision::PreconditionNode("slave_enemy_detected_condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsEnemyDetected()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::SelectorNode> slave_shoot_selector(new roborts_decision::SelectorNode("slave_shoot_selector", blackboard_ptr_));
  std::shared_ptr<roborts_decision::PreconditionNode> slave_shoot_condition_(new roborts_decision::PreconditionNode("slave_shoot condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsBulletLeft()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::PreconditionNode> slave_guard_condition_(new roborts_decision::PreconditionNode("slave_guard condition", blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->IsBulletLeft()) {
																																																	return false;
																																																} else {
																																																	return true;
																																																}
																																															}, roborts_decision::AbortType::BOTH));        
  slave_enemy_selector->AddChildren(slave_enemy_detected_condition_);
  slave_enemy_detected_condition_->SetChild(slave_shoot_selector);
  slave_shoot_selector->AddChildren(slave_shoot_condition_);
  slave_shoot_selector->AddChildren(slave_guard_condition_);
	slave_shoot_condition_->SetChild(shoot_action_);
  slave_guard_condition_->SetChild(guard_action_);


  std::shared_ptr<roborts_decision::PreconditionNode> slave_master_enemy_detected_condition_(new roborts_decision::PreconditionNode("slave_master_enemy_detected_condition_",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsPartnerDetectEnemy()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  
  slave_enemy_selector->AddChildren(slave_master_enemy_detected_condition_);
  slave_master_enemy_detected_condition_->SetChild(support_action_);	  


  roborts_decision::BehaviorTree behaviortree(root_node, 20);

  while(1)
    behaviortree.Run();
  
  ros::waitForShutdown();
  return 0;
}