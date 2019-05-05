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
/****************************************/
#include "example_behavior/turntohurt_behavior.h"
#include "example_behavior/buff_behavior.h"
/********************************************/
int main(int argc, char **argv) {
  ros::init(argc, argv, "dual_behavior_tree_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto gimbal_executor = new roborts_decision::GimbalExecutor;
  auto shoot_executor = new roborts_decision::ShootExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);
	
  std::shared_ptr<roborts_decision::Blackboard> blackboard_ptr_(blackboard);

 // behavior
  roborts_decision::PatrolBehavior        patrol_behavior_(chassis_executor, blackboard, full_path);
  roborts_decision::SupportBehavior       support_behavior_(chassis_executor, gimbal_executor, blackboard);
  roborts_decision::ShootBehavior         shoot_behavior_(shoot_executor, chassis_executor, blackboard, full_path);
  roborts_decision::SupplyGoalBehavior    supply_goal_behavior_(chassis_executor, blackboard);
  roborts_decision::SupplyBehavior        supply_application_behavior_(blackboard);
	roborts_decision::GainBuffGoalBehavior  gain_buff_goal_behavior_(chassis_executor, blackboard);
	roborts_decision::RoundBehavior         gain_buff_round_behavior_(chassis_executor, blackboard, 2);
  roborts_decision::RoundBehavior         guard_behavior_(chassis_executor, blackboard, 3);

/***************************************************************/
  roborts_decision::ChaseBehavior         chase_behavior_(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior         escape_behavior_(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior         search_behavior_(chassis_executor, gimbal_executor, blackboard, full_path);
  roborts_decision::GainBuffBehavior         gain_buff_behavior_(chassis_executor, blackboard);
  
  roborts_decision::BackBootAreaBehavior         back_boot_area_behavior_(chassis_executor, blackboard, full_path);
  roborts_decision::TurnToHurtBehavior         turn_to_hurt_behavior_(chassis_executor, blackboard);
  roborts_decision::TurnBackBehavior         turn_back_behavior_(chassis_executor, blackboard);
/***************************************************************/
 //action
  auto patrol_action_ = std::make_shared<roborts_decision::PatrolAction>(blackboard_ptr_, patrol_behavior_);
  auto support_action_ = std::make_shared<roborts_decision::SupportAction>(blackboard_ptr_, support_behavior_);
  auto shoot_action_ = std::make_shared<roborts_decision::ShootAction>(blackboard_ptr_, shoot_behavior_);
	auto supply_goal_action_ = std::make_shared<roborts_decision::SupplyGoalAction>(blackboard_ptr_, supply_goal_behavior_);
  auto supply_application_action_ = std::make_shared<roborts_decision::SupplyApplicateNode>(blackboard_ptr_, supply_application_behavior_);
  auto gain_buff_goal_action_ = std::make_shared<roborts_decision::GainBuffGoalAction>(blackboard_ptr_, gain_buff_goal_behavior_);
  auto gain_buff_action_ = std::make_shared<roborts_decision::GainBuffAction>(blackboard_ptr_, gain_buff_behavior_);
  auto guard_action_ = std::make_shared<roborts_decision::GuardAction>(blackboard_ptr_, guard_behavior_);

  /***************************************************************/

  auto chase_action_ = std::make_shared<roborts_decision::ChaseAction>(blackboard_ptr_, chase_behavior_);
  auto escape_action_ = std::make_shared<roborts_decision::EscapeAction>(blackboard_ptr_, escape_behavior_);
  auto search_action_ = std::make_shared<roborts_decision::SearchAction>(blackboard_ptr_, search_behavior_);
  auto wait_action_ = std::make_shared<roborts_decision::BackBootAreaAction>(blackboard_ptr_, back_boot_area_behavior_);
  auto turn_to_hurt_action_ = std::make_shared<roborts_decision::TurnToHurtAction>(blackboard_ptr_, turn_to_hurt_behavior_);
  auto turn_back_action_ = std::make_shared<roborts_decision::TurnBackAction>(blackboard_ptr_, turn_back_behavior_);
  /***************************************************************/

//  
	
  std::shared_ptr<roborts_decision::SelectorNode> root_node(new roborts_decision::SelectorNode("root_selector", blackboard_ptr_));
  //game stop
  std::shared_ptr<roborts_decision::PreconditionNode> game_stop_condition_(new roborts_decision::PreconditionNode("game_stop_condition",blackboard_ptr_,
																																															[&]() {return false;
																																																/*if (blackboard_ptr_->GetGameStatus()
                                                                                      != roborts_decision::GameStatus::ROUND) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}*/
																																															} , roborts_decision::AbortType::BOTH));
  root_node->AddChildren(game_stop_condition_);  
  game_stop_condition_->SetChild(wait_action_);   

  //game start
  std::shared_ptr<roborts_decision::SelectorNode> game_start_selector(new roborts_decision::SelectorNode("game_start_selector", blackboard_ptr_));                                                                                            
  root_node->AddChildren(game_start_selector);


  //game_start_selector
  // no bullet left
  std::shared_ptr<roborts_decision::PreconditionNode> no_bullet_left_condition_(new roborts_decision::PreconditionNode("no_bullet_left_condition",blackboard_ptr_,
																																															[&]() {if (blackboard_ptr_->IsSupplyCondition() && blackboard_ptr_->IsMasterCondition()) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
                                                                                                /*if (!blackboard_ptr_->IsBulletLeft()) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}*/
																																															} , roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::SelectorNode> no_bullet_left_selector(new roborts_decision::SelectorNode("no_bullet_left_selector", blackboard_ptr_));          
  game_start_selector->AddChildren(no_bullet_left_condition_);
  no_bullet_left_condition_->SetChild(no_bullet_left_selector);
  std::shared_ptr<roborts_decision::PreconditionNode> bullet_supply_condition_(new roborts_decision::PreconditionNode("bullet_supply_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->IsSupplyCondition() && blackboard_ptr_->IsMasterCondition()/* && blackboard_ptr_->GetSupplierStatus()
                                                                                                == roborts_decision::SupplierStatus::PREPARING*/) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::SequenceNode> supply_sequence(new roborts_decision::SequenceNode("supply", blackboard_ptr_));

  no_bullet_left_selector->AddChildren(bullet_supply_condition_);
  bullet_supply_condition_->SetChild(supply_sequence); 
  supply_sequence->AddChildren(supply_goal_action_);
	supply_sequence->AddChildren(supply_application_action_);

  // no_bullet_left_selector->AddChildren(guard_action_);


  //bullet left
  std::shared_ptr<roborts_decision::SelectorNode> bullet_left_selector(new roborts_decision::SelectorNode("bullet_left_selector", blackboard_ptr_));          
  game_start_selector->AddChildren(bullet_left_selector);

  
  //obtain buff
  std::shared_ptr<roborts_decision::PreconditionNode> obtain_buff_condition_(new roborts_decision::PreconditionNode("obtain_buff_condition",blackboard_ptr_,
																																															[&]() {return true;/*if (blackboard_ptr_->GetRobotBonus()) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}*/
																																															} , roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::SelectorNode> offensive_selector(new roborts_decision::SelectorNode("offensive_selector", blackboard_ptr_));          
  bullet_left_selector->AddChildren(obtain_buff_condition_);
  obtain_buff_condition_->SetChild(offensive_selector);  
  std::shared_ptr<roborts_decision::PreconditionNode> offensive_dmp_condition_(new roborts_decision::PreconditionNode("offensive_dmp_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->HurtedPerSecond() > 400) {
                                                                                                  ROS_INFO("DMP: %f",blackboard_ptr_->dmp_);
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::PreconditionNode> offensive_detect_enemy_condition_(new roborts_decision::PreconditionNode("offensive_detect_enemy_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->EnemyDetected()
                                                                                                == roborts_decision::EnemyStatus::FRONT) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::BOTH));
  std::shared_ptr<roborts_decision::PreconditionNode> offensive_under_attack_condition_(new roborts_decision::PreconditionNode("offensive_under_attack_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->GetDamageSource()
                                                                                                    != roborts_decision::DamageSource::NONE
                                                                                                    && blackboard_ptr_->GetDamageSource()
                                                                                                        != roborts_decision::DamageSource::FORWARD) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::LOW_PRIORITY));
  std::shared_ptr<roborts_decision::PreconditionNode> offensive_detected_condition_(new roborts_decision::PreconditionNode("offensive_detected_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->EnemyDetected()
                                                                                                == roborts_decision::EnemyStatus::BACK) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::LOW_PRIORITY));                                                                                              
  std::shared_ptr<roborts_decision::PreconditionNode> master_receive_condition_(new roborts_decision::PreconditionNode("master_receive_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->IsPartnerDetectEnemy() && 
                                                                                                 blackboard_ptr_-> EnemyDetected()== roborts_decision::EnemyStatus::NONE) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::LOW_PRIORITY));
  std::shared_ptr<roborts_decision::PreconditionNode> offensive_search_condition_(new roborts_decision::PreconditionNode("offensive_search_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->EnemyDetected()
                                                                                                == roborts_decision::EnemyStatus::NONE && blackboard_ptr_->EnemyDisappear()) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::LOW_PRIORITY));                                                                                            
  std::shared_ptr<roborts_decision::PreconditionNode> offensive_patrol_condition_(new roborts_decision::PreconditionNode("offensive_patrol_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->EnemyDetected()
                                                                                                == roborts_decision::EnemyStatus::NONE) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
                                                                                              } , roborts_decision::AbortType::LOW_PRIORITY));
  // std::shared_ptr<roborts_decision::SequenceNode> offensive_detect_enemy_sequence(new roborts_decision::SequenceNode("offensive_detect_enemy_sequence", blackboard_ptr_));
 
  offensive_selector->AddChildren(offensive_dmp_condition_);
  offensive_selector->AddChildren(offensive_detect_enemy_condition_);
  offensive_selector->AddChildren(offensive_under_attack_condition_);
  offensive_selector->AddChildren(offensive_detected_condition_);
  //offensive_selector->AddChildren(master_receive_condition_);
  offensive_selector->AddChildren(offensive_search_condition_);
  offensive_selector->AddChildren(offensive_patrol_condition_);

  offensive_dmp_condition_->SetChild(escape_action_);
  offensive_detect_enemy_condition_->SetChild(chase_action_);
  // offensive_detect_enemy_sequence->AddChildren(chase_action_);
  // offensive_detect_enemy_sequence->AddChildren(shoot_action_);
  offensive_under_attack_condition_->SetChild(turn_to_hurt_action_);
  offensive_detected_condition_->SetChild(turn_back_action_);
  //master_receive_condition_->SetChild(support_action_);
  offensive_search_condition_->SetChild(search_action_);
  offensive_patrol_condition_->SetChild(patrol_action_);

  //  //without buff
  // std::shared_ptr<roborts_decision::SelectorNode> without_buff_selector(new roborts_decision::SelectorNode("without_buff_selector", blackboard_ptr_));          
  // bullet_left_selector->AddChildren(without_buff_selector);
  // std::shared_ptr<roborts_decision::PreconditionNode> buff_ready_condition_(new roborts_decision::PreconditionNode("buff_ready_condition_",blackboard_ptr_,
	// 																																														[&]() {return true;
	// 																																															/*if (blackboard_ptr_->GetBonusStatus()
  //                                                                                               == roborts_decision::BonusStatus::UNOCCUPIED) {
	// 																																																return true;
	// 																																															} else {
	// 																																																return false;
	// 																																															}*/
	// 																																														} , roborts_decision::AbortType::BOTH));
 	// std::shared_ptr<roborts_decision::SequenceNode> gain_buff_sequence(new roborts_decision::SequenceNode("gain_buff", blackboard_ptr_));
  // without_buff_selector->AddChildren(buff_ready_condition_);
  // buff_ready_condition_->SetChild(gain_buff_sequence);
  // gain_buff_sequence->AddChildren(gain_buff_goal_action_);
	// //gain_buff_sequence->AddChildren(gain_buff_action_);

  // //need detailed information if rfid is not detected  


  // // enemy buff condition //enemy_buff_selector
  // std::shared_ptr<roborts_decision::SelectorNode> enemy_obtain_buff_selector(new roborts_decision::SelectorNode("enemy_obtain_buff_selector", blackboard_ptr_)); 

  // without_buff_selector->AddChildren(enemy_obtain_buff_selector);

  // std::shared_ptr<roborts_decision::PreconditionNode> emy_buff_condition_(new roborts_decision::PreconditionNode("emy_buff_condition",blackboard_ptr_,
	// 																																														[&]() {
	// 																																															if (blackboard_ptr_->GetEnemyBonusStatus()
  //                                                                                                == roborts_decision::BonusStatus::OCCUPIED) {
	// 																																																return true;
	// 																																															} else {
	// 																																																return false;
	// 																																															}
	// 																																														} , roborts_decision::AbortType::LOW_PRIORITY));

  // std::shared_ptr<roborts_decision::PreconditionNode> emy_buff_dmp_condition_(new roborts_decision::PreconditionNode("emy_buff_dmp_condition",blackboard_ptr_,
	// 																																														[&]() {
	// 																																															if (blackboard_ptr_->HurtedPerSecond() > 600 ) {
	// 																																																return true;
	// 																																															} else {
	// 																																																return false;
	// 																																															}
	// 																																														} , roborts_decision::AbortType::LOW_PRIORITY));
  // std::shared_ptr<roborts_decision::PreconditionNode> inferior_detect_enemy_condition_(new roborts_decision::PreconditionNode("inferior_detect_enemy_condition",blackboard_ptr_,
	// 																																														[&]() {
	// 																																															if (blackboard_ptr_->EnemyDetected()
  //                                                                                               == roborts_decision::EnemyStatus::FRONT) {
	// 																																																return true;
	// 																																															} else {
	// 																																																return false;
	// 																																															}
	// 																																														} , roborts_decision::AbortType::BOTH));
  // std::shared_ptr<roborts_decision::PreconditionNode> inferior_detected_condition_(new roborts_decision::PreconditionNode("inferior_detected_condition",blackboard_ptr_,
	// 																																														[&]() {
	// 																																															if (blackboard_ptr_->EnemyDetected()
  //                                                                                               == roborts_decision::EnemyStatus::BACK) {
	// 																																																return true;
	// 																																															} else {
	// 																																																return false;
	// 																																															}
	// 																																														} , roborts_decision::AbortType::LOW_PRIORITY));
  // std::shared_ptr<roborts_decision::PreconditionNode> emy_buff_attack_condition_(new roborts_decision::PreconditionNode("emy_buff_attack_condition",blackboard_ptr_,
	// 																																														[&]() {
	// 																																															if (blackboard_ptr_->GetDamageSource()
  //                                                                                            != roborts_decision::DamageSource::NONE
  //                                                                                            && blackboard_ptr_->GetDamageSource()
  //                                                                                                != roborts_decision::DamageSource::FORWARD) {
	// 																																																return true;
	// 																																															} else {
	// 																																																return false;
	// 																																															}
	// 																																														} , roborts_decision::AbortType::BOTH));
  // std::shared_ptr<roborts_decision::PreconditionNode> master_inferior_receive_condition_(new roborts_decision::PreconditionNode("master_inferior_receive_condition",blackboard_ptr_,
	// 																																														[&]() {
	// 																																															if (blackboard_ptr_->IsPartnerDetectEnemy() && 
  //                                                                                                blackboard_ptr_-> EnemyDetected()== roborts_decision::EnemyStatus::NONE) {
	// 																																																return true;
	// 																																															} else {
	// 																																																return false;
	// 																																															}
	// 																																														} , roborts_decision::AbortType::LOW_PRIORITY));
  // enemy_obtain_buff_selector->AddChildren(emy_buff_condition_);
  // enemy_obtain_buff_selector->AddChildren(emy_buff_dmp_condition_);
  // enemy_obtain_buff_selector->AddChildren(inferior_detect_enemy_condition_);
  // enemy_obtain_buff_selector->AddChildren(inferior_detected_condition_);
  // enemy_obtain_buff_selector->AddChildren(emy_buff_attack_condition_);
  // enemy_obtain_buff_selector->AddChildren(master_inferior_receive_condition_);
  // enemy_obtain_buff_selector->AddChildren(guard_action_);
  // emy_buff_condition_->SetChild(guard_action_);
  // emy_buff_dmp_condition_->SetChild(escape_action_);
  // inferior_detect_enemy_condition_->SetChild(chase_action_);
  // inferior_detected_condition_->SetChild(turn_back_action_);
  // emy_buff_attack_condition_->SetChild(turn_to_hurt_action_);
  // master_inferior_receive_condition_->SetChild(support_action_); 




  roborts_decision::BehaviorTree behaviortree(root_node, 20);

  while(1)
    behaviortree.Run();
  
  ros::waitForShutdown();
  return 0;
}
