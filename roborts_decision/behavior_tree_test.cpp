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

#include "behavior_tree/behavior_tree.h"
#include "behavior_tree/behavior_node.h"
#include "blackboard/blackboard.h"
#include "behavior_tree/action_node.h"

#include "std_msgs/Int16.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto gimbal_executor = new roborts_decision::GimbalExecutor;
  auto shoot_executor = new roborts_decision::ShootExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);
	
  //ros::NodeHandle nh;
  //ros::Publisher bullet_num_pub = nh.advertise<std_msgs::Int16>("bullet_num", 1);
  //ros::Publisher remain_HP_pub = nh.advertise<std_msgs::Int16>("remain_HP", 1);

  std::shared_ptr<roborts_decision::Blackboard> blackboard_ptr_(blackboard);
 // behavior
  //roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  //roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  //roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);

	roborts_decision::PatrolBehavior        patrol_behavior(chassis_executor, gimbal_executor, blackboard, full_path);
  roborts_decision::ShootBehavior         shoot_behavior(shoot_executor, chassis_executor, blackboard, full_path);
  roborts_decision::SupportBehavior       support_behavior(chassis_executor, gimbal_executor, blackboard);
  
 //action
	
  //auto back_boot_area_action_ = std::make_shared<roborts_decision::BackBootAreaAction>(blackboard_ptr_, back_boot_area_behavior);
  //auto chase_action_ = std::make_shared<roborts_decision::ChaseAction>(blackboard_ptr_, chase_behavior);
  //auto search_action_ = std::make_shared<roborts_decision::SearchAction>(blackboard_ptr_, search_behavior);

  auto support_action_ = std::make_shared<roborts_decision::SupportAction>(blackboard_ptr_, support_behavior);
  auto patrol_action_ = std::make_shared<roborts_decision::PatrolAction>(blackboard_ptr_, patrol_behavior);

  auto shoot_action_ = std::make_shared<roborts_decision::ShootAction>(blackboard_ptr_, shoot_behavior);
	
  //
  
  std::shared_ptr<roborts_decision::SelectorNode> root_node(new roborts_decision::SelectorNode("root_selector", blackboard_ptr_));

	std::shared_ptr<roborts_decision::PreconditionNode> supply_condition_(new roborts_decision::PreconditionNode("supply_condition",blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->IsSupplyCondition()) {
																																																	return true;
																																																} else {
																																																	return false;
																																																}
																																															} , roborts_decision::AbortType::BOTH));
	
	std::shared_ptr<roborts_decision::SequenceNode> supply_sequence(new roborts_decision::SequenceNode("supply", blackboard_ptr_));
	roborts_decision::SupplyGoalBehavior supply_goal_behavior(chassis_executor, blackboard);
	auto supply_goal_action_ = std::make_shared<roborts_decision::SupplyGoalAction>(blackboard_ptr_, supply_goal_behavior);
	
	roborts_decision::SupplyBehavior supply_application_behavior(blackboard);
	auto supply_application_action_ = std::make_shared<roborts_decision::SupplyApplicateNode>(blackboard_ptr_, supply_application_behavior);
	
	std::shared_ptr<roborts_decision::PreconditionNode> gain_buff_condition_(new roborts_decision::PreconditionNode("gain_buff_condition_",blackboard_ptr_,
																																							[&]() {
																																								if (!blackboard_ptr_->IsSupplyCondition() && blackboard_ptr_->IsGainBuffCondition()) {
																																									return true;
																																								} else {
																																									return false;
																																								}
																																							} , roborts_decision::AbortType::BOTH));
	std::shared_ptr<roborts_decision::SequenceNode> gain_buff_sequence(new roborts_decision::SequenceNode("gain_buff", blackboard_ptr_));
	roborts_decision::GainBuffGoalBehavior gain_buff_goal_behavior(chassis_executor, blackboard);
	auto gain_buff_goal_action_ = std::make_shared<roborts_decision::GainBuffGoalAction>(blackboard_ptr_, gain_buff_goal_behavior);
	roborts_decision::RoundBehavior gain_buff_round_behavior(chassis_executor, blackboard, 10);
	auto gain_buff_guard_action_ = std::make_shared<roborts_decision::GainBuffGuardAction>(blackboard_ptr_, gain_buff_round_behavior);
	
	std::shared_ptr<roborts_decision::PreconditionNode> shoot_selector_condition_(new roborts_decision::PreconditionNode("shoot_selector_condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsEnemyDetected()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
	
	std::shared_ptr<roborts_decision::PreconditionNode> auxiliary_condition_(new roborts_decision::PreconditionNode("auxiliary_condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsPartnerDetectEnemy()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));

	std::shared_ptr<roborts_decision::SelectorNode> shoot_selector(new roborts_decision::SelectorNode("shoot_selector", blackboard_ptr_));
  /*std::shared_ptr<roborts_decision::PreconditionNode> escape_condition_(new roborts_decision::PreconditionNode("escape_condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsBulletLeft()) {
                                                                                                 return false;
                                                                                               } else {
                                                                                                 return true;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
  roborts_decision::EscapeBehavior escape_behavior(chassis_executor, blackboard, full_path);
  auto escape_action_ = std::make_shared<roborts_decision::EscapeAction>(blackboard_ptr_, escape_behavior);*/
	
  std::shared_ptr<roborts_decision::PreconditionNode> shoot_condition_(new roborts_decision::PreconditionNode("shoot condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsBulletLeft()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
	
	
  std::shared_ptr<roborts_decision::PreconditionNode> patrol_condition_(new roborts_decision::PreconditionNode("patrol condition",blackboard_ptr_,
                                                                                              [&]() {
                                                                                               if (blackboard_ptr_->IsEnemyDetected()) {
                                                                                                 return false;
                                                                                               } else {
                                                                                                 return true;
                                                                                               }
                                                                                             }, roborts_decision::AbortType::BOTH));
	
	std::shared_ptr<roborts_decision::PreconditionNode> guard_condition_(new roborts_decision::PreconditionNode("guard condition", blackboard_ptr_,
																																															[&]() {
																																																if (blackboard_ptr_->IsBulletLeft()) {
																																																	return false;
																																																} else {
																																																	return true;
																																																}
																																															}, roborts_decision::AbortType::BOTH));
	roborts_decision::RoundBehavior guard_behavior(chassis_executor, blackboard, 10);
	auto guard_action_ = std::make_shared<roborts_decision::GuardAction>(blackboard_ptr_, guard_behavior);
	

  roborts_decision::BehaviorTree behaviortree(root_node, 20);
	//root_node->AddChildren(supply_condition_);
	//root_node->AddChildren(gain_buff_condition_);
  root_node->AddChildren(shoot_selector_condition_);
  root_node->AddChildren(auxiliary_condition_);
	root_node->AddChildren(patrol_condition_);

	

	supply_condition_->SetChild(supply_sequence);
	supply_sequence->AddChildren(supply_goal_action_);
	supply_sequence->AddChildren(supply_application_action_);
	
	gain_buff_condition_->SetChild(gain_buff_sequence);
	gain_buff_sequence->AddChildren(gain_buff_goal_action_);
	gain_buff_sequence->AddChildren(gain_buff_guard_action_);
	
  shoot_selector_condition_->SetChild(shoot_selector);
  //shoot_selector->AddChildren(escape_condition_);
  shoot_selector->AddChildren(shoot_condition_);
  //shoot_selector->AddChildren(guard_condition_);
	shoot_condition_->SetChild(shoot_action_);
  guard_condition_->SetChild(guard_action_);

  auxiliary_condition_->SetChild(support_action_);

  //escape_condition_->SetChild(escape_action_);
	patrol_condition_->SetChild(patrol_action_);

  while(ros::ok()){
    behaviortree.Run();
    //bullet_num_pub.publish(blackboard->GetBulletNum());
    //remain_HP_pub.publish(blackboard->GetRemainHP());
  }  
  ros::waitForShutdown();
  return 0;
}