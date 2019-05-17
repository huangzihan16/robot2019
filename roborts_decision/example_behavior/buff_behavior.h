#ifndef ROBORTS_DECISION_BUFF_BEHAVIOR_H
#define ROBORTS_DECISION_BUFF_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
namespace roborts_decision {
class GainBuffBehavior {
 public:
  GainBuffBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard, const std::string & proto_file_path) :
      chassis_executor_(chassis_executor), blackboard_(blackboard), have_execute_(false), i_(0), buff_count_(1){
    fix_goals_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
  }

  void Run() {
    blackboard_->partner_msg_pub_.status = (char)PartnerStatus::GAINBUFF;
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();
    if(!have_execute_){
      buff_start_time_ = ros::Time::now();
      buff_count_ = 1;
			have_execute_ = true;
    }

    if (!blackboard_->have_gone_to_gainbuff_) {
      blackboard_->have_gone_to_gainbuff_ = true;
      blackboard_->go_to_gainbuff_time_ = ros::Time::now();
    }
    BehaviorState executor_state = Update();
    if (executor_state != BehaviorState::RUNNING && blackboard_->GetBonusStatus() == BonusStatus::UNOCCUPIED)
      chassis_executor_->Execute(GetAddGuardGoal());
 }

  void Cancel() {
		have_execute_ = false;
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
		if (blackboard_->GetBonusStatus() == BonusStatus::BEING_OCCUPIED) {
			have_execute_ = false;
			return BehaviorState::SUCCESS;
		} else {
			BehaviorState state = chassis_executor_->Update();
			return state;
		}
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    fix_goals_size_ = (unsigned int)(decision_config.buff_point().size());
    fix_goals_.resize(fix_goals_size_);
    for (int i = 0; i != fix_goals_size_; i++) {
      fix_goals_[i].header.frame_id = "map";
      fix_goals_[i].pose.position.x = decision_config.buff_point(i).x();
      fix_goals_[i].pose.position.y = decision_config.buff_point(i).y();
      fix_goals_[i].pose.position.z = decision_config.buff_point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.buff_point(i).roll(),
                                                              decision_config.buff_point(i).pitch(),
                                                              decision_config.buff_point(i).yaw());
      fix_goals_[i].pose.orientation.x = quaternion.x();
      fix_goals_[i].pose.orientation.y = quaternion.y();
      fix_goals_[i].pose.orientation.z = quaternion.z();
      fix_goals_[i].pose.orientation.w = quaternion.w();
    }
    return true;
  }

  ~GainBuffBehavior() = default;
  /*geometry_msgs::PoseStamped GetAddGuardGoal() {
    buff_time_duration_ = ros::Time::now() - buff_start_time_;
    if (blackboard_->GetBonusStatus() == BonusStatus::UNOCCUPIED){
      buff_count_ += 1;
      i += 1;
    }

    float goal_x_[5] = {6.30, 6.2, 6.40, 6.40, 6.2};
    float goal_y_[5] = {1.85, 1.95, 1.75, 1.95, 1.75};  
    float goal_q_[5] = {180, 135, 225, 180, 135};      
    geometry_msgs::PoseStamped fix_goal;
    ros::Time current_time = ros::Time::now();
    fix_goal.header.stamp = current_time;
    fix_goal.pose.position.x = goal_x_[i % 5];
    fix_goal.pose.position.y = goal_y_[i % 5];
    fix_goal.pose.position.z = 0.0;
    fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_q_[i]/180*3.14);

    return fix_goal;
  }*/

  geometry_msgs::PoseStamped GetAddGuardGoal() {
    buff_time_duration_ = ros::Time::now() - buff_start_time_;
    if (blackboard_->GetBonusStatus() == BonusStatus::UNOCCUPIED){
      buff_count_ += 1;
      i_ += 1;
    }

    geometry_msgs::PoseStamped fix_goal;
    ros::Time current_time = ros::Time::now();
    fix_goal = fix_goals_[i_ % fix_goals_size_];
    fix_goal.header.stamp = current_time;

    return fix_goal;
  }

public:
		bool have_execute_;

    int i_;
    double buff_count_;
    ros::Time buff_start_time_;
    ros::Duration buff_time_duration_;

    std::vector<geometry_msgs::PoseStamped> fix_goals_;
    unsigned int fix_goals_size_;
 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
};


}

#endif //ROBORTS_DECISION_BUFF_BEHAVIOR_H