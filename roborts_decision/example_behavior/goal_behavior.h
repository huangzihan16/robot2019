#ifndef ROBORTS_DECISION_GOAL_BEHAVIOR_H
#define ROBORTS_DECISION_GOAL_BEHAVIOR_H


#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"


namespace roborts_decision {
class GoalBehavior {
 public:
  GoalBehavior(ChassisExecutor* &chassis_executor,
               Blackboard* &blackboard) :
      chassis_executor_(chassis_executor),
      blackboard_(blackboard) { }

  void Run() {
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();
    if(blackboard_->IsNewGoal()){
      chassis_executor_->Execute(blackboard_->GetGoal());
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~GoalBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! planning goal
  geometry_msgs::PoseStamped planning_goal_;

};

class SupplyGoalBehavior {
 public:
  SupplyGoalBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard) :
      chassis_executor_(chassis_executor), blackboard_(blackboard), have_execute_(false){ }

  void Run() {
    blackboard_->partner_msg_pub_.status = (char)PartnerStatus::SUPPLY;
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();
    if(!have_execute_){
      geometry_msgs::PoseStamped goal = blackboard_->GetSupplyGoal();
      goal.pose.position.x = goal.pose.position.x- 0.15;
      goal.pose.orientation = tf::createQuaternionMsgFromYaw(-95.0/180*3.14);
      chassis_executor_->Execute(goal);
			have_execute_ = true;
    }
  }

  void Cancel() {
		have_execute_ = false;
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
		BehaviorState state = chassis_executor_->Update();
		if (state != BehaviorState::RUNNING)
			have_execute_ = false;
    return state;
  }

  ~SupplyGoalBehavior() = default;

public:
		bool have_execute_;
 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
};

class SupplyGoalOutBehavior {
 public:
  SupplyGoalOutBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard) :
      chassis_executor_(chassis_executor), blackboard_(blackboard), have_execute_(false){ }

  void Run() {
    blackboard_->partner_msg_pub_.status = (char)PartnerStatus::SUPPLY;
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();
    if(!have_execute_){
      goalout_start_time_ = ros::Time::now();
      goalout_count_ = 1;      
      chassis_executor_->Execute(GetOutGoal());
			have_execute_ = true;
    }
  }

  void Cancel() {
		have_execute_ = false;
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
		BehaviorState state = chassis_executor_->Update();
		if (state != BehaviorState::RUNNING)
			have_execute_ = false;
    return state;
  }

  ~SupplyGoalOutBehavior() = default;

  geometry_msgs::PoseStamped GetOutGoal() {
    goalout_time_duration_ = ros::Time::now() - goalout_start_time_;
    float goal_x_[3] = {3.75, 4.25, 3.25};
    float goal_y_[3] = {3.5, 3.5, 4};  
    float goal_q_[3] = {0, 180, 180}; 
    unsigned char cell_cost_[3];
    int map_x_self, map_y_self, min_cost = 10000, min_i = 0;  
    for (i = 0; i < 3; i++ ){
      blackboard_->costmap_2d_->World2MapWithBoundary(goal_x_[i], goal_y_[i], map_x_self, map_y_self);
      cell_cost_[i] = blackboard_->costmap_2d_->GetCost(map_x_self, map_y_self);
      if (cell_cost_[i] < min_cost){
        min_cost = cell_cost_[i];
        min_i = i;
      }
    }
    geometry_msgs::PoseStamped fix_goal;
    ros::Time current_time = ros::Time::now();
    fix_goal.header.stamp = current_time;
    fix_goal.pose.position.x = goal_x_[min_i];
    fix_goal.pose.position.y = goal_y_[min_i];
    fix_goal.pose.position.z = 0.0;
    fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_q_[min_i]/180*3.14);

    return fix_goal;
  }

public:
		bool have_execute_;
    int i;
    double goalout_count_;
    ros::Time goalout_start_time_;
    ros::Duration goalout_time_duration_;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
};



class GainBuffGoalBehavior {
 public:
  GainBuffGoalBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard) :
      chassis_executor_(chassis_executor), blackboard_(blackboard), have_execute_(false){ }

  void Run() {
    blackboard_->SuggestGimbalPatrol();
    chassis_executor_->SetMode(ChassisExecutor::ExcutionMode::GOAL_MODE);
    if(!have_execute_){
      geometry_msgs::PoseStamped fix_goal = blackboard_->GetRobotMapPose();
      ros::Time current_time = ros::Time::now();
      fix_goal.header.stamp = current_time;
      fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(190.0/180*3.14);
      chassis_executor_->Execute(fix_goal);
			have_execute_ = true;
    }
    // ROS_INFO("have_execute_:%d",(int)have_execute_);
  }

  void Cancel() {
		have_execute_ = false;
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
		if (blackboard_->ArriveGainBuff()) {
			chassis_executor_->Cancel();
			
			have_execute_ = false;
			return BehaviorState::SUCCESS;
		} else {
			BehaviorState state = chassis_executor_->Update();
			return state;
		}
  }

  ~GainBuffGoalBehavior() = default;

public:
		bool have_execute_;
 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
};

class GuardGoalBehavior {
 public:
  GuardGoalBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard) :
      chassis_executor_(chassis_executor), blackboard_(blackboard), have_execute_(false){ }

  void Run() {
    blackboard_->SuggestGimbalPatrol();
    chassis_executor_->SetMode(ChassisExecutor::ExcutionMode::GOAL_MODE);
    if(!have_execute_){
      geometry_msgs::PoseStamped fix_goal;
      ros::Time current_time = ros::Time::now();
      fix_goal.header.stamp = current_time;
      if (blackboard_->IsMasterCondition()){
        fix_goal.pose.position.x = 7.5;
        fix_goal.pose.position.y = 2.5;
        fix_goal.pose.position.z = 0.0;
        fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(270.0/180*3.14);
      } else {
        fix_goal.pose.position.x = 1;
        fix_goal.pose.position.y = 4.5;
        fix_goal.pose.position.z = 0.0;
        fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0/180*3.14);
      }
      chassis_executor_->Execute(fix_goal);
			have_execute_ = true;
    }
    // ROS_INFO("have_execute_:%d",(int)have_execute_);
  }

  void Cancel() {
		have_execute_ = false;
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
		BehaviorState state = chassis_executor_->Update();
		if (state != BehaviorState::RUNNING)
			have_execute_ = false;
    return state;
  }

  ~GuardGoalBehavior() = default;

public:
		bool have_execute_;
 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
};


class AccurSupplyBehavior {
 public:
  AccurSupplyBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {
    speed_.linear.x = 0;
    speed_.linear.y = 0;
    speed_.linear.z = 0;
    speed_.angular.x = 0;
    speed_.angular.y = 0;
    speed_.angular.z = 0;

  }

  void Run() {
    blackboard_->partner_msg_pub_.status = (char)PartnerStatus::STOP;
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();

    geometry_msgs::PoseStamped robot_pose = blackboard_->GetRobotMapPose();
    geometry_msgs::PoseStamped goal = blackboard_->GetSupplyGoal();
    double yaw = tf::getYaw(robot_pose.pose.orientation) + 1.57;    //--1.57


    double vel = 0.35,angular_vel = 0.4;

    if(robot_pose.pose.position.x - goal.pose.position.x < -0.02)
      speed_.linear.y = vel;
    else if(robot_pose.pose.position.x - goal.pose.position.x > 0.02)
      speed_.linear.y = -vel;
    else
      speed_.linear.y = 0.0;

    if(robot_pose.pose.position.y - goal.pose.position.y < -0.02)
      speed_.linear.x = -vel;
    else if(robot_pose.pose.position.y - goal.pose.position.y > 0.02)
      speed_.linear.x = vel;
    else
      speed_.linear.x = 0.0;    

    if(yaw < -0.03)
      speed_.angular.z = angular_vel;
    else if(yaw > 0.03)
      speed_.angular.z = -angular_vel;
    else
      speed_.angular.z = 0.0;

    chassis_executor_->Execute(speed_);  

    // if(blackboard_->IsArriveSupplyGoal())
    //   ROS_INFO("arrive arrive");  
    // else
    //   ROS_INFO("no no no no no");  

    
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {

		if (blackboard_->IsArriveSupplyGoal()) {
			chassis_executor_->Cancel();
			return BehaviorState::SUCCESS;
		} else {
      return chassis_executor_->Update();
		}


  }

  ~AccurSupplyBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! candidate speed twist
  geometry_msgs::Twist speed_;


};

}



#endif //ROBORTS_DECISION_GOAL_BEHAVIOR_H
