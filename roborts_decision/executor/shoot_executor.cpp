#include "shoot_executor.h"
namespace roborts_decision{
ShootExecutor::ShootExecutor():excution_mode_(ExcutionMode::IDLE_MODE),
                                 execution_state_(BehaviorState::IDLE), is_published_(false) {
  ros::NodeHandle nh;
  cmd_shoot_client_ = nh.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
  ROS_INFO("Shoot server start!");
  zero_shoot_cmd_.request.mode = 0;
	zero_shoot_cmd_.request.number = 0;
}

void ShootExecutor::Execute(roborts_msgs::ShootCmd &shoot_cmd){
  excution_mode_ = ExcutionMode::SHOOT_MODE;
  if(cmd_shoot_client_.call(shoot_cmd)) {
    ROS_INFO("yes %d", (int)shoot_cmd.response.received);
		is_published_ = true;
	}
  else {
    ROS_INFO("no");
		is_published_ = false;
	}
}


BehaviorState ShootExecutor::Update(){
  switch (excution_mode_){
    case ExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::SHOOT_MODE:
			if (is_published_)
				execution_state_ = BehaviorState::SUCCESS;
			else
				execution_state_ = BehaviorState::FAILURE;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;
}

void ShootExecutor::Cancel(){
  switch (excution_mode_){
    case ExcutionMode::IDLE_MODE:
      ROS_WARN("Nothing to be canceled.");
      break;

    case ExcutionMode::SHOOT_MODE:
      cmd_shoot_client_.call(zero_shoot_cmd_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }

}
}