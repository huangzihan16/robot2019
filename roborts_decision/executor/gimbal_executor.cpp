#include "gimbal_executor.h"
namespace roborts_decision{
GimbalExecutor::GimbalExecutor():excution_mode_(ExcutionMode::IDLE_MODE),
                                 execution_state_(BehaviorState::IDLE){
  ros::NodeHandle nh;
  cmd_gimbal_angle_pub_ = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
  cmd_gimbal_rate_pub_  = nh.advertise<roborts_msgs::GimbalRate>("cmd_gimbal_rate", 1);

}

void GimbalExecutor::Execute(const roborts_msgs::GimbalAngle &gimbal_angle){
  excution_mode_ = ExcutionMode::ANGLE_MODE;
  cmd_gimbal_angle_pub_.publish(gimbal_angle);
}

void GimbalExecutor::Execute(const roborts_msgs::GimbalRate &gimbal_rate){
  excution_mode_ = ExcutionMode::RATE_MODE;
  cmd_gimbal_rate_pub_.publish(gimbal_rate);
}

#define MAX_MIN_WHIRL_ANGLE     60*3.1415926/180
#define WHIRL_SCAN_DELTA_ANGLE  42*3.1415926/180
void GimbalExecutor::WhirlScan()
{
  static float direction = 1;
  static ros::Time last_time = ros::Time::now();
  static roborts_msgs::GimbalAngle gimbal_goal;

  ros::Duration duration = ros::Time::now() - last_time;
  float delta_time = duration.toSec();

  if(delta_time > 0.2)
  {

    gimbal_goal.yaw_mode = false;
    gimbal_goal.pitch_mode = false;
    // gimbal_goal.yaw_angle = 3.1415926/4;
    gimbal_goal.pitch_angle = 0;

    if(gimbal_goal.yaw_angle > MAX_MIN_WHIRL_ANGLE)
    {
      direction = -1.0;
    }
    if(gimbal_goal.yaw_angle < -MAX_MIN_WHIRL_ANGLE)
    {
      direction = 1.0;
    }
    gimbal_goal.yaw_angle = gimbal_goal.yaw_angle + direction*WHIRL_SCAN_DELTA_ANGLE;

    Execute(gimbal_goal);

    last_time = ros::Time::now();
  }
}

BehaviorState GimbalExecutor::Update(){
  switch (excution_mode_){
    case ExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::ANGLE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::RATE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;
}

void GimbalExecutor::Cancel(){
  switch (excution_mode_){
    case ExcutionMode::IDLE_MODE:
      ROS_WARN("Nothing to be canceled.");
      break;

    case ExcutionMode::ANGLE_MODE:
      cmd_gimbal_rate_pub_.publish(zero_gimbal_rate_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::RATE_MODE:
      cmd_gimbal_rate_pub_.publish(zero_gimbal_rate_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }

}
}