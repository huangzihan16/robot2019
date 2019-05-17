#include <actionlib/client/simple_action_client.h>
#include "chassis_executor.h"

namespace roborts_decision{

ChassisExecutor::ChassisExecutor():execution_mode_(ExcutionMode::IDLE_MODE), execution_state_(BehaviorState::IDLE),
                                   global_planner_client_("global_planner_node_action", true),
                                   local_planner_client_("local_planner_node_action", true)
{
  ros::NodeHandle nh;
  cmd_vel_acc_pub_ = nh.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 100);
  cmd_vel_pub_     = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  global_planner_client_.waitForServer();
  ROS_INFO("Global planer server start!");
  local_planner_client_.waitForServer();
  ROS_INFO("Local planer server start!");
}

void ChassisExecutor::Execute(const geometry_msgs::PoseStamped &goal){
  execution_mode_ = ExcutionMode::GOAL_MODE;
  global_planner_goal_.goal = goal;
  global_planner_client_.sendGoal(global_planner_goal_,
                                  GlobalActionClient::SimpleDoneCallback(),
                                  GlobalActionClient::SimpleActiveCallback(),
                                  boost::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, _1));
}

void ChassisExecutor::Execute(const geometry_msgs::Twist &twist){
  if( execution_mode_ == ExcutionMode::GOAL_MODE){
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_MODE;
  cmd_vel_pub_.publish(twist);
}

void ChassisExecutor::Execute(const roborts_msgs::TwistAccel &twist_accel){
  if( execution_mode_ == ExcutionMode::GOAL_MODE){
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_WITH_ACCEL_MODE;

  cmd_vel_acc_pub_.publish(twist_accel);
}

void ChassisExecutor::Execute(const double yaw){
  static float angular_vel = 1.0;
	if( execution_mode_ == ExcutionMode::GOAL_MODE){
	  Cancel();
  }
	execution_mode_ = ExcutionMode::SPEED_MODE;
	
  float limitangle_ = M_PI/4;
  float incre_angle_ = 2.5*M_PI/180;
  
  // if(yaw>-limitangle_&&yaw<=-limitangle_+3*incre_angle_){
  //     // angular_vel = -0.08*dir_flag_;
  // // }else if(yaw>-limitangle_+incre_angle_&&yaw<=-limitangle_+2*incre_angle_){
  // //      angular_vel = -0.34*dir_flag_;
  // // }else if(yaw>-limitangle_+2*incre_angle_&&yaw<=-limitangle_+3*incre_angle_){
  //     angular_vel = -0.52*dir_flag_;
  // }else if(yaw>-limitangle_+3*incre_angle_&&yaw<=-limitangle_+4*incre_angle_){
  //     angular_vel = -0.75*dir_flag_;
  // }else if(yaw>-limitangle_+4*incre_angle_&&yaw<=-limitangle_+5*incre_angle_){
  //     angular_vel = -0.95*dir_flag_;
  // }else if(yaw>-limitangle_+5*incre_angle_&&yaw<=-limitangle_+9*incre_angle_){
  //     angular_vel = -1.43*dir_flag_;
  // }else if(yaw>-limitangle_+9*incre_angle_&&yaw<=-limitangle_+13*incre_angle_){
  //     angular_vel = -2.08*dir_flag_;
  // }else if(yaw>-limitangle_+13*incre_angle_&&yaw<=-limitangle_+18*incre_angle_){
  //     angular_vel = -2.34*dir_flag_;
  // }
  
  // else if(yaw<limitangle_&&yaw>=limitangle_-3*incre_angle_){
  // //     angular_vel = -0.08*dir_flag_;
  // // }else if(yaw<limitangle_-incre_angle_&&yaw>=limitangle_-2*incre_angle_){
  // //     angular_vel = -0.34*dir_flag_;
  // // }else if(yaw<limitangle_-2*incre_angle_&&yaw>=limitangle_-3*incre_angle_){
  //     angular_vel = -0.52*dir_flag_;
  // }else if(yaw<limitangle_-3*incre_angle_&&yaw>=limitangle_-4*incre_angle_){
  //     angular_vel = -0.75*dir_flag_;
  // }else if(yaw<limitangle_-4*incre_angle_&&yaw>=limitangle_-5*incre_angle_){
  //     angular_vel = -0.95*dir_flag_;
  // }else if(yaw<limitangle_-5*incre_angle_&&yaw>=limitangle_-9*incre_angle_){
  //     angular_vel = -1.43*dir_flag_;
  // }else if(yaw<limitangle_-9*incre_angle_&&yaw>=limitangle_-13*incre_angle_){
  //     angular_vel = -2.08*dir_flag_;
  // }else if(yaw<limitangle_-13*incre_angle_&&yaw>=limitangle_-18*incre_angle_){
  //     angular_vel = -2.34*dir_flag_;
  // }
  // else if(yaw <= -limitangle_){
  //     angular_vel = -0.08;
  //     dir_flag_ = 1;
  // }else if(yaw >= limitangle_){
  //     angular_vel = 0.08;
  //     dir_flag_ =-1;
  // }

  
  if(yaw <= -limitangle_){
      angular_vel = -1;
  }
  if(yaw >= limitangle_){
      angular_vel = 1;
  }

  set_w.angular.z=angular_vel;
  cmd_vel_pub_.publish(set_w);
}

BehaviorState ChassisExecutor::Update(){
  actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::LOST;
  ROS_INFO("execution_mode:  %d",(int)execution_mode_);
  // std::cout << "mode" << (int)execution_mode_ << std::endl;
  switch (execution_mode_){
    case ExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::GOAL_MODE:
      state = global_planner_client_.getState();
      if (state == actionlib::SimpleClientGoalState::ACTIVE){
        ROS_INFO("%s : ACTIVE", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;

      } else if (state == actionlib::SimpleClientGoalState::PENDING) {
        ROS_INFO("%s : PENDING", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;

      } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("%s : SUCCEEDED", __FUNCTION__);
        execution_state_ = BehaviorState::SUCCESS;

      } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("%s : ABORTED", __FUNCTION__);
        execution_state_ = BehaviorState::FAILURE;

      } else {
        ROS_ERROR("Error: %s", state.toString().c_str());
        execution_state_ = BehaviorState::FAILURE;
      }
      break;

    case ExcutionMode::SPEED_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;

};

void ChassisExecutor::Cancel(){
  switch (execution_mode_){
    case ExcutionMode::IDLE_MODE:
      ROS_WARN("Nothing to be canceled.");
      break;

    case ExcutionMode::GOAL_MODE:
      global_planner_client_.cancelGoal();
      local_planner_client_.cancelGoal();
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_MODE:
      cmd_vel_pub_.publish(zero_twist_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:
      cmd_vel_acc_pub_.publish(zero_twist_accel_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      usleep(50000);
      break;
    default:
      ROS_ERROR("Wrong Execution Mode");
  }

}

void ChassisExecutor::SetMode(ExcutionMode mode)
{
  execution_mode_ = mode;
}

ChassisExecutor::ExcutionMode ChassisExecutor::GetMode(){
  return execution_mode_;
}


void ChassisExecutor::GlobalPlannerFeedbackCallback(const roborts_msgs::GlobalPlannerFeedbackConstPtr& global_planner_feedback){
  if (!global_planner_feedback->path.poses.empty()) {
    local_planner_goal_.route = global_planner_feedback->path;
    local_planner_client_.sendGoal(local_planner_goal_);
  }
}

}