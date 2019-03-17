#ifndef ROBORTS_DECISION_PATROL_BEHAVIOR_H
#define ROBORTS_DECISION_PATROL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class PatrolBehavior {
 public:
  PatrolBehavior(ChassisExecutor* &chassis_executor, GimbalExecutor* &gimbal_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor), gimbal_executor_(gimbal_executor),
                                                        blackboard_(blackboard), have_time_(false) {

    patrol_count_ = 0;
    point_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();

    std::cout << "state: " << (int)(executor_state) << std::endl;

    // ros::Duration patrol_duration = ros::Time::now() - start_time_;
		// int patrol_gimbal_goal = patrol_duration.toNSec() % 2;
			
		// gimbal_executor_->Execute(patrol_gimbal_goals_[patrol_count_][patrol_gimbal_goal]);
    if (executor_state != BehaviorState::RUNNING) {

      if (patrol_goals_.empty()) {
        ROS_ERROR("patrol goal is empty");
        return;
      }

      std::cout << "send goal" << std::endl;
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
			
			
			
      patrol_count_ = ++patrol_count_ % point_size_;
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
		gimbal_executor_->Cancel();
		have_time_ = false;
  }

  BehaviorState Update() {
		/*BehaviorState chassis_state = chassis_executor_->Update();
		BehaviorState gimbal_state = gimbal_executor_->Update();
		if (chassis_state == BehaviorState::RUNNING || gimbal_state == BehaviorState::RUNNING)
			return BehaviorState::RUNNING;
		else if (chassis_state == BehaviorState::FAILURE || gimbal_state == BehaviorState::FAILURE)
			return BehaviorState::FAILURE;
		else 
			return BehaviorState::SUCCESS;*/
		return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    point_size_ = (unsigned int)(decision_config.point().size());
    patrol_goals_.resize(point_size_);
		patrol_gimbal_goals_.resize(point_size_);
		

    for (int i = 0; i != point_size_; i++) {
      patrol_goals_[i].header.frame_id = "map";
      patrol_goals_[i].pose.position.x = decision_config.point(i).x();
      patrol_goals_[i].pose.position.y = decision_config.point(i).y();
      patrol_goals_[i].pose.position.z = decision_config.point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                              decision_config.point(i).pitch(),
                                                              decision_config.point(i).yaw());
      patrol_goals_[i].pose.orientation.x = quaternion.x();
      patrol_goals_[i].pose.orientation.y = quaternion.y();
      patrol_goals_[i].pose.orientation.z = quaternion.z();
      patrol_goals_[i].pose.orientation.w = quaternion.w();
			
			patrol_gimbal_goals_[i].resize(2);

			patrol_gimbal_goals_[i][0].yaw_mode = false;
			patrol_gimbal_goals_[i][0].pitch_mode = false;
			patrol_gimbal_goals_[i][0].yaw_angle = decision_config.point(i).yaw_angle_min();
			patrol_gimbal_goals_[i][0].pitch_angle = 0;
			
			patrol_gimbal_goals_[i][1].yaw_mode = false;
			patrol_gimbal_goals_[i][1].pitch_mode = false;
			patrol_gimbal_goals_[i][1].yaw_angle = decision_config.point(i).yaw_angle_max();
			patrol_gimbal_goals_[i][1].pitch_angle = 0;
    }

    return true;
  }

  ~PatrolBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;
	GimbalExecutor* const gimbal_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
	std::vector<std::vector<roborts_msgs::GimbalAngle>> patrol_gimbal_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;
	
public:
	bool have_time_;
	ros::Time start_time_;

};
}

#endif //ROBORTS_DECISION_PATROL_BEHAVIOR_H
