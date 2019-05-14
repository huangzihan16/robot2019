#ifndef ROBORTS_DECISION_PATROL_BEHAVIOR_H
#define ROBORTS_DECISION_PATROL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
/*class PatrolBehavior {
 public:
  PatrolBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {

    patrol_count_ = 0;
    point_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();
    auto executor_state = Update();

    std::cout << "state: " << (int)(executor_state) << std::endl;

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
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    point_size_ = (unsigned int)(decision_config.point().size());
    patrol_goals_.resize(point_size_);
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
    }

    return true;
  }

  ~PatrolBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;

};*/
class PatrolBehavior {
 public:
  PatrolBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {

    patrol_count_ = 0;
    master_point_size_ = 0;
    slave_point_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {
    blackboard_->SuggestGimbalPatrol();
    blackboard_->PublishPartnerInformation();
    auto executor_state = Update();

    std::cout << "state: " << (int)(executor_state) << std::endl;

    if (executor_state != BehaviorState::RUNNING) {

      if (blackboard_->self_identity_ == Identity::MASTER) {
        if (master_patrol_goals_.empty()) {
          ROS_ERROR("master patrol goal is empty");
          return;
        }
        if (patrol_count_ <= blackboard_->partner_patrol_count_) {
          chassis_executor_->Execute(master_patrol_goals_[patrol_count_]);
          patrol_count_ = ++patrol_count_ % master_point_size_;
        }
      } else if (blackboard_->self_identity_ == Identity::SLAVE) {
        if (slave_patrol_goals_.empty()) {
          ROS_ERROR("slave patrol goal is empty");
          return;
        }
        if (patrol_count_ <= blackboard_->partner_patrol_count_) {
          chassis_executor_->Execute(slave_patrol_goals_[patrol_count_]);
          patrol_count_ = ++patrol_count_ % slave_point_size_;
        }
      }
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    master_point_size_ = (unsigned int)(decision_config.master_point().size());
    master_patrol_goals_.resize(master_point_size_);
    for (int i = 0; i != master_point_size_; i++) {
      master_patrol_goals_[i].header.frame_id = "map";
      master_patrol_goals_[i].pose.position.x = decision_config.master_point(i).x();
      master_patrol_goals_[i].pose.position.y = decision_config.master_point(i).y();
      master_patrol_goals_[i].pose.position.z = decision_config.master_point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.master_point(i).roll(),
                                                              decision_config.master_point(i).pitch(),
                                                              decision_config.master_point(i).yaw());
      master_patrol_goals_[i].pose.orientation.x = quaternion.x();
      master_patrol_goals_[i].pose.orientation.y = quaternion.y();
      master_patrol_goals_[i].pose.orientation.z = quaternion.z();
      master_patrol_goals_[i].pose.orientation.w = quaternion.w();
    }

    slave_point_size_ = (unsigned int)(decision_config.slave_point().size());
    slave_patrol_goals_.resize(slave_point_size_);
    for (int i = 0; i != slave_point_size_; i++) {
      slave_patrol_goals_[i].header.frame_id = "map";
      slave_patrol_goals_[i].pose.position.x = decision_config.slave_point(i).x();
      slave_patrol_goals_[i].pose.position.y = decision_config.slave_point(i).y();
      slave_patrol_goals_[i].pose.position.z = decision_config.slave_point(i).z();
      
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.slave_point(i).roll(),
                                                              decision_config.slave_point(i).pitch(),
                                                              decision_config.slave_point(i).yaw());
      slave_patrol_goals_[i].pose.orientation.x = quaternion.x();
      slave_patrol_goals_[i].pose.orientation.y = quaternion.y();
      slave_patrol_goals_[i].pose.orientation.z = quaternion.z();
      slave_patrol_goals_[i].pose.orientation.w = quaternion.w();
    }

    return true;
  }

  ~PatrolBehavior() = default;

 public:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> master_patrol_goals_;
  std::vector<geometry_msgs::PoseStamped> slave_patrol_goals_;
  unsigned int patrol_count_;
  unsigned int master_point_size_;
  unsigned int slave_point_size_;

};
}

#endif //ROBORTS_DECISION_PATROL_BEHAVIOR_H
