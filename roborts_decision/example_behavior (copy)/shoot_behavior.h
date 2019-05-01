#ifndef ROBORTS_DECISION_SHOOT_BEHAVIOR_H
#define ROBORTS_DECISION_SHOOT_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/shoot_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "roborts_msgs/ShootCmd.h"

#include "line_iterator.h"

namespace roborts_decision {

class ShootBehavior {
 public:
  ShootBehavior(ShootExecutor* &shoot_executor, ChassisExecutor* &chassis_executor, Blackboard* &blackboard, 
								const std::string &proto_file_path) :
		shoot_executor_(shoot_executor), chassis_executor_(chassis_executor), blackboard_(blackboard) {
		if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
  }

  void Run() {
		// float enemy_x_shooter = (float)blackboard_->GetEnemyInCamera().pose.position.x/1000 + offset_x_;
    // std::cout << "enemy_x_shooter" << enemy_x_shooter << std::endl;
		// if (enemy_x_shooter < half_robot_length_ && enemy_x_shooter > -half_robot_length_) {
		
		// 	auto executor_state = Update();
		// 	std::cout << "state: " << (int)(executor_state) << std::endl;

		// 	std::cout << "send shoot_cmd" << std::endl;
			
		// 	//shootdecision();
		// 	shoot_cmd_.request.mode = 1;
		// 	shoot_cmd_.request.number = 1;
		// 	shoot_executor_->Execute(shoot_cmd_);
    //   //if(shoot_executor_->GetIsPublished()){
    //     //blackboard_->MinusShootNum(shoot_cmd_);
    //   //}
			
		// }
    chassis_executor_->Execute(blackboard_->GetGimbalYaw());
  }

  void Cancel() {
		shoot_executor_->Cancel();
  }

  BehaviorState Update() {
    return shoot_executor_->Update();
  }

  ~ShootBehavior() = default;

	bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }
		offset_x_ = decision_config.offset_x();
		half_robot_length_ = decision_config.half_robot_length();
		return true;
  }
 private:
	void shootdecision() {
    int CD;
    int f_max;
    //update parameters
    if (blackboard_->GetRemainHP() >= 400){
        CD = 120;
    }
    else{
        CD = 240;
    }
    // v~d determine v by d

    //determine max f
    if((360 - blackboard_->GetShooterHeat() - blackboard_->GetShootVel() * blackboard_->GetBulletNum()) >= 0){
        f_max = 20;
    }
    else{
        f_max = CD * blackboard_->GetBulletNum() / (blackboard_->GetShootVel() * blackboard_->GetBulletNum() + blackboard_->GetShooterHeat() - 360);
    }
    
    //mode 0 stop 1 once 2 continuous 
    shoot_cmd_.request.mode = 1;
    shoot_cmd_.request.number = 1;
	}
  //! executor
  ShootExecutor* const shoot_executor_;
	ChassisExecutor* const chassis_executor_;

  roborts_msgs::ShootCmd shoot_cmd_;

  //! perception information
  Blackboard* const blackboard_;

	float offset_x_;
	float half_robot_length_;
};
}


#endif //ROBORTS_DECISION_SHOOT_BEHAVIOR_H
