#ifndef ROBORTS_DECISION_SHOOT_EXECUTOR_H
#define ROBORTS_DECISION_SHOOT_EXECUTOR_H
#include "ros/ros.h"

#include "roborts_msgs/ShootCmd.h"
#include "../../roborts_base/gimbal/gimbal.h"
#include "../../roborts_base/chassis/chassis.h"
#include "../../roborts_base/roborts_base_config.h"
#include "../../roborts_base/roborts_sdk/sdk.h"
#include "../behavior_tree/behavior_state.h"
namespace roborts_decision{

class ShootExecutor{
 public:
  enum class ExcutionMode{
    IDLE_MODE,   ///< Default idle mode with no task
    SHOOT_MODE,  ///< 
  };

  ShootExecutor();
  ~ShootExecutor() = default;

  void Execute(roborts_msgs::ShootCmd &shoot_cmd);


  BehaviorState Update();

  void Cancel();

 private:
 //! execution mode of the executor
  ExcutionMode excution_mode_;
  //! execution state of the executor (same with behavior state)
  BehaviorState execution_state_;
  roborts_msgs::ShootCmd zero_shoot_cmd_;

  ros::ServiceClient cmd_shoot_client_;

	bool is_published_;

};
}


#endif //ROBORTS_DECISION_SHOOT_EXECUTOR_H