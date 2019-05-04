/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <cmath>
#include <unistd.h>
#include "armor_detection_node.h"
#include "ros/ros.h"

namespace roborts_detection {

ArmorDetectionNode::ArmorDetectionNode():
    node_state_(roborts_common::IDLE),
    demensions_(3),
    initialized_(false),
    detected_enemy_(false),
    undetected_count_(0),
    as_(nh_, "armor_detection_node_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false){
          tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));


  initialized_ = false;
  enemy_nh_ = ros::NodeHandle();
  if (Init().IsOK()) {
    initialized_ = true;
    node_state_ = roborts_common::IDLE;
  } else {
    ROS_ERROR("armor_detection_node initalized failed!");
    node_state_ = roborts_common::FAILURE;
  }
  as_.start();
    
}

ErrorInfo ArmorDetectionNode::Init() {
  enemy_info_pub_ = enemy_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
  ArmorDetectionAlgorithms armor_detection_param;

  std::string file_name = ros::package::getPath("roborts_detection") + "/armor_detection/config/armor_detection.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &armor_detection_param);
  if (!read_state) {
    ROS_ERROR("Cannot open %s", file_name.c_str());
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  }
  gimbal_control_.Init(armor_detection_param.camera_gimbal_transform().offset_x(),
                       armor_detection_param.camera_gimbal_transform().offset_y(),
                       armor_detection_param.camera_gimbal_transform().offset_z(),
                       armor_detection_param.camera_gimbal_transform().offset_pitch(),
                       armor_detection_param.camera_gimbal_transform().offset_yaw(), 
                       armor_detection_param.projectile_model_info().init_v(),
                       armor_detection_param.projectile_model_info().init_k());

  //create the selected algorithms
  std::string selected_algorithm = armor_detection_param.selected_algorithm();
  // create image receiver
  cv_toolbox_ =std::make_shared<CVToolbox>(armor_detection_param.camera_name());
  // create armor detection algorithm
  armor_detector_ = roborts_common::AlgorithmFactory<ArmorDetectionBase,std::shared_ptr<CVToolbox>>::CreateAlgorithm
      (selected_algorithm, cv_toolbox_);

  undetected_armor_delay_ = armor_detection_param.undetected_armor_delay();
  if (armor_detector_ == nullptr) {
    ROS_ERROR("Create armor_detector_ pointer failed!");
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  } else
    return ErrorInfo(ErrorCode::OK);
}

void ArmorDetectionNode::ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr &data) {
  roborts_msgs::ArmorDetectionFeedback feedback;
  roborts_msgs::ArmorDetectionResult result;
  bool undetected_msg_published = false;

  if(!initialized_){
    feedback.error_code = error_info_.error_code();
    feedback.error_msg  = error_info_.error_msg();
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    ROS_INFO("Initialization Failed, Failed to execute action!");
    return;
  }

  switch (data->command) {
    case 1:
      StartThread();
      break;
    case 2:
      PauseThread();
      break;
    case 3:
      StopThread();
      break;
    default:
      break;
  }
  ros::Rate rate(25);
  while(ros::ok()) {

    if(as_.isPreemptRequested()) {
      as_.setPreempted();
      return;
    }

    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (undetected_count_ != 0) {
        feedback.detected = true;
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();
        feedback.enemy_info.clear();


	for(int i=0;i<enemy_armor_.size();i++){
	  roborts_msgs::EnemyInfo enemyinfo;
	  enemyinfo.enemy_pos.header.frame_id = "camera0";
          enemyinfo.enemy_pos.header.stamp    = ros::Time::now();
          

          enemyinfo.enemy_pos.pose.position.x = enemy_armor_[i].target_3d.x;
          enemyinfo.enemy_pos.pose.position.y = enemy_armor_[i].target_3d.y;
          enemyinfo.enemy_pos.pose.position.z = enemy_armor_[i].target_3d.z;
          enemyinfo.enemy_pos.pose.orientation.w = 1;

	  enemyinfo.num = enemy_armor_[i].num;
	  feedback.enemy_info.push_back(enemyinfo);
	}


        
        as_.publishFeedback(feedback);
        undetected_msg_published = false;
      } else if(!undetected_msg_published) {
        feedback.detected = false;
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();

        
        as_.publishFeedback(feedback);
        undetected_msg_published = true;
      }
    }
    rate.sleep();
  }
}

#define MAX_MIN_WHIRL_ANGLE     60*3.1415926/180
#define WHIRL_SCAN_DELTA_ANGLE  3*3.1415926/180
void ArmorDetectionNode::ExecuteLoop() {
  undetected_count_ = 0;
                   static float direction = 1;
        static ros::Time last_time = ros::Time::now();
        ros::Duration duration, enemy_duration;
        float delta_time;
        float BoundKy[6] = {-0.5,-0.05,0,0.05,0.2,0.75};
        float BoundKs[6] = {-0.5,-0.1,0,0.1,0.2,0.75};
        float Ky_[6] = {0.3,0.35,0.7,0.35,0.35,0.3};
        float Ks_[6] = {1,20,5000,20,10,1};
        float Ky, Ks;
        float enemy_duration_time;
        static ros::Time last_enemy_time = ros::Time::now();

  while(running_) {
    usleep(1);
    if (node_state_ == NodeState::RUNNING) {
      std::vector<ArmorInfo> armors;
      ErrorInfo error_info = armor_detector_->DetectArmor(detected_enemy_, armors);
      {
        std::lock_guard<std::mutex> guard(mutex_);
        enemy_armor_ = armors;
      }

      if(detected_enemy_) {
        float pitch, yaw, speed, delta_yaw, all_yaw;
        enemy_duration = ros::Time::now() - last_enemy_time;
        enemy_duration_time = enemy_duration.toSec();
        ROS_INFO("enemy_duration_time:%f",enemy_duration_time);
        gimbal_control_.Transform(armors[0].target_3d, pitch, yaw);
        
        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = false;
        gimbal_angle_.pitch_angle = pitch;
          double gimbal_yaw = GetGimbalYaw();
          delta_yaw = yaw;
          all_yaw = gimbal_yaw + delta_yaw;
          speed = (all_yaw - last_yaw_) / 1;
          ROS_INFO("all_yaw:%f, last_yaw_:%f, speed:%f",all_yaw,last_yaw_,speed);
          last_yaw_ = all_yaw;

        if (enemy_duration_time > 2 && yaw < 0.1){
          //speed = kalmanfilter_.Update(all_yaw, speed);
          CalcMembership(yaw, MembershipKy, BoundKy);
          CalcMembership(yaw, MembershipKs, BoundKs);
          Ky = 0;
          Ks = 0;
          for (int i = 0; i < 6; i++){
  		        if (MembershipKy[i] != 0){
                Ky += Ky_[i] * MembershipKy[i] / 100;
                Ks += Ks_[i] * MembershipKs[i] / 100;
  		        }
          }
        } else{
          speed = 0;
        }
        ROS_INFO("yaw:%f, speed:%f",yaw,speed);
        gimbal_angle_.yaw_angle = 0.3 * yaw + 0.8 * speed;
        std::lock_guard<std::mutex> guard(mutex_);
        undetected_count_ = undetected_armor_delay_;
        PublishMsgs();
        //TODO ff

        float enemy_x_shooter = armors[0].target_3d.x/1000.0 + 0.03;  //offset 0.03
        ROS_INFO("enemy_x_shooter:%f",enemy_x_shooter);
		    if (enemy_x_shooter < 0.3 && enemy_x_shooter > -0.3 && armors[0].target_3d.z < 4000) {
          shoot_executor_.Execute();
        }
      } else if(undetected_count_ != 0) {

        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = true;
        gimbal_angle_.yaw_angle = 0;//kalmanfilter_.UpdateNoMesurement();
        gimbal_angle_.pitch_angle = 0;

        undetected_count_--;
        PublishMsgs();
      } else {
        last_enemy_time = ros::Time::now();
        last_yaw_ = GetGimbalYaw();
        duration = ros::Time::now() - last_time;
        delta_time = duration.toSec();
        double yaw = GetGimbalYaw();
        // if(delta_time > 0.2)  {
          gimbal_angle_.yaw_mode = true;
          gimbal_angle_.pitch_mode = true;
          gimbal_angle_.pitch_angle = 0;

          if(yaw > MAX_MIN_WHIRL_ANGLE){
            direction = -1.0;
          }
          if(yaw < -MAX_MIN_WHIRL_ANGLE){
            direction = 1.0;
          }
          gimbal_angle_.yaw_angle = direction * WHIRL_SCAN_DELTA_ANGLE;
          last_time = ros::Time::now();
          PublishMsgs();
        // }
      }
    } else if (node_state_ == NodeState::PAUSE) {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_var_.wait(lock);
    }
  }
}

void ArmorDetectionNode::CalcMembership(float value, float *membership, float *bound)
{
	int i;
	for (i = 0; i <= 6 - 1; i++){
		membership[i] = 0;
	}

	if (value < bound[0]){
		membership[0] = 100;
	} else if(value >= bound[6 - 1]){
		membership[6 - 1] = 100;
	} else{
		for (i = 1; i <= 6 - 1; i++){
			if (value < bound[i]){
				membership[i-1] = (bound[i] - value) * 100 / (bound[i] - bound[i - 1]);
				membership[i]   = 100 - membership[i-1];
        break;
			}
		}
	}
	
}

void ArmorDetectionNode::PublishMsgs() {
  enemy_info_pub_.publish(gimbal_angle_);
  
}

  double ArmorDetectionNode::GetGimbalYaw()
  {
    UpdateGimbalPose();
    tf::Quaternion q;
    tf::quaternionMsgToTF(gimbal_base_pose_.pose.orientation, q);
    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  void ArmorDetectionNode::UpdateGimbalPose() {
    tf::Stamped<tf::Pose> gimbal_tf_pose;
    gimbal_tf_pose.setIdentity();

    gimbal_tf_pose.frame_id_ = "gimbal";
    gimbal_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped gimbal_pose;
      tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
      tf_ptr_->transformPose("base_link", gimbal_pose, gimbal_base_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }




void ArmorDetectionNode::StartThread() {
  ROS_INFO("Armor detection node started!");
  running_ = true;
  armor_detector_->SetThreadState(true);
  if(node_state_ == NodeState::IDLE) {    armor_detection_thread_ = std::thread(&ArmorDetectionNode::ExecuteLoop, this);
  }
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}

void ArmorDetectionNode::PauseThread() {
  ROS_INFO("Armor detection thread paused!");
  node_state_ = NodeState::PAUSE;
}

void ArmorDetectionNode::StopThread() {
  node_state_ = NodeState::IDLE;
  running_ = false;
  armor_detector_->SetThreadState(false);
  if (armor_detection_thread_.joinable()) {
    armor_detection_thread_.join();
  }
}

ArmorDetectionNode::~ArmorDetectionNode() {
  StopThread();
}
} //namespace roborts_detection

void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  
  ros::init(argc, argv, "armor_detection_node", ros::init_options::NoSigintHandler);

  roborts_detection::ArmorDetectionNode armor_detection;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  armor_detection.StopThread();
  return 0;
}


