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
#include<stdio.h>
#include<stdlib.h>
namespace roborts_detection {

ArmorDetectionNode::ArmorDetectionNode():
    node_state_(roborts_common::IDLE),
    demensions_(3),
    initialized_(false),
    detected_enemy_(false),
    undetected_count_(0),
    as_(nh_, "armor_detection_node_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false){
          tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  patrol_dir_ = 1;
  patrol_mode_ = 0;
  initialized_ = false;
  enemy_nh_ = ros::NodeHandle();

    mode_angle.push_back(mode0_angle);
   mode_angle.push_back(mode1_angle);
   mode_angle.push_back(mode2_angle);
   mode_angle.push_back(mode3_angle);
   mode_angle.push_back(mode4_angle);
   mode_angle.push_back(mode5_angle);
   mode_angle.push_back(mode6_angle);



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
  patrol_suggest_sub_ = enemy_nh_.subscribe<std_msgs::Int32>("patrol_suggest", 1, &ArmorDetectionNode::PatrolSuggestCallback, this);

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
                       armor_detection_param.projectile_model_info().init_k(),
                       armor_detection_param.offset_pitch_fuzzy().offset0(),
                       armor_detection_param.offset_pitch_fuzzy().offset1(),
                       armor_detection_param.offset_pitch_fuzzy().offset2(),
                       armor_detection_param.offset_pitch_fuzzy().offset3(),
                       armor_detection_param.offset_pitch_fuzzy().offset4(),
                       armor_detection_param.offset_pitch_fuzzy().offset5());

  //Set information of ky and ks
  BoundKy_[0] = armor_detection_param.boundky().boundky0();
  BoundKy_[1] = armor_detection_param.boundky().boundky1();
  BoundKy_[2] = armor_detection_param.boundky().boundky2();
  BoundKy_[3] = armor_detection_param.boundky().boundky3();
  BoundKy_[4] = armor_detection_param.boundky().boundky4();
  BoundKy_[5] = armor_detection_param.boundky().boundky5();
  BoundKs_[0] = armor_detection_param.boundks().boundks0();
  BoundKs_[1] = armor_detection_param.boundks().boundks1();
  BoundKs_[2] = armor_detection_param.boundks().boundks2();
  BoundKs_[3] = armor_detection_param.boundks().boundks3();
  BoundKs_[4] = armor_detection_param.boundks().boundks4();
  BoundKs_[5] = armor_detection_param.boundks().boundks5();
  Ky_[0] = armor_detection_param.ky().ky0();
  Ky_[1] = armor_detection_param.ky().ky1();
  Ky_[2] = armor_detection_param.ky().ky2();
  Ky_[3] = armor_detection_param.ky().ky3();
  Ky_[4] = armor_detection_param.ky().ky4();
  Ky_[5] = armor_detection_param.ky().ky5();
  Ks_[0] = armor_detection_param.ks().ks0();
  Ks_[1] = armor_detection_param.ks().ks1();
  Ks_[2] = armor_detection_param.ks().ks2();
  Ks_[3] = armor_detection_param.ks().ks3();
  Ks_[4] = armor_detection_param.ks().ks4();
  Ks_[5] = armor_detection_param.ks().ks5();

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

void ArmorDetectionNode::PatrolSuggestCallback (const std_msgs::Int32::ConstPtr &mode){
  if (mode->data <= 6)
    patrol_mode_ = mode->data;
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
  ros::Duration duration, enemy_duration, enemy_disappear_duration;
  float delta_time;
  float Ky, Ks;
  int patrol_count = 1;
  float enemy_duration_time, gimbal_control_time;
  static ros::Time last_enemy_time = ros::Time::now(),last_time_ = ros::Time::now(), enemy_disappear_time = ros::Time::now();
  char yawArray[20],speedArray[20];
    // FILE *file_fd = fopen("filter.txt","a+"); 
    // if(file_fd == NULL) {  
    //    ROS_ERROR("File Open failed!"); 
    // } else {  
    //    ROS_INFO("File Open successed!");  
    // } 
  while(running_) {
    usleep(1);
    if (node_state_ == NodeState::RUNNING/* && shoot_executor_.game_status_ == roborts_detection::GameStatus::ROUND*/) {
      std::vector<ArmorInfo> armors;
      ErrorInfo error_info = armor_detector_->DetectArmor(detected_enemy_, armors);
      {
        std::lock_guard<std::mutex> guard(mutex_);
        enemy_armor_ = armors;
      }

      if(detected_enemy_) {
        ROS_INFO("detect");
        start_patrol_=false;
        float pitch, yaw, speed, delta_yaw, all_yaw;
        float speed_estimate;
        enemy_duration = ros::Time::now() - last_enemy_time;
        enemy_duration_time = enemy_duration.toSec();
        gimbal_control_.Transform(armors[0].target_3d, pitch, yaw);
        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = false;
        gimbal_angle_.pitch_angle = pitch;
        double gimbal_yaw = GetGimbalYaw();
        delta_yaw = yaw;
        all_yaw = gimbal_yaw + delta_yaw;
        // sprintf(yawArray,"%.10lf",all_yaw);
        // ROS_INFO("yawArray:%s",yawArray);
        // fwrite(yawArray,sizeof(yawArray),1,file_fd);
        // fwrite("\n",sizeof("\n"),1,file_fd);
        duration = ros::Time::now() - last_time_;
        gimbal_control_time = duration.toSec();
        speed = (all_yaw - last_yaw_) / gimbal_control_time;
        last_yaw_ = all_yaw;
        last_time_ = ros::Time::now();

        if (enemy_duration_time > 2 && yaw < 0.2){
          CalcMembership(yaw, MembershipKy, BoundKy_);
          CalcMembership(yaw, MembershipKs, BoundKs_);
          Ky = 0;
          Ks = 0;
          for (int i = 0; i < 6; i++){
  		        if (MembershipKy[i] != 0){
                Ky += Ky_[i] * MembershipKy[i] / 100;
                Ks += Ks_[i] * MembershipKs[i] / 100;
  		        }
          }
          speed_estimate = kalmanfilter_.Update(all_yaw, speed);
          if (speed_estimate > 2 || speed_estimate < -1)
             speed_estimate = 0;
        } else {
          speed_estimate = 0;
        }
        gimbal_angle_.yaw_angle =  0.3 * yaw + 0.05 * speed_estimate; 
        PublishMsgs();
        std::lock_guard<std::mutex> guard(mutex_);
        undetected_count_ = undetected_armor_delay_;


        //TODO ff

        float enemy_x_shooter = armors[0].target_3d.x/1000.0 + 0.03;  //offset 0.03
		    if (enemy_x_shooter < 0.3 && enemy_x_shooter > -0.3 && armors[0].target_3d.z < 2500) {
          shoot_executor_.Execute();
        
              int diff_fre_=shoot_executor_.frequency_-shoot_executor_.last_fre_;
              float diff_spd_=shoot_executor_.speed_-shoot_executor_.last_speed_;
               if(diff_fre_!=0 && diff_spd_!=0){//如果射弹了
               shoot_executor_.shoot_last_time = ros::Time::now();
             }
           ros::Time now_time = ros::Time::now();
           ros::Duration shoot_duration=now_time-shoot_executor_.shoot_last_time;
           float shoot_pause_time = shoot_duration.toSec();
           if(shoot_pause_time>3){//
             shoot_executor_.bullet_vacant_.bullet_vacant=true;
             shoot_executor_.bullet_status_pub_.publish(shoot_executor_.bullet_vacant_);
            }
           shoot_executor_.last_fre_=shoot_executor_.frequency_;
           shoot_executor_.last_speed_=shoot_executor_.speed_;
        }
       
          
      } else if(undetected_count_ != 0) {

        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = true;
        gimbal_angle_.yaw_angle = 0.05 * kalmanfilter_.UpdateNoMesurement();
        gimbal_angle_.pitch_angle = 0;

        undetected_count_--;
        // PublishMsgs();
        enemy_disappear_time = ros::Time::now();
      } else {
        last_enemy_time = ros::Time::now();
        last_yaw_ = GetGimbalYaw();
        duration = ros::Time::now() - last_time;
        enemy_disappear_duration = ros::Time::now() - enemy_disappear_time;
        delta_time = duration.toSec();
        double yaw = GetGimbalYaw();
        if(!start_patrol_){
          patrol_count=FindNearestAngle(patrol_mode_, patrol_count, yaw);
            start_patrol_=true;
        }
        if(delta_time > 0.4 && enemy_disappear_duration.toSec() > 0.5)  {
          gimbal_angle_.yaw_mode = false;
          gimbal_angle_.pitch_mode = true;
          gimbal_angle_.pitch_angle = 0;
          gimbal_angle_.yaw_angle = GetPatrolAngle(patrol_mode_, patrol_count, yaw);
          patrol_count += patrol_dir_;
          last_time = ros::Time::now();
          PublishMsgs();
        }
      }
    } else if (node_state_ == NodeState::PAUSE) {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_var_.wait(lock);
    }
  }
}

int ArmorDetectionNode::FindNearestAngle(int mode, int &patrol_count,float angle){
      int nearcount=0;
      if (patrol_count >= 5 ){
         patrol_count=5;
         patrol_dir_ = -1;
      }
      if( patrol_count <= 0){
        patrol_count=0;
        patrol_dir_ = 1;
      }
      int diff_angle=1000;
      for(int i=0;i<mode_angle[mode].size();++i){
        int diff_temp=abs(mode_angle[mode][i]-angle);
        if(diff_temp<diff_angle){
          nearcount=i;
          diff_angle=diff_temp;
        }
      }




      // patrol_angle = mode0_angle[patrol_count % 6];
      // if (patrol_count >= 5 || patrol_count <= 0){
      //   patrol_dir_ *= -1;
      // }
    // std::cout<<"shuzuchangdu:"<<sizeof( mode0_angle)/sizeof( mode0_angle[0])<<std::endl;
  return nearcount;



}


float ArmorDetectionNode::GetPatrolAngle(int mode, int &patrol_count, float angle){
  float patrol_angle;
  // float mode0_angle[6] = {70, 45, 20, -20, -45, -70};
  // float mode1_angle[4] = {70, 45, 20, -20};
  // float mode2_angle[2] = {20, -20};
  // float mode3_angle[4] = {20, -20, -45, -70};
  // float mode4_angle[4] = {70, 45, -45, -70};
  // float mode5_angle[2] = {70, 45};
  // float mode6_angle[2] = {-45, -70};

     if (patrol_count >= mode_angle[mode].size()-1 ){
         patrol_count = mode_angle[mode].size() -1;
         patrol_dir_ = -1;
      }
      if( patrol_count <= 0){
        patrol_count=0;
        patrol_dir_ = 1;
      }
    patrol_angle = mode_angle[mode][patrol_count];

   
   //std::cout<<"shuzuchangdu:"<<mode_angle.size()<<std::endl;
  // switch(mode){
  //   case 0:
        
  //     //patrol_count=FindNearestAngle(mode,angle);
  //     patrol_angle = mode0_angle[patrol_count % 6];
      
  //     break;
  //   case 1:
  //     if (patrol_count >= 3 ){
  //        patrol_count = 3;
  //        patrol_dir_ = -1;
  //     }
  //     if( patrol_count <= 0){
  //       patrol_count=0;
  //       patrol_dir_ = 1;
  //     }
  //     patrol_angle = mode1_angle[patrol_count % 4];
  //     break;
  //   case 2:
  //     if (patrol_count >= 1 ){
  //        patrol_count = 1;
  //        patrol_dir_ = -1;
  //     }
  //     if( patrol_count <= 0){
  //       patrol_count=0;
  //       patrol_dir_ = 1;
  //     }
  //     patrol_angle = mode2_angle[patrol_count % 2];
  //     break;
  //   case 3:
  //      if (patrol_count >= 3 ){
  //        patrol_count = 3;
  //        patrol_dir_ = -1;
  //     }
  //     if( patrol_count <= 0){
  //       patrol_count=0;
  //       patrol_dir_ = 1;
  //     }
  //    patrol_angle = mode3_angle[patrol_count % 4];
  //     break;
  //   case 4:
  //     if (patrol_count >= 3 ){
  //        patrol_count = 3;
  //        patrol_dir_ = -1;
  //     }
  //     if( patrol_count <= 0){
  //       patrol_count=0;
  //       patrol_dir_ = 1;
  //     }
  //     patrol_angle = mode4_angle[patrol_count % 4];
  //     break;
  //   case 5:
  //     if (patrol_count >= 1 ){
  //        patrol_count = 1;
  //        patrol_dir_ = -1;
  //     }
  //     if( patrol_count <= 0){
  //       patrol_count=0;
  //       patrol_dir_ = 1;
  //     }
  //     patrol_angle = mode5_angle[patrol_count % 2];
  //     break;
  //   case 6:
  //     if (patrol_count >= 1 ){
  //        patrol_count = 1;
  //        patrol_dir_ = -1;
  //     }
  //     if( patrol_count <= 0){
  //       patrol_count=0;
  //       patrol_dir_ = 1;
  //     }
  //     patrol_angle = mode6_angle[patrol_count % 2];
  //     break;
  //   default:
  //     break;
  // }
  return patrol_angle / 180 * 3.1415926;
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


