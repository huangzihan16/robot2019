#include "./blackboard.h"

namespace roborts_decision {
  /*******************Enemy Information from roborts_detection*******************/
  void Blackboard::ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback) {
    if (feedback->detected){
      enemy_detected_ = true;
      last_enemy_disappear_time_ = ros::Time::now();
      // ROS_INFO("Find Enemy!");
      search_count_ = 5;
      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_info[0].enemy_pos;

      //
      std::cout << "ARMOR NUMBER :" << std::endl << feedback->enemy_info.size() << std::endl;
      //

      //剔除无效值
      if (camera_pose_msg.pose.position.z < 1.0)
        return;

			enemy_pose_camera_ = camera_pose_msg;

      camera_pose_msg.header.frame_id = "base_link";
      double yaw = GetGimbalYaw();
      double pos_x = camera_pose_msg.pose.position.z/1000;
      double pos_y = -camera_pose_msg.pose.position.x/1000;
      camera_pose_msg.pose.position.x = pos_x * cos(yaw) - pos_y * sin(yaw);
      camera_pose_msg.pose.position.y = pos_x * sin(yaw) + pos_y * cos(yaw);
			camera_pose_msg.pose.position.z = 0;

      // double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
      //     camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      // double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);
      //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
      yaw = 0;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();

      // std::cout <<"camera_pose_msg:" << camera_pose_msg << std::endl;

      poseStampedMsgToTF(camera_pose_msg, tf_pose);
			
      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        //tf_ptr_->transformPose("map", camera_pose_msg, global_pose_msg);
        if (GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
          enemy_pose_ = global_pose_msg;
          
        }
        // std::cout <<"enemy_pose:" << enemy_pose_ << std::endl;
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    } else{
      enemy_detected_ = false;
      // ROS_INFO(" nnnnnnnnnnnnnnnnFind Enemy!");

    }
    PublishPartnerInformation();
  }

  /*******************Referee System Interaction(Callback and Send Cmd)*******************/
  void Blackboard::GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr& game_status) {
    game_status_ = (GameStatus)game_status->game_status;
    remaining_time_ = game_status->remaining_time;
  }

  void Blackboard::GameResultCallback(const roborts_msgs::GameResult::ConstPtr& game_result) {
    game_result_ = (GameResult)game_result->result;
  }

  void Blackboard::GameSurvivorCallback(const roborts_msgs::GameSurvivor::ConstPtr& game_survival) {
    red3_ = game_survival->red3;
    red4_ = game_survival->red4;
    blue3_ = game_survival->blue3;
    blue3_ = game_survival->blue4;
  }

  void Blackboard::BonusStatusCallback(const roborts_msgs::BonusStatus::ConstPtr& bonus_status) {
    red_bonus_status_ = (BonusStatus)bonus_status->red_bonus;
    blue_bonus_status_ = (BonusStatus)bonus_status->blue_bonus;
  }

  void Blackboard::SupplierStatusCallback(const roborts_msgs::SupplierStatus::ConstPtr& supplier_status) {
    supplier_status_ = (SupplierStatus)supplier_status->status;
  }

  void Blackboard::RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr& robot_status) {
    id_ = robot_status->id;
    level_ = robot_status->level;
    remain_hp_ = robot_status->remain_hp;
    max_hp_ = robot_status->max_hp;
    heat_cooling_limit_ = robot_status->heat_cooling_limit;
    heat_cooling_rate_ = robot_status->heat_cooling_rate;
    gimbal_output_ = robot_status->gimbal_output;
    chassis_output_ = robot_status->chassis_output;
    shooter_output_ = robot_status->shooter_output;
  }

  void Blackboard::RobotHeatCallback(const roborts_msgs::RobotHeat::ConstPtr& robot_heat) {
    chassis_volt_ = robot_heat->chassis_volt;
    chassis_current_ = robot_heat->chassis_current;
    chassis_power_ = robot_heat->chassis_power;
    chassis_power_buffer_ = robot_heat->chassis_power_buffer;
    last_shooter_heat_ = shooter_heat_;
    shooter_heat_ = robot_heat->shooter_heat;
  }

  void Blackboard::RobotBonusCallback(const roborts_msgs::RobotBonus::ConstPtr& robot_bonus) {
    bonus_ = robot_bonus->bonus;
  }

  void Blackboard::RobotDamageCallback(const roborts_msgs::RobotDamage::ConstPtr& robot_damage) {
    last_armor_attacked_time_ = ros::Time::now();
    damage_type_ = (DamageType)robot_damage->damage_type;
    armor_attacked_ = (DamageSource)robot_damage->damage_source;
  }

  void Blackboard::RobotShootCallback(const roborts_msgs::RobotShoot::ConstPtr& robot_shoot) {
    frequency_ = robot_shoot->frequency;
    speed_ = robot_shoot->speed;
    if (speed_ > 12){
      bullet_num_ -= 1;
    }
  }

  void Blackboard::SendSupplyCmd() {
    projectilesupply_.supply = 1;
    projectile_supply_pub_.publish(projectilesupply_);
  }

  /*******************Localization Information for Supply and Gain Buff*******************/
  bool Blackboard::ArriveGainBuff() {
		UpdateRobotPose();
		double delta_x = robot_map_pose_.pose.position.x - 6.3;
		double delta_y = robot_map_pose_.pose.position.y - 1.75;
		// if (red_bonus_status_ == roborts_decision::BonusStatus::BEING_OCCUPIED)
    //   return true;
    // else 
    //   return false;
		if (delta_x * delta_x + delta_y * delta_y < 0.01) 
			return true;
		else 
			return false;
	}

  geometry_msgs::PoseStamped Blackboard::GetSupplyGoal() {

    geometry_msgs::PoseStamped fix_goal;
    ros::Time current_time = ros::Time::now();
    fix_goal.header.stamp = current_time;
    fix_goal.pose.position.x = 4 + 0.075;
    fix_goal.pose.position.y = 4.5 - 0.08;
    fix_goal.pose.position.z = 0.0;
    fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(-90.0/180*3.14);

    return fix_goal;
  }

  geometry_msgs::PoseStamped Blackboard::GetGuardGoal() {

    geometry_msgs::PoseStamped fix_goal;
    ros::Time current_time = ros::Time::now();
    fix_goal.header.stamp = current_time;
    fix_goal.pose.position.x = 6.3;
    fix_goal.pose.position.y = 1.75;
    fix_goal.pose.position.z = 0.0;
    fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(90.0/180*3.14);

    return fix_goal;
  }

  //测试用，给固定位置
  geometry_msgs::PoseStamped Blackboard::GetFixedGoal() {

    geometry_msgs::PoseStamped fix_goal;
    ros::Time current_time = ros::Time::now();
    fix_goal.header.stamp = current_time;
    fix_goal.pose.position.x = 4;
    fix_goal.pose.position.y = 0.5;
    fix_goal.pose.position.z = 0.0;
    fix_goal.pose.orientation = tf::createQuaternionMsgFromYaw(90.0/180*3.14);

    return fix_goal;
  }

  /*******************Referee System Information Preliminarily Process(Not Used directly in Behavior Tree)*******************/
  double Blackboard::HurtedPerSecond() {
    if (ros::Time::now()-last_get_hp_time_ > ros::Duration(0.5)) {
        auto reduce_hp = last_hp_ - remain_hp_;
        dmp_ = reduce_hp / (ros::Time::now()-last_get_hp_time_).toSec();
        last_hp_ = remain_hp_;
        last_get_hp_time_ = ros::Time::now();
        return dmp_;
    } else {
        return dmp_;
    }
  }

  DamageSource Blackboard::GetDamageSource() const{
    if (ros::Time::now()-last_armor_attacked_time_>ros::Duration(0.1)){
        // ROS_INFO("%s: %s", __FUNCTION__, ": NONE");
        return DamageSource::NONE;
    } else {
        ROS_INFO("%s: %d", __FUNCTION__, (int)armor_attacked_);
        return armor_attacked_;
    }
  }

  EnemyStatus Blackboard::EnemyDetected() const{
    // ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    // ROS_INFO("%s: %d", __FUNCTION__, (int)back_enemy_detected_);
    if (enemy_detected_ && back_enemy_detected_)
      return EnemyStatus::BOTH;
    else if (enemy_detected_)
      return EnemyStatus::FRONT;
    else if (back_enemy_detected_)
      return EnemyStatus::BACK;
    else
      return EnemyStatus::NONE;   
  }

  bool Blackboard::EnemyDisappear(){
    if(ros::Time::now() - last_enemy_disappear_time_ < ros::Duration(1) && ros::Time::now() - last_enemy_disappear_time_ > ros::Duration(0) ){
      return true;
    } else
      return false;
  }
  
  /*******************Other(Not Referee System) Output Interface Functions*******************/
  double Blackboard::GetGimbalYaw() {
    UpdateGimbalPose();
    tf::Quaternion q;
    tf::quaternionMsgToTF(gimbal_base_pose_.pose.orientation, q);
    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  /*******************Partner Interaction*******************/
  void Blackboard::PartnerCallback(const roborts_msgs::PartnerInformationConstPtr& partner_info) {
		partner_status_ = (PartnerStatus)partner_info->status;
		partner_detect_enemy_ = partner_info->enemy_detected;
		partner_enemy_pose_ = partner_info->enemy_pose;
		partner_pose_ = partner_info->partner_pose;
		partner_patrol_count_ = partner_info->patrol_count;
	}

  void Blackboard::PublishPartnerInformation() {
    partner_msg_pub_.enemy_detected = enemy_detected_;
    partner_msg_pub_.enemy_pose = enemy_pose_;
    UpdateRobotPose();
    partner_msg_pub_.partner_pose = robot_map_pose_;
    partner_msg_pub_.header.stamp = ros::Time::now();
    partner_pub_.publish(partner_msg_pub_);
  }

  geometry_msgs::PoseStamped Blackboard::GetPartnerEnemyPose() { 
    float Yaw;
    UpdateRobotPose();
    Yaw= atan2(partner_enemy_pose_.pose.position.y - robot_map_pose_.pose.position.y , partner_enemy_pose_.pose.position.x - robot_map_pose_.pose.position.x);
    partner_enemy_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(Yaw);
    return partner_enemy_pose_;
  }

  bool Blackboard::IsMasterCondition() {
    ros::Duration time_past = ros::Time::now() - start_time_;
	  if (time_past.toSec() >= 60 * identity_number_) {
      if (self_identity_ == Identity::MASTER) {
        self_identity_ = Identity::SLAVE;
      } else {
        self_identity_ = Identity::MASTER;
      }
      identity_number_++;
    }
    if (self_identity_ == Identity::MASTER) {
      // ROS_INFO(" MASTER MASTER!");
      return true;
    } else {
      // ROS_INFO(" SLAVE SLAVE!");
      return false;
    }
  }

  bool Blackboard::IsMasterSupplyCondition() {
    if (GetPartnerStatus() == PartnerStatus::SUPPLY)
      return true;
    else
      return false;
  }
  bool Blackboard::IsMasterGainBuffCondition() {
    if (GetPartnerStatus()==PartnerStatus::GAINBUFF)
      return true;
    else
      return false;
  }

  /*******************Math Tools*******************/
  double Blackboard::GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double Blackboard::GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  /*******************Update Chassis Pose and Gimbal Pose*******************/
  void Blackboard::UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }

  void Blackboard::UpdateGimbalPose() {
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

  /*******************Functions Used in Behavior Tree*******************/
  bool Blackboard::IsSupplyCondition() {
		ros::Duration time_past = ros::Time::now() - start_time_;
		if (time_past.toSec() >= 60 * supply_number_){
      return true;
      supplier_status_ = SupplierStatus::PREPARING;
    }
		else
			return false;
	}

  bool Blackboard::IsGainBuffCondition() {
		ros::Duration time_past = ros::Time::now() - start_time_;
		if (time_past.toSec() >= 60 * gain_buff_number_)
			return true;
		else
			return false;
	}

  bool Blackboard::IsBulletLeft() const{
    // ROS_INFO("%s: %d", __FUNCTION__, (int)bullet_num_);
    //return true;
    if (bullet_num_ > 5){
      return true;
    } else{
      return false;
    }
  }
}