#include "./blackboard.h"

namespace roborts_decision {
  double CachedMapCell::GetDistanceSquareBetweenCells(const int x1, const int y1, const int x2, const int y2) {
		double dx = (x1 - x2) * resolution_;
		double dy = (y1 - y2) * resolution_;
		return (dx * dx + dy * dy);
	}
	
	double CachedMapCell::GetAngleBetweenNonZeroVectorandCachedVector(double x, double y, int num) {
		double norm = sqrt(x * x + y * y);
		double x_normal = x / norm, y_normal = y / norm;
		return acos(x_normal * cached_x_normal_[num] + y_normal * cached_y_normal_[num]);
	}//cached vector is enemy to goal, if calculate angle_goal_enemy_partner, (x,y) is (dx,dy) of enemy to partner
	
	double CachedMapCell::GetDistanceBetweenPointandLine(double x, double y,
																											 double x1, double y1, double x2, double y2) {
		double parallelogram_area = fabs((x1 - x) * (y2 - y) - (x2 - x) * (y1 - y));
		double bottom_length = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
		return parallelogram_area / bottom_length;
	}//(x,y) is the point and (x1,y1) and (x2,y2) form the line
	
	double CachedMapCell::GetDistanceBetweenPointandLineWithBottomLength(double x, double y,
							double x1, double y1, double x2, double y2, double bottom_length) {
		double parallelogram_area = fabs((x1 - x) * (y2 - y) - (x2 - x) * (y1 - y));
		return parallelogram_area / bottom_length;
	}

	bool CachedMapCell::FindChaseGoal(geometry_msgs::PoseStamped enemy_pose, geometry_msgs::PoseStamped self_pose,
																		geometry_msgs::PoseStamped& goal_pose) {
		double x_enemy = enemy_pose.pose.position.x;
		double y_enemy = enemy_pose.pose.position.y;
		double x_self = self_pose.pose.position.x;
		double y_self = self_pose.pose.position.y;
		
		int map_x_enemy, map_y_enemy, map_x_self, map_y_self;
		costmap2d_->World2MapWithBoundary(x_self, y_self, map_x_self, map_y_self);
		costmap2d_->World2MapWithBoundary(x_enemy, y_enemy, map_x_enemy, map_y_enemy);
		std::vector<unsigned char> cached_cost;
		//We select cell from cached cell
		for (int i = 0; i < cached_number_; i++) {
			int cached_map_x_cell = cached_map_x_bias_[i] + map_x_enemy, cached_map_y_cell = cached_map_y_bias_[i] + map_y_enemy;
			//Check if the cell is in competition area, if not, set highest cost
			if (cached_map_x_cell < 0 || cached_map_x_cell > map_x_size_
				|| cached_map_y_cell < 0 || cached_map_y_cell > map_y_size_) {
				cached_cost.push_back(255);
				continue;
			}
			unsigned char cached_cell_cost = costmap2d_->GetCost(cached_map_x_cell, cached_map_y_cell);
			if (cached_cell_cost > freespace_threshold_) {
				cached_cost.push_back(cached_cell_cost);
				continue;			
			}
			//Check angle between lines (goal to enemy and goal to self), which should be obtuse, if not, set highest cost
			int map_dx_goal_self = map_x_self - cached_map_x_cell, map_dy_goal_self = map_y_self - cached_map_y_cell,
					map_dx_goal_enemy = map_x_enemy - cached_map_x_cell, map_dy_goal_enemy = map_y_enemy - cached_map_y_cell;
			if (map_dx_goal_self * map_dx_goal_enemy + map_dy_goal_self * map_dy_goal_enemy >= 0)
				cached_cost.push_back(255);
			else
				cached_cost.push_back(cached_cell_cost);
		}
		double min_loss = 10000;
		double x_goal, y_goal;
		int goal_cached = -1;
		double distance_enemy_self =
			sqrt((x_enemy - x_self) * (x_enemy - x_self) + (y_enemy - y_self) * (y_enemy - y_self));
		for (int i = 0; i < cached_number_; i++) {
			//Check the cost of goal which should be low enough
			if (cached_cost[i] <= freespace_threshold_) {
				double x_candidate, y_candidate;
				int cached_map_x_cell = cached_map_x_bias_[i] + map_x_enemy, cached_map_y_cell = cached_map_y_bias_[i] + map_y_enemy;
				costmap2d_->Map2World(cached_map_x_cell, cached_map_y_cell, x_candidate, y_candidate);
				//Check if goal is valid for attacking
				int obstacle_num = 0;
				for (FastLineIterator line(map_x_enemy, map_y_enemy, cached_map_x_cell, cached_map_y_cell); line.IsValid(); line.Advance()) {
					if (costmap2d_->GetCost(line.GetX(), line.GetY()) > 253) 
						obstacle_num++;
				}
				if (obstacle_num * resolution_ > 0.25)
					continue;
			
				double error_best_distance = fabs(cached_distance_[i] - best_distance_goal_enemy_);
				double distance_candidate_to_line_enemy_self = 
					GetDistanceBetweenPointandLineWithBottomLength(x_candidate, y_candidate, x_enemy, y_enemy, x_self, y_self, distance_enemy_self);
				double loss = error_best_distance + distance_candidate_to_line_enemy_self;
				//Select a goal which has the lowest error to best distance between goal and enemy
				if (loss < min_loss) {
					min_loss = loss;
					x_goal = x_candidate;
					y_goal = y_candidate;
					goal_cached = i;
				}
			}
		}
		if (goal_cached == -1)
			return false;
		goal_pose.header.frame_id = "map";
		goal_pose.header.stamp = ros::Time::now();
		goal_pose.pose.position.x = x_goal;
		goal_pose.pose.position.y = y_goal;
		goal_pose.pose.position.z = 0;
		goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(cached_yaw_[goal_cached]);
		return true;
	}
	
	bool CachedMapCell::FindSupportGoal(geometry_msgs::PoseStamped enemy_pose, geometry_msgs::PoseStamped partner_pose,
																			geometry_msgs::PoseStamped self_pose, geometry_msgs::PoseStamped& goal_pose) {
		double x_enemy = enemy_pose.pose.position.x;
		double y_enemy = enemy_pose.pose.position.y;
		double x_partner = partner_pose.pose.position.x;
		double y_partner = partner_pose.pose.position.y;
    double x_self = self_pose.pose.position.x;
    double y_self = self_pose.pose.position.y;
		double distance_enemy_partner = sqrt((x_enemy - x_partner) * (x_enemy - x_partner) + (y_enemy - y_partner) * (y_enemy - y_partner));
		
		double dx_enemy_partner = x_partner - x_enemy, dy_enemy_partner = y_partner- y_enemy;//vector from enemy to partner
		double dx_enemy_self = x_self - x_enemy, dy_enemy_self = y_self - y_enemy;					//vector from enemy to self
		double dx_self_enemy = -dx_enemy_self, dy_self_enemy = -dy_enemy_self;							//vector from self to enemy
		double dx_self_partner = x_partner - x_self, dy_self_partner = y_partner - y_self;	//vector from self to partner
		
		int map_x_enemy, map_y_enemy, map_x_partner, map_y_partner;
		costmap2d_->World2MapWithBoundary(x_enemy, y_enemy, map_x_enemy, map_y_enemy);
		costmap2d_->World2MapWithBoundary(x_partner, y_partner, map_x_partner, map_y_partner);
		std::vector<unsigned char> cached_cost;
		//We select cell from cached cell
		for (int i = 0; i < cached_number_; i++) {
			int cached_map_x_cell = cached_map_x_bias_[i] + map_x_enemy, cached_map_y_cell = cached_map_y_bias_[i] + map_y_enemy;
			//Check if the cell is in competition area, if not, set highest cost
			if (cached_map_x_cell < 0 || cached_map_x_cell > map_x_size_
				|| cached_map_y_cell < 0 || cached_map_y_cell > map_y_size_) {
				cached_cost.push_back(255);
				continue;
			}
			unsigned char cached_cell_cost = costmap2d_->GetCost(cached_map_x_cell, cached_map_y_cell);
			if (cached_cell_cost > freespace_threshold_) {
				cached_cost.push_back(cached_cell_cost);
				continue;			
			}
			//Check if goal is valid for attacking, if not, set highest cost
			int obstacle_num = 0;
			for (FastLineIterator line(map_x_enemy, map_y_enemy, cached_map_x_cell, cached_map_y_cell); line.IsValid(); line.Advance()) {
				if (costmap2d_->GetCost(line.GetX(), line.GetY()) > 253) 
					obstacle_num++;
			}
			if (obstacle_num * resolution_ > 0.25) {
				cached_cost.push_back(255);
				continue;
			}
			
			double x_candidate, y_candidate;
			costmap2d_->Map2World(cached_map_x_cell, cached_map_y_cell, x_candidate, y_candidate);
			
			//Check the line of goal and self is not intersect with the line of partner and enemy
			double dx_enemy_candidate = x_candidate - x_enemy, dy_enemy_candidate = y_candidate - y_enemy;
			double dx_self_candidate = x_candidate - x_self, dy_self_candidate = y_candidate - y_self;
			if (VectorCrossProduct(dx_enemy_partner, dy_enemy_partner, dx_enemy_self, dy_enemy_self)
				* VectorCrossProduct(dx_enemy_partner, dy_enemy_partner, dx_enemy_candidate, dy_enemy_candidate) > 0 
			|| VectorCrossProduct(dx_self_candidate, dy_self_candidate, dx_self_partner, dy_self_partner)
				* VectorCrossProduct(dx_self_candidate, dy_self_candidate, dx_self_enemy, dy_self_enemy) > 0) {
				cached_cost.push_back(255);
				continue;
			}
			
			//Select goal which is good for support, if not, set highest cost
			double angle_goal_enemy_partner = 
				GetAngleBetweenNonZeroVectorandCachedVector(dx_enemy_partner, dy_enemy_partner, i);
			if (angle_goal_enemy_partner <= 0)
				cached_cost.push_back(cached_cell_cost);
			else {
				double distance_partner_to_line_enemy_goal, distance_goal_to_line_enemy_partner;
				distance_partner_to_line_enemy_goal =
					GetDistanceBetweenPointandLineWithBottomLength(x_partner, y_partner, x_enemy, y_enemy, x_candidate, y_candidate, cached_distance_[i]);
				distance_goal_to_line_enemy_partner =
					GetDistanceBetweenPointandLineWithBottomLength(x_candidate, y_candidate, x_enemy, y_enemy, x_partner, y_partner, distance_enemy_partner);
				if (distance_partner_to_line_enemy_goal > 0.3548 && distance_goal_to_line_enemy_partner > 0.3548) //0.3548 is half of diagonal.
					cached_cost.push_back(cached_cell_cost);
				else
					cached_cost.push_back(255);
			}
		}
		double min_loss = 10000;
		double x_goal, y_goal;
		int goal_cached = -1;
		for (int i = 0; i < cached_number_; i++) {
			if (cached_cost[i] < freespace_threshold_) {
        int cached_map_x_cell = cached_map_x_bias_[i] + map_x_enemy, cached_map_y_cell = cached_map_y_bias_[i] + map_y_enemy;
				double x_candidate, y_candidate;
				costmap2d_->Map2World(cached_map_x_cell, cached_map_y_cell, x_candidate, y_candidate);

        double dx_self_candidate = x_candidate - x_self, dy_self_candidate = y_candidate - y_self;
        double loss = fabs(cached_distance_[i] - best_distance_goal_enemy_) / best_distance_goal_enemy_
                    + sqrt(dx_self_candidate * dx_self_candidate + dy_self_candidate * dy_self_candidate) / around_area_radius_;
				if (loss < min_loss) {
					min_loss = loss;
					x_goal = x_candidate;
					y_goal = y_candidate;
					goal_cached = i;
				}
			}
		}
		if (goal_cached == -1)
			return false;
		goal_pose.header.frame_id = "map";
		goal_pose.header.stamp = ros::Time::now();
		goal_pose.pose.position.x = x_goal;
		goal_pose.pose.position.y = y_goal;
		goal_pose.pose.position.z = 0;
		goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(cached_yaw_[goal_cached]);
		return true;
	}
	
	bool CachedMapCell::IsGoalStillAvailable(geometry_msgs::PoseStamped enemy_pose, geometry_msgs::PoseStamped last_goal) {
		double x_goal = last_goal.pose.position.x;
		double y_goal = last_goal.pose.position.y;
		double yaw = tf::getYaw(last_goal.pose.orientation);
		double x_enemy = enemy_pose.pose.position.x;
		double y_enemy = enemy_pose.pose.position.y;
		double dx_goal_enemy = x_enemy - x_goal, dy_goal_enemy = y_enemy - y_goal; //vector from goal to enemy
		if (dx_goal_enemy * dx_goal_enemy + dy_goal_enemy * dy_goal_enemy > max_lost_radius_ * max_lost_radius_)
			return false;
		double delta_yaw = yaw - atan2(dy_goal_enemy, dx_goal_enemy);
		if (delta_yaw > M_PI/2 || delta_yaw < -M_PI/2)
			return false;
		int map_x_goal, map_y_goal, map_x_enemy, map_y_enemy;
		costmap2d_->World2MapWithBoundary(x_goal, y_goal, map_x_goal, map_y_goal);
		costmap2d_->World2MapWithBoundary(x_enemy, y_enemy, map_x_enemy, map_y_enemy);
		if (costmap2d_->GetCost(map_x_goal, map_y_goal) > freespace_threshold_)
			return false;
		int obstacle_num = 0;
		for (FastLineIterator line(map_x_enemy, map_y_enemy, map_x_goal, map_y_goal); line.IsValid(); line.Advance()) {
			if (costmap2d_->GetCost(line.GetX(), line.GetY()) > 253) 
				obstacle_num++;
		}
		if (obstacle_num * resolution_ > 0.25)
			return false;
		return true;
	}
  /*******************Enemy Information from roborts_detection*******************/
  void Blackboard::ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback) {
    if (feedback->detected){
      enemy_detected_ = true;
      last_enemy_disappear_time_ = ros::Time::now();
      // ROS_INFO("Find Enemy!");
      //enemy_info_ = feedback->enemy_info;
      search_count_ = 5;
      enemy_info_.clear();
      int update_number = 0;
      for (int i = 0; i < feedback->enemy_info.size(); i++) {
        tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
        geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
        //剔除无效值
        if (feedback->enemy_info[i].enemy_pos.pose.position.z < 1.0) {
          update_number++;
          continue;
        }
        camera_pose_msg = feedback->enemy_info[i].enemy_pos;
        //如果在
        if (update_number == i)
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
          if (update_number == i) {
            if (GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2)
              enemy_pose_ = global_pose_msg;
          }
          roborts_msgs::EnemyInfo enemy_info;
          enemy_info.enemy_pos = global_pose_msg;
          enemy_info.num = feedback->enemy_info[i].num;
          enemy_info_.push_back(enemy_info);
          // std::cout <<"enemy_pose:" << enemy_pose_ << std::endl;
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("tf error when transform enemy pose from camera to map");
        }
      }
    } else {
      enemy_detected_ = false;
      // ROS_INFO(" nnnnnnnnnnnnnnnnFind Enemy!");
    }
    PublishPartnerInformation();
  }

  void Blackboard::BackCameraCallback(const roborts_msgs::BackCameraFeedbackConstPtr& feedback){ 
    if(feedback->detected)
    {
      back_enemy_detected_ = true;
    }
    else
    {
      back_enemy_detected_ = false;
    }
    tag_id_ = feedback->tag_id;

    geometry_msgs::PoseStamped supply_goal, robot_pose;
    supply_goal = GetSupplyGoal();
    robot_pose = GetRobotMapPose();
    if (GetDistance(robot_pose, supply_goal)>0.2 || GetAngle(robot_pose, supply_goal) > 0.3){
      SetBackCameraDetect();
    }else{
      SetBackCameraLocalization();
    }
  }

  void Blackboard::SetBackCameraDetect(){
    if(back_camera_mode_ != 1){
      back_camera_mode_ = 1;
      back_camera_goal_.command = 1;
      back_camera_client_.sendGoal(back_camera_goal_,
                                    actionlib::SimpleActionClient<roborts_msgs::BackCameraAction>::SimpleDoneCallback(),
                                    actionlib::SimpleActionClient<roborts_msgs::BackCameraAction>::SimpleActiveCallback(),
                                    boost::bind(&Blackboard::BackCameraCallback, this, _1));
    }
  }

  void Blackboard::SetBackCameraLocalization(){
    if(back_camera_mode_ != 5){
      back_camera_mode_ = 5;
      back_camera_goal_.command = 5;
      back_camera_client_.sendGoal(back_camera_goal_,
                                    actionlib::SimpleActionClient<roborts_msgs::BackCameraAction>::SimpleDoneCallback(),
                                    actionlib::SimpleActionClient<roborts_msgs::BackCameraAction>::SimpleActiveCallback(),
                                    boost::bind(&Blackboard::BackCameraCallback, this, _1));
    }
  }

  void Blackboard::InitParameter() {
    last_hp_ = 2000;
    dmp_ = 0;
		supply_number_ = 0;
    identity_number_ = 1;
 		gain_buff_number_ = 0;

    bullet_num_ = decision_config_.initial_bullet_num();
    if (decision_config_.master())
      self_identity_ = Identity::MASTER;
    else
      self_identity_ = Identity::SLAVE;
  }

  /*******************Referee System Interaction(Callback and Send Cmd)*******************/
  void Blackboard::GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr& game_status) {
    game_status_ = (GameStatus)game_status->game_status;
    remaining_time_ = game_status->remaining_time;

    if (game_status_ == GameStatus::FIVE_SEC_CD && remaining_time_ > 4)
      InitParameter();
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
    projectilesupply_.number = 100;
    projectile_supply_pub_.publish(projectilesupply_);
  }

  /*******************Localization Information for Supply and Gain Buff*******************/
  bool Blackboard::ArriveGainBuff() {
		UpdateRobotPose();
		double delta_x = robot_map_pose_.pose.position.x - 6.3;
		double delta_y = robot_map_pose_.pose.position.y - 1.75;
		if (red_bonus_status_ == roborts_decision::BonusStatus::BEING_OCCUPIED)
      return true;
    else 
      return false;
		// if (delta_x * delta_x + delta_y * delta_y < 0.01) 
		// 	return true;
		// else 
		// 	return false;
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

  double Blackboard::GetChassisYaw() {
    UpdateRobotPose();
    tf::Quaternion q;
    tf::quaternionMsgToTF(robot_map_pose_.pose.orientation, q);
    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  /*******************Partner Interaction*******************/
  void Blackboard::PartnerCallback(const roborts_msgs::PartnerInformationConstPtr& partner_info) {
		partner_status_ = (PartnerStatus)partner_info->status;
		partner_detect_enemy_ = partner_info->enemy_detected;
		partner_enemy_info_ = partner_info->enemy_info;
    if (!partner_enemy_info_.empty()) {
      UpdateRobotPose();
      geometry_msgs::PoseStamped goal_pose;
      partner_enemy_pose_ = partner_enemy_info_[0].enemy_pos;
      test_enemy_publisher_.publish(partner_enemy_pose_);
      bool success = cachedmapforchaseandsupport_ptr_->FindSupportGoal(partner_enemy_pose_, partner_info->partner_pose, robot_map_pose_, goal_pose);
      if (success)
        test_support_publisher_.publish(goal_pose);
      else
        test_support_publisher_.publish(partner_enemy_pose_);
    }
		partner_pose_ = partner_info->partner_pose;
		partner_patrol_count_ = partner_info->patrol_count;
	}

  void Blackboard::PublishPartnerInformation() {
    partner_msg_pub_.enemy_detected = enemy_detected_;
    partner_msg_pub_.enemy_info = enemy_info_;
    UpdateRobotPose();
    partner_msg_pub_.partner_pose = robot_map_pose_;
    partner_msg_pub_.header.stamp = ros::Time::now();
    partner_pub_.publish(partner_msg_pub_);
  }

  geometry_msgs::PoseStamped Blackboard::GetPartnerEnemyPose() { 
    float Yaw;
    UpdateRobotPose();
    if (!partner_enemy_info_.empty())
      Yaw= atan2(partner_enemy_info_[0].enemy_pos.pose.position.y - robot_map_pose_.pose.position.y, partner_enemy_info_[0].enemy_pos.pose.position.x - robot_map_pose_.pose.position.x);
    else
      Yaw = 0;
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

    ros::Time transform_time = ros::Time();
    std::string tf_error;

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = transform_time;

    if (!tf_ptr_->waitForTransform("map", "base_link", transform_time, ros::Duration(0.3),
                            ros::Duration(0.01), &tf_error)) {
      ROS_ERROR("Transform with tolerance 0.3s failed: %s.", tf_error.c_str());
      return;
    }
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