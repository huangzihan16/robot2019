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
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include "obstacle_layer_setting.pb.h"
#include "obstacle_layer.h"

namespace roborts_costmap {
	
CachedDistanceMap::CachedDistanceMap(double scale,
									 double max_dist) {
	scale_ = scale;
	max_dist_ = max_dist;
	cell_radius_ = static_cast<int>(max_dist / scale);
	distances_mat_.resize(cell_radius_ + 2);
	for (auto it = distances_mat_.begin(); it != distances_mat_.end(); ++it) {
		it->resize(cell_radius_ + 2, max_dist_);
	}

	for (int i = 0; i < cell_radius_ + 2; i++) {
		for (int j = 0; j < cell_radius_ + 2; j++) {
			distances_mat_[i][j] = std::sqrt(i * i + j * j);
		}
	}
}

DistanceMap::DistanceMap(std::string map_topic, double max_occ_dist) {
	ros::NodeHandle nh;
	ParaObstacleLayer para_obstacle;
	std::string obstacle_map = ros::package::getPath("roborts_costmap") + \
      "/config/obstacle_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(obstacle_map.c_str(), &para_obstacle);
	
	map_topic_ = map_topic;
	max_occ_dist_ = max_occ_dist;
	
	is_map_received_ = false;
	map_sub_ = nh.subscribe(map_topic_.c_str(), 1, &DistanceMap::InComingMap, this);
	ros::Rate temp_rate(10);
	while (!is_map_received_) {
		ros::spinOnce();
    temp_rate.sleep();
	}
			
	UpdateCSpace();
}

void DistanceMap::InComingMap(const nav_msgs::OccupancyGridConstPtr& new_map) {
	size_x_ = new_map->info.width;
	size_y_ = new_map->info.height;
	scale_ = new_map->info.resolution;
	origin_x_ = new_map->info.origin.position.x + (size_x_ / 2) * scale_;
	origin_y_ = new_map->info.origin.position.y + (size_y_ / 2) * scale_;
	cells_vec_.resize(size_x_ * size_y_);
	max_x_distance_ = static_cast<double>(size_x_) * scale_;
	max_y_distance_ = static_cast<double>(size_y_) * scale_;
	diag_distance_ = sqrt(max_x_distance_ * max_x_distance_ + max_y_distance_ * max_y_distance_);
	for (int i = 0; i < size_x_ * size_y_; i++) {
		auto tmp_msg = static_cast<int>(new_map->data[i]);
		if (tmp_msg == 0) {
			cells_vec_[i].occ_state = -1;
		} else if (tmp_msg == 100) {
			cells_vec_[i].occ_state = +1;
		} else {
			cells_vec_[i].occ_state = 0;
		}
	}
	
	is_map_received_ = true;
}

int DistanceMap::GetSizeX() const {
	return size_x_;
}
int DistanceMap::GetSizeY() const {
	return size_y_;
}

const double DistanceMap::GetCellOccDistByIndex(int cell_index) {
	// return cells_vec_[cell_index].occ_distance;
  double dis = 0.0;
  try {
    dis = cells_vec_.at(cell_index).occ_distance;
  } catch (const std::out_of_range &oor) {
    std::cerr << "Out of Range error: " << oor.what() << '\n';
    dis = 0.0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    dis = 0.0;
  } catch (...) {
    ROS_ERROR("Unknown exception when getting distance to wall");
    dis = 0.0;
  }
  return dis;
}

int DistanceMap::ComputeCellIndexByMap(const int& i, const int& j) {
	return i + j * size_x_;
}

double DistanceMap::GetCellOccDistByCoord(const int& i, const int& j) {
	return GetCellOccDistByIndex(ComputeCellIndexByMap(i, j));
}

bool DistanceMap::CheckIndexFree(const int& i, const int& j) {
	if (cells_vec_[ComputeCellIndexByMap(i, j)].occ_state == -1)
		return true;
	else
		return false;
}

void DistanceMap::UpdateCSpace() {
	mark_vec_ = std::make_unique<std::vector<unsigned char>>();
	mark_vec_->resize(size_x_ * size_y_);
	std::priority_queue<MapCellData, std::vector<MapCellData>, CompareByOccDist> Q;
	BuildDistanceMap(scale_, max_occ_dist_);

	// Enqueue all the obstacle cells
	MapCellData cell(0,
				  0,
				  0,
				  0,
				  std::bind(&DistanceMap::GetCellOccDistByCoord, this, std::placeholders::_1, std::placeholders::_2));
	for (int i = 0; i < size_x_; i++) {
		cell.src_i_ = cell.i_ = i;
		for (int j = 0; j < size_y_; j++) {
			auto map_index_tmp = ComputeCellIndexByMap(i, j);
			if (cells_vec_[map_index_tmp].occ_state == +1) {
//				cell.occ_distance = this->cells_vec_[map_index_tmp].occ_distance = 0.0;
				cells_vec_[map_index_tmp].occ_distance = 0.0;
				cell.src_j_ = cell.j_ = j;
				mark_vec_->at(map_index_tmp) = 1;
				Q.push(cell);
			} else {
//				cell.occ_distance = this->cells_vec_[map_index_tmp].occ_distance = max_occ_dist_;
				cells_vec_[map_index_tmp].occ_distance = max_occ_dist_;
			}
		}
	}

	while (!Q.empty()) {
		MapCellData current_cell_data = Q.top();
		if (current_cell_data.i_ > 0) {
			Enqueue(current_cell_data.i_ - 1, current_cell_data.j_,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q);
		}
		if (current_cell_data.j_ > 0) {
			Enqueue(current_cell_data.i_, current_cell_data.j_ - 1,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q);
		}
		if (current_cell_data.i_ < size_x_ - 1) {
			Enqueue(current_cell_data.i_ + 1, current_cell_data.j_,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q);
		}
		if (current_cell_data.j_ < size_y_ - 1) {
			Enqueue(current_cell_data.i_, current_cell_data.j_ + 1,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q);
		}
		Q.pop();
	}

	cached_distance_map_.reset();
	mark_vec_.reset();
}

void DistanceMap::Enqueue(int i, int j, int src_i, int src_j,
															 std::priority_queue<MapCellData, std::vector<MapCellData>, CompareByOccDist> &Q) {

	auto index = ComputeCellIndexByMap(i, j);
	if (mark_vec_->at(index)) {
		return;
	}

	int di = std::abs(i - src_i);
	int dj = std::abs(j - src_j);
	double distance = cached_distance_map_->distances_mat_[di][dj];

	if (distance > cached_distance_map_->cell_radius_) {
		return;
	}

	cells_vec_[index].occ_distance = distance * scale_;

	MapCellData cell_data(static_cast<unsigned int>(i),
					   static_cast<unsigned int>(j),
					   static_cast<unsigned int>(src_i),
					   static_cast<unsigned int>(src_j),
					   std::bind(&DistanceMap::GetCellOccDistByCoord, this, std::placeholders::_1, std::placeholders::_2));
	Q.push(cell_data);
	mark_vec_->at(index) = 1;
}

void DistanceMap::BuildDistanceMap(double scale, double max_dist) {
	cached_distance_map_.reset();
	if (cached_distance_map_ == nullptr || cached_distance_map_->scale_ != scale || cached_distance_map_->max_dist_) {
		if (cached_distance_map_ != nullptr) {
			cached_distance_map_.reset();
		}
		cached_distance_map_ = std::make_unique<CachedDistanceMap>(scale, max_dist);
	}
}

void DistanceMap::ConvertWorldToMap(const double&x, const double& y, int& mx, int& my) {
	mx = (std::floor((x - origin_x_) / scale_ + 0.5) + size_x_ / 2);
	my = (std::floor((y - origin_y_) / scale_ + 0.5) + size_y_ / 2);
}

double DistanceMap::GetDistance(double wx, double wy) {
  if (wx < 0.075 || wx > 8.075 || wy < 0.075 || wy > 5.075) {
    return 0;
  }
  
	int mx, my;
	ConvertWorldToMap(wx, wy, mx, my);
	
	return GetCellOccDistByCoord(mx, my);
}

void ObstacleLayer::OnInitialize() {
  ros::NodeHandle nh;
  ParaObstacleLayer para_obstacle;

  std::string obstacle_map = ros::package::getPath("roborts_costmap") + \
      "/config/obstacle_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(obstacle_map.c_str(), &para_obstacle);
  double observation_keep_time = 0.1, expected_update_rate = 10.0, min_obstacle_height = 0.2, \
 max_obstacle_height = 0.6, obstacle_range = 2.5, raytrace_range = 3.0, transform_tolerance = 0.2;
  observation_keep_time = para_obstacle.observation_keep_time();
  expected_update_rate = para_obstacle.expected_update_rate();
  transform_tolerance = para_obstacle.transform_tolerance();
  max_obstacle_height = para_obstacle.max_obstacle_height();
  min_obstacle_height = para_obstacle.min_obstacle_height();
  obstacle_range = para_obstacle.obstacle_range();
  raytrace_range = para_obstacle.raytrace_range();
  max_obstacle_height_ = max_obstacle_height;
  footprint_clearing_enabled_ = para_obstacle.footprint_clearing_enabled();
  std::string topic_string = "LaserScan", sensor_frame = "laser_frame";
  topic_string = para_obstacle.topic_string();
  sensor_frame = para_obstacle.sensor_frame();

  bool inf_is_valid = false, clearing = false, marking = true;
  inf_is_valid = para_obstacle.inf_is_valid();
  clearing = para_obstacle.clearing();
  marking = para_obstacle.marking();
  rolling_window_ = layered_costmap_->IsRollingWindow();
  bool track_unknown_space = layered_costmap_->IsTrackingUnknown();
  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }
  is_current_ = true;
  global_frame_ = layered_costmap_->GetGlobalFrameID();
  ObstacleLayer::MatchSize();
  observation_buffers_.push_back(std::shared_ptr<ObservationBuffer>(new ObservationBuffer(topic_string,
                                                                                            observation_keep_time,
                                                                                            expected_update_rate,
                                                                                            min_obstacle_height,
                                                                                            max_obstacle_height,
                                                                                            obstacle_range,
                                                                                            raytrace_range,
                                                                                            *tf_,
                                                                                            global_frame_,
                                                                                            sensor_frame,
                                                                                            transform_tolerance)));
  if (marking) {
    marking_buffers_.push_back(observation_buffers_.back());
  }
  if (clearing) {
    clearing_buffers_.push_back(observation_buffers_.back());
  } 
  reset_time_ = std::chrono::system_clock::now();
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>
  > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, topic_string, 50));
  std::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan>
  > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));
  if (inf_is_valid) {
    filter->registerCallback(
        boost::bind(&ObstacleLayer::LaserScanValidInfoCallback, this, _1, observation_buffers_.back()));
  } else {
    filter->registerCallback(
        boost::bind(&ObstacleLayer::LaserScanCallback, this, _1, observation_buffers_.back()));
  }
  observation_subscribers_.push_back(sub);
  observation_notifiers_.push_back(filter);
  observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
  std::vector<std::string> target_frames;
  target_frames.push_back(global_frame_);
  target_frames.push_back(sensor_frame);
  observation_notifiers_.back()->setTargetFrames(target_frames);
  is_enabled_ = true;

  std::string map_topic = para_obstacle.map_topic();
	double max_occ_dist = para_obstacle.max_occ_dist();
	distance_map_ptr_ = std::shared_ptr<DistanceMap>(new DistanceMap(map_topic, max_occ_dist));
	
	enemy_inflation_ = para_obstacle.enemy_inflation();	

  if (strstr(global_frame_.c_str(), "map") == NULL) {
    map_is_global_ = false;
    reset_thre = 15;
  }
  else {
    map_is_global_ = true;
    reset_thre = 8;
  }
  
}

void ObstacleLayer::LaserScanCallback(const sensor_msgs::LaserScanConstPtr &message,
                                      const std::shared_ptr<ObservationBuffer> &buffer) {
  sensor_msgs::PointCloud2 temp_cloud;
  temp_cloud.header = message->header;
  try {
    projector_.transformLaserScanToPointCloud(temp_cloud.header.frame_id, *message, temp_cloud, *tf_);
  }
  catch (tf::TransformException &ex) {
    projector_.projectLaser(*message, temp_cloud);
  }
  buffer->Lock();
  buffer->BufferCloud(temp_cloud);
  buffer->Unlock();
}

void ObstacleLayer::LaserScanValidInfoCallback(const sensor_msgs::LaserScanConstPtr &raw_message,
                                               const std::shared_ptr<ObservationBuffer> &buffer) {
  float epsilon = 0.0001, range;
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    range = message.ranges[i];
    if (!std::isfinite(range) && range > 0) {
      message.ranges[i] = message.range_max - epsilon;
    }
  }
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;
  try {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("High fidelity enabled, but TF returned a transform exception to frame %s: %s", \
        global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  }
  buffer->Lock();
  buffer->BufferCloud(cloud);
  buffer->Unlock();
}

void ObstacleLayer::UpdateBounds(double robot_x,
                                 double robot_y,
                                 double robot_yaw,
                                 double *min_x,
                                 double *min_y,
                                 double *max_x,
                                 double *max_y) {
  if (rolling_window_) {
    UpdateOrigin(robot_x - GetSizeXWorld() / 2, robot_y - GetSizeYWorld() / 2);
  } else if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - reset_time_) > std::chrono::seconds(2)){
    reset_time_ = std::chrono::system_clock::now();
    ResetMaps();
  }
  if (!is_enabled_) {
    ROS_ERROR("Obstacle layer is not enabled.");
    return;
  }
  UseExtraBounds(min_x, min_y, max_x, max_y);
  bool temp_is_current = true;
  std::vector<Observation> observations, clearing_observations;
  temp_is_current = temp_is_current && GetMarkingObservations(observations);
  temp_is_current = temp_is_current && GetClearingObservations(clearing_observations);
  is_current_ = temp_is_current;

  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    RaytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  // reset obstacle layer each time
  static int reset_cnt = 0;
  reset_cnt++;
  if (reset_cnt > reset_thre) {
    reset_cnt = 0;
    memset(costmap_, FREE_SPACE, size_x_ * size_y_ * sizeof(unsigned char));
  }

  double resolution = layered_costmap_->GetCostMap()->GetResolution();
	enemy_inflation_grid_ = (int)std::ceil(enemy_inflation_ / resolution);

  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); it++) {
    const Observation obs = *it;
    const pcl::PointCloud<pcl::PointXYZ> &cloud = *(obs.cloud_);
    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;
    for (unsigned int i = 0; i < cloud.points.size(); ++i) {
      double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_) {
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range) {
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!World2Map(px, py, mx, my)) {
        continue;
      }

      double dist_to_wall = 0;
      if (map_is_global_) {
        dist_to_wall = distance_map_ptr_->GetDistance(px, py);
      } else {
        // stamped point to be transformed
        geometry_msgs::PointStamped ps;
        pcl_conversions::fromPCL(cloud.header.stamp, ps.header.stamp);
        ps.header.frame_id = cloud.header.frame_id;
        ps.point.x = px;
        ps.point.y = py;

        // stamped point after transformation
        geometry_msgs::PointStamped ps_map;

        // transform from odom to map
        bool is_transform = true;
        tf_->waitForTransform(ps.header.frame_id, "map", ros::Time(0), ros::Duration(0.05));
        try {
          tf_->transformPoint("map", ros::Time(0), ps, ps.header.frame_id, ps_map);
        } catch (tf::ExtrapolationException &ex) {
          // ROS_ERROR("Extrapolation Error looking up stamped point: %s", ex.what());
          is_transform = false;
        } catch (tf::TransformException &tfe) {
          // ROS_ERROR("TF Exception that should never happen from frame [%s] to [%s], %s", ps.header.frame_id.c_str(),
          //       "map", tfe.what());
          is_transform = false;
        } catch (const std::exception &e) {
          // std::cerr << e.what() << '\n';
          is_transform = false;
        } catch (...) {
          // ROS_ERROR("Unknown exception when transforming stamped point");
          is_transform = false;
        }

        // compute dist_to_wall
        if (is_transform) {
          dist_to_wall = distance_map_ptr_->GetDistance(ps_map.point.x, ps_map.point.y);
        }
        else {
          dist_to_wall = 0.0;
        }
      }

      if (dist_to_wall > 1.5 * enemy_inflation_) {
        if (mx > enemy_inflation_grid_)
  				mx = mx - enemy_inflation_grid_;
				else
  				mx = 0;
				if (my > enemy_inflation_grid_)
  				my = my - enemy_inflation_grid_;
				else
  				my = 0;
  
			  for (unsigned int row = 0; row <= 2 * enemy_inflation_grid_; row++) {
				  for (unsigned int col = 0; col <= 2 * enemy_inflation_grid_; col++) {
  					unsigned int index = GetIndex(mx + row, my + col);
					  costmap_[index] = LETHAL_OBSTACLE;
				  }
				 }
			} else {
  			unsigned int index = GetIndex(mx, my);
				costmap_[index] = LETHAL_OBSTACLE;
			}
      //unsigned int index = GetIndex(mx, my);
				//costmap_[index] = LETHAL_OBSTACLE;

      Touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
  UpdateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!is_enabled_) {
    ROS_WARN("Obstacle layer is not enabled");
    return;
  }

  if (footprint_clearing_enabled_) {
    SetConvexRegionCost(transformed_footprint_, FREE_SPACE);
  }
  combination_method_ = 1;
  switch (combination_method_) {
    case 0:  // Overwrite
      UpdateOverwriteByValid(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void ObstacleLayer::Activate() {
  for (size_t i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != nullptr) {
      observation_subscribers_[i]->subscribe();
    }
  }
  for (size_t i = 0; i < observation_buffers_.size(); ++i) {
    if (observation_buffers_[i] != nullptr) {
      observation_buffers_[i]->ResetLastUpdated();
    }
  }
}

void ObstacleLayer::Deactivate() {
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != nullptr)
      observation_subscribers_[i]->unsubscribe();
  }
}

void ObstacleLayer::Reset() {
  Deactivate();
  ResetMaps();
  is_current_ = true;
  Activate();
}

bool ObstacleLayer::GetMarkingObservations(std::vector<Observation> &marking_observations) const {
  bool current = true;
  // get the marking observations
  for (size_t i = 0; i < marking_buffers_.size(); ++i) {
    marking_buffers_[i]->Lock();
    marking_buffers_[i]->GetObservations(marking_observations);
    current = marking_buffers_[i]->IsCurrent() && current;
    marking_buffers_[i]->Unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool ObstacleLayer::GetClearingObservations(std::vector<Observation> &clearing_observations) const {
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) {
    clearing_buffers_[i]->Lock();
    clearing_buffers_[i]->GetObservations(clearing_observations, true);
    current = clearing_buffers_[i]->IsCurrent() && current;
    clearing_buffers_[i]->Unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                               static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void ObstacleLayer::RaytraceFreespace(const Observation &clearing_observation,
                                      double *min_x,
                                      double *min_y,
                                      double *max_x,
                                      double *max_y) {
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  pcl::PointCloud<pcl::PointXYZ> cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!World2Map(ox, oy, x0, y0)) {
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;

  Touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  for (unsigned int i = 0; i < cloud.points.size(); ++i) {
    double wx = cloud.points[i].x;
    double wy = cloud.points[i].y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the map and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!World2Map(wx, wy, x1, y1))
      continue;

    unsigned int cell_raytrace_range = World2Cell(clearing_observation.raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    RaytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

    UpdateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void ObstacleLayer::UpdateRaytraceBounds(double ox,
                                         double oy,
                                         double wx,
                                         double wy,
                                         double range,
                                         double *min_x,
                                         double *min_y,
                                         double *max_x,
                                         double *max_y) {
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  Touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::UpdateFootprint(double robot_x,
                                    double robot_y,
                                    double robot_yaw,
                                    double *min_x,
                                    double *min_y,
                                    double *max_x,
                                    double *max_y) {
  if (!footprint_clearing_enabled_)
    return;
  TransformFootprint(robot_x, robot_y, robot_yaw, GetFootprint(), transformed_footprint_);

  for (size_t i = 0; i < transformed_footprint_.size(); i++) {
    Touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

} //namespace roborts_costmap