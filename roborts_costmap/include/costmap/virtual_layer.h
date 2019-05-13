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
#ifndef ROBORTS_COSTMAP_VIRTUAL_LAYER_H
#define ROBORTS_COSTMAP_VIRTUAL_LAYER_H

#include <chrono>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <string>
#include "footprint.h"
#include "map_common.h"
#include "costmap_layer.h"
#include "layered_costmap.h"
#include "observation_buffer.h"
#include "map_common.h"
#include "roborts_msgs/PartnerInformation.h"

namespace roborts_costmap {

// class MapCell {
// public:
// 	//occ_state = -1为空闲, 其他则为占用
// 	int occ_state = 0;

// 	double occ_distance = 0;
// };

// class MapCellData {
//  public:
//   MapCellData(unsigned int i,
//            unsigned int j,
//            unsigned int src_i,
//            unsigned int src_j,
//            const std::function<double(unsigned int, unsigned int)> &GetOccDistFunc) {
//     i_ = i;
//     j_ = j;
//     src_i_ = src_i;
//     src_j_ = src_j;
//     GetOccDist = GetOccDistFunc;
//   }

//  public:
//   std::function<double(unsigned int, unsigned int)> GetOccDist;
//   unsigned int i_ = 0;
//   unsigned int j_ = 0;
//   unsigned int src_i_ = 0;
//   unsigned int src_j_ = 0;
// };

// struct CompareByOccDist {
//   bool operator()(const MapCellData &a, const MapCellData &b) {
//     return a.GetOccDist(a.i_, a.j_) > b.GetOccDist(b.i_, b.j_);
//   };
// };

// class CachedDistanceMap {
//  public:
//   CachedDistanceMap(double scale, double max_dist);
//  public:
//   double scale_ = 0, max_dist_ = 0;
//   int cell_radius_ = 0;
//   std::vector<std::vector<double>> distances_mat_;
// };

// class DistanceMap {
// 	public:
// 	typedef std::shared_ptr<DistanceMap> Ptr;
// 	DistanceMap(std::string map_topic, double max_occ_dist);
// 	void InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map);
	
// 	int GetSizeX() const;
// 	int GetSizeY() const;
	
// 	bool CheckIndexFree(const int& i, const int& j);
	
// 	const double GetCellOccDistByIndex(int cell_index);
	
// 	int ComputeCellIndexByMap(const int& i, const int& j);
// 	double GetCellOccDistByCoord(const int& i, const int& j);
	
// 	void ConvertWorldToMap(const double&x, const double& y, int& mx, int& my);
// 	double GetDistance(double wx, double wy);

// private:
// 	void UpdateCSpace();
	
//   void Enqueue(int i, int j, int src_i, int src_j,
// 							 std::priority_queue<MapCellData, std::vector<MapCellData>, CompareByOccDist> &Q);

//   void BuildDistanceMap(double scale, double max_dist);
// public:
// 	int size_x_;
// 	int size_y_;
// 	double scale_;
// 	double origin_x_ = 0;
// 	double origin_y_ = 0;
	
// 	std::vector<MapCell> cells_vec_;
	
// 	double max_x_distance_;
// 	double max_y_distance_;
// 	double diag_distance_;
// private:
// 	std::string map_topic_;
// 	ros::Subscriber map_sub_;
// 	bool is_map_received_;
	
// 	double max_occ_dist_;
// 	std::unique_ptr<CachedDistanceMap> cached_distance_map_;
// 	std::unique_ptr<std::vector<unsigned char>> mark_vec_;
// };

class VirtualLayer : public CostmapLayer {
 public:
  VirtualLayer() {
    costmap_ = nullptr;
  }

  virtual ~VirtualLayer() {}
  virtual void OnInitialize();
  virtual void Activate();
  virtual void Deactivate();
  virtual void Reset();
  virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                            double *max_x, double *max_y) override;
  void PartnerInfoCallback(const roborts_msgs::PartnerInformationConstPtr &partner_info);

 protected:
  bool GetMarkingObservations(std::vector<Observation> &marking_observations) const;
  bool GetClearingObservations(std::vector<Observation> &clearing_observations) const;
  virtual void RaytraceFreespace(const Observation &clearing_observation, double *min_x, double *min_y,
                                 double *max_x, double *max_y);
  void UpdateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double *min_x, double *min_y,
                            double *max_x, double *max_y);
  void UpdateFootprint(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                       double *max_x, double *max_y);
  bool footprint_clearing_enabled_, rolling_window_;
  int combination_method_;
  std::string global_frame_;
  double max_virtual_height_;
  std::vector<geometry_msgs::Point> transformed_footprint_;
  laser_geometry::LaserProjection projector_;

  std::vector<std::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;
  std::vector<std::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;
  std::vector<std::shared_ptr<ObservationBuffer> > observation_buffers_;
  std::vector<std::shared_ptr<ObservationBuffer> > marking_buffers_;
  std::vector<std::shared_ptr<ObservationBuffer> > clearing_buffers_;

  std::vector<Observation> static_clearing_observations_, static_marking_observations_;
  std::chrono::system_clock::time_point reset_time_;
  std::string sensor_frame_;

  // DistanceMap::Ptr distance_map_ptr_;
	double enemy_inflation_;
	int enemy_inflation_grid_; 

  bool map_is_global_;
  int reset_thre;

private:
  ros::Subscriber partner_info_sub_;
};

} //namespace roborts_costmap


#endif //ROBORTS_COSTMAP_VIRTUAL_LAYER_H
