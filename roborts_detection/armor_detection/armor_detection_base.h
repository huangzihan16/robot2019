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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_BASE_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_BASE_H

#include <vector>
#include "state/error_code.h"
#include "../util/cv_toolbox.h"

namespace roborts_detection {

using roborts_common::ErrorInfo;

/**
 *  This class describes the armor information, including maximum bounding box, vertex, standard deviation.
 */
class ArmorInfo {
 public:
  ArmorInfo(cv::RotatedRect armor_rect, std::vector<cv::Point2f> armor_vertex, float armor_stddev = 0.0, int armor_num = 0, cv::Point3f target_pos = cv::Point3f(0,0,0)) {
    rect = armor_rect;
    vertex = armor_vertex;
    stddev = armor_stddev;
    num = armor_num;
    target_3d = target_pos;
    
  }
 public:
  cv::RotatedRect rect;
  std::vector<cv::Point2f> vertex;
  float stddev;
  int num;
  cv::Point3f target_3d;

};

class ArmorDetectionBase {
 public:
  ArmorDetectionBase(std::shared_ptr<CVToolbox> cv_toolbox)
      : cv_toolbox_(cv_toolbox)
  {  };
  virtual void LoadParam() = 0;
  virtual ErrorInfo DetectArmor(bool &detected, std::vector<ArmorInfo> &armors) = 0;
  virtual void SetThreadState(bool thread_state) = 0;
  virtual ~ArmorDetectionBase() = default;
 protected:
  std::shared_ptr<CVToolbox> cv_toolbox_;
};
} //namespace roborts_detection

#endif //ROBORTS_DETECTION_ARMOR_DETECTION_BASE_H
