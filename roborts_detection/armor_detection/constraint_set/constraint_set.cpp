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
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "constraint_set.h"

#include "timer/timer.h"
#include "io/io.h"

#include "svm.h"


namespace roborts_detection {

ConstraintSet::ConstraintSet(std::shared_ptr<CVToolbox> cv_toolbox):
    ArmorDetectionBase(cv_toolbox){
  filter_x_count_ = 0;
  filter_y_count_ = 0;
  filter_z_count_ = 0;
  filter_distance_count_ = 0;
  filter_pitch_count_ = 0;
  filter_yaw_count_ = 0;
  read_index_ = -1;
  detection_time_ = 0;
  thread_running_ = false;

  LoadParam();
  error_info_ = ErrorInfo(roborts_common::OK);
}

void ConstraintSet::LoadParam() {
  //read parameters
  std::string svm_path = ros::package::getPath("roborts_detection") + \
      "/armor_detection/constraint_set/contourHOG_SVM";
  model = svm_load_model(svm_path.c_str());
  ConstraintSetConfig constraint_set_config_;
  CameraMatrix cameramatrix_;
  std::string file_name = ros::package::getPath("roborts_detection") + \
      "/armor_detection/constraint_set/config/constraint_set.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &constraint_set_config_);
  ROS_ASSERT_MSG(read_state, "Cannot open %s", file_name.c_str());

  enable_debug_ = constraint_set_config_.enable_debug();
  enemy_color_ = constraint_set_config_.enemy_color();
  using_hsv_ = constraint_set_config_.using_hsv();

  //armor info
  float armor_width = constraint_set_config_.armor_size().width();
  float armor_height = constraint_set_config_.armor_size().height();
  SolveArmorCoordinate(armor_width, armor_height);

  //algorithm threshold parameters
  light_max_aspect_ratio_ = constraint_set_config_.threshold().light_max_aspect_ratio();
  light_min_area_ = constraint_set_config_.threshold().light_min_area();
  light_max_angle_ = constraint_set_config_.threshold().light_max_angle();
  light_max_angle_diff_ = constraint_set_config_.threshold().light_max_angle_diff();
  armor_max_angle_ = constraint_set_config_.threshold().armor_max_angle();
  armor_min_area_ = constraint_set_config_.threshold().armor_min_area();
  armor_max_aspect_ratio_ = constraint_set_config_.threshold().armor_max_aspect_ratio();
  armor_max_pixel_val_ = constraint_set_config_.threshold().armor_max_pixel_val();
  armor_max_stddev_ = constraint_set_config_.threshold().armor_max_stddev();
  armor_max_mean_   = constraint_set_config_.threshold().armor_max_mean();
  
  thresh_depth_ = constraint_set_config_.camera_matrix().thresh_depth();
  fx_ = constraint_set_config_.camera_matrix().fx();
  fy_ = constraint_set_config_.camera_matrix().fy();
  cx_ = constraint_set_config_.camera_matrix().cx(); 
  cy_ = constraint_set_config_.camera_matrix().cy();


  color_thread_ = constraint_set_config_.threshold().color_thread();
  blue_thread_ = constraint_set_config_.threshold().blue_thread();
  red_thread_ = constraint_set_config_.threshold().red_thread();

  int get_intrinsic_state = -1;
  int get_distortion_state = -1;

  while ((get_intrinsic_state < 0) || (get_distortion_state < 0)) {
    ROS_WARN("Wait for camera driver launch %d", get_intrinsic_state);
    usleep(50000);
    ros::spinOnce();
    get_intrinsic_state = cv_toolbox_->GetCameraMatrix(intrinsic_matrix_);
    get_distortion_state = cv_toolbox_->GetCameraDistortion(distortion_coeffs_);
  }
}



ErrorInfo ConstraintSet::DetectArmor(bool &detected, std::vector<ArmorInfo> &armors) {
  // std::cout<<"00000000000000000"<<std::endl;
  
  std::vector<cv::RotatedRect> lights;

  auto img_begin = std::chrono::high_resolution_clock::now();
  bool sleep_by_diff_flag = true;
  while (true) {
    // Ensure exit this thread while call Ctrl-C
    if (!thread_running_) {
      ErrorInfo error_info(ErrorCode::STOP_DETECTION);
      return error_info;
    }
    cv::Mat srcImg=cv::Mat::zeros(480,640,CV_8UC3);
    read_index_ = cv_toolbox_->NextImage(srcImg);
    depth_img_=cv_toolbox_->depthImg;
    cv::Mat roi = Mat::zeros(srcImg.size(),CV_8UC1);
    cv::Rect roirect(0,roiy_,640,480-roiy_);
    src_img_=srcImg(roirect);


    if (read_index_ < 0) {
      // Reducing lock and unlock when accessing function 'NextImage'
      if (detection_time_ == 0) {
        usleep(20000);
        continue;
      } else {
        double capture_time = 0;
        cv_toolbox_->GetCaptureTime(capture_time);
        if (capture_time == 0) {
          // Make sure the driver is launched and the image callback is called
          usleep(20000);
          continue;
        } else if (capture_time > detection_time_ && sleep_by_diff_flag) {
//          ROS_WARN("time sleep %lf", (capture_time - detection_time_));
          usleep((unsigned int)(capture_time - detection_time_));
          sleep_by_diff_flag = false;
          continue;
        } else {
          //For real time request when image call back called, the function 'NextImage' should be called.
          usleep(500);
          continue;
        }
      }
    } else {
      break;
    }
  }
  /*ROS_WARN("time get image: %lf", std::chrono::duration<double, std::ratio<1, 1000>>
      (std::chrono::high_resolution_clock::now() - img_begin).count());*/

  auto detection_begin = std::chrono::high_resolution_clock::now();

    
    cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);
    if (enable_debug_) {
      show_lights_before_filter_ = src_img_.clone();
      show_lights_after_filter_ = src_img_.clone();
      show_armors_befor_filter_ = src_img_.clone();
      show_armors_after_filter_ = src_img_.clone();
      //cv::waitKey(1);
    }
    armors.clear();

    //  std::cout<<"1111111111111"<<std::endl;
    DetectLights(src_img_, lights);
    //  std::cout<<"22222222222222222"<<std::endl;
    //FilterLights(lights);
    PossibleArmors(lights, armors);
    //  std::cout<<"333333333333333333333333333"<<std::endl;
    FilterArmors(armors); 
 //svm load
    vector<Point2f> ones, twos;  
    //  std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"<<std::endl;         
    detect12FromImage(src_img_, ones, twos);//svm
    // std::cout<<"bbbbbbbbbbbbbbbbbbbbbbb"<<std::endl;
    Add12Label(armors, ones, twos);
    // std::cout<<"ccccccccccccccccccccccc"<<std::endl;
    

    std::vector<ArmorInfo> final_armor;
    detected = false;
    if(!armors.empty()) {
      final_armor = SlectFinalArmor(armors);
      //  std::cout<<"ddddddddddddddddddddddd"<<std::endl;

      if(!final_armor.empty()){
      if(final_armor.size()>0) detected = true;
      
      for(int i=0;i<final_armor.size();i++){
	      if(final_armor[i].num == 1) {
            cv_toolbox_->DrawRotatedRectwithnum(src_img_, final_armor[i].rect, cv::Scalar(0, 255, 0), 2,1);
        }
        else if(final_armor[i].num == 2) {
            cv_toolbox_->DrawRotatedRectwithnum(src_img_, final_armor[i].rect, cv::Scalar(0, 0, 255), 2,2);
            // std::cout<<"armor x : "<<final_armor[i].rect.center.x<<std::endl;
        }
        else if(final_armor[i].num == 0) {
            cv_toolbox_->DrawRotatedRectwithnum(src_img_, final_armor[i].rect, cv::Scalar(255, 0, 0), 2,0);
        }    
        //  std::cout<<"eeeeeeeeeeeeeeeeeeeeeeeee"<<std::endl;

        CalcControlInfo(final_armor[i]);
        //  std::cout<<"ffffffffffffffffffffff"<<std::endl;

      }
      }
      
    }
    else{
        detected = false;
    }     
    if(enable_debug_) {
      cv::imshow("relust_img_", src_img_);
    }

  lights.clear();
  //armors.clear();
  armors = final_armor;
  final_armor.clear();
  //  std::cout<<"gggggggggggggggggggggggggggg"<<std::endl;
  cv_toolbox_->ReadComplete(read_index_);
  ROS_INFO("read complete");
  detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
      (std::chrono::high_resolution_clock::now() - detection_begin).count();
      // std::cout<<"hhhhhhhhhhh"<<std::endl;

  return error_info_;
}

void ConstraintSet::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) {
  //std::cout << "********************************************DetectLights********************************************" << std::endl;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(src, src, element, cv::Point(-1, -1), 1);//3x3膨胀？？？彩色图膨胀原理是啥？
  cv::Mat binary_brightness_img, binary_light_img, binary_color_img;
  if(using_hsv_) {
    binary_color_img = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, CV_THRESH_BINARY);
  }else {
    auto light = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, CV_THRESH_BINARY);
    float thresh;
    if (enemy_color_ == BLUE)
      thresh = blue_thread_;
    else
      thresh = red_thread_;
    cv::threshold(light, binary_color_img, thresh, 255, CV_THRESH_BINARY);
    if(enable_debug_)
      cv::imshow("light", light);
  }
  //binary_light_img = binary_color_img & binary_brightness_img;
  if (enable_debug_) {
    cv::imshow("binary_brightness_img", binary_brightness_img);
    //cv::imshow("binary_light_img", binary_light_img);
    cv::imshow("binary_color_img", binary_color_img);
  }

  auto contours_light = cv_toolbox_->FindContours(binary_color_img);
  auto contours_brightness = cv_toolbox_->FindContours(binary_brightness_img);

  lights.reserve(contours_light.size());
  lights_info_.reserve(contours_light.size());
  // TODO: To be optimized
  //std::vector<int> is_processes(contours_light.size());




  for (unsigned int i = 0; i < contours_brightness.size(); ++i) {
    for (unsigned int j = 0; j < contours_light.size(); ++j) {

        if (cv::pointPolygonTest(contours_light[j], contours_brightness[i][0], false) >= 0.0) {
          cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[i]);
          cv::Point2f vertices_point[4];
          single_light.points(vertices_point);
          LightInfo light_info(vertices_point);

          if (enable_debug_)
            cv_toolbox_->DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0, 255, 0), 2, light_info.angle_);
          single_light.angle = light_info.angle_;
          lights.push_back(single_light);
          break;
        }
    }
  }

  if (enable_debug_)
    cv::imshow("show_lights_before_filter", show_lights_before_filter_);

  auto c = cv::waitKey(1);
  if (c == 'a') {
    cv::waitKey(0);
  }
}


void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights) {
  //std::cout << "********************************************FilterLights********************************************" << std::endl;
  std::vector<cv::RotatedRect> rects;
  rects.reserve(lights.size());

  for (const auto &light : lights) {
    float angle;
    auto light_aspect_ratio =
        std::max(light.size.width, light.size.height) / std::min(light.size.width, light.size.height);
    //https://stackoverflow.com/questions/15956124/minarearect-angles-unsure-about-the-angle-returned/21427814#21427814
    if(light.size.width < light.size.height) {
      angle = light.angle; // -light.angle
    } else
      angle = light.angle; // light.angle + 90
    //std::cout << "light angle: " << angle << std::endl;
    //std::cout << "light_aspect_ratio: " << light_aspect_ratio << std::endl;
    //std::cout << "light_area: " << light.size.area() << std::endl;
    if (light_aspect_ratio < light_max_aspect_ratio_ &&
        light.size.area() >= light_min_area_) { //angle < light_max_angle_ &&
          rects.push_back(light);
      if (enable_debug_)
        cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, light, cv::Scalar(0, 255, 0), 2, angle);
    }
  }
  if (enable_debug_)
    cv::imshow("lights_after_filter", show_lights_after_filter_);

  lights = rects;
}

void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors) {
  //std::cout << "********************************************PossibleArmors********************************************" << std::endl;
  for (unsigned int i = 0; i < lights.size(); i++) {
    for (unsigned int j = i + 1; j < lights.size(); j++) {
      cv::RotatedRect light1 = lights[i];
      cv::RotatedRect light2 = lights[j];
      auto edge1 = std::minmax(light1.size.width, light1.size.height);
      auto edge2 = std::minmax(light2.size.width, light2.size.height);
      auto lights_dis = std::sqrt((light1.center.x - light2.center.x) * (light1.center.x - light2.center.x) +
          (light1.center.y - light2.center.y) * (light1.center.y - light2.center.y));
      auto center_angle = std::atan(std::abs(light1.center.y - light2.center.y) / std::abs(light1.center.x - light2.center.x)) * 180 / CV_PI;
      center_angle = center_angle > 90 ? 180 - center_angle : center_angle;
      //std::cout << "center_angle: " << center_angle << std::endl;

      cv::RotatedRect rect;
      rect.angle = static_cast<float>(center_angle);
      rect.center.x = (light1.center.x + light2.center.x) / 2;
      rect.center.y = (light1.center.y + light2.center.y) / 2;
      float armor_width = std::abs(static_cast<float>(lights_dis) - std::max(edge1.first, edge2.first));
      float armor_height = std::max<float>(edge1.second, edge2.second);

      rect.size.width = std::max<float>(armor_width, armor_height);
      rect.size.height = std::min<float>(armor_width, armor_height);

      float light1_angle = light1.angle; //light1.size.width < light1.size.height ? -light1.angle : light1.angle + 90
      float light2_angle = light2.angle; //light2.size.width < light2.size.height ? -light2.angle : light2.angle + 90
      // std::cout << "light1_angle: " << light1_angle << std::endl;
      // std::cout << "light2_angle: " << light2_angle << std::endl;

      // if (enable_debug_) {
      //   std::cout << "*******************************" << std::endl;
      //   std::cout << "light_angle_diff_: " << std::abs(light1_angle - light2_angle) << std::endl;
      //   std::cout << "radio: " << std::max<float>(edge1.second, edge2.second)/std::min<float>(edge1.second, edge2.second) << std::endl;
      //   std::cout << "armor_angle_: " << std::abs(center_angle) << std::endl;
      //  std::cout << "armor_aspect_ratio_: " << rect.size.width / (float) (rect.size.height) << std::endl;
      //   std::cout << "armor_area_: " << std::abs(rect.size.area()) << std::endl;
      //   std::cout << "armor_pixel_val_: " << (float)(gray_img_.at<uchar>(static_cast<int>(rect.center.y), static_cast<int>(rect.center.x))) << std::endl;
      //   std::cout << "pixel_y" << static_cast<int>(rect.center.y) << std::endl;
      //   std::cout << "pixel_x" << static_cast<int>(rect.center.x) << std::endl;
      // }
      
      auto angle_diff = std::abs(light1_angle - light2_angle);
      // Avoid incorrect calculation at 180 and 0.
      if (angle_diff > 175) {
        angle_diff = 180 -angle_diff;
      }

      if (angle_diff < light_max_angle_diff_ &&
          std::max<float>(edge1.second, edge2.second)/std::min<float>(edge1.second, edge2.second) < 2.0 &&
          rect.size.width / (rect.size.height) < armor_max_aspect_ratio_ &&
          std::abs(rect.size.area()) > armor_min_area_ &&std::abs(center_angle) < armor_max_angle_ &&
          gray_img_.at<uchar>(static_cast<int>(rect.center.y), static_cast<int>(rect.center.x))
              < armor_max_pixel_val_) { //std::abs(center_angle) < armor_max_angle_ &&

        if (light1.center.x < light2.center.x) {
          std::vector<cv::Point2f> armor_points;
          CalcArmorInfo(armor_points, light1, light2);
          armors.emplace_back(ArmorInfo(rect, armor_points));
          if (enable_debug_)
            cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
          armor_points.clear();
        } else {
          std::vector<cv::Point2f> armor_points;
          CalcArmorInfo(armor_points, light2, light1);
          armors.emplace_back(ArmorInfo(rect, armor_points));
          if (enable_debug_)
            cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
          armor_points.clear();
        }
      }
    }
  }
  if (enable_debug_)
    cv::imshow("armors_before_filter", show_armors_befor_filter_);
}

void ConstraintSet::Add12Label(std::vector<ArmorInfo> &armors,std::vector<Point2f> ones, std::vector<Point2f> twos) {
  //首先根据当前帧中的12检测情况对装甲板进行标注，如同时包含两个则视为0
  for (unsigned int i = 0; i < armors.size(); i++) {
    cv::Point2f corners[4];
    armors[i].rect.points(corners);
    
    cv::Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
    std::vector<Point2f> contours(corners , lastItemPointer);

    for(int j=0;j<ones.size();j++){
      double indicator = pointPolygonTest(contours,ones[j],false);
      if(indicator>=0){
        armors[i].num = 1;
        break;
      }
    }
    for(int j=0;j<twos.size();j++){
      double indicator = pointPolygonTest(contours,twos[j],false);
      if(indicator>=0){
        if(armors[i].num == 1) armors[i].num = 0;
        else armors[i].num = 2;
        break;
      }  
    }
  }

  //接着根据历史数据进行标注，选取迭代数最少的满足要求的历史结果进行标注。
  for (unsigned int i = 0; i < armors.size(); i++) {
    if(armors[i].num != 0) continue;//若已通过当前帧进行过标注则直接略过
    cv::Point2f corners[4];
    armors[i].rect.points(corners);
    
    cv::Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
    std::vector<Point2f> contours(corners , lastItemPointer);

    for(int j=historical_nums.size()-1;j>=0;j--){
      double indicator = pointPolygonTest(contours,historical_nums_position[j],false);
      if(indicator>=0){
        if(historical_nums[j] == 1) armors[i].num = 1;
        else armors[i].num = 2;
        break;
      }
    }
  }

  //最后更新历史信息，此外仅保留最近的num_cell_max个历史信息
  for(int j=0;j<ones.size();j++){
    if(historical_nums.size()>=num_cell_max){//如果历史序列已满，则首先清除历史项，再添加新项
      historical_nums_iternum.erase(historical_nums_iternum.begin());
      historical_nums.erase(historical_nums.begin());
      historical_nums_position.erase(historical_nums_position.begin());
    }
    historical_nums.push_back(1);
    historical_nums_iternum.push_back(0);
    historical_nums_position.push_back(ones[j]);
  }

  //最后更新历史信息，此外仅保留最近的num_cell_max个历史信息
  for(int j=0;j<twos.size();j++){
    if(historical_nums.size()>=num_cell_max){//如果历史序列已满，则首先清除历史项，再添加新项
      historical_nums_iternum.erase(historical_nums_iternum.begin());
      historical_nums.erase(historical_nums.begin());
      historical_nums_position.erase(historical_nums_position.begin());
    }
    historical_nums.push_back(2);
    historical_nums_iternum.push_back(0);
    historical_nums_position.push_back(twos[j]);
  }

  //最后的最后，更新迭代次数信息，并将历史信息里大于一个预设迭代值的过于久远历史信息去除
  for(int j=0;j<historical_nums_iternum.size();j++) historical_nums_iternum[j]++;

  while(historical_nums_iternum.size()>0){
    if(historical_nums_iternum[0]>num_iternum_max){
      historical_nums_iternum.erase(historical_nums_iternum.begin());
      historical_nums.erase(historical_nums.begin());
      historical_nums_position.erase(historical_nums_position.begin());
    }
    else break;
  }

}

void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) {
  //std::cout << "********************************************FilterArmors********************************************" << std::endl;
  cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8UC1);
  for (auto armor_iter = armors.begin(); armor_iter != armors.end();) {
    cv::Point pts[4];
    for (unsigned int i = 0; i < 4; i++) {
      pts[i].x = (int) armor_iter->vertex[i].x;
      pts[i].y = (int) armor_iter->vertex[i].y;
    }
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255), 8, 0);

    cv::Mat mat_mean;
    cv::Mat mat_stddev;
    cv::meanStdDev(gray_img_, mat_mean, mat_stddev, mask);

    auto stddev = mat_stddev.at<double>(0, 0);
    auto mean = mat_mean.at<double>(0, 0);
    // std::cout << "stddev: " << stddev << std::endl;
     std::cout << "mean: " << mean << std::endl;

    if (stddev > armor_max_stddev_ || mean > armor_max_mean_) {
      armor_iter = armors.erase(armor_iter);
      std::cout<<"filter fot stddev or mean"<<std::endl;
    } else {
      armor_iter++;
    }
  }

  std::vector<bool> is_armor(armors.size(), true);

    //========depth filter============

  for (int i = 0; i < armors.size() ; i++) {
     
    cv::Point2f corners[4];
    armors[i].rect.points(corners);//0为左下角，1为左上，2为右上角

    int yd=armors[i].rect.center.y+roiy_ ;
    int xd=armors[i].rect.center.x;
    float depthz=cv_toolbox_->depthImg.at<ushort>(yd,xd);
    float depthy=(yd-cy_)*depthz/fy_;

    int sumvertex=0;
    int nflag=0;
    if(depthz>4500){
      is_armor[i] = false;
    }
    if(depthz!=0){
      sumvertex=sumvertex/nflag;
      int diff =abs(sumvertex-depthz);
      //  std::cout<<thresh_depth_<<fx_<<"========="<<std::endl;
      if(depthy<thresh_depth_){//225,307   中间大框115
        is_armor[i] = false;
        continue;
      }
      for(int k=0;k<4;k++){
          for(int kk=0;kk<4;kk++){
            if(yd+k<0||yd+k>479||xd+kk<0||xd+kk>639){
              continue;
            }
              float depth3z=cv_toolbox_->depthImg.at<ushort>(yd+k,xd+kk);
              float depth3y=(yd+k-cy_)*depth3z/fy_;
              if(depth3y!=0){
                if(depth3y<thresh_depth_){
                  is_armor[i] = false;
                  break;
                }
              }
          }
      }
          if(!is_armor[i]){
            break;
          }
    }
    else{
      is_armor[i] = false;
       
    }
    if(armors[i].rect.size.width>160){
      is_armor[i] = false;
    }


  }

  

  
  // nms
  
  for (int i = 0; i < armors.size() && is_armor[i] == true; i++) {
    for (int j = i + 1; j < armors.size() && is_armor[j]; j++) {
      float dx = armors[i].rect.center.x - armors[j].rect.center.x;
      float dy = armors[i].rect.center.y - armors[j].rect.center.y;
      float dis = std::sqrt(dx * dx + dy * dy);
      if (dis < armors[i].rect.size.width + armors[j].rect.size.width) {
        if (armors[i].rect.angle > armors[j].rect.angle) {
          
          is_armor[i] = false;
         
          //std::cout << "i: " << i << std::endl;
        } else {
          is_armor[j] = false;
          //std::cout << "j: " << j << std::endl;
        }
      }
    }
  }

  //std::cout << armors.size() << std::endl;
  for (unsigned int i = 0; i < armors.size(); ) {
    if (!is_armor[i]) {
      armors.erase(armors.begin() + i);
      is_armor.erase(is_armor.begin() + i);
      //std::cout << "index: " << i << std::endl;
    } else {
	if (enable_debug_) {
          cv_toolbox_->DrawRotatedRect(show_armors_after_filter_, armors[i].rect, cv::Scalar(0, 255, 0), 2);
	}
      	i++;
    }

  }
  if (enable_debug_)
    cv::imshow("armors_after_filter", show_armors_after_filter_);
}

// std::vector<ArmorInfo> ConstraintSet::SlectFinalArmor(std::vector<ArmorInfo> &armors) {

//    std::vector<ArmorInfo> final_armor; 
//    std::sort(armors.begin(),
//             armors.end(),
//             [](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });

//   for(int i=0;i<armors.size();i++){
//     if(armors[i].num==1){
//       final_armor.push_back(armors[i]);
//       break;
//     }
//   }
//   for(int i=0;i<armors.size();i++){
//     if(armors[i].num==2){
//       final_armor.push_back(armors[i]);
//       break;
//     }
//   }

//   if(final_armor.size()==0) 
//   for(int i=0;i<armors.size();i++){
//     final_armor.push_back(armors[i]);
//   }
//   return final_armor;
// }
std::vector<ArmorInfo> ConstraintSet::SlectFinalArmor(std::vector<ArmorInfo> &armors) {
   std::vector<cv::Point2f> final_armor_center; 
   std::vector<ArmorInfo> final_armor0,final_armor1,final_armor2,final_armor; 
   std::vector<ArmorInfo> final_armor_fine; 

   std::sort(armors.begin(),
            armors.end(),
            [](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });

    int armor_1_size = 0;
    int armor_2_size = 0;
    int armor_0_size = 0;
    
      for(int i=0;i<armors.size();i++){
    if(armors[i].num==1){
       armor_1_size++;
       final_armor1.push_back(armors[i]);
       
       std::cout<<"armor_num1_"<<i<<" : "<<armors[i].rect.size.area()<<std::endl;
       if(armor_1_size>1)
       { 
         if((left_flag&&(final_armor1[0].rect.center.x<final_armor1[1].rect.center.x)))
         { 
           
          final_armor1.pop_back();
          final_armor.pop_back();
          final_armor.push_back(final_armor1[0]);
          break;
         }
         
         else if(left_flag)//if last time choose the left one but this time the left area is smaller than the right one's
         {

          if((final_armor1[1].rect.size.area()/final_armor1[0].rect.size.area())>0.6)//left.area()/right.area()>0.8 
          {//select the left but not largest one,left_flag= true
            
            final_armor1[0] = final_armor1[1];
            final_armor1.pop_back();
            final_armor.pop_back();
            final_armor.push_back(final_armor1[0]);
            break;
          }
          else
          {//selectthe right and largest one,left_flag=false
            final_armor1.pop_back();  
            left_flag = false;
            final_armor.pop_back();
            final_armor.push_back(final_armor1[0]);
            break;
          }
          //final_armor[1] = fiinal_armor[0];
          
          }
          if(((left_flag==false)&&(final_armor1[0].rect.center.x>final_armor1[1].rect.center.x)))
         { 
           //std::cout<<" right,don't change!"<<std::endl;
           left_flag=false;
          //do not change the value of the left_flag just choose the first one
          final_armor1.pop_back();
          final_armor.pop_back();
          final_armor.push_back(final_armor1[0]);
          //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
          break;
         }
         else if (!left_flag)//last time is the right one,but this time the largest is the left one
          {
            if((final_armor1[1].rect.size.area()/final_armor1[0].rect.size.area())>0.6)//left.area()/right.area()>0.8 
          {//select the left but not largest one,left_flag= true
            //std::cout<<"right,area decrease don't change!"<<std::endl;
            final_armor1[0] = final_armor1[1];
            //std::vector<ArmorInfo>::iterator iter = final_armor.begin();
            //final_armor.erase(iter);
            //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
            final_armor1.pop_back();
            final_armor.pop_back();
            final_armor.push_back(final_armor1[0]);
            break;
          }
          else
          {//select the left and largest one,left_flag=true
            final_armor1.pop_back();
            //std::cout<<"right,change!"<<std::endl;
            left_flag = true;
            final_armor.pop_back();
            final_armor.push_back(final_armor1[0]);
            //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
            break;
          }

          }
        
       }
       if(armor_1_size==1)
       {
         final_armor.push_back(armors[i]);
       }
       //final_armor_center.clear();
       //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
       
    }
  }
    
      
   
  for(int i=0;i<armors.size();i++){
    if(armors[i].num==2){
       armor_2_size++;
       final_armor2.push_back(armors[i]);
       
      //  std::cout<<"armor_num2_"<<i<<" : "<<armors[i].rect.size.area()<<std::endl;
       if(armor_2_size>1)
       { 
         if((left_flag&&(final_armor2[0].rect.center.x<final_armor2[1].rect.center.x)))
         { 
           
          final_armor2.pop_back();
          final_armor.pop_back();
          final_armor.push_back(final_armor2[0]);
          break;
         }
         
         else if(left_flag)//if last time choose the left one but this time the left area is smaller than the right one's
         {

          if((final_armor2[1].rect.size.area()/final_armor2[0].rect.size.area())>0.6)//left.area()/right.area()>0.8 
          {//select the left but not largest one,left_flag= true
            
            final_armor2[0] = final_armor2[1];
            final_armor2.pop_back();
            final_armor.pop_back();
            final_armor.push_back(final_armor2[0]);
            break;
          }
          else
          {//selectthe right and largest one,left_flag=false
            final_armor2.pop_back();  
            left_flag = false;
            final_armor.pop_back();
            final_armor.push_back(final_armor2[0]);
            break;
          }
          //final_armor[1] = fiinal_armor[0];
          
          }
          if(((left_flag==false)&&(final_armor2[0].rect.center.x>final_armor2[1].rect.center.x)))
         { 
           //std::cout<<" right,don't change!"<<std::endl;
           left_flag=false;
          //do not change the value of the left_flag just choose the first one
          final_armor2.pop_back();
          final_armor.pop_back();
          final_armor.push_back(final_armor2[0]);
          //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
          break;
         }
         else if (!left_flag)//last time is the right one,but this time the largest is the left one
          {
            if((final_armor2[1].rect.size.area()/final_armor2[0].rect.size.area())>0.6)//left.area()/right.area()>0.8 
          {//select the left but not largest one,left_flag= true
            //std::cout<<"right,area decrease don't change!"<<std::endl;
            final_armor2[0] = final_armor2[1];
            //std::vector<ArmorInfo>::iterator iter = final_armor.begin();
            //final_armor.erase(iter);
            //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
            final_armor2.pop_back();
            final_armor.pop_back();
            final_armor.push_back(final_armor2[0]);
            break;
          }
          else
          {//select the left and largest one,left_flag=true
            final_armor2.pop_back();
            //std::cout<<"right,change!"<<std::endl;
            left_flag = true;
            final_armor.pop_back();
            final_armor.push_back(final_armor2[0]);
            //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
            break;
          }

          }
        
       }
       if(armor_2_size==1)
       {
         final_armor.push_back(armors[i]);
       }
       //final_armor_center.clear();
       //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
    }
  }
  
    for(int i=0;i<armors.size();i++){
    if(armors[i].num==0){
       armor_0_size++;
       final_armor0.push_back(armors[i]);
       
      //  std::cout<<"armor_num0_"<<i<<" : "<<armors[i].rect.size.area()<<std::endl;
       if(armor_0_size>1)
       { 
         if((left_flag&&(final_armor0[0].rect.center.x<final_armor0[1].rect.center.x)))
         { 
           
          final_armor0.pop_back();
          final_armor.pop_back();
          final_armor.push_back(final_armor0[0]);
          break;
         }
         
         else if(left_flag)//if last time choose the left one but this time the left area is smaller than the right one's
         {

          if((final_armor0[1].rect.size.area()/final_armor0[0].rect.size.area())>0.6)//left.area()/right.area()>0.8 
          {//select the left but not largest one,left_flag= true
            
            final_armor0[0] = final_armor0[1];
            
            final_armor0.pop_back();
            final_armor.pop_back();
            final_armor.push_back(final_armor0[0]);
            break;
          }
          else
          {//selectthe right and largest one,left_flag=false
            final_armor0.pop_back(); 
            final_armor.pop_back();
            final_armor.push_back(final_armor0[0]); 
            left_flag = false;
            break;
          }
          //final_armor[1] = fiinal_armor[0];
          
          }
          if(((left_flag==false)&&(final_armor0[0].rect.center.x>final_armor0[1].rect.center.x)))
         { 
           //std::cout<<" right,don't change!"<<std::endl;
           left_flag=false;
          //do not change the value of the left_flag just choose the first one
          final_armor0.pop_back();
          final_armor.pop_back();
          final_armor.push_back(final_armor0[0]);
          //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
          break;
         }
         else if (!left_flag)//last time is the right one,but this time the largest is the left one
          {
            if((final_armor0[1].rect.size.area()/final_armor0[0].rect.size.area())>0.6)//left.area()/right.area()>0.8 
          {//select the left but not largest one,left_flag= true
            //std::cout<<"right,area decrease don't change!"<<std::endl;
            final_armor0[0] = final_armor0[1];
            final_armor0.pop_back();
            //std::vector<ArmorInfo>::iterator iter = final_armor.begin();
            //final_armor.erase(iter);
            //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
            final_armor.pop_back();
            final_armor.push_back(final_armor0[0]);
            final_armor0.pop_back();
            break;
          }
          else
          {//select the left and largest one,left_flag=true
            final_armor0.pop_back();
            final_armor.pop_back();
            final_armor.push_back(final_armor0[0]);
            //std::cout<<"right,change!"<<std::endl;
            left_flag = true;
            //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
            break;
          }

          }
        
       }
       if(armor_0_size==1)
       {
         final_armor.push_back(armors[i]);
       }
       //final_armor_center.clear();
       //std::cout<<"final_armor size is :"<<final_armor.size()<<std::endl;
    }
  }
  // std::cout<<"left_flag:"<<left_flag<<std::endl;
  // std::cout<<"final_armor size:"<<final_armor.size()<<std::endl;
  // std::cout<<"select armor center:"<<final_armor[0].rect.center.x;

  final_armor_center.clear(); 
  final_armor0.clear();
  final_armor1.clear();
  final_armor2.clear(); 
  final_armor_fine.clear(); 
  
  return final_armor;

  
}


void ConstraintSet::CalcControlInfo( ArmorInfo & armor) {
  cv::Mat rvec;
  cv::Mat tvec;
  
  //=================================================depth================
  
  
  int yd=armor.rect.center.y+roiy_;
  int xd=armor.rect.center.x;
  float depthz=cv_toolbox_->depthImg.at<ushort>(yd,xd);
  float depthy=(yd-240)*depthz/615;
  if(depthy!=0){
  float depthx=(xd-320)*depthz/615;
  // cv::line(depth_img_,cv::Point(xd-10,yd),cv::Point(xd+10,yd),cv::Scalar(255),3);
  // cv::line(depth_img_,cv::Point(xd,yd-10),cv::Point(xd,yd+10),cv::Scalar(255),3);
  // cv::imshow("depth",depth_img_*20);
  armor.target_3d.z=depthz;
  armor.target_3d.y=depthy;
  armor.target_3d.x=depthx;
  // std::cout<<"==========from depth map=========="<<std::endl;
  std::cout<<"x-"<<armor.target_3d.x<<std::endl;
  std::cout<<"y-"<<armor.target_3d.y<<std::endl;
  std::cout<<"z-"<<armor.target_3d.z<<std::endl;
  }else{
  for (unsigned int i = 0; i < 4; i++) {
      armor.vertex[i].y=armor.vertex[i].y+roiy_;
    }
  cv::solvePnP(armor_points_,
               armor.vertex,
               intrinsic_matrix_,
               distortion_coeffs_,
               rvec,
               tvec);
  armor.target_3d = cv::Point3f(tvec);
  // std::cout<<"==========from PnP=========="<<std::endl;
  // std::cout<<"x-"<<armor.target_3d.x<<std::endl;
  // std::cout<<"y-"<<armor.target_3d.y<<std::endl;
  // std::cout<<"z-"<<armor.target_3d.z<<std::endl;
  }
  

}

void ConstraintSet::CalcArmorInfo(std::vector<cv::Point2f> &armor_points,
                                 cv::RotatedRect left_light,
                                 cv::RotatedRect right_light) {
  cv::Point2f left_points[4], right_points[4];
  left_light.points(left_points);
  right_light.points(right_points);

  cv::Point2f right_lu, right_ld, lift_ru, lift_rd;
  std::sort(left_points, left_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  std::sort(right_points, right_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  if (right_points[0].y < right_points[1].y) {
    right_lu = right_points[0];
    right_ld = right_points[1];
  } else {
    right_lu = right_points[1];
    right_ld = right_points[0];
  }

  if (left_points[2].y < left_points[3].y) {
    lift_ru = left_points[2];
    lift_rd = left_points[3];
  } else {
    lift_ru = left_points[3];
    lift_rd = left_points[2];
  }
  armor_points.push_back(lift_ru);
  armor_points.push_back(right_lu);
  armor_points.push_back(right_ld);
  armor_points.push_back(lift_rd);

}

void ConstraintSet::SolveArmorCoordinate(const float width,
                                         const float height) {
  armor_points_.emplace_back(cv::Point3f(-width/2, height/2,  0.0));
  armor_points_.emplace_back(cv::Point3f(width/2,  height/2,  0.0));
  armor_points_.emplace_back(cv::Point3f(width/2,  -height/2, 0.0));
  armor_points_.emplace_back(cv::Point3f(-width/2, -height/2, 0.0));
}

void ConstraintSet::SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff) {
  if(fabs(new_num - old_num) > max_diff && filter_count < 2) {
    filter_count++;
    new_num += max_diff;
  } else {
    filter_count = 0;
    old_num = new_num;
  }
}

void ConstraintSet::SetThreadState(bool thread_state) {
  thread_running_ = thread_state;
}

/************输入一个彩色图和两个空的序列，返回识别出的装甲板的中心坐标序列*************/
void ConstraintSet::detect12FromImage(Mat colorImg, vector<Point2f>& ones, vector<Point2f>& twos)
{
    //获得灰度图
    Mat gray;
    cv::cvtColor(colorImg, gray, CV_BGR2GRAY);
    double mean = cv::mean(gray)[0];
    // std::cout <<"mean:" <<mean <<endl;

    //获得canny边缘检测结果
    Mat canny;
    Canny(gray, canny, 150, 100);
    //Canny(gray, canny, 70, 50);//night
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
    dilate(canny, canny, element);//对canny膨胀，可防止提取轮廓时不闭合的情况
    //imshow("canny",canny);
    //imshow("gray",gray);


    //从canny图中提取轮廓
    vector< vector<Point> > contours;
    findContours(canny, contours, noArray(), CV_RETR_LIST, CHAIN_APPROX_SIMPLE);//还可以用CV_CHAIN_APPROX_NONE

    //temp是用来给人看的，比赛时可注释掉
    //vector< vector<Point> > tempContours;//可注释，存储经过面积筛选的轮廓结果
   // Mat tempImage = colorImg.clone();//可注释

    //初始化两个序列用来存储1和2的检测结果
    sortLinkList *one_sort = new sortLinkList;
    one_sort->next = NULL;
    sortLinkList *two_sort = new sortLinkList;
    two_sort->next = NULL;

    // vector<Mat> bgr;
    // split(colorImg, bgr);

    // Mat redImg,blueImg;
    // redImg = bgr[2]-bgr[0];
    // blueImg = bgr[0]-bgr[2];

    // threshold(redImg,redImg,50,255,CV_THRESH_BINARY);
    // threshold(blueImg,blueImg,50,255,CV_THRESH_BINARY);
    // imshow("redImg",redImg);
    // imshow("blueImg",blueImg);

    //在这个循环里获得候选矩形框并存入链表，从而进行下一步非极大值抑制
    for(unsigned int i=0;i<contours.size();i++)
    {
        double contArea = contourArea(contours[i]);//这个函数能够估计一个轮廓占的像素面积
        if(contArea>80)//用面积做初步筛选，剔除无关轮廓,500是经验值，可以调整&&contArea<1500
        {
            //由于轮廓序列存储的是轮廓点在原图的坐标，因此先生成一个等大空图，画上轮廓，再从中截取轮廓外接矩形部分，resize成固定大大小进行HOG识别。
	          Rect boundRect = boundingRect(Mat(contours[i]));
	          if(boundRect.width>boundRect.height*1.2) continue;

      // int brightCount = 0;
      // for(int k=boundRect.x;k<boundRect.x+boundRect.width;k++)
      // for(int l=boundRect.y;l<boundRect.y+boundRect.height;l++)
      // {
      //   if(gray.at<uchar>(k,l)>150) brightCount++;
      // }
      // if(brightCount > boundRect.width*boundRect.height/16) {
      //   continue;
      //   std::cout<<"erase for bright"<<std::endl;}

           // tempContours.push_back(contours[i]);
            Mat tempGray = Mat::zeros(gray.rows, gray.cols, CV_8UC1);
            drawContours(tempGray, contours, i, cv::Scalar(255));
            
            Mat grayCandidate = tempGray(boundRect).clone();

            Mat element = getStructuringElement(MORPH_ELLIPSE, Size(grayCandidate.rows/HogLength+1, grayCandidate.cols/HogHeight+1));//先膨胀处理使得resize后的24x32图中各边缘尽量等宽
            dilate(grayCandidate, grayCandidate, element);
            resize(grayCandidate, grayCandidate, Size( HogLength, HogHeight ) );//24x32
                
            //由于是三分类识别，SVM会在两两类间共建立3个分类器，我们将对应分类结果中置信度的低值作为该类的分类置信度
            svm_node* node = getHogFeatures( grayCandidate );
            int vote_max_idx;//分类器投票结果，通过这个值找出与当前类相关的两个置信度值
            double *dec_values = new double[model->nr_class*(model->nr_class-1)/2];//存储3个分类器的置信度值，需要处理正负
            double pred2 = svm_predict_values(model, node, dec_values, vote_max_idx);
            free(node);

            double predNum;//存储置信度值
            if(vote_max_idx == 0)       predNum = min(dec_values[0],dec_values[1]);
            else if(vote_max_idx == 1)  predNum = min(-dec_values[0],dec_values[2]);
            else                        predNum = min(-dec_values[1],-dec_values[2]);
            free(dec_values);


            //这段程序有风险，尤其是在当1/2在右侧和下侧边缘的时候！！需要测试！！！
            //将框的x放大到三倍，从而获取颜色信息,这里三倍宽的数字1矩形仍然框不住灯柱，可考虑改成数字2三倍，数字1五倍
	    Rect tempBoundRect = boundRect;
            // if(tempBoundRect.x < tempBoundRect.width)
            // {
            //     tempBoundRect.width += tempBoundRect.x + tempBoundRect.width*2;
            //     tempBoundRect.x = 0;
            // }
            // else if(tempBoundRect.x + tempBoundRect.width*2 > colorImg.cols)
            // {
            //     tempBoundRect.width = colorImg.cols - tempBoundRect.x + tempBoundRect.width;
            //     tempBoundRect.x -= tempBoundRect.width;
            // }
            // else
            // {
            //     tempBoundRect.x -= tempBoundRect.width;
            //     tempBoundRect.width = tempBoundRect.width*3;
            // }

            if(pred2!=0)//如果检测到了数字则进行颜色筛选
            {
                // unsigned int redCount=0,blueCount=0;
                // for(int k=tempBoundRect.x;k<tempBoundRect.x+tempBoundRect.width;k++)
                // for(int j=tempBoundRect.y;j<tempBoundRect.y+tempBoundRect.height;j++)
                // {
                //     if(redImg.at<uchar>(j,k)>127) redCount++;
                //     if(blueImg.at<uchar>(j,k)>127) blueCount++;
                // }
                // //if(blueCount>3*redCount&&enemy_color_==BLUE) rectangle(tempImage,boundRect,Scalar(255,0,0),4);//如果蓝色比红色多三倍,且需要识别蓝色
                // //else if(redCount>3*blueCount&&enemy_color_==RED) rectangle(tempImage,boundRect,Scalar(0,0,255),4);//如果红色比蓝色多三倍,且需要识别红色
                // if(!((blueCount>2*redCount&&enemy_color_==BLUE)||(redCount>2*blueCount&&enemy_color_==RED))) continue;

                //根据SVM结果将信息存储于对应链表中
                if(pred2>0)
                {
                    sortLinkList *p;
                    for (p = one_sort;p->next!=NULL;p=p->next)
                    if(p->next->probablity<predNum) break;
                    sortLinkList *q = new sortLinkList;
                    q->next = p->next;
                    p->next = q;
                    q->probablity = predNum;
                    q->rect = boundRect;
                    //cout <<"One contour area:" <<contArea <<endl;
                }
                else if(pred2<0)
                {
                    sortLinkList *p;
                    for (p = two_sort;p->next!=NULL;p=p->next)
                    if(p->next->probablity<predNum) break;
                    sortLinkList *q = new sortLinkList;
                    q->next = p->next;
                    p->next = q;
                    q->probablity = predNum;
                    q->rect = boundRect;
                    //cout <<"Two contour area:" <<contArea <<endl;
                }
            }
            
            
        }
    }

    NonMaximumSuppression(one_sort);//非极大值抑制
    NonMaximumSuppression(two_sort);//非极大值抑制

    //返回检测到的1和2的中心点坐标
    for(sortLinkList *p = one_sort->next;p!=NULL;p = p->next)
    {
        Point2f one;
        one.x = p->rect.x+p->rect.width/2;
        one.y = p->rect.y+p->rect.height/2;
        ones.push_back(one);
    }
    for(sortLinkList *p = two_sort->next;p!=NULL;p = p->next)
    {
        Point2f two;
        two.x = p->rect.x+p->rect.width/2;
        two.y = p->rect.y+p->rect.height/2;
        twos.push_back(two);
    }


    //for(sortLinkList *p = one_sort->next;p!=NULL;p = p->next) rectangle(tempImage,p->rect,Scalar(0,255,0),2);//可注释，在图中画出识别出的1和2外接矩形框
    //for(sortLinkList *p = two_sort->next;p!=NULL;p = p->next) rectangle(tempImage,p->rect,Scalar(255,0,0),2);//可注释，在图中画出识别出的1和2外接矩形框
    //drawContours(tempImage, tempContours, -1, cv::Scalar(0,0,255));//可注释，在图中画出识别出的1和2外接矩形框
    //cv::imshow("Contours", tempImage);//可注释，在图中画出识别出的1和2外接矩形框
}


void ConstraintSet::NonMaximumSuppression(sortLinkList *possib_sort)
{
    for(sortLinkList *p = possib_sort->next;p!=NULL;p = p->next)//非极大值抑制
    {
        for(sortLinkList *q = p;q->next!=NULL;)
        {
            int width;
            if(p->rect.x>q->next->rect.x) width = (p->rect.x-q->next->rect.x)<q->next->rect.width?q->next->rect.width-p->rect.x+q->next->rect.x:0;
            else width = (q->next->rect.x-p->rect.x)<p->rect.width?p->rect.width-q->next->rect.x+p->rect.x:0;
            int height;
            if(p->rect.y>q->next->rect.y) height = (p->rect.y-q->next->rect.y)<q->next->rect.height?q->next->rect.height-p->rect.y+q->next->rect.y:0;
            else height = (q->next->rect.y-p->rect.y)<p->rect.height?p->rect.height-q->next->rect.y+p->rect.y:0;
            if(width*height*2>p->rect.width*p->rect.height)//如果重叠面积大于判断值较大那个的四分之一，则去除。
            {
                sortLinkList *r = q->next;
                q->next = r->next;
                free(r);
            }
            else q = q->next;
        }
    }

}

svm_node* ConstraintSet::getHogFeatures(Mat grayImg)
{  

//  HOGDescriptor hog(Size win_size=Size(96, 160),          //win_size：检测窗口大小。
// 　　　　　　　　　　　　　　　　　　　　　　Size block_size=Size(16, 16),      //block_size：块大小，目前只支持Size(16, 16)。
// 　　　　　　　　　　　　　　　　　　　　　　Size block_stride=Size(8, 8),          //block_stride：块的滑动步长，大小只支持是单元格cell_size大小的倍数。
// 　　　　　　　　　　　　　　　　　　　　　　Size cell_size=Size(8, 8),             //cell_size：单元格的大小，目前只支持Size(8, 8)。
// 　　　　　　　　　　　　　　　　　　　　　　int nbins=9,                           //nbins：直方图bin的数量(投票箱的个数)，目前每个单元格Cell只支持9个。
//                    int derivAperture = 1,
// 　　　　　　　　　　　　　　　　　　　　　　double win_sigma=DEFAULT_WIN_SIGMA,    //win_sigma：高斯滤波窗口的参数。
//                    int histogramNormType=HOGDescriptor::L2Hys,
// 　　　　　　　　　　　　　　　　　　　　　　double threshold_L2hys=0.2,            //threshold_L2hys：块内直方图归一化类型L2-Hys的归一化收缩率
// 　　　　　　　　　　　　　　　　　　　　　　bool gamma_correction=true,            //gamma_correction：是否gamma校正
// 　　　　　　　　　　　　　　　　　　　　　　int nlevels=DEFAULT_NLEVELS,           //nlevels：检测窗口的最大数量
//                    bool notknow = false);
    HOGDescriptor hog(Size(HogLength,HogHeight),Size(16, 16),Size(8, 8),Size(8, 8),9);
    vector<float> descriptors;//HOG描述子向量
    hog.compute(grayImg,descriptors,Size(8,8));

    svm_node* node = new svm_node[1 + featureDim_];//Save data in svm format
    for(unsigned int j=0; j<featureDim_; j++)
    {
        node[j].index = j + 1;
        node[j].value = descriptors[j];
    }
    node[featureDim_].index = -1;

    return node;
}


ConstraintSet::~ConstraintSet() {
  
}
} //namespace roborts_detection
