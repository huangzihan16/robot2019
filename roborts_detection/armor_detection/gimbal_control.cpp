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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/
#include <cmath>
#include <stdio.h>

#include "gimbal_control.h"
#include "roborts_msgs/ShootCmd.h"
#include "../../roborts_base/gimbal/gimbal.h"

namespace roborts_detection {

void GimbalContrl::Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k) {
  offset_.x = x;
  offset_.y = y;
  offset_.z = z;
  offset_pitch_ = pitch;
  offset_yaw_ = yaw;
  init_v_ = init_v;
  init_k_ = init_k;

}

//air friction is considered
float GimbalContrl::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
  float t, y;
  t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
  y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return y;
}

//x:distance , y: height
float GimbalContrl::GetPitch(float x, float y, float v) {
  float y_temp, y_actual, dy;
  float a;
  y_temp = y;
  // by iteration
  for (int i = 0; i < 20; i++) {
    a = (float) atan2(y_temp, x);
    y_actual = BulletModel(x, v, a);
    dy = y - y_actual;
    y_temp = y_temp + dy;
    if (fabsf(dy) < 0.001) {
      break;
    }
    //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
  }
  return a;

}

/*
void ShootCallBack(const roborts_msgs::Shootfk::ConstPtr& msg)
{
roborts_msgs::ShootCmd srv;
 
    srv.request.mode = msg->mode;
    srv.request.number = msg->number;
}
*/

  float Bound[6]  = {0 , 1000 , 2000 , 3000 , 4000 , 5000};
  float OffsetPitch[6]  = {0, -1, -1.5, -2.5, -3.5, -4.5};

void GimbalContrl::CalcMembership(float value, float *membership, float *bound)
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

void GimbalContrl::Transform(cv::Point3f &postion, float &pitch, float &yaw) {
//  pitch =
//      GetPitch((postion.z + offset_.z) / 100, -(postion.y + offset_.y) / 100, 15) + (float)(offset_pitch_ * 3.1415926535 / 180);


//pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);



    CalcMembership(postion.z ,Membership  ,Bound);
    int i;
    offset_pitch_ = 0;
  	for (i = 0; i < NUM; i++){
  		if (Membership[i] != 0){
        offset_pitch_ += OffsetPitch[i] * Membership[i] / 100;
  		}
    }
    //ROS_INFO("postion.x: %f  postion.y: %f postion.z: %f ", postion.x, postion.y,postion.z);
    //ROS_INFO("postion.z: %f     offset_pitch_: %f", postion.z, offset_pitch_);
    // ROS_INFO("offset_pitch_: %f", offset_pitch_);    
    pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);

// if(postion.z <=1000 )
// {
// offset_pitch_ = 5.5;
// pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
// }

// if(postion.z > 1000 && postion.z <=2000)
// {
// offset_pitch_ = 4.5;
// pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
// }


// if(postion.z > 2000 && postion.z <=3000)
// {
// offset_pitch_ = 3.5                                               ;
// pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
// }

// if(postion.z > 3000 && postion.z <=4000)
// {
// offset_pitch_ = 3;
// pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
// }

// if(postion.z > 4000 && postion.z <=5000)
// {
// offset_pitch_ = 2;
// pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
// }


// if(postion.z >5000 )
// {
// offset_pitch_ = 1;
// pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
// }
// ROS_INFO("offset_pitch_: %f",offset_pitch_);


  //yaw positive direction :anticlockwise
  yaw = -(float) (atan2(postion.x + offset_.x, postion.z + offset_.z)) + (float)(offset_yaw_ * 3.1415926535 / 180);
  yaw = yaw * 1.0;
// std::cout<<"pitch is: "<<pitch<<std::endl;
// std::cout<<"yaw is: "<<yaw<<std::endl;

}

} // roborts_detection



