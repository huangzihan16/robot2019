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
/*
float GimbalContrl::GetPitch(float x, float y, float v) {
  float y_temp, y_actual, dy;
  float a;
  y_temp = y;
  // by iteration
  for (int i = 0; i < 20; i++) {
    a = (float) atan2(y_temp, x);
    y_actual = BulletModel(x, v, a);
    dy = y - y_actual;process[master]: started with pid [2406]

    y_temp = y_temp + dy;
    if (fabsf(dy) < 0.001) {
      break;
    }
    //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
  }
  return a;

}*/

/*
void ShootCallBack(const roborts_msgs::Shootfk::ConstPtr& msg)
{
roborts_msgs::ShootCmd srv;
 
    srv.request.mode = msg->mode;
    srv.request.number = msg->number;
}
*/

void GimbalContrl::Transform(cv::Point3f &postion, float &pitch, float &yaw) {
//  pitch =
//      GetPitch((postion.z + offset_.z) / 100, -(postion.y + offset_.y) / 100, 15) + (float)(offset_pitch_ * 3.1415926535 / 180);


//pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);

if(postion.z <=1000 )
{
offset_pitch_ = 3.5;
pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
}

if(postion.z > 1000 && postion.z <=2000)
{
offset_pitch_ = 3;
pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
}


if(postion.z > 2000 && postion.z <=3000)
{
offset_pitch_ = 2.5;
pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
}

if(postion.z > 3000 && postion.z <=4000)
{
offset_pitch_ = 1.5;
pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
}


if(postion.z >4000 )
{
offset_pitch_ = 0.5;
pitch = (float) (atan2(postion.y + offset_.y, postion.z + offset_.z)) + (float)(offset_pitch_ * 3.1415926535 / 180);
}


  //yaw positive direction :anticlockwise
  yaw = -(float) (atan2(postion.x + offset_.x, postion.z + offset_.z)) + (float)(offset_yaw_ * 3.1415926535 / 180);
  yaw = yaw * 0.7;
std::cout<<"pitch is: "<<pitch<<std::endl;
std::cout<<"yaw is: "<<yaw<<std::endl;

}

} // roborts_detection



