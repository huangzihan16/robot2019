#ifndef ROBORTS_DECISION_FILTER_H
#define ROBORTS_DECISION_FILTER_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <stdio.h>
using namespace cv;
using namespace std;

class Filter {
public:
  Filter(): stateNum_(4), measureNum_(2), T_(0.03){
    kalmanfilter_.init(stateNum_,measureNum_,0);
    kalmanfilter_.transitionMatrix = (Mat_<float>(4, 4) <<1,T_,0,0,0,1,0,0,0,0,1,T_,0,0,0,1);  //转移矩阵A
    kalmanfilter_.measurementMatrix = (Mat_<float>(2, 4) <<1,0,0,0,0,0,1,0);//测量矩阵H
	  setIdentity(kalmanfilter_.processNoiseCov, Scalar::all(1));                            //系统噪声方差矩阵Q
	  setIdentity(kalmanfilter_.measurementNoiseCov, Scalar::all(1e-2));                        //测量噪声方差矩阵R
	  setIdentity(kalmanfilter_.errorCovPost, Scalar::all(1));                                  //后验估计协方差矩阵P
    kalmanfilter_.statePost = (Mat_<float>(4, 1) <<0,0,0,0);      //初始状态值x(0)
	  measurement = (Mat_<float>(2, 1) <<1,1);                          //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
   }


  ~Filter() = default;

  void Update(cv::Point3f &enemyPosition){
    //2.kalman prediction
	kalmanfilter_.predict(); //返回的是下一时刻的状态值KF.statePost (k+1) 

	
	//3.update measurement
	measurement.at<float>(0) = (float)enemyPosition.z / 1000;
	measurement.at<float>(1) = (float)enemyPosition.x / 1000;		
 
	//4.update
	kalmanfilter_.correct(measurement);  //最终结果是更新后的statePost
  enemyPosition.z = kalmanfilter_.statePost.at<float>(0) * 1000 + kalmanfilter_.statePost.at<float>(1) * 10 * 1000;
  enemyPosition.x = kalmanfilter_.statePost.at<float>(2) * 1000 + kalmanfilter_.statePost.at<float>(3) * 10 * 1000;
  //enemyPosition.z = 150;

  }

  cv::Point3f UpdateNoMesurement(){
    //2.kalman prediction
	kalmanfilter_.predict(); //返回的是下一时刻的状态值KF.statePost (k+1) 
  enemyPosition_.z = kalmanfilter_.statePost.at<float>(0) * 1000 + kalmanfilter_.statePost.at<float>(1) * 10 * 1000;
  enemyPosition_.x = kalmanfilter_.statePost.at<float>(2) * 1000 + kalmanfilter_.statePost.at<float>(3) * 10 * 1000;
  //enemyPosition_.y = 150;
  }


  
  //1.kalman filter setup
  const int stateNum_;                                      //状态值4×1向量(x,y,△x,△y)
  const int measureNum_;                                    //测量值2×1向量(x,y)	
  const int T_;
  KalmanFilter kalmanfilter_;
  Mat measurement;
  cv::Point3f enemyPosition_;


};

#endif //ROBORTS_DECISION_FILTER_H
