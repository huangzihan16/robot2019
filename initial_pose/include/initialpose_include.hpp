#ifndef INITIALPOSE_INCLUDE_H
#define INITIALPOSE_INCLUDE_H

#define CHECK_VALUE

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>

#include <roborts_msgs/GameStatus.h>

#include <vector>
#include <string.h>
#include <math.h>
#include <memory>
#include <queue>
#include <thread>
#include <mutex>
#include <boost/timer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include </usr/local/include/g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <cmath>
#include <chrono>

#endif