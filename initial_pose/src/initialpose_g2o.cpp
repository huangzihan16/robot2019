#include <initialpose_g2o.hpp>

void BoundaryEdge_Bottom::computeError() {
	const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
	const Eigen::Vector3d pose_estimation = pose->estimate();
	_error(0,0) = pose_estimation(1) + _measurement(1) * sin( _measurement(0) + pose_estimation(2) ) ;
}

void BoundaryEdge_Bottom::linearizeOplus() {
	const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
	const Eigen::Vector3d pose_estimation = pose->estimate();

	_jacobianOplusXi(0,0) = 0;
	_jacobianOplusXi(0,1) = 1;
	_jacobianOplusXi(0,2) = _point_range * cos(_point_theta + pose_estimation(2));
}

void BoundaryEdge_Left::computeError() {
	const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
	const Eigen::Vector3d pose_estimation = pose->estimate();
	_error(0,0) = pose_estimation(0) + _measurement(1) * cos( _measurement(0) + pose_estimation(2) ) ;
}

void BoundaryEdge_Left::linearizeOplus() {
	const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
	const Eigen::Vector3d pose_estimation = pose->estimate();
	
	_jacobianOplusXi(0,0) = 1;
	_jacobianOplusXi(0,1) = 0;
	_jacobianOplusXi(0,2) = -_point_range * sin(_point_theta + pose_estimation(2));
}

void BoundaryEdge_Right::computeError() {
	const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
	const Eigen::Vector3d pose_estimation = pose->estimate();
	_error(0,0) = 8.0 - pose_estimation(0) - _measurement(1) * cos( _measurement(0) + pose_estimation(2) );
	
}

void BoundaryEdge_Right::linearizeOplus() {
	const BoundaryPoseVertex* pose = static_cast<const BoundaryPoseVertex*> (_vertices[0]);
	const Eigen::Vector3d pose_estimation = pose->estimate();
	
	_jacobianOplusXi(0,0) = 0;
	_jacobianOplusXi(0,1) = -1;
	_jacobianOplusXi(0,2) = _point_range * sin(_point_theta + pose_estimation(2));
}