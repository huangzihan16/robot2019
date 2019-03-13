#ifndef INITIALPOSE_G2O_H
#define INITIALPOSE_G2O_H

#include <initialpose_include.hpp>

class BoundaryPoseVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 重置
    virtual void setToOriginImpl() {
        _estimate << 0,0,0;
    }
     // 更新
    virtual void oplusImpl( const double* update ) {
        _estimate += Eigen::Vector3d(update);
    }
    // 存盘和读盘：留空
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
//_measurement是角度和距离的二维向量
//场地下边的点所构成的图优化边
class BoundaryEdge_Bottom: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, BoundaryPoseVertex>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	BoundaryEdge_Bottom(double point_theta, double point_range):
		BaseUnaryEdge(), _point_theta(point_theta), _point_range(point_range) {}
	// 计算曲线模型误差
	void computeError();
	virtual void linearizeOplus();
	
	virtual bool read( std::istream& in ) {}
	virtual bool write( std::ostream& out ) const {}  
    
public:
	double _point_range;
	double _point_theta;
	//measurement(0)其实和point_theta是一样的，measurement(1)其实和point_range是一样的
};

//场地左边的点所构成的图优化边
class BoundaryEdge_Left: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, BoundaryPoseVertex> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	BoundaryEdge_Left(double point_theta, double point_range): 
		BaseUnaryEdge(), _point_theta(point_theta), _point_range(point_range) {}
	// 计算曲线模型误差
	void computeError();
	virtual void linearizeOplus();
	
	virtual bool read( std::istream& in ) {}
	virtual bool write( std::ostream& out ) const {}  

public:
	double _point_range;
	double _point_theta;
	//measurement(0)其实和point_theta是一样的，measurement(1)其实和point_range是一样的
};

//场地右边的点所构成的图优化边
class BoundaryEdge_Right: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, BoundaryPoseVertex>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	BoundaryEdge_Right(double point_theta, double point_range): 
		BaseUnaryEdge(), _point_theta(point_theta), _point_range(point_range) {}
	// 计算曲线模型误差
	void computeError();
	virtual void linearizeOplus();
	
	virtual bool read( std::istream& in ) {}
	virtual bool write( std::ostream& out ) const {}  
    
public:
	double _point_range;
	double _point_theta;
	//measurement(0)其实和point_theta是一样的，measurement(1)其实和point_range是一样的
};

#endif