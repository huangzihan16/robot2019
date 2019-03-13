#ifndef LIKELIHOOD_MATCH_H
#define LIKELIHOOD_MATCH_H

#include <initialpose_include.hpp>

class MapCell {
public:
	//occ_state = -1为空闲, 其他则为占用
	int occ_state = 0;

	double occ_distance = 0;
};

class MapCellData {
 public:
  MapCellData(unsigned int i,
           unsigned int j,
           unsigned int src_i,
           unsigned int src_j,
           const std::function<double(unsigned int, unsigned int)> &GetOccDistFunc) {
    i_ = i;
    j_ = j;
    src_i_ = src_i;
    src_j_ = src_j;
    GetOccDist = GetOccDistFunc;
  }

 public:
  std::function<double(unsigned int, unsigned int)> GetOccDist;
  unsigned int i_ = 0;
  unsigned int j_ = 0;
  unsigned int src_i_ = 0;
  unsigned int src_j_ = 0;
};

struct CompareByOccDist {
  bool operator()(const MapCellData &a, const MapCellData &b) {
    return a.GetOccDist(a.i_, a.j_) > b.GetOccDist(b.i_, b.j_);
  };
};

class CachedDistanceMap {
 public:
  CachedDistanceMap(double scale, double max_dist);
 public:
  double scale_ = 0, max_dist_ = 0;
  int cell_radius_ = 0;
  std::vector<std::vector<double>> distances_mat_;
};

class likelihood_match {
public:
	typedef std::shared_ptr<likelihood_match> Ptr;
	likelihood_match(const nav_msgs::OccupancyGrid &map_msg, const double max_occ_dist);
	
	int GetSizeX() const;
	int GetSizeY() const;
	
	bool CheckIndexFree(const int& i, const int& j);
	
	const double GetCellOccDistByIndex(int cell_index);
	
	int ComputeCellIndexByMap(const int& i, const int& j);
	double GetCellOccDistByCoord(const int& i, const int& j);
	
	void ConvertWorldToMap(const double&x, const double& y, int& mx, int& my);
	double ProbMatch(const std::vector<double>& x, const std::vector<double>& y);

private:
	void UpdateCSpace(double max_occ_dist);
	
  void Enqueue(int i, int j, int src_i, int src_j,
							 std::priority_queue<MapCellData, std::vector<MapCellData>, CompareByOccDist> &Q);

  void BuildDistanceMap(double scale, double max_dist);
public:
	int size_x_;
	int size_y_;
	double scale_;
	double origin_x_ = 0;
	double origin_y_ = 0;
	
	std::vector<MapCell> cells_vec_;
	
	double max_x_distance_;
	double max_y_distance_;
	double diag_distance_;
private:
	double sigma_hit_;
	
	double max_occ_dist_;
	std::unique_ptr<CachedDistanceMap> cached_distance_map_;
	std::unique_ptr<std::vector<unsigned char>> mark_vec_;
};

#endif