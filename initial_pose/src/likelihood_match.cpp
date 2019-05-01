#include <likelihood_match.hpp>

CachedDistanceMap::CachedDistanceMap(double scale,
									 double max_dist) {
	scale_ = scale;
	max_dist_ = max_dist;
	cell_radius_ = static_cast<int>(max_dist / scale);
	distances_mat_.resize(cell_radius_ + 2);
	for (auto it = distances_mat_.begin(); it != distances_mat_.end(); ++it) {
		it->resize(cell_radius_ + 2, max_dist_);
	}

	for (int i = 0; i < cell_radius_ + 2; i++) {
		for (int j = 0; j < cell_radius_ + 2; j++) {
			distances_mat_[i][j] = std::sqrt(i * i + j * j);
		}
	}
}

likelihood_match::likelihood_match(const nav_msgs::OccupancyGrid& map_msg, const double max_occ_dist):
																	 sigma_hit_(0.2){
	size_x_ = map_msg.info.width;
	size_y_ = map_msg.info.height;
	scale_ = map_msg.info.resolution;
	origin_x_ = map_msg.info.origin.position.x + (size_x_ / 2) * scale_;
	origin_y_ = map_msg.info.origin.position.y + (size_y_ / 2) * scale_;
	cells_vec_.resize(size_x_ * size_y_);
	max_x_distance_ = static_cast<double>(size_x_) * scale_;
	max_y_distance_ = static_cast<double>(size_y_) * scale_;
	diag_distance_ = sqrt(max_x_distance_ * max_x_distance_ + max_y_distance_ * max_y_distance_);

	std::cout << "Map size_x_:" << size_x_ << "\tsize_y_:" << size_y_ << "\tscale_:" << scale_ << "\torigin_x_:" << origin_x_ << "\torigin_y_:" << origin_y_ << std::endl;
	
	for (int i = 0; i < size_x_ * size_y_; i++) {
		auto tmp_msg = static_cast<int>(map_msg.data[i]);
		if (tmp_msg == 0) {
			cells_vec_[i].occ_state = -1;
		} else if (tmp_msg == 100) {
			cells_vec_[i].occ_state = +1;
		} else {
			cells_vec_[i].occ_state = 0;
		}
		/*
		if (cells_vec_[i].occ_state == -1)
			std::cout << cells_vec_[i].occ_state << " ";
		else
			std::cout << " " << cells_vec_[i].occ_state << " ";
		if ((i+1) % size_x_ == 0)
			std::cout << std::endl;*/
	}
	UpdateCSpace(max_occ_dist);
}

int likelihood_match::GetSizeX() const {
	return size_x_;
}
int likelihood_match::GetSizeY() const {
	return size_y_;
}

const double likelihood_match::GetCellOccDistByIndex(int cell_index) {
	return cells_vec_[cell_index].occ_distance;
}

int likelihood_match::ComputeCellIndexByMap(const int& i, const int& j) {
	return i + j * size_x_;
}

double likelihood_match::GetCellOccDistByCoord(const int& i, const int& j) {
	return GetCellOccDistByIndex(ComputeCellIndexByMap(i, j));
}

bool likelihood_match::CheckIndexFree(const int& i, const int& j) {
	if (cells_vec_[ComputeCellIndexByMap(i, j)].occ_state == -1)
		return true;
	else
		return false;
}

void likelihood_match::UpdateCSpace(double max_occ_dist) {
	mark_vec_ = std::make_unique<std::vector<unsigned char>>();
	mark_vec_->resize(size_x_ * size_y_);
	std::priority_queue<MapCellData, std::vector<MapCellData>, CompareByOccDist> Q;
	max_occ_dist_ = max_occ_dist;
	BuildDistanceMap(scale_, max_occ_dist_);

	// Enqueue all the obstacle cells
	MapCellData cell(0,
				  0,
				  0,
				  0,
				  std::bind(&likelihood_match::GetCellOccDistByCoord, this, std::placeholders::_1, std::placeholders::_2));
	for (int i = 0; i < size_x_; i++) {
		cell.src_i_ = cell.i_ = i;
		for (int j = 0; j < size_y_; j++) {
			auto map_index_tmp = ComputeCellIndexByMap(i, j);
			if (cells_vec_[map_index_tmp].occ_state == +1) {
//				cell.occ_distance = this->cells_vec_[map_index_tmp].occ_distance = 0.0;
				cells_vec_[map_index_tmp].occ_distance = 0.0;
				cell.src_j_ = cell.j_ = j;
				mark_vec_->at(map_index_tmp) = 1;
				Q.push(cell);
			} else {
//				cell.occ_distance = this->cells_vec_[map_index_tmp].occ_distance = max_occ_dist;
				cells_vec_[map_index_tmp].occ_distance = max_occ_dist;
			}
		}
	}

	while (!Q.empty()) {
		MapCellData current_cell_data = Q.top();
		if (current_cell_data.i_ > 0) {
			Enqueue(current_cell_data.i_ - 1, current_cell_data.j_,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q);
		}
		if (current_cell_data.j_ > 0) {
			Enqueue(current_cell_data.i_, current_cell_data.j_ - 1,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q);
		}
		if (current_cell_data.i_ < size_x_ - 1) {
			Enqueue(current_cell_data.i_ + 1, current_cell_data.j_,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q);
		}
		if (current_cell_data.j_ < size_y_ - 1) {
			Enqueue(current_cell_data.i_, current_cell_data.j_ + 1,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q);
		}
		Q.pop();
	}

	cached_distance_map_.reset();
	mark_vec_.reset();
}

void likelihood_match::Enqueue(int i, int j, int src_i, int src_j,
															 std::priority_queue<MapCellData, std::vector<MapCellData>, CompareByOccDist> &Q) {

	auto index = ComputeCellIndexByMap(i, j);
	if (mark_vec_->at(index)) {
		return;
	}

	int di = std::abs(i - src_i);
	int dj = std::abs(j - src_j);
	double distance = cached_distance_map_->distances_mat_[di][dj];

	if (distance > cached_distance_map_->cell_radius_) {
		return;
	}

	cells_vec_[index].occ_distance = distance * scale_;

	MapCellData cell_data(static_cast<unsigned int>(i),
					   static_cast<unsigned int>(j),
					   static_cast<unsigned int>(src_i),
					   static_cast<unsigned int>(src_j),
					   std::bind(&likelihood_match::GetCellOccDistByCoord, this, std::placeholders::_1, std::placeholders::_2));
	Q.push(cell_data);
	mark_vec_->at(index) = 1;

}

void likelihood_match::BuildDistanceMap(double scale, double max_dist) {
	cached_distance_map_.reset();
	if (cached_distance_map_ == nullptr || cached_distance_map_->scale_ != scale || cached_distance_map_->max_dist_) {
		if (cached_distance_map_ != nullptr) {
			cached_distance_map_.reset();
		}
		cached_distance_map_ = std::make_unique<CachedDistanceMap>(scale, max_dist);
	}
}

void likelihood_match::ConvertWorldToMap(const double&x, const double& y, int& mx, int& my) {
	mx = (std::floor((x - origin_x_) / scale_ + 0.5) + size_x_ / 2) + 1;
	my = (std::floor((y - origin_y_) / scale_ + 0.5) + size_y_ / 2) + 1;
}

double likelihood_match::ProbMatch(const std::vector<double>& x, const std::vector<double>& y) {
	int mx, my;
	double log_prob = 0;
	double z_hit_denom = 2 * sigma_hit_ * sigma_hit_;
	for (int i = 0; i < x.size(); i++) {
		ConvertWorldToMap(x[i], y[i], mx, my);
		double distance = GetCellOccDistByCoord(mx, my);
		double prob = std::exp(-(distance * distance) / z_hit_denom);
		log_prob += log(prob);
	}
	log_prob /= x.size();
	return std::exp(log_prob);	
}
