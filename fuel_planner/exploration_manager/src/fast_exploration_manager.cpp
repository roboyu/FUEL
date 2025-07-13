// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {
}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FastExplorationManager::setCurrentPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& yaw) {
  current_pos_ = pos;
  current_vel_ = vel;
  current_yaw_ = yaw;
}

bool FastExplorationManager::isNearTargetArea(const Eigen::Vector3d& current_pos, const Eigen::Vector3d& target_center, double radius) const {
  return (current_pos - target_center).norm() < radius;
}

bool FastExplorationManager::findBestHoverPoint(const Eigen::Vector3d& target_center, double search_radius, Eigen::Vector3d& best_point) {
  ROS_WARN("Searching for best hover point around target center: %f, %f, %f", 
           target_center.x(), target_center.y(), target_center.z());
  
  if (!sdf_map_) {
    ROS_ERROR("SDF map not initialized!");
    return false;
  }

  double resolution = sdf_map_->getResolution();
  double step_size = resolution * search_step_multiplier_;
  int steps = static_cast<int>(search_radius / step_size);
  
  std::vector<Eigen::Vector3d> candidate_points;
  std::vector<double> candidate_scores;
  
  // 在搜索半径内进行网格搜索
  for (int x = -steps; x <= steps; ++x) {
    for (int y = -steps; y <= steps; ++y) {
      for (int z = 0; z <= static_cast<int>((max_search_height_ - min_search_height_) / step_size); ++z) {
        Eigen::Vector3d candidate = target_center + Eigen::Vector3d(x * step_size, y * step_size, 
                                                                    min_search_height_ + z * step_size);
        
        // 基础检查
        if ((candidate - target_center).norm() > search_radius || 
            !sdf_map_->isInMap(candidate) ||
            sdf_map_->getInflateOccupancy(candidate) == 1) {
          continue;
        }
        
        // 使用三个公式评估候选点
        double flatness_score = calculateFlatnessScore(candidate);
        double safety_score = calculateSafetyScore(candidate);
        double proximity_score = calculateProximityScore(candidate, target_center, search_radius);
        
        // 综合评分（可配置权重）
        double total_score = flatness_weight_ * flatness_score + 
                           safety_weight_ * safety_score + 
                           proximity_weight_ * proximity_score;
        
        candidate_points.push_back(candidate);
        candidate_scores.push_back(total_score);
        
        ROS_DEBUG("Candidate (%f, %f, %f): flatness=%.3f, safety=%.3f, proximity=%.3f, total=%.3f",
                 candidate.x(), candidate.y(), candidate.z(), 
                 flatness_score, safety_score, proximity_score, total_score);
      }
    }
  }
  
  if (candidate_points.empty()) {
    ROS_ERROR("No suitable hover points found in the target area!");
    return false;
  }
  
  // 选择评分最高的点
  auto max_score_it = std::max_element(candidate_scores.begin(), candidate_scores.end());
  int best_idx = std::distance(candidate_scores.begin(), max_score_it);
  best_point = candidate_points[best_idx];
  
  ROS_WARN("Best hover point found at: %f, %f, %f with score: %f", 
           best_point.x(), best_point.y(), best_point.z(), *max_score_it);
  
  // 可視化懸停點
  visualizeHoverPoint(best_point);
  
  return true;
}



void FastExplorationManager::visualizeTargetPoint() {
  if (!has_target_) return;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "target_point";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.pose.position.x = target_point_.point.x;
  marker.pose.position.y = target_point_.point.y;
  marker.pose.position.z = target_point_.point.z;
  marker.pose.orientation.w = 1.0;
  
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.8;
  
  target_point_pub_.publish(marker);
}

void FastExplorationManager::visualizeSearchArea() {
  if (!has_target_) return;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "search_area";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.pose.position.x = target_point_.point.x;
  marker.pose.position.y = target_point_.point.y;
  marker.pose.position.z = target_point_.point.z;
  marker.pose.orientation.w = 1.0;
  
  marker.scale.x = target_area_radius_ * 2.0;
  marker.scale.y = target_area_radius_ * 2.0;
  marker.scale.z = 0.1;
  
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.3;
  
  search_area_pub_.publish(marker);
}

void FastExplorationManager::visualizeHoverPoint(const Eigen::Vector3d& hover_point) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "hover_point";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.pose.position.x = hover_point.x();
  marker.pose.position.y = hover_point.y();
  marker.pose.position.z = hover_point.z();
  marker.pose.orientation.w = 1.0;
  
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.8;
  
  hover_point_pub_.publish(marker);
}

void FastExplorationManager::updateVisualization() {
  visualizeTargetPoint();
  visualizeSearchArea();
}

void FastExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
  nh.param("exploration/target_area_radius", target_area_radius_, 2.0);

  // 懸停點搜尋相關參數
  nh.param("exploration/min_safety_dist", min_safety_dist_, 0.5);
  nh.param("exploration/max_search_height", max_search_height_, 3.0);
  nh.param("exploration/min_search_height", min_search_height_, 1.0);
  nh.param("exploration/search_step_multiplier", search_step_multiplier_, 2.0);
  nh.param("exploration/stability_radius", stability_radius_, 0.8);
  nh.param("exploration/approach_distance_ratio", approach_distance_ratio_, 0.8);

  // 三個公式的權重參數
  nh.param("exploration/flatness_weight", flatness_weight_, 0.3);
  nh.param("exploration/safety_weight", safety_weight_, 0.4);
  nh.param("exploration/proximity_weight", proximity_weight_, 0.3);

  // 平坦度公式參數
  nh.param("exploration/flatness_check_radius", flatness_check_radius_, 1.0);
  nh.param("exploration/flatness_variance_threshold", flatness_variance_threshold_, 0.1);

  // 安全性公式參數
  nh.param("exploration/safety_check_radius", safety_check_radius_, 0.8);

  // 鄰近度公式參數
  nh.param("exploration/max_height_diff", max_height_diff_, 2.0);
  nh.param("exploration/proximity_distance_weight", proximity_distance_weight_, 0.5);
  nh.param("exploration/proximity_height_weight", proximity_height_weight_, 0.3);
  nh.param("exploration/proximity_accessibility_weight", proximity_accessibility_weight_, 0.2);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";

  // 初始化可視化發布器
  ros::NodeHandle nh_vis;
  target_point_pub_ = nh_vis.advertise<visualization_msgs::Marker>("/target_point_marker", 1);
  search_area_pub_ = nh_vis.advertise<visualization_msgs::Marker>("/search_area_marker", 1);
  hover_point_pub_ = nh_vis.advertise<visualization_msgs::Marker>("/hover_point_marker", 1);

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

void FastExplorationManager::setTargetPoint(const geometry_msgs::PointStamped& pt) {
  target_point_ = pt;
  has_target_ = true;
  
  // 更新可視化
  updateVisualization();
  
  ROS_WARN("Target point set to: %f, %f, %f", pt.point.x, pt.point.y, pt.point.z);
}

bool FastExplorationManager::hasTarget() const {
  return has_target_;
}

Eigen::Vector3d FastExplorationManager::getTargetPosition() const {
  if (!has_target_) return Eigen::Vector3d::Zero();
  return Eigen::Vector3d(target_point_.point.x, target_point_.point.y, target_point_.point.z);
}

int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  // === 根據是否有目標點，選擇不同的探索策略 ===
  if (hasTarget()) {
    setCurrentPose(pos, vel, yaw);
    Eigen::Vector3d target_area_center = getTargetPosition();
    
    if (isNearTargetArea(current_pos_, target_area_center, target_area_radius_)) {
      ROS_WARN("Inside target area. Searching for best hover point...");
      Eigen::Vector3d hover_point;
      if (findBestHoverPoint(target_area_center, target_area_radius_, hover_point)) {
        ROS_WARN_STREAM("Best hover point found at: " << hover_point.transpose());
        
        // 規劃到懸停點的軌跡
        return planToHoverPoint(hover_point, pos, vel, acc, yaw);
      } else {
        ROS_ERROR("Failed to find a suitable hover point in the target area.");
        return FAIL;
      }
    } else {
      ROS_WARN("Navigating towards target area...");
      
      // 規劃到目標區域的軌跡
      return planToTargetArea(target_area_center, pos, vel, acc, yaw);
    }
  } else {
    // ******** 傳統自由探索邏輯 ********
    ROS_WARN("Executing free exploration logic...");
    return planner_manager_->planExploreMotion(pos, vel, acc, yaw);
  }
}

int FastExplorationManager::planToTargetArea(const Eigen::Vector3d& target_center, 
                                            const Vector3d& pos, const Vector3d& vel, 
                                            const Vector3d& acc, const Vector3d& yaw) {
  ROS_WARN("Planning path to target area center: %f, %f, %f", 
           target_center.x(), target_center.y(), target_center.z());
  
  // 計算目標區域邊緣的點（避免直接飛到中心）
  Eigen::Vector3d direction = (target_center - pos).normalized();
  double approach_distance = target_area_radius_ * approach_distance_ratio_;  // 使用配置參數
  Eigen::Vector3d approach_point = target_center - direction * approach_distance;
  
  // 確保接近點在地圖範圍內
  if (!sdf_map_->isInMap(approach_point)) {
    ROS_ERROR("Approach point is outside map bounds!");
    return FAIL;
  }
  
  // 使用kinodynamic replanning規劃到接近點的路徑
  bool success = planner_manager_->kinodynamicReplan(pos, vel, acc, approach_point, 
                                                     Eigen::Vector3d::Zero(), -1.0);
  
  if (success) {
    ROS_WARN("Successfully planned path to target area approach point");
    return SUCCEED;
  } else {
    ROS_ERROR("Failed to plan path to target area");
    return FAIL;
  }
}

int FastExplorationManager::planToHoverPoint(const Eigen::Vector3d& hover_point, 
                                            const Vector3d& pos, const Vector3d& vel, 
                                            const Vector3d& acc, const Vector3d& yaw) {
  ROS_WARN("Planning path to hover point: %f, %f, %f", 
           hover_point.x(), hover_point.y(), hover_point.z());
  
  // 使用kinodynamic replanning規劃到懸停點的路徑
  bool success = planner_manager_->kinodynamicReplan(pos, vel, acc, hover_point, 
                                                     Eigen::Vector3d::Zero(), -1.0);
  
  if (success) {
    ROS_WARN("Successfully planned path to hover point");
    return SUCCEED;
  } else {
    ROS_ERROR("Failed to plan path to hover point");
    return FAIL;
  }
}

void FastExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findGlobalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    vector<int>& indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->updateFrontierCostMatrix();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  if (false) {
    // Use symmetric TSP
    for (int i = 1; i < dimension; ++i) {
      for (int j = 0; j < i; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }

  } else {
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  if (false) {
    // Read path for Symmetric TSP formulation
    getline(res_file, res);  // Skip current pose
    getline(res_file, res);
    int id = stoi(res);
    bool rev = (id == dimension);  // The next node is virutal depot?

    while (id != -1) {
      indices.push_back(id - 2);
      getline(res_file, res);
      id = stoi(res);
    }
    if (rev) reverse(indices.begin(), indices.end());
    indices.pop_back();  // Remove the depot

  } else {
    // Read path for ATSP formulation
    while (getline(res_file, res)) {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1)  // Ignore the current state
        continue;
      if (id == -1) break;
      indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
    }
  }

  res_file.close();

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

void FastExplorationManager::refineLocalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
    const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
    vector<Vector3d>& refined_pts, vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local tour graph: ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

// 公式1：平坦度公式 - 用高度標準差法
// FlatnessScore = 1 / (1 + Z_Standard_Deviation)
double FastExplorationManager::calculateFlatnessScore(const Eigen::Vector3d& candidate) {
  double flatness_radius = flatness_check_radius_;
  double resolution = sdf_map_->getResolution();
  std::vector<double> heights;

  // 以候選點為中心，半徑flatness_radius，提取所有地面高度
  for (double dx = -flatness_radius; dx <= flatness_radius; dx += resolution) {
    for (double dy = -flatness_radius; dy <= flatness_radius; dy += resolution) {
      Eigen::Vector3d sample_point = candidate + Eigen::Vector3d(dx, dy, 0);
      if (sdf_map_->isInMap(sample_point)) {
        double ground_height = findGroundHeight(sample_point);
        if (ground_height > -1000) {
          heights.push_back(ground_height);
        }
      }
    }
  }

  if (heights.size() < 3) {
    return 0.0; // 樣本不足，視為不平坦
  }

  // 計算標準差
  double mean = std::accumulate(heights.begin(), heights.end(), 0.0) / heights.size();
  double sum_sq = 0.0;
  for (double h : heights) sum_sq += (h - mean) * (h - mean);
  double stddev = sqrt(sum_sq / heights.size());

  // 標準差越小越平坦
  double flatness_score = 1.0 / (1.0 + stddev);
  return flatness_score;
}

// 公式2：安全性公式 - 评估周围空间的安全性
double FastExplorationManager::calculateSafetyScore(const Eigen::Vector3d& candidate) {
  double safety_radius = safety_check_radius_;
  double resolution = sdf_map_->getResolution();
  int free_voxels = 0;
  int total_voxels = 0;
  
  // 检查周围空间占用情况
  for (double dx = -safety_radius; dx <= safety_radius; dx += resolution) {
    for (double dy = -safety_radius; dy <= safety_radius; dy += resolution) {
      for (double dz = -safety_radius; dz <= safety_radius; dz += resolution) {
        Eigen::Vector3d check_point = candidate + Eigen::Vector3d(dx, dy, dz);
        
        if (sdf_map_->isInMap(check_point)) {
          total_voxels++;
          if (sdf_map_->getInflateOccupancy(check_point) == 0) {
            free_voxels++;
          }
        }
      }
    }
  }
  
  if (total_voxels == 0) {
    return 0.0;
  }
  
  // 基础安全性分数
  double safety_score = static_cast<double>(free_voxels) / total_voxels;
  
  // 额外检查：确保有足够的垂直空间
  double vertical_space = checkVerticalSpace(candidate);
  safety_score *= vertical_space;
  
  return safety_score;
}

// 公式3：鄰近度公式 - 與目標點距離成反比
// ProximityScore = 1 / (1 + Distance)
double FastExplorationManager::calculateProximityScore(const Eigen::Vector3d& candidate,
                                                      const Eigen::Vector3d& target_center,
                                                      double search_radius) {
  // 歐氏距離
  double distance = (candidate - target_center).norm();
  double proximity_score = 1.0 / (1.0 + distance);
  return proximity_score;
}

// 辅助函数：找到地面高度
double FastExplorationManager::findGroundHeight(const Eigen::Vector3d& point) {
  double max_search_depth = 5.0;  // 最大搜索深度
  double resolution = sdf_map_->getResolution();
  
  for (double z = point.z(); z >= point.z() - max_search_depth; z -= resolution) {
    Eigen::Vector3d check_point(point.x(), point.y(), z);
    if (sdf_map_->isInMap(check_point)) {
      if (sdf_map_->getInflateOccupancy(check_point) == 1) {
        return z + resolution;  // 返回地面高度
      }
    }
  }
  
  return -1000;  // 未找到地面
}

// 辅助函数：检查垂直空间
double FastExplorationManager::checkVerticalSpace(const Eigen::Vector3d& candidate) {
  double min_vertical_space = 1.5;  // 最小垂直空间
  double resolution = sdf_map_->getResolution();
  int free_vertical_voxels = 0;
  int total_vertical_voxels = 0;
  
  for (double dz = 0; dz <= min_vertical_space; dz += resolution) {
    Eigen::Vector3d check_point = candidate + Eigen::Vector3d(0, 0, dz);
    if (sdf_map_->isInMap(check_point)) {
      total_vertical_voxels++;
      if (sdf_map_->getInflateOccupancy(check_point) == 0) {
        free_vertical_voxels++;
      }
    }
  }
  
  return total_vertical_voxels > 0 ? 
         static_cast<double>(free_vertical_voxels) / total_vertical_voxels : 0.0;
}

// 辅助函数：检查可达性
double FastExplorationManager::checkAccessibility(const Eigen::Vector3d& candidate, 
                                                 const Eigen::Vector3d& target_center) {
  if (!ViewNode::caster_) {
    return 0.5;  // 如果raycaster不可用，返回中等分数
  }
  
  // 检查从候选点到目标中心的直线路径是否通畅
  ViewNode::caster_->input(target_center, candidate);
  Eigen::Vector3i idx;
  int obstacle_count = 0;
  int total_points = 0;
  
  while (ViewNode::caster_->nextId(idx)) {
    total_points++;
    if (sdf_map_->getInflateOccupancy(idx) == 1 || 
        sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
      obstacle_count++;
    }
  }
  
  if (total_points == 0) {
    return 0.0;
  }
  
  // 无障碍物比例越高，可达性越好
  return 1.0 - static_cast<double>(obstacle_count) / total_points;
}

}  // namespace fast_planner
