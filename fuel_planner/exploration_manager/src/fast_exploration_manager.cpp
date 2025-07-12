// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
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
  ROS_WARN("Placeholder: Searching for best hover point...");
  // TODO: 在這裡實現詳細的搜尋邏輯
  // 目前直接返回目標點作為懸停點
  best_point = target_center;
  ROS_WARN("Placeholder: Found a potential hover point.");
  return true;
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

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

void FastExplorationManager::setTargetPoint(const geometry_msgs::PointStamped& pt) {
  target_point_ = pt;
  has_target_ = true;
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
        // TODO: 調用 planner_manager_ 規劃到 hover_point 的路徑
        ROS_WARN("Placeholder: Planning path to hover point...");
        return SUCCEED;
      } else {
        ROS_ERROR("Failed to find a suitable hover point in the target area.");
        return FAIL;
      }
    } else {
      ROS_WARN("Navigating towards target area...");
      // TODO: 調用 planner_manager_ 規劃到 target_area_center 的路徑
      ROS_WARN("Placeholder: Planning path to target area...");
      return SUCCEED;
    }
  } else {
    // ******** 傳統自由探索邏輯 ********
    ROS_WARN("Executing free exploration logic...");
    return planner_manager_->planExploreMotion(pos, vel, acc, yaw);
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

}  // namespace fast_planner
