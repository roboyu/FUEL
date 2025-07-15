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

// ===================== 新增辅助函数实现 START =====================
bool fast_planner::FastExplorationManager::findGroundHeight(const Vector3d& p_center, Vector3d& ground_pt) {
    // 从当前无人机高度上方2米开始向下搜索
    // 使用函数参数中的当前位置，而不是从ExplorationData获取
    double Z_START = 3.0; // 使用固定高度，实际应该从FSM获取当前位置
    double Z_END = -1.0; // 地图下边界
    double Z_RESOLUTION = sdf_map_->getResolution();

    Vector3d current_pos(p_center.x(), p_center.y(), Z_START);
    while(current_pos.z() > Z_END) {
        // 如果距离值小于等于0，说明我们碰到了表面
        if (sdf_map_->getDistance(current_pos) <= 0.0) {
            ground_pt = current_pos;
            return true;
        }
        current_pos.z() -= Z_RESOLUTION;
    }
    return false; // 搜索到底也没找到地面
}

double fast_planner::FastExplorationManager::calculateFlatness(const Vector3d& center_ground_pt) {
    const double analysis_radius = 0.5; // 在1x1m的区域内分析平坦度
    const double grid_resolution = 0.25; // 采样分辨率
    
    vector<double> z_values;
    z_values.push_back(center_ground_pt.z());

    for (double dx = -analysis_radius; dx <= analysis_radius; dx += grid_resolution) {
        for (double dy = -analysis_radius; dy <= analysis_radius; dy += grid_resolution) {
            if (dx == 0 && dy == 0) continue;

            Vector3d sample_pt_center(center_ground_pt.x() + dx, center_ground_pt.y() + dy, 0);
            Vector3d found_ground_pt;
            if (findGroundHeight(sample_pt_center, found_ground_pt)) {
                z_values.push_back(found_ground_pt.z());
            }
        }
    }

    if (z_values.size() < 5) {
        return 100.0; // 样本太少，认为不平坦
    }

    // 计算z值的均值和标准差
    double sum = std::accumulate(z_values.begin(), z_values.end(), 0.0);
    double mean = sum / z_values.size();
    double sq_sum = std::inner_product(z_values.begin(), z_values.end(), z_values.begin(), 0.0);
    double std_dev = std::sqrt(sq_sum / z_values.size() - mean * mean);

    return std_dev;
}

int fast_planner::FastExplorationManager::planFineDelivery(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_acc) {
    ROS_INFO("[Delivery] Analyzing drop-off area around target...");

    Vector3d target_pos = this->getTargetPosition();
    vector<Vector3d> candidates;
    
    const double search_radius = 5.0;     // 圆形搜索半径
    const double grid_resolution = 0.5;   // 候选点网格分辨率
    const double hover_height = 1.8;      // 投放时的悬停高度

    // Step 1: 在圆形区域内生成候选点
    for (double dx = -search_radius; dx <= search_radius; dx += grid_resolution) {
        for (double dy = -search_radius; dy <= search_radius; dy += grid_resolution) {
            if (dx * dx + dy * dy > search_radius * search_radius) continue;

            Vector3d ground_pt;
            Vector3d p_center(target_pos.x() + dx, target_pos.y() + dy, 0);
            if (findGroundHeight(p_center, ground_pt)) {
                candidates.push_back(ground_pt);
            }
        }
    }

    if (candidates.empty()) {
        ROS_ERROR("[Delivery] Can't find any ground within the search radius.");
        return FAIL;
    }

    // Step 2: 计算每个候选点的各项得分（未归一化）
    vector<double> prox_scores, flat_scores, safe_scores;
    for (const auto& p_cand_ground : candidates) {
        Vector3d p_cand_hover = p_cand_ground + Vector3d(0, 0, hover_height);

        // a) 邻近度得分 (越小越好)
        double dist_to_target = (p_cand_ground.head<2>() - target_pos.head<2>()).norm();
        prox_scores.push_back(dist_to_target);

        // b) 不平坦度得分 (越小越好)
        double flatness = calculateFlatness(p_cand_ground);
        flat_scores.push_back(flatness);

        // c) 不安全性得分 (越小越好)
        double dist_to_obs = edt_environment_->evaluateCoarseEDT(p_cand_hover, -1.0);
        double safety_cost = (dist_to_obs < 0.5) ? 100.0 : 1.0 / dist_to_obs;
        safe_scores.push_back(safety_cost);
    }
    
    // Step 3: 归一化所有成本，并计算综合得分
    auto normalize = [](vector<double>& scores) {
        double min_val = *min_element(scores.begin(), scores.end());
        double max_val = *max_element(scores.begin(), scores.end());
        if (max_val - min_val < 1e-6) return;
        for (auto& score : scores) {
            score = (score - min_val) / (max_val - min_val);
        }
    };

    normalize(prox_scores);
    normalize(flat_scores);
    normalize(safe_scores);

    int best_idx = -1;
    double min_total_cost = std::numeric_limits<double>::max();

    for (int i = 0; i < candidates.size(); ++i) {
        double total_cost = w_proximity_ * prox_scores[i] + w_flatness_ * flat_scores[i] + w_safety_ * safe_scores[i];
        if (total_cost < min_total_cost) {
            min_total_cost = total_cost;
            best_idx = i;
        }
    }

    if (best_idx == -1) {
        ROS_ERROR("[Delivery] No suitable delivery spot found after scoring!");
        return FAIL;
    }

    // Step 4: 找到了最佳点，规划最终轨迹
    Vector3d best_drop_point_ground = candidates[best_idx];
    Vector3d final_hover_goal = best_drop_point_ground + Vector3d(0, 0, hover_height);
    
    ROS_INFO_STREAM("[Delivery] Found best spot! Hovering at: " << final_hover_goal.transpose());

    ed_->next_goal_ = final_hover_goal;

    // 如果距离很近，就不需要复杂的轨迹规划了
    if ((cur_pos - final_hover_goal).norm() < 0.3) {
        // 让FSM知道我们已经到了
        ROS_INFO("[Delivery] Already close to final goal, no trajectory planning needed.");
    } else {
        // Step 1: 创建一个包含起点和终点的基本路径
        vector<Vector3d> path_to_final = { cur_pos, final_hover_goal };
        
        // Step 2: 调用现有的工具函数来处理路径，它会自动添加中间点
        shortenPath(path_to_final);
        
        // Step 3: 将处理好的路径传递给规划器
        ROS_INFO("[Delivery] Planning trajectory with %zu waypoints", path_to_final.size());
        planner_manager_->planExploreTraj(path_to_final, cur_vel, cur_acc, 0.0);
    }

    return FINAL_GOAL_FOUND; // 返回新状态
}
// ===================== 新增辅助函数实现 END =====================

int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();
  ed_->global_tour_.clear();

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  // =================== 任务逻辑分流 ===================
  if (this->hasTarget()) {
    // ========== 新逻辑入口：混合引导探索 =============
    const double SURVEY_RADIUS = 5.0; // 勘探半径
    Vector3d target_pos = this->getTargetPosition();
    double dist_to_target = (pos - target_pos).norm();

    // Step 1: 远距离混合引导探索
    if (dist_to_target > SURVEY_RADIUS) {
      // 保留前沿点检测与候选视点生成
      frontier_finder_->searchFrontiers();
      frontier_finder_->computeFrontiersToVisit();
      frontier_finder_->getFrontiers(ed_->frontiers_);
      frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

      if (ed_->frontiers_.empty()) {
        ROS_WARN("No coverable frontier.");
        return NO_FRONTIER;
      }
      frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
      for (int i = 0; i < ed_->points_.size(); ++i)
        ed_->views_.push_back(
            ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

      // ===================== FIX: START =====================
      // 保持FrontierFinder内部状态同步，防止后续定时器回调崩溃
      if (!ed_->points_.empty()) {
        frontier_finder_->updateFrontierCostMatrix();
      }
      // ===================== FIX: END =======================

      // Step 2: 视点评分与选择
      // 权重参数（可后续参数化）
      const double w_info = 1.0;
      const double w_target = 2.0;
      const double w_cost = 1.0;
      int best_idx = -1;
      double best_score = -1e9;
      for (int i = 0; i < ed_->points_.size(); ++i) {
        // 信息增益（这里用averages_，可根据实际定义）
        double info_gain = (i < ed_->averages_.size()) ? ed_->averages_[i].norm() : 0.0;
        // 目标导向分
        Eigen::Vector3d A = ed_->points_[i] - pos;
        Eigen::Vector3d B = target_pos - pos;
        double target_progress = 0.0;
        if (A.norm() > 1e-3 && B.norm() > 1e-3) {
          target_progress = A.normalized().dot(B.normalized());
        }
        // 飞行代价（距离）
        double travel_cost = A.norm();
        // 综合评分
        double score = w_info * info_gain + w_target * target_progress - w_cost * travel_cost;
        if (score > best_score) {
          best_score = score;
          best_idx = i;
        }
      }
      if (best_idx < 0) {
        ROS_WARN("No valid viewpoint found in guided exploration.");
        return FAIL;
      }
      Vector3d next_pos = ed_->points_[best_idx];
      double next_yaw = ed_->yaws_[best_idx];
      // 后续轨迹生成与原逻辑一致
      // ... 轨迹生成代码 ...
      // 复制原有的轨迹生成部分
      t1 = ros::Time::now();
      double diff = fabs(next_yaw - yaw[0]);
      double time_lb = std::min(diff, 2 * M_PI - diff) / ViewNode::yd_;
      planner_manager_->path_finder_->reset();
      if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END) {
        ROS_ERROR("No path to next viewpoint");
        return FAIL;
      }
      ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
      shortenPath(ed_->path_next_goal_);
      const double radius_far = 5.0;
      const double radius_close = 1.5;
      const double len = Astar::pathLength(ed_->path_next_goal_);
      if (len < radius_close) {
        planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
        ed_->next_goal_ = next_pos;
      } else if (len > radius_far) {
        double len2 = 0.0;
        vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
        for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
          auto cur_pt = ed_->path_next_goal_[i];
          len2 += (cur_pt - truncated_path.back()).norm();
          truncated_path.push_back(cur_pt);
        }
        ed_->next_goal_ = truncated_path.back();
        planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
      } else {
        ed_->next_goal_ = next_pos;
        if (!planner_manager_->kinodynamicReplan(
                pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
          return FAIL;
      }
      if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
        ROS_ERROR("Lower bound not satified!");
      planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);
      double traj_plan_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();
      double yaw_time = (ros::Time::now() - t1).toSec();
      ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
      double total = (ros::Time::now() - t2).toSec();
      ROS_WARN("Total time: %lf", total);
      ROS_ERROR_COND(total > 0.1, "Total time too long!!!");
      return SUCCEED;
    } else {
      // ========== 近距离精细规划模式 =============
      ROS_INFO("[Manager] Switched to fine delivery planning mode. Distance to target: %.2f m", dist_to_target);
      
      // 直接调用精细投放规划函数
      int delivery_result = this->planFineDelivery(pos, vel, acc);
      
      if (delivery_result == FINAL_GOAL_FOUND) {
        ROS_INFO("[Manager] Fine delivery planning completed successfully.");
        return SUCCEED;
      } else {
        ROS_ERROR("[Manager] Fine delivery planning failed with result: %d", delivery_result);
        return FAIL;
      }
    }
  }
  
  // 如果没有目标点，返回失败
  return FAIL;
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
