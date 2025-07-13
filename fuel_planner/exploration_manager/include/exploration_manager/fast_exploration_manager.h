#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <geometry_msgs/PointStamped.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
class FrontierFinder;
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED };

class FastExplorationManager {
public:
  FastExplorationManager();
  ~FastExplorationManager();

  void initialize(ros::NodeHandle& nh);

  int planExploreMotion(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
                        const Vector3d& yaw);

  // 新增：设置目标点接口
  void setTargetPoint(const geometry_msgs::PointStamped& pt);
  bool hasTarget() const;
  Eigen::Vector3d getTargetPosition() const;

  // Benchmark method, classic frontier and rapid frontier
  int classicFrontier(const Vector3d& pos, const double& yaw);
  int rapidFrontier(const Vector3d& pos, const Vector3d& vel, const double& yaw, bool& classic);

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  // unique_ptr<ViewFinder> view_finder_;

  void setCurrentPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& yaw);
  bool isNearTargetArea(const Eigen::Vector3d& current_pos, const Eigen::Vector3d& target_center, double radius) const;
  // 懸停點搜尋器接口
  bool findBestHoverPoint(const Eigen::Vector3d& target_center, double search_radius, Eigen::Vector3d& best_point);

private:
  // 懸停點評估相關函數
  double evaluateHoverPoint(const Eigen::Vector3d& candidate, const Eigen::Vector3d& target_center, double search_radius);
  double calculateVisibilityScore(const Eigen::Vector3d& candidate, const Eigen::Vector3d& target_center, double search_radius);
  double calculateStabilityScore(const Eigen::Vector3d& candidate);

  // 三個核心公式函數
  double calculateFlatnessScore(const Eigen::Vector3d& candidate);
  double calculateSafetyScore(const Eigen::Vector3d& candidate);
  double calculateProximityScore(const Eigen::Vector3d& candidate, const Eigen::Vector3d& target_center, double search_radius);

  // 輔助函數
  double findGroundHeight(const Eigen::Vector3d& point);
  double checkVerticalSpace(const Eigen::Vector3d& candidate);
  double checkAccessibility(const Eigen::Vector3d& candidate, const Eigen::Vector3d& target_center);

  // 目標導向路徑規劃函數
  int planToTargetArea(const Eigen::Vector3d& target_center, const Vector3d& pos, const Vector3d& vel, 
                       const Vector3d& acc, const Vector3d& yaw);
  int planToHoverPoint(const Eigen::Vector3d& hover_point, const Vector3d& pos, const Vector3d& vel, 
                       const Vector3d& acc, const Vector3d& yaw);

  shared_ptr<EDTEnvironment> edt_environment_;
  shared_ptr<SDFMap> sdf_map_;

  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
                      vector<int>& indices);

  // Refine local tour for next few frontiers, using more diverse viewpoints
  void refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
                       const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
                       vector<Vector3d>& refined_pts, vector<double>& refined_yaws);

  void shortenPath(vector<Vector3d>& path);

public:
  typedef shared_ptr<FastExplorationManager> Ptr;

  // 新增：目标点相关成员
  geometry_msgs::PointStamped target_point_;
  bool has_target_ = false;
  Eigen::Vector3d current_pos_;
  Eigen::Vector3d current_vel_;
  Eigen::Vector3d current_yaw_;
  double target_area_radius_ = 2.0; // 默認值，可通過參數加載

  // 懸停點搜尋配置參數
  double min_safety_dist_ = 0.5;
  double max_search_height_ = 3.0;
  double min_search_height_ = 1.0;
  double search_step_multiplier_ = 2.0;
  double stability_radius_ = 0.8;
  double approach_distance_ratio_ = 0.8;

  // 三個公式的權重參數
  double flatness_weight_ = 0.3;
  double safety_weight_ = 0.4;
  double proximity_weight_ = 0.3;

  // 平坦度公式參數
  double flatness_check_radius_ = 1.0;
  double flatness_variance_threshold_ = 0.1;

  // 安全性公式參數
  double safety_check_radius_ = 0.8;

  // 鄰近度公式參數
  double max_height_diff_ = 2.0;
  double proximity_distance_weight_ = 0.5;
  double proximity_height_weight_ = 0.3;
  double proximity_accessibility_weight_ = 0.2;

  // 可視化相關
  ros::Publisher target_point_pub_;
  ros::Publisher search_area_pub_;
  ros::Publisher hover_point_pub_;
  
  void visualizeTargetPoint();
  void visualizeSearchArea();
  void visualizeHoverPoint(const Eigen::Vector3d& hover_point);
  void updateVisualization();
};

}  // namespace fast_planner

#endif