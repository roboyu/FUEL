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

// ========== 新增：探索状态枚举 ===========
enum ExplorationState {
  SUCCEED = 1,
  FAIL = 2,
  NO_FRONTIER = 0,
  FINAL_GOAL_FOUND = 3 // 新增：最终投放点找到
};
// ========== 新增 END ===========

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

private:
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

  /**
   * @brief 在目标点附近规划精细的投放/降落任务
   */
  int planFineDelivery(const Eigen::Vector3d& cur_pos, const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc);

  /**
   * @brief 使用向下射线投射找到指定(x,y)位置的地面高度
   * @param p_center 投射的中心点 (只使用 x, y)
   * @param ground_pt 输出的地面点
   * @return true 如果找到地面, false 如果没有
   */
  bool findGroundHeight(const Eigen::Vector3d& p_center, Eigen::Vector3d& ground_pt);

  /**
   * @brief 计算一个点周围地面的平坦度 (通过高度标准差)
   * @param center_ground_pt 要分析的地面中心点
   * @return 地面高度的标准差，值越小越平坦
   */
  double calculateFlatness(const Eigen::Vector3d& center_ground_pt);

  // 投放点决策的权重参数 (可以在launch文件中配置)
  double w_proximity_ = 0.4;
  double w_flatness_  = 0.3;
  double w_safety_    = 0.3;

public:
  typedef shared_ptr<FastExplorationManager> Ptr;

  // 新增：目标点相关成员
  geometry_msgs::PointStamped target_point_;
  bool has_target_ = false;
};

}  // namespace fast_planner

#endif