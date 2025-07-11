#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* Target point related members */
  geometry_msgs::PointStamped target_point_;
  bool has_target_;
  bool target_in_map_frame_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, targetpoint_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, target_vis_pub_;

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);
  void visualizeTargetPoint();
  bool isValidTargetPoint(const geometry_msgs::PointStamped& point);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void targetPointCallback(const geometry_msgs::PointStampedConstPtr& msg);
  void visualize();
  void clearVisMarker();

public:
  FastExplorationFSM(/* args */) {
    has_target_ = false;
    target_in_map_frame_ = false;
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);
  
  // Getter functions for target point
  bool hasTarget() const { return has_target_; }
  geometry_msgs::PointStamped getTargetPoint() const { return target_point_; }
  Vector3d getTargetPosition() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif