#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_simulator/Quadrotor.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <uav_utils/geometry_utils.h>
#include <vector>
#include <signal.h>
#include <fstream>
#include <cstdio>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef struct _Control { double rpm[4]; } Control;

typedef struct _Command {
  float force[3];
  float qx, qy, qz, qw;
  float kR[3];
  float kOm[3];
  float corrections[3];
  float current_yaw;
  bool use_external_yaw;
} Command;

typedef struct _Disturbance {
  Eigen::Vector3d f;
  Eigen::Vector3d m;
} Disturbance;

static Command command;
static Disturbance disturbance;

// 气泡模型参数
struct Bubble {
  Eigen::Vector3d center; // 气泡中心（世界坐标系）
  double radius;          // 气泡半径
};
std::vector<Bubble> bubbles;

// 气泡分布参数
const double drone_bubble_radius = 0.25; // 无人机本体气泡半径
const double load_bubble_radius = 0.15;  // 吊载气泡半径
const double rod_bubble_radius = 0.05;   // 杆/绳气泡半径
const int rod_bubble_num = 5;            // 杆/绳分几个气泡
const double rod_length = 1.0;           // 杆/绳长度

// 碰撞统计变量
int collision_count = 0;
bool last_collided = false; // 新增：用于统计碰撞次数

// 全局变量
std::shared_ptr<fast_planner::EDTEnvironment> edt_environment_;
std::shared_ptr<fast_planner::SDFMap> sdf_map_;

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State& state, nav_msgs::Odometry& odom);
void quadToImuMsg(const QuadrotorSimulator::Quadrotor& quad, sensor_msgs::Imu& imu);

static Control getControl(const QuadrotorSimulator::Quadrotor& quad, const Command& cmd) {
  const double _kf = quad.getPropellerThrustCoefficient();
  const double _km = quad.getPropellerMomentCoefficient();
  const double kf = _kf - cmd.corrections[0];
  const double km = _km / _kf * kf;

  const double d = quad.getArmLength();
  const Eigen::Matrix3f J = quad.getInertia().cast<float>();
  const float I[3][3] = { { J(0, 0), J(0, 1), J(0, 2) },
                          { J(1, 0), J(1, 1), J(1, 2) },
                          { J(2, 0), J(2, 1), J(2, 2) } };
  const QuadrotorSimulator::Quadrotor::State state = quad.getState();

  // Rotation, may use external yaw
  Eigen::Vector3d _ypr = uav_utils::R_to_ypr(state.R);
  Eigen::Vector3d ypr = _ypr;
  if (cmd.use_external_yaw) ypr[0] = cmd.current_yaw;
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
  float R11 = R(0, 0);
  float R12 = R(0, 1);
  float R13 = R(0, 2);
  float R21 = R(1, 0);
  float R22 = R(1, 1);
  float R23 = R(1, 2);
  float R31 = R(2, 0);
  float R32 = R(2, 1);
  float R33 = R(2, 2);
  /*
    float R11 = state.R(0,0);
    float R12 = state.R(0,1);
    float R13 = state.R(0,2);
    float R21 = state.R(1,0);
    float R22 = state.R(1,1);
    float R23 = state.R(1,2);
    float R31 = state.R(2,0);
    float R32 = state.R(2,1);
    float R33 = state.R(2,2);
  */
  float Om1 = state.omega(0);
  float Om2 = state.omega(1);
  float Om3 = state.omega(2);

  float Rd11 = cmd.qw * cmd.qw + cmd.qx * cmd.qx - cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd12 = 2 * (cmd.qx * cmd.qy - cmd.qw * cmd.qz);
  float Rd13 = 2 * (cmd.qx * cmd.qz + cmd.qw * cmd.qy);
  float Rd21 = 2 * (cmd.qx * cmd.qy + cmd.qw * cmd.qz);
  float Rd22 = cmd.qw * cmd.qw - cmd.qx * cmd.qx + cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd23 = 2 * (cmd.qy * cmd.qz - cmd.qw * cmd.qx);
  float Rd31 = 2 * (cmd.qx * cmd.qz - cmd.qw * cmd.qy);
  float Rd32 = 2 * (cmd.qy * cmd.qz + cmd.qw * cmd.qx);
  float Rd33 = cmd.qw * cmd.qw - cmd.qx * cmd.qx - cmd.qy * cmd.qy + cmd.qz * cmd.qz;

  float Psi = 0.5f * (3.0f - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 + Rd12 * R12 + Rd22 * R22 +
                              Rd32 * R32 + Rd13 * R13 + Rd23 * R23 + Rd33 * R33));

  float force = 0;
  if (Psi < 1.0f)  // Position control stability guaranteed only when Psi < 1
    force = cmd.force[0] * R13 + cmd.force[1] * R23 + cmd.force[2] * R33;

  float eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 + R32 * Rd33 - R33 * Rd32);
  float eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 + R23 * Rd21 - R31 * Rd33 + R33 * Rd31);
  float eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 + R31 * Rd32 - R32 * Rd31);

  float eOm1 = Om1;
  float eOm2 = Om2;
  float eOm3 = Om3;

  float in1 = Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) -
      Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
  float in2 = Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) -
      Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
  float in3 = Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) -
      Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);
  /*
    // Robust Control --------------------------------------------
    float c2       = 0.6;
    float epsilonR = 0.04;
    float deltaR   = 0.1;
    float eA1 = eOm1 + c2 * 1.0/I[0][0] * eR1;
    float eA2 = eOm2 + c2 * 1.0/I[1][1] * eR2;
    float eA3 = eOm3 + c2 * 1.0/I[2][2] * eR3;
    float neA = sqrt(eA1*eA1 + eA2*eA2 + eA3*eA3);
    float muR1 = -deltaR*deltaR * eA1 / (deltaR * neA + epsilonR);
    float muR2 = -deltaR*deltaR * eA2 / (deltaR * neA + epsilonR);
    float muR3 = -deltaR*deltaR * eA3 / (deltaR * neA + epsilonR);
    // Robust Control --------------------------------------------
  */
  float M1 = -cmd.kR[0] * eR1 - cmd.kOm[0] * eOm1 + in1;  // - I[0][0]*muR1;
  float M2 = -cmd.kR[1] * eR2 - cmd.kOm[1] * eOm2 + in2;  // - I[1][1]*muR2;
  float M3 = -cmd.kR[2] * eR3 - cmd.kOm[2] * eOm3 + in3;  // - I[2][2]*muR3;

  float w_sq[4];
  w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
  w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

  Control control;
  for (int i = 0; i < 4; i++) {
    if (w_sq[i] < 0) w_sq[i] = 0;

    control.rpm[i] = sqrtf(w_sq[i]);
  }
  return control;
}

static void cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr& cmd) {
  command.force[0] = cmd->force.x;
  command.force[1] = cmd->force.y;
  command.force[2] = cmd->force.z;
  command.qx = cmd->orientation.x;
  command.qy = cmd->orientation.y;
  command.qz = cmd->orientation.z;
  command.qw = cmd->orientation.w;
  command.kR[0] = cmd->kR[0];
  command.kR[1] = cmd->kR[1];
  command.kR[2] = cmd->kR[2];
  command.kOm[0] = cmd->kOm[0];
  command.kOm[1] = cmd->kOm[1];
  command.kOm[2] = cmd->kOm[2];
  command.corrections[0] = cmd->aux.kf_correction;
  command.corrections[1] = cmd->aux.angle_corrections[0];
  command.corrections[2] = cmd->aux.angle_corrections[1];
  command.current_yaw = cmd->aux.current_yaw;
  command.use_external_yaw = cmd->aux.use_external_yaw;
}

static void force_disturbance_callback(const geometry_msgs::Vector3::ConstPtr& f) {
  disturbance.f(0) = f->x;
  disturbance.f(1) = f->y;
  disturbance.f(2) = f->z;
}

static void moment_disturbance_callback(const geometry_msgs::Vector3::ConstPtr& m) {
  disturbance.m(0) = m->x;
  disturbance.m(1) = m->y;
  disturbance.m(2) = m->z;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "quadrotor_simulator_so3");

  ros::NodeHandle n("~");

  // ROS Publishers and Subscribers
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  // TODO 球位置发布
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
  ros::Publisher bubbles_pub = n.advertise<visualization_msgs::MarkerArray>("bubble_markers", 1);
  ros::Publisher rod_pub = n.advertise<visualization_msgs::Marker>("rod_marker", 1);
  ros::Subscriber cmd_sub = n.subscribe("cmd", 100, &cmd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber f_sub = n.subscribe("force_disturbance", 100, &force_disturbance_callback,
                                      ros::TransportHints().tcpNoDelay());
  ros::Subscriber m_sub = n.subscribe("moment_disturbance", 100, &moment_disturbance_callback,
                                      ros::TransportHints().tcpNoDelay());

  // Quadrotor and simulation setup
  QuadrotorSimulator::Quadrotor quad;
  double _init_x, _init_y, _init_z;
  n.param("simulator/init_state_x", _init_x, 0.0);
  n.param("simulator/init_state_y", _init_y, 0.0);
  n.param("simulator/init_state_z", _init_z, 1.0);
  Eigen::Vector3d position = Eigen::Vector3d(_init_x, _init_y, _init_z);
  quad.setStatePos(position);

  double simulation_rate;
  n.param("rate/simulation", simulation_rate, 1000.0);
  ROS_ASSERT(simulation_rate > 0);

  double odom_rate;
  n.param("rate/odom", odom_rate, 100.0);
  const ros::Duration odom_pub_duration(1 / odom_rate);

  std::string quad_name;
  n.param("quadrotor_name", quad_name, std::string("quadrotor"));

  QuadrotorSimulator::Quadrotor::State state = quad.getState();
  ros::Rate r(simulation_rate);
  const double dt = 1 / simulation_rate;

  Control control;
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "/simulator";
  odom_msg.child_frame_id = "/" + quad_name;
  sensor_msgs::Imu imu;
  imu.header.frame_id = "/simulator";

  // ================== 气泡参数通过rosparam读取 ==================
  double drone_bubble_radius, load_bubble_radius, rod_bubble_radius, rod_length, dist0;
  int rod_bubble_num;
  n.param("drone_bubble_radius", drone_bubble_radius, 0.25);
  n.param("load_bubble_radius", load_bubble_radius, 0.15);
  n.param("rod_bubble_radius", rod_bubble_radius, 0.05);
  n.param("rod_length", rod_length, 1.0);
  n.param("rod_bubble_num", rod_bubble_num, 5);
  n.param("dist0", dist0, 0.7); // 默认为0.7，建议与优化器一致

  // ESDF Map initialization
  edt_environment_.reset(new fast_planner::EDTEnvironment());
  sdf_map_.reset(new fast_planner::SDFMap());
  sdf_map_->initMap(n);
  std::string map_file;
  n.param("map_path", map_file, std::string("")); 
  if (map_file.empty()) {
      ROS_WARN("Map file path is empty, no map loaded.");
  } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file, *cloud) == -1) {
          ROS_ERROR_STREAM("Cannot load map file from: " << map_file);
      } else {
          ROS_INFO_STREAM("Loaded map with " << cloud->size() << " points from: " << map_file);
          sdf_map_->inputPointCloud(*cloud, cloud->size(), Eigen::Vector3d(0,0,0));
      }
  }
  edt_environment_->setMap(sdf_map_);

  // ========== 调试：打印地图边界（origin 与 max）到控制台 ==========
  {
    if (edt_environment_ && edt_environment_->sdf_map_) {
      Eigen::Vector3d map_ori, map_size;
      edt_environment_->sdf_map_->getRegion(map_ori, map_size);
      Eigen::Vector3d map_min = map_ori;
      Eigen::Vector3d map_max = map_ori + map_size;
      ROS_INFO("[SDF DEBUG] Map region origin:(%.2f, %.2f, %.2f) size:(%.2f, %.2f, %.2f)",
               map_ori.x(), map_ori.y(), map_ori.z(), map_size.x(), map_size.y(), map_size.z());
      ROS_INFO("[SDF DEBUG] Map boundaries  min:(%.2f, %.2f, %.2f)  max:(%.2f, %.2f, %.2f)",
               map_min.x(), map_min.y(), map_min.z(), map_max.x(), map_max.y(), map_max.z());
    } else {
      ROS_WARN("[SDF DEBUG] edt_environment_ or sdf_map_ is null, cannot print map boundaries.");
    }
  }

  ros::Time next_odom_pub_time = ros::Time::now();

  // ================== 打开碰撞检测日志文件 ==================
  std::ofstream collision_log_file("/tmp/collision_debug.log", std::ios::trunc);
  if (!collision_log_file.is_open()) {
    ROS_ERROR("Failed to open collision debug log file!");
  } else {
    collision_log_file << "========== Collision Detection Debug Log ==========" << std::endl;
    collision_log_file << "Log started at: " << ros::Time::now() << std::endl;
    collision_log_file << "Format: [Event_Type] Frame:XXX | Key:Value | ..." << std::endl;
    collision_log_file << "===================================================" << std::endl;
    collision_log_file.flush();

    // 启动时也把地图边界写入原始日志文件
    if (edt_environment_ && edt_environment_->sdf_map_) {
      Eigen::Vector3d map_ori, map_size;
      edt_environment_->sdf_map_->getRegion(map_ori, map_size);
      Eigen::Vector3d map_min = map_ori;
      Eigen::Vector3d map_max = map_ori + map_size;
      char buf[256];
      snprintf(buf, sizeof(buf),
               "[SDF DEBUG] Map region origin:(%.2f, %.2f, %.2f) size:(%.2f, %.2f, %.2f)",
               map_ori.x(), map_ori.y(), map_ori.z(), map_size.x(), map_size.y(), map_size.z());
      collision_log_file << buf << std::endl;
      snprintf(buf, sizeof(buf),
               "[SDF DEBUG] Map boundaries  min:(%.2f, %.2f, %.2f)  max:(%.2f, %.2f, %.2f)",
               map_min.x(), map_min.y(), map_min.z(), map_max.x(), map_max.y(), map_max.z());
      collision_log_file << buf << std::endl;
      collision_log_file.flush();
    }
  }

  // ========== 常量 ==========
  const Eigen::Vector3d gravity_vec(0.0, 0.0, -9.81);
  
  // ================== 帧计数器（用于日志） ==================
  int frame_counter = 0;

  while (n.ok()) {
    ros::spinOnce();

    // Flight control and dynamics simulation
    auto last = control;
    control = getControl(quad, command);
    for (int i = 0; i < 4; ++i) {
      if (std::isnan(control.rpm[i])) control.rpm[i] = last.rpm[i];
    }
    quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2], control.rpm[3]);
    quad.setExternalForce(disturbance.f);
    quad.setExternalMoment(disturbance.m);
    quad.step(dt);

    // ================== 气泡分布逻辑（与优化器完全一致） ==================
    state = quad.getState();
    const Eigen::Vector3d drone_pos = state.x;
    const Eigen::Vector3d acc_drone = quad.getAcc();
    Eigen::Vector3d rod_direction = gravity_vec - acc_drone;
    if (rod_direction.squaredNorm() < 1e-8) {
        rod_direction = Eigen::Vector3d(0.0, 0.0, -1.0);
    } else {
        rod_direction.normalize();
    }
    const Eigen::Vector3d load_pos = drone_pos + rod_length * rod_direction;
    bubbles.clear();
    bubbles.push_back({drone_pos, drone_bubble_radius});
    bubbles.push_back({load_pos, load_bubble_radius});
    for (int i = 1; i <= rod_bubble_num; ++i) {
      double alpha = double(i) / (rod_bubble_num + 1);
      Eigen::Vector3d rod_pos = drone_pos * (1 - alpha) + load_pos * alpha;
      bubbles.push_back({rod_pos, rod_bubble_radius});
    }

    // ================== 碰撞统计逻辑（与优化器一致，含dist0） ==================
    frame_counter++;
    
    bool frame_collided = false;
    double collided_dist = 1e6;
    double collided_radius = 0.0;
    int collided_bubble_idx = -1;
    
    // 记录最小距离和对应的气泡信息（用于周期性日志）
    double min_dist_all = 1e6;
    double min_dist_radius = 0.0;
    int min_dist_bubble_idx = -1;
    
    for (size_t i = 0; i < bubbles.size(); ++i) {
      const auto& bubble = bubbles[i];
      double dist = 1e6;
      if (edt_environment_ && edt_environment_->sdf_map_) {
        dist = edt_environment_->sdf_map_->getDistance(bubble.center);
      }
      
      // 记录最小距离
      if (dist < min_dist_all) {
        min_dist_all = dist;
        min_dist_radius = bubble.radius;
        min_dist_bubble_idx = i;
      }
      
      if (dist < bubble.radius) {
        frame_collided = true;
        collided_dist = dist;
        collided_radius = bubble.radius;
        collided_bubble_idx = i;
        break;
      }
    }
    
    // 记录状态变化和关键数据
    bool state_changed = (frame_collided != last_collided);
    
    if (state_changed) {
      if (frame_collided) {
        // 状态从 SAFE -> COLLIDED
        collision_count++;
        char log_buf[512];
        snprintf(log_buf, sizeof(log_buf), 
                 "[COLLISION_STATE_CHANGE] Frame:%d | SAFE->COLLIDED | collision_count:%d->%d | dist:%.4f | radius:%.4f | bubble_idx:%d",
                 frame_counter, collision_count-1, collision_count, collided_dist, collided_radius, collided_bubble_idx);
        ROS_ERROR("%s", log_buf);
        if (collision_log_file.is_open()) {
          collision_log_file << log_buf << std::endl;
          collision_log_file.flush();
        }
      } else {
        // 状态从 COLLIDED -> SAFE
        char log_buf[512];
        snprintf(log_buf, sizeof(log_buf),
                 "[COLLISION_STATE_CHANGE] Frame:%d | COLLIDED->SAFE | collision_count:%d | last_collided:true->false",
                 frame_counter, collision_count);
        ROS_WARN("%s", log_buf);
        if (collision_log_file.is_open()) {
          collision_log_file << log_buf << std::endl;
          collision_log_file.flush();
        }
      }
    }
    
    // 记录当前状态（每100帧记录一次，便于追踪连续状态）
    if (frame_counter % 100 == 0) {
      char log_buf[512];
      if (frame_collided) {
        // 有碰撞时，记录碰撞气泡的信息
        snprintf(log_buf, sizeof(log_buf),
                 "[COLLISION_STATUS] Frame:%d | frame_collided:%s | last_collided:%s | collision_count:%d | dist:%.4f | radius:%.4f | bubble_idx:%d",
                 frame_counter, frame_collided ? "true" : "false", last_collided ? "true" : "false", collision_count,
                 collided_dist, collided_radius, collided_bubble_idx);
      } else {
        // 无碰撞时，记录最小距离气泡的信息
        snprintf(log_buf, sizeof(log_buf),
                 "[COLLISION_STATUS] Frame:%d | frame_collided:%s | last_collided:%s | collision_count:%d | min_dist:%.4f | min_dist_radius:%.4f | min_dist_bubble_idx:%d",
                 frame_counter, frame_collided ? "true" : "false", last_collided ? "true" : "false", collision_count,
                 min_dist_all, min_dist_radius, min_dist_bubble_idx);
      }
      ROS_INFO("%s", log_buf);
      if (collision_log_file.is_open()) {
        collision_log_file << log_buf << std::endl;
        collision_log_file.flush();
      }
    }
    
    last_collided = frame_collided;

    // ========== 调试：周期性打印每个气泡是否在地图内 ==========
    if (frame_counter % 200 == 0) {
      if (edt_environment_ && edt_environment_->sdf_map_) {
        for (size_t i = 0; i < bubbles.size(); ++i) {
          const auto& b = bubbles[i];
          bool in_map = edt_environment_->sdf_map_->isInMap(b.center);
          ROS_INFO("[SDF DEBUG] Bubble %zu pos:(%.2f, %.2f, %.2f) r:%.3f in_map:%s", i,
                   b.center.x(), b.center.y(), b.center.z(), b.radius, in_map ? "true" : "false");
          if (collision_log_file.is_open()) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                     "[SDF DEBUG] Bubble %zu pos:(%.2f, %.2f, %.2f) r:%.3f in_map:%s",
                     i, b.center.x(), b.center.y(), b.center.z(), b.radius,
                     in_map ? "true" : "false");
            collision_log_file << buf << std::endl;
          }
        }
        if (collision_log_file.is_open()) collision_log_file.flush();
      }
    }

    // ================== bubbles 绳索rviz可视化 ====================
    // 1. 气泡可视化，所有气泡都画成透明球体，颜色区分无人机/载荷/绳子
    visualization_msgs::MarkerArray bubble_arr;
    for(size_t i = 0; i < bubbles.size(); ++i) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world"; // 显示在world坐标系，适配你的环境
      marker.header.stamp = ros::Time::now();
      marker.ns = "bubbles";
      marker.id = i;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = bubbles[i].center.x();
      marker.pose.position.y = bubbles[i].center.y();
      marker.pose.position.z = bubbles[i].center.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = bubbles[i].radius * 2;
      marker.scale.y = bubbles[i].radius * 2;
      marker.scale.z = bubbles[i].radius * 2;
      // 区分颜色：无人机本体0为紫/载荷1为橙，其余为绿色
      if(i == 0)     { marker.color.r = 0.6; marker.color.g = 0.2; marker.color.b = 1.0; } //无人机
      else if(i==1)  { marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.05;} //载荷
      else           { marker.color.r = 0.2; marker.color.g = 1.0; marker.color.b = 0.2; } //绳气泡
      marker.color.a = 0.4;
      marker.lifetime = ros::Duration(0.07);
      bubble_arr.markers.push_back(marker);
    }
    bubbles_pub.publish(bubble_arr);
    // 2. 绳索可视化（连接无人机本体、绳气泡、载荷气泡)
    visualization_msgs::Marker rod_marker;
    rod_marker.header.frame_id = "world";
    rod_marker.header.stamp = ros::Time::now();
    rod_marker.ns = "rod";
    rod_marker.id = 0;
    rod_marker.type = visualization_msgs::Marker::LINE_STRIP;
    rod_marker.action = visualization_msgs::Marker::ADD;
    rod_marker.scale.x = 0.04; //线宽
    rod_marker.color.r = 1.0;  // 黄色（明显和球区分）
    rod_marker.color.g = 1.0;
    rod_marker.color.b = 0.0;
    rod_marker.color.a = 1.0;
    rod_marker.lifetime = ros::Duration(0.07);
    for(const auto& bubble : bubbles) {
      geometry_msgs::Point pt; pt.x = bubble.center.x(); pt.y = bubble.center.y(); pt.z = bubble.center.z();
      rod_marker.points.push_back(pt);
    }
    rod_pub.publish(rod_marker);

    // ROS message publishing
    ros::Time tnow = ros::Time::now();
    if (tnow >= next_odom_pub_time) {
      next_odom_pub_time += odom_pub_duration;
      odom_msg.header.stamp = tnow;
      stateToOdomMsg(state, odom_msg);
      quadToImuMsg(quad, imu);
      odom_pub.publish(odom_msg);
      imu_pub.publish(imu);
    }

    r.sleep();
  }

  // 关闭碰撞检测日志文件
  if (collision_log_file.is_open()) {
    collision_log_file << "===================================================" << std::endl;
    collision_log_file << "Log ended at: " << ros::Time::now() << std::endl;
    collision_log_file << "Total frames: " << frame_counter << std::endl;
    collision_log_file << "Total collision events: " << collision_count << std::endl;
    collision_log_file << "===================================================" << std::endl;
    collision_log_file.close();
    ROS_INFO("Collision debug log saved to: /tmp/collision_debug.log");
  }
  
  // File output (原有的统计文件)
  std::ofstream fout("/tmp/collision_countt.txt", std::ios::app);
  fout << "Collision count for this run: " << collision_count << std::endl;
  fout.close();

  return 0;
}

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State& state, nav_msgs::Odometry& odom) {
  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);

  Eigen::Quaterniond q(state.R);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = state.v(0);
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = state.v(2);

  odom.twist.twist.angular.x = state.omega(0);
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);
}

void quadToImuMsg(const QuadrotorSimulator::Quadrotor& quad, sensor_msgs::Imu& imu)

{
  QuadrotorSimulator::Quadrotor::State state = quad.getState();
  Eigen::Quaterniond q(state.R);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();

  imu.angular_velocity.x = state.omega(0);
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);

  imu.linear_acceleration.x = quad.getAcc()[0];
  imu.linear_acceleration.y = quad.getAcc()[1];
  imu.linear_acceleration.z = quad.getAcc()[2];
}