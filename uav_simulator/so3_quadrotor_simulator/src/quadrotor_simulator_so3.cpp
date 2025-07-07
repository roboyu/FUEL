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
#include <iomanip> // 用于设置输出精度
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>

// =========================================================================
//  最终版评估仿真器 (Final Evaluation Simulator)
//  - 功能: 
//    1. 使用与最终版优化器完全一致的、基于加速度的动态摆动模型。
//    2. 同时统计"独立碰撞次数"和"碰撞率"两个核心安全指标。
//    3. 所有关键参数均可配置，代码健壮且注释清晰。
//  - 作者: AI助手
//  - 日期: [当前日期]
// =========================================================================

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

// 气泡模型参数
struct Bubble {
  Eigen::Vector3d center; // 气泡中心（世界坐标系）
  double radius;          // 气泡半径
};

// 全局变量
static Command command;
static Disturbance disturbance;

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
  // 建议为评估节点使用一个独特的名称
  ros::init(argc, argv, "quadrotor_simulator_eval_node");

  ros::NodeHandle n("~");

  // --- ROS 通信设置 ---
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
  ros::Subscriber cmd_sub = n.subscribe("cmd", 100, &cmd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber f_sub = n.subscribe("force_disturbance", 100, &force_disturbance_callback,
                                      ros::TransportHints().tcpNoDelay());
  ros::Subscriber m_sub = n.subscribe("moment_disturbance", 100, &moment_disturbance_callback,
                                      ros::TransportHints().tcpNoDelay());

  // --- 仿真器和物理参数初始化 ---
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

  // --- 加载仿真和评估所需的所有参数 ---
  double drone_bubble_radius, load_bubble_radius, rod_bubble_radius, rod_length, dist0;
  int rod_bubble_num;
  std::string map_path;
  n.param("collision_check/drone_bubble_radius", drone_bubble_radius, 0.25);
  n.param("collision_check/load_bubble_radius", load_bubble_radius, 0.15);
  n.param("collision_check/rod_bubble_radius", rod_bubble_radius, 0.05);
  n.param("collision_check/rod_length", rod_length, 1.0);
  n.param("collision_check/rod_bubble_num", rod_bubble_num, 5);
  n.param("collision_check/dist0", dist0, 0.0); // 额外安全边际
  n.param("map/path", map_path, std::string("")); // 地图路径

  // --- 初始化ESDF地图 ---
  edt_environment_.reset(new fast_planner::EDTEnvironment());
  sdf_map_.reset(new fast_planner::SDFMap());
  sdf_map_->initMap(n);
  if (map_path.empty()) {
      ROS_WARN("Map path is not set, no map loaded for collision check.");
  } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *cloud) == -1) {
          ROS_ERROR_STREAM("Cannot load map file from: " << map_path);
      } else {
          ROS_INFO_STREAM("Loaded map with " << cloud->size() << " points from: " << map_path);
          sdf_map_->inputPointCloud(*cloud, cloud->size(), Eigen::Vector3d(0,0,0));
      }
  }
  edt_environment_->setMap(sdf_map_);

  // --- 初始化评估指标变量 ---
  int collision_events = 0;       // 独立碰撞事件计数
  bool last_frame_collided = false;
  long long total_sim_frames = 0;   // 总仿真帧数
  long long risky_sim_frames = 0;   // 处于危险状态的帧数

  ros::Time next_odom_pub_time = ros::Time::now();

  // --- 常量 ---
  const Eigen::Vector3d gravity_vec(0.0, 0.0, -9.81);

  // --- 主循环 ---
  while (n.ok()) {
    ros::spinOnce();
    total_sim_frames++;

    // --- 飞行控制与动力学仿真 ---
    auto last = control;
    control = getControl(quad, command);
    for (int i = 0; i < 4; ++i) {
      if (std::isnan(control.rpm[i])) control.rpm[i] = last.rpm[i];
    }
    quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2], control.rpm[3]);
    quad.setExternalForce(disturbance.f);
    quad.setExternalMoment(disturbance.m);
    quad.step(dt);

    // ======================================================================
    //  核心评估逻辑: 模型构建 -> 碰撞判断 -> 指标统计
    // ======================================================================
    
    // 1. 模型构建 (与最终版优化器完全一致)
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
    
    std::vector<Bubble> bubbles;
    bubbles.push_back({drone_pos, drone_bubble_radius});
    bubbles.push_back({load_pos, load_bubble_radius});
    for (int i = 1; i <= rod_bubble_num; ++i) {
      double alpha = double(i) / (rod_bubble_num + 1);
      bubbles.push_back({drone_pos * (1 - alpha) + load_pos * alpha, rod_bubble_radius});
    }

    // 2. 碰撞判断
    bool current_frame_collided = false;
    if (edt_environment_ && edt_environment_->sdf_map_) {
        for (const auto& bubble : bubbles) {
            double dist = edt_environment_->sdf_map_->getDistance(bubble.center);
            // 判断是否侵入 (气泡半径 + 额外安全边际)
            if (dist < bubble.radius + dist0) {
                current_frame_collided = true;
                break; 
            }
        }
    }

    // 3. 更新统计指标
    if (current_frame_collided) {
        risky_sim_frames++;
    }
    if (current_frame_collided && !last_frame_collided) {
        collision_events++;
    }
    last_frame_collided = current_frame_collided;

    // --- ROS消息发布 ---
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

  // --- 仿真结束，输出最终评估结果 ---
  // 使用一个独特的、易于识别的文件名
  std::ofstream fout("/tmp/safety_evaluation_metrics.txt", std::ios::app);
  
  double collision_rate = 0.0;
  if (total_sim_frames > 0) {
    collision_rate = static_cast<double>(risky_sim_frames) / total_sim_frames;
  }

  // 结构化输出，方便解析和对比
  fout << "--- New Evaluation Run ---" << std::endl;
  fout << "Timestamp: " << ros::Time::now() << std::endl; // 记录时间戳
  fout << "Collision Events: " << collision_events << std::endl;
  fout << "Collision Rate: " << std::fixed << std::setprecision(2) << collision_rate * 100.0 << " %";
  fout << " (" << risky_sim_frames << " / " << total_sim_frames << " risky frames)" << std::endl;
  fout << "--------------------------" << std::endl << std::endl;
  
  fout.close();
  ROS_INFO_STREAM("Safety evaluation results saved to /tmp/safety_evaluation_metrics.txt");

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
