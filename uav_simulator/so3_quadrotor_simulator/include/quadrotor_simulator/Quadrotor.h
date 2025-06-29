#ifndef __QUADROTOR_SIMULATOR_QUADROTOR_H__
#define __QUADROTOR_SIMULATOR_QUADROTOR_H__

#include <Eigen/Core>
#include <boost/array.hpp>

namespace QuadrotorSimulator {

class Quadrotor {
public:
  struct State {
    Eigen::Vector3d x;  //无人机位置
    Eigen::Vector3d v;  //无人机速度
    Eigen::Matrix3d R;  //无人机姿态矩阵
    Eigen::Vector3d omega;  //无人机角速度
    Eigen::Array4d motor_rpm; //电机转速

    Eigen::Vector3d xl; //负载的位置
    Eigen::Vector3d vl; //负载的速度
    Eigen::Vector3d ql; //缆绳的方向向量
    Eigen::Vector3d dql; //缆绳方向向量变化率
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  Quadrotor();

  const Quadrotor::State& getState(void) const;

  void setState(const Quadrotor::State& state);

  void setStatePos(const Eigen::Vector3d& Pos);

  double getMass(void) const;
  void setMass(double mass);

  double getLoadMass(void) const;
  void setLoadMass(double mass_l);  //负载质量相关

  double getCableLength(void) const;
  void setCableLength(double l_length);  //缆绳长度相关

  double getGravity(void) const;
  void setGravity(double g);

  const Eigen::Matrix3d& getInertia(void) const;
  void setInertia(const Eigen::Matrix3d& inertia);

  double getArmLength(void) const;
  void setArmLength(double d);

  double getPropRadius(void) const;
  void setPropRadius(double r);

  double getPropellerThrustCoefficient(void) const;
  void setPropellerThrustCoefficient(double kf);

  double getPropellerMomentCoefficient(void) const;
  void setPropellerMomentCoefficient(double km);

  double getMotorTimeConstant(void) const;
  void setMotorTimeConstant(double k);

  const Eigen::Vector3d& getExternalForce(void) const;
  void setExternalForce(const Eigen::Vector3d& force);

  const Eigen::Vector3d& getExternalMoment(void) const;
  void setExternalMoment(const Eigen::Vector3d& moment);

  double getMaxRPM(void) const;
  void setMaxRPM(double max_rpm);

  double getMinRPM(void) const;
  void setMinRPM(double min_rpm);

  // 获取负载状态的方法
  Eigen::Vector3d getLoadPos(void) const;
  Eigen::Vector3d getLoadVel(void) const;
  Eigen::Vector3d getCableDirection(void) const;
  Eigen::Vector3d getCableDirectionRate(void) const;

  // Inputs are desired RPM for the motors
  // Rotor numbering is:
  //   *1*    Front
  // 3     4
  //    2
  // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
  void setInput(double u1, double u2, double u3, double u4);

  // Runs the actual dynamics simulation with a time step of dt
  void step(double dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 34> InternalState; //负载状态 22+12
  void operator()(const Quadrotor::InternalState& x, Quadrotor::InternalState& dxdt,
                  const double /* t */);

  Eigen::Vector3d getAcc() const;

private:
  void updateInternalState(void);

  double alpha0;  // AOA
  double g_;      // gravity
  double mass_;
  double mass_l_;  //负载质量
  double l_length_;  //缆绳长度
  Eigen::Matrix3d J_;  // Inertia
  double kf_;
  double km_;
  double prop_radius_;
  double arm_length_;
  double motor_time_constant_;  // unit: sec
  double max_rpm_;
  double min_rpm_;

  Quadrotor::State state_;

  Eigen::Vector3d acc_;

  Eigen::Array4d input_;
  Eigen::Vector3d external_force_;
  Eigen::Vector3d external_moment_;

  InternalState internal_state_;
};
}
#endif
