#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;//规划离散步长，10ms一步
constexpr int HALF_HORIZON = 50;//半个规划窗口长度，50步
constexpr int HORIZON = HALF_HORIZON * 2;//总规划窗口长度，100步，也就是1s的规划时间

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel，对应的列时时间点，共1s

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  Planner(const std::string & config_path);

  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

  TinySolver * yaw_solver_;//两个tiny solver，分别用于yaw和pitch的MPC规划
  TinySolver * pitch_solver_;

  void setup_yaw_solver(const std::string & config_path);//初始化mpc模型，代价和约束
  void setup_pitch_solver(const std::string & config_path);

  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);//计算目标的yaw和pitch角度
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);//生成未来一段的yaw和pitch轨迹，作为MPC的参考轨迹
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP