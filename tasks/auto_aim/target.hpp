#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;//前面detector的结果
  bool jumped;//jumped表示是否发生了装甲板切换
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);//主构造函数，用第一块观测到的装甲板初始化目标状态

  void predict(std::chrono::steady_clock::time_point t);//EKF预测函数，输入当前时间，计算时间差dt，调用ekf_.predict()进行预测
  void predict(double dt);
  void update(const Armor & armor);//更新EKF状态

  Eigen::VectorXd ekf_x() const;//获取EKF状态向量，对外暴漏的接口
  const tools::ExtendedKalmanFilter & ekf() const;//EKF滤波器，对外暴漏的接口
  std::vector<Eigen::Vector4d> armor_xyza_list() const;//获取装甲板位置列表
  //判断滤波是否发散，是否收敛
  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;//私有状态，装甲板数量，切换次数，更新次数，是否切换，是否收敛，EKF滤波器，时间戳
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle，更新

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;//观测模型
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;//观测模型雅可比
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP