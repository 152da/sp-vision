#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;//云台系到世界系的旋转矩阵

  void set_R_gimbal2world(const Eigen::Quaterniond & q);//设置云台系到世界系的旋转矩阵

  void solve(Armor & armor) const;//pnp

  std::vector<cv::Point2f> reproject_armor(//重投影装甲板的四个角点到像素坐标系
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double oupost_reprojection_error(Armor armor, const double & picth);
    //世界坐标点投影到像素坐标系，计算重投影误差
  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  cv::Mat camera_matrix_;//相机内参矩阵
  cv::Mat distort_coeffs_;//相机畸变系数
  Eigen::Matrix3d R_gimbal2imubody_;//云台系到IMU机体系的旋转矩阵
  Eigen::Matrix3d R_camera2gimbal_;//相机系到云台系的旋转矩阵
  Eigen::Vector3d t_camera2gimbal_;//相机系到云台系的平移向量
  Eigen::Matrix3d R_gimbal2world_;//云台系到世界系的旋转矩阵

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP