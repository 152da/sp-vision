#ifndef AUTO_AIM__DETECTOR_HPP
#define AUTO_AIM__DETECTOR_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "classifier.hpp"

namespace auto_aim
{

class Detector
{
public:
  Detector(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count = -1);

  bool detect(Armor & armor, const cv::Mat & bgr_img);

  friend class YOLOV8;

private:
  Classifier classifier_;

  double threshold_;//二值化阈值
  double max_angle_error_;//灯条倾角误差范围
  double min_lightbar_ratio_, max_lightbar_ratio_;//灯条长宽比范围
  double min_lightbar_length_;//灯条最小长度
  double min_armor_ratio_, max_armor_ratio_;//装甲板长宽比范围
  double max_side_ratio_;//装甲板长短边比范围
  double min_confidence_;//分类器最小置信度
  double max_rectangular_error_;//装甲板矩形误差范围

  bool debug_;
  std::string save_path_;

  // 利用PCA回归角点，参考自https://github.com/CSU-FYT-Vision/FYT2024_vision
  void lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const;

  bool check_geometry(const Lightbar & lightbar) const;//过滤函数
  bool check_geometry(const Armor & armor) const;
  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  Color get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const;//获取灯条颜色
  ArmorType get_type_by_name(const Armor & armor) const;//根据装甲板名称获取类型
  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;//获取装甲板图案
  ArmorType get_type(const Armor & armor);
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  void save(const Armor & armor) const;
  void show_result(
    const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
    const std::list<Armor> & armors, int frame_count) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DETECTOR_HPP