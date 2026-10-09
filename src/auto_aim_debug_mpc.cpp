#include <fmt/core.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <list>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>
#include <vector>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

namespace
{

cv::Point top_view_point(
  const Eigen::Vector2d & point, const Eigen::Vector2d & center, const cv::Point & origin,
  double scale)
{
  const Eigen::Vector2d rel = point - center;
  return {origin.x + cvRound(rel.y() * scale), origin.y - cvRound(rel.x() * scale)};
}

void draw_top_view(
  cv::Mat & img, const auto_aim::Target & target, const std::list<auto_aim::Armor> & observed_armors)
{
  if (img.empty()) return;

  constexpr int margin = 12;
  int panel_size = 260;
  panel_size = std::min(panel_size, img.cols - 2 * margin);
  panel_size = std::min(panel_size, img.rows - 2 * margin);
  if (panel_size < 140) return;

  const cv::Rect panel_rect(img.cols - panel_size - margin, margin, panel_size, panel_size);
  cv::rectangle(img, panel_rect, {18, 18, 18}, -1);
  cv::rectangle(img, panel_rect, {180, 180, 180}, 1);

  const auto x = target.ekf_x();
  const Eigen::Vector2d center{x[0], x[2]};
  const auto armor_xyza_list = target.armor_xyza_list();

  double max_radius = 0.35;
  for (const auto & xyza : armor_xyza_list) {
    const Eigen::Vector2d armor_xy{xyza[0], xyza[1]};
    max_radius = std::max(max_radius, (armor_xy - center).norm());
  }
  const double scale = panel_size * 0.38 / max_radius;
  const cv::Point origin(panel_rect.x + panel_size / 2, panel_rect.y + panel_size / 2);

  for (double r = 0.1; r <= 0.5; r += 0.1) {
    cv::circle(img, origin, cvRound(r * scale), {45, 45, 45}, 1, cv::LINE_AA);
  }
  cv::arrowedLine(img, origin, {origin.x, origin.y - 44}, {90, 90, 90}, 1, cv::LINE_AA);
  cv::arrowedLine(img, origin, {origin.x + 44, origin.y}, {90, 90, 90}, 1, cv::LINE_AA);
  cv::putText(img, "x", {origin.x + 4, origin.y - 46}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {130, 130, 130}, 1);
  cv::putText(img, "y", {origin.x + 47, origin.y - 4}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {130, 130, 130}, 1);

  std::vector<cv::Point> predicted_points;
  for (const auto & xyza : armor_xyza_list) {
    const Eigen::Vector2d armor_xy{xyza[0], xyza[1]};
    predicted_points.push_back(top_view_point(armor_xy, center, origin, scale));
  }
  for (std::size_t i = 0; i < predicted_points.size(); i++) {
    cv::line(
      img, predicted_points[i], predicted_points[(i + 1) % predicted_points.size()], {80, 120, 80},
      1, cv::LINE_AA);
  }

  for (const auto & armor : observed_armors) {
    const Eigen::Vector2d observed{armor.xyz_in_world[0], armor.xyz_in_world[1]};
    const auto point = top_view_point(observed, center, origin, scale);
    cv::circle(img, point, 4, {0, 165, 255}, 1, cv::LINE_AA);
  }

  cv::circle(img, origin, 4, {255, 255, 255}, -1, cv::LINE_AA);
  cv::putText(
    img, "C", {origin.x + 6, origin.y - 6}, cv::FONT_HERSHEY_SIMPLEX, 0.38, {255, 255, 255}, 1);

  for (std::size_t i = 0; i < armor_xyza_list.size(); i++) {
    const auto & xyza = armor_xyza_list[i];
    const Eigen::Vector2d armor_xy{xyza[0], xyza[1]};
    const Eigen::Vector2d normal{std::cos(xyza[3]), std::sin(xyza[3])};
    const auto point = top_view_point(armor_xy, center, origin, scale);
    const auto normal_end = top_view_point(armor_xy + normal * 0.12, center, origin, scale);
    const cv::Scalar color = (static_cast<int>(i) == target.last_id) ? cv::Scalar(0, 0, 255)
                                                                     : cv::Scalar(0, 255, 0);

    cv::circle(img, point, 5, color, -1, cv::LINE_AA);
    cv::arrowedLine(img, point, normal_end, color, 1, cv::LINE_AA, 0, 0.25);
    cv::putText(
      img, fmt::format("{}", i), {point.x + 6, point.y + 4}, cv::FONT_HERSHEY_SIMPLEX, 0.38,
      color, 1);
  }

  cv::putText(
    img, "tracker top view", {panel_rect.x + 8, panel_rect.y + 16}, cv::FONT_HERSHEY_SIMPLEX, 0.42,
    {220, 220, 220}, 1);
  cv::putText(
    img, "green:red=last  orange=obs", {panel_rect.x + 8, panel_rect.y + panel_size - 8},
    cv::FONT_HERSHEY_SIMPLEX, 0.35, {180, 180, 180}, 1);
}

}  // namespace

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }"
  "{top-view tv    | false                  | draw tracker top view }"
  "{future-view fv | false                  | draw future model projection }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  const bool enable_top_view = cli.get<bool>("top-view");
  const bool enable_future_view = cli.get<bool>("future-view");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    std::optional<Eigen::Vector2d> first_center;
    std::optional<Eigen::Vector2d> last_center;
    std::optional<double> last_center_yaw;

    while (!quit) {
      if (gimbal.mode() != io::GimbalMode::AUTO_AIM) {
        std::this_thread::sleep_for(200ms);
        continue;
      }

      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["target_valid"] = target.has_value() ? 1 : 0;
      data["plan_control"] = plan.control ? 1 : 0;

      if (target.has_value()) {
        const auto x = target->ekf_x();
        const Eigen::Vector2d center{x[0], x[2]};
        const auto center_yaw = x[6];
        if (!first_center) first_center = center;

        data["center_x"] = x[0];
        data["center_vx"] = x[1];
        data["center_y"] = x[2];
        data["center_vy"] = x[3];
        data["center_z"] = x[4];
        data["center_vz"] = x[5];
        data["center_a"] = x[6];
        data["center_w"] = x[7];
        data["center_r"] = x[8];
        data["center_l"] = x[9];
        data["center_h"] = x[10];

        data["center_speed"] = std::hypot(x[1], x[3]);
        data["center_distance"] = std::hypot(x[0], x[2]);
        data["center_drift"] = (center - first_center.value()).norm();
        data["center_drift_x"] = center.x() - first_center->x();
        data["center_drift_y"] = center.y() - first_center->y();
        if (last_center) {
          data["center_step"] = (center - last_center.value()).norm();
          data["center_step_x"] = center.x() - last_center->x();
          data["center_step_y"] = center.y() - last_center->y();
        }
        if (last_center_yaw) data["center_a_step"] = tools::limit_rad(center_yaw - *last_center_yaw);
        last_center = center;
        last_center_yaw = center_yaw;

        data["center_x_pred_100ms"] = x[0] + x[1] * 0.1;
        data["center_y_pred_100ms"] = x[2] + x[3] * 0.1;
        data["center_x_pred_300ms"] = x[0] + x[1] * 0.3;
        data["center_y_pred_300ms"] = x[2] + x[3] * 0.3;
        data["center_x_pred_500ms"] = x[0] + x[1] * 0.5;
        data["center_y_pred_500ms"] = x[2] + x[3] * 0.5;

        data["ekf_residual_yaw"] = target->ekf().data.at("residual_yaw");
        data["ekf_residual_pitch"] = target->ekf().data.at("residual_pitch");
        data["ekf_residual_distance"] = target->ekf().data.at("residual_distance");
        data["ekf_residual_angle"] = target->ekf().data.at("residual_angle");
        data["ekf_nis"] = target->ekf().data.at("nis");
        data["ekf_nees"] = target->ekf().data.at("nees");
        data["ekf_nis_fail"] = target->ekf().data.at("nis_fail");
        data["ekf_nees_fail"] = target->ekf().data.at("nees_fail");
        data["ekf_recent_nis_failures"] = target->ekf().data.at("recent_nis_failures");

        if (plan.control) {
          data["aim_x"] = planner.debug_xyza.x();
          data["aim_y"] = planner.debug_xyza.y();
          data["aim_z"] = planner.debug_xyza.z();
          data["aim_yaw"] = planner.debug_xyza.w();
        }

        data["target_z"] = x[4];   // z
        data["target_vz"] = x[5];  // vz
        data["w"] = x[7];
      } else {
        first_center = std::nullopt;
        last_center = std::nullopt;
        last_center_yaw = std::nullopt;
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    std::optional<auto_aim::Target> top_view_target;
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();
      top_view_target = target;

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      if (enable_future_view) {
        auto future_target = target;
        for (int i = 1; i <= 20; i++) {
          future_target.predict(0.025);

          Eigen::Vector4d nearest_xyza;
          double min_dist = 1e10;
          for (const auto & xyza : future_target.armor_xyza_list()) {
            auto dist = xyza.head<2>().norm();
            if (dist < min_dist) {
              min_dist = dist;
              nearest_xyza = xyza;
            }
          }

          auto future_points = solver.reproject_armor(
            nearest_xyza.head(3), nearest_xyza[3], future_target.armor_type, future_target.name);
          cv::Point2f center{0, 0};
          for (const auto & point : future_points) center += point;
          center *= 0.25f;
          cv::circle(img, center, 2, {255, 255, 0}, -1);
          if (i % 5 == 0) tools::draw_points(img, future_points, {255, 255, 0}, 1);
        }
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    if (enable_top_view && top_view_target) draw_top_view(img, top_view_target.value(), armors);
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
