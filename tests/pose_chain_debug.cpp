#include <fmt/core.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <list>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                         | print help message }"
  "{config-path c  | configs/standard3.yaml  | yaml config path }"
  "{output o       | pose_chain_debug.jsonl  | output jsonl path }"
  "{delay-ms       | 1.0                     | imu query delay in ms }"
  "{display d      |                         | show debug window }";

namespace
{
nlohmann::json vec3_json(const Eigen::Vector3d & v)
{
  return {v.x(), v.y(), v.z()};
}

nlohmann::json vec4_json(const Eigen::Vector4d & v)
{
  return {v[0], v[1], v[2], v[3]};
}

nlohmann::json vecx_json(const Eigen::VectorXd & v)
{
  nlohmann::json result = nlohmann::json::array();
  for (int i = 0; i < v.size(); ++i) result.push_back(v[i]);
  return result;
}

void ensure_parent_dir(const std::string & path)
{
  const std::filesystem::path fs_path(path);
  const auto parent = fs_path.parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
}

std::chrono::steady_clock::duration ms_delay(double ms)
{
  return std::chrono::microseconds(static_cast<int64_t>(std::llround(ms * 1000.0)));
}

void draw_armor_summary(cv::Mat & img, const auto_aim::Armor & armor, int index)
{
  tools::draw_points(img, armor.points, {0, 255, 255}, 2);

  const cv::Point text_pos(
    static_cast<int>(armor.center.x) + 8, static_cast<int>(armor.center.y) + 22 + index * 18);
  tools::draw_text(
    img,
    fmt::format(
      "{} {} c[{:.2f},{:.2f},{:.2f}] w[{:.2f},{:.2f},{:.2f}]", index,
      auto_aim::ARMOR_NAMES[armor.name], armor.xyz_in_camera.x(), armor.xyz_in_camera.y(),
      armor.xyz_in_camera.z(), armor.xyz_in_world.x(), armor.xyz_in_world.y(),
      armor.xyz_in_world.z()),
    text_pos, {0, 255, 255}, 0.45, 1);
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>("config-path");
  const auto output_path = cli.get<std::string>("output");
  const auto delay_ms = cli.get<double>("delay-ms");
  const auto display = cli.has("display");

  ensure_parent_dir(output_path);
  std::ofstream log_file(output_path);
  if (!log_file.is_open()) {
    fmt::print(stderr, "Failed to open output file: {}\n", output_path);
    return 1;
  }

  tools::Exiter exiter;
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  const auto t0 = std::chrono::steady_clock::now();
  int frame = 0;

  fmt::print("Writing pose chain log to {}\n", output_path);
  fmt::print("Press q in the debug window to quit.\n");

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) continue;

    const auto q = cboard.imu_at(timestamp - ms_delay(delay_ms));
    solver.set_R_gimbal2world(q);
    const Eigen::Vector3d gimbal_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto detected_armors = detector.detect(img, frame);
    auto solved_armors = detected_armors;

    nlohmann::json data;
    data["frame"] = frame;
    data["t"] = tools::delta_time(timestamp, t0);
    data["delay_ms"] = delay_ms;
    data["gimbal_ypr_deg"] = vec3_json(gimbal_ypr * 57.3);
    data["armors"] = nlohmann::json::array();

    cv::Mat debug_img = img.clone();
    int armor_index = 0;
    for (auto & armor : solved_armors) {
      solver.solve(armor);

      nlohmann::json armor_data;
      armor_data["index"] = armor_index;
      armor_data["name"] = auto_aim::ARMOR_NAMES[armor.name];
      armor_data["type"] = auto_aim::ARMOR_TYPES[armor.type];
      armor_data["confidence"] = armor.confidence;
      armor_data["center"] = {armor.center.x, armor.center.y};
      armor_data["xyz_camera"] = vec3_json(armor.xyz_in_camera);
      armor_data["xyz_gimbal"] = vec3_json(armor.xyz_in_gimbal);
      armor_data["xyz_world"] = vec3_json(armor.xyz_in_world);
      armor_data["ypd_world"] = vec3_json(armor.ypd_in_world);
      armor_data["ypr_world"] = vec3_json(armor.ypr_in_world);
      armor_data["yaw_raw"] = armor.yaw_raw;
      data["armors"].push_back(armor_data);

      draw_armor_summary(debug_img, armor, armor_index);

      const auto pnp_reprojected =
        solver.reproject_armor(armor.xyz_in_world, armor.ypr_in_world[0], armor.type, armor.name);
      tools::draw_points(debug_img, pnp_reprojected, {255, 255, 0}, 2);

      ++armor_index;
    }

    auto tracker_armors = detected_armors;
    const auto targets = tracker.track(tracker_armors, timestamp);
    data["tracker_state"] = tracker.state();
    data["target_valid"] = !targets.empty();

    if (!targets.empty()) {
      const auto & target = targets.front();
      data["target_last_id"] = target.last_id;
      data["target_jumped"] = target.jumped;
      data["target_ekf_x"] = vecx_json(target.ekf_x());
      data["target_armors"] = nlohmann::json::array();

      for (const auto & xyza : target.armor_xyza_list()) {
        data["target_armors"].push_back(vec4_json(xyza));
        const auto model_reprojected =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(debug_img, model_reprojected, {0, 255, 0}, 2);
      }

      tools::draw_text(
        debug_img,
        fmt::format(
          "tracker:{} last_id:{} jumped:{} w:{:.2f}", tracker.state(), target.last_id,
          target.jumped ? 1 : 0, target.ekf_x()[7]),
        {40, 40}, {0, 255, 0}, 0.7, 2);
    } else {
      tools::draw_text(debug_img, "tracker:no target", {40, 40}, {0, 0, 255}, 0.7, 2);
    }

    tools::draw_text(
      debug_img,
      fmt::format(
        "gimbal ypr deg [{:.2f}, {:.2f}, {:.2f}] delay {:.2f}ms", gimbal_ypr[0] * 57.3,
        gimbal_ypr[1] * 57.3, gimbal_ypr[2] * 57.3, delay_ms),
      {40, 75}, {0, 255, 0}, 0.7, 2);

    log_file << data.dump() << '\n';
    log_file.flush();

    if (display) {
      cv::resize(debug_img, debug_img, {}, 0.6, 0.6);
      cv::imshow("pose_chain_debug", debug_img);
      if (cv::waitKey(1) == 'q') break;
    }

    ++frame;
  }

  return 0;
}
