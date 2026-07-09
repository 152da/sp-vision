#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);//normal  non-omni version of track, only return the current target, if no target, return empty list

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);//带全向感知的追踪入口，相比普通版，多了一个switch_target的输出，表示是否发生了目标切换

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;//连续检测到目标的最小次数，超过这个次数才会进入tracking状态
  int max_temp_lost_count_;//连续丢失目标的最大次数，超过这个次数才会进入lost状态
  int detect_count_;//连续检测到目标的次数
  int temp_lost_count_;//连续丢失目标的次数
  int outpost_max_temp_lost_count_;//前哨站连续丢失目标的最大次数
  int normal_temp_lost_count_;//普通目标连续丢失目标的最大次数
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;

  void state_machine(bool found);//状态机

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);//当没有目标时，用当前装甲板初始化一个新target

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);//已有目标时，用当前帧装甲板更新target
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP