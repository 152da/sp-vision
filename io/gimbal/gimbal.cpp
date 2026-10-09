#include "gimbal.hpp"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <vector>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
constexpr float kRad2Deg = 57.29577951308232f;
constexpr float kDeg2Rad = 0.017453292519943295f;
}  // namespace

//构造函数，开启串口，以及读取线程
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  auto baud_rate = tools::read<int>(yaml, "baud_rate");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baud_rate);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}
//插值计算拍摄机姿态，返回时间点t的四元数
Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}
//两种封装的发送
void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  send(
    VisionToGimbal.mode != 0, VisionToGimbal.mode == 2, VisionToGimbal.yaw,
    VisionToGimbal.yaw_vel, VisionToGimbal.yaw_acc, VisionToGimbal.pitch, VisionToGimbal.pitch_vel,
    VisionToGimbal.pitch_acc);
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_frame_.target_yaw_deg = yaw * kRad2Deg;
  tx_frame_.target_yaw_vel_deg_s = yaw_vel * kRad2Deg;
  tx_frame_.target_yaw_acc_deg_s2 = yaw_acc * kRad2Deg;
  tx_frame_.target_pitch_deg = pitch * kRad2Deg;
  tx_frame_.target_pitch_vel_deg_s = pitch_vel * kRad2Deg;
  tx_frame_.target_pitch_acc_deg_s2 = pitch_acc * kRad2Deg;

  if (control) {
    // 控制时发目标角，TRACKING/SHOOTING由fire决定
    tx_frame_.state = fire ? 3 : 2;  // 3: SHOOTING, 2: TRACKING
  } else {
    // 对齐RM：不控制时（lost/idle）发当前云台角 + STATE_LOST，云台保持位置不甩回零
    std::lock_guard<std::mutex> lock(mutex_);
    tx_frame_.target_yaw_deg = state_.yaw * kRad2Deg;
    tx_frame_.target_pitch_deg = state_.pitch * kRad2Deg;
    tx_frame_.target_yaw_vel_deg_s = 0;
    tx_frame_.target_yaw_acc_deg_s2 = 0;
    tx_frame_.target_pitch_vel_deg_s = 0;
    tx_frame_.target_pitch_acc_deg_s2 = 0;
    tx_frame_.state = 1;  // LOST
  }

  // CRC校验范围：len + payload，共 sizeof - 帧头2 - crc2 = 26字节
  tx_frame_.crc16 = tools::get_crc16_modbus(
    reinterpret_cast<uint8_t *>(&tx_frame_) + offsetof(PcToCtlFrame, len),
    sizeof(PcToCtlFrame) - offsetof(PcToCtlFrame, len) - sizeof(tx_frame_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_frame_), sizeof(tx_frame_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}
//读取，只要是读取失败就返回false，读取成功返回true
bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    //如果读取到的字节数相等
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    return false;
  }
}
//读取线程：逐字节滑动找帧头，读取len，读取payload+crc，CRC校验通过后更新状态
void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  // 解析状态机用缓冲：帧头2 + len 1 + payload 30 + crc 2 = 35字节
  std::vector<uint8_t> buf;
  buf.reserve(sizeof(CtlToPcFrame));

  while (!quit_) {
    if (error_count > 50000) {  //看门狗
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      buf.clear();
      continue;
    }

    uint8_t byte;
    if (!read(&byte, 1)) {
      error_count++;
      continue;
    }

    buf.push_back(byte);
    if (buf.size() > sizeof(CtlToPcFrame)) buf.erase(buf.begin());

    // 不足最小帧长（帧头2 + len 1）则继续收
    if (buf.size() < 3) continue;

    // 帧头不对，滑动窗口丢弃最老的一个字节
    if (buf[0] != 0xAA || buf[1] != 0x55) {
      buf.erase(buf.begin());
      continue;
    }

    auto len = buf[2];
    auto frame_len = 3 + len + 2;
    if (len != sizeof(GimbalToVision)) {
      // 长度不符合预期，丢弃帧头继续找（防止假帧头导致长时间等待）
      tools::logger()->debug("[Gimbal] Unexpected payload len: {}", len);
      buf.erase(buf.begin(), buf.begin() + 2);
      continue;
    }

    // 整帧未收齐，继续收
    if (buf.size() < frame_len) continue;

    // CRC校验，范围为 len + payload（从buf[2]开始共1+len字节）
    auto received_crc = static_cast<uint16_t>(buf[frame_len - 2]) |
                        static_cast<uint16_t>(buf[frame_len - 1]) << 8;
    auto calculated_crc =
      tools::get_crc16_modbus(buf.data() + 2, 1 + len);
    if (received_crc != calculated_crc) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      buf.erase(buf.begin());
      continue;
    }

    // 到这里算是校验成功
    error_count = 0;
    auto t = std::chrono::steady_clock::now();

    GimbalToVision payload;
    std::memcpy(&payload, buf.data() + 3, sizeof(payload));
    buf.erase(buf.begin(), buf.begin() + frame_len);

    // deg -> rad
    auto yaw = payload.yaw * kDeg2Rad;
    auto pitch = payload.pitch * kDeg2Rad;
    auto yaw_vel = payload.yaw_vel * kDeg2Rad;
    auto pitch_vel = payload.pitch_vel * kDeg2Rad;
    auto roll = payload.roll * kDeg2Rad;
    auto delta_bigyaw_angle = payload.delta_bigyaw_angle * kDeg2Rad;

    // 对齐RM tf_broadcaster: q = q_yaw * q_pitch * q_roll（ZYX），构造云台系姿态四元数
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    q.normalize();
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = yaw;
    state_.yaw_vel = yaw_vel;
    state_.pitch = pitch;
    state_.pitch_vel = pitch_vel;
    state_.roll = roll;
    state_.delta_bigyaw_angle = delta_bigyaw_angle;
    state_.bullet_speed = payload.bullet_speed;

    switch (payload.mode) {
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 0:
      default:
        mode_ = GimbalMode::IDLE;
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
