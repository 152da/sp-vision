#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
// 串口协议（对齐 RM_Vision_Aiming）：
//   帧格式: 0xAA 0x55 + len(1字节, payload字节数) + payload(N字节) + crc16(2字节)
//   CRC16为Modbus(0xA001)，校验范围为 len + payload，帧头不参与校验
//   多字节数值均为小端

// Control -> PC payload, 30字节, 总帧长35字节
struct __attribute__((packed)) GimbalToVision
{
  uint8_t mode;  // 0: UNKNOWN, 1: AIMING
  float yaw;                 // deg
  float pitch;               // deg, 向下为正
  float yaw_vel;             // deg/s
  float pitch_vel;           // deg/s
  float roll;                // deg
  float delta_bigyaw_angle;  // deg, 大yaw与小yaw的角度差
  float bullet_speed;        // m/s
  int8_t color;              // 己方颜色: 0=UNKNOWN, 1=BLUE, 2=RED, 当前仅占位
};

static_assert(sizeof(GimbalToVision) == 30);

// PC -> Control 上层控制指令（内部使用rad，发送时由Gimbal转换为协议要求的deg）
struct VisionToGimbal
{
  float yaw;        // rad
  float yaw_vel;    // rad/s
  float yaw_acc;    // rad/s^2
  float pitch;      // rad, 向下为正
  float pitch_vel;  // rad/s
  float pitch_acc;  // rad/s^2
  uint8_t mode;     // 0: 不控制, 1: 控制云台但不开火, 2: 控制云台且开火
};

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符（当前协议不含该模式，保留以兼容上层代码）
  BIG_BUFF     // 大符（当前协议不含该模式，保留以兼容上层代码）
};

struct GimbalState
{
  float yaw;                 // rad
  float yaw_vel;             // rad/s
  float pitch;               // rad, 向下为正
  float pitch_vel;           // rad/s
  float roll;                // rad
  float delta_bigyaw_angle;  // rad
  float bullet_speed;        // m/s
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

private:
  // PC -> Control 完整帧, 30字节
  struct __attribute__((packed)) PcToCtlFrame
  {
    uint8_t head[2] = {0xAA, 0x55};
    uint8_t len = 25;
    float target_yaw_deg;
    float target_yaw_vel_deg_s;
    float target_yaw_acc_deg_s2;
    float target_pitch_deg;
    float target_pitch_vel_deg_s;
    float target_pitch_acc_deg_s2;
    uint8_t state;  // 0: ERROR, 1: LOST, 2: TRACKING, 3: SHOOTING
    uint16_t crc16;
  };

  // Control -> PC 完整帧, 35字节
  struct __attribute__((packed)) CtlToPcFrame
  {
    uint8_t head[2];
    uint8_t len;
    GimbalToVision payload;
    uint16_t crc16;
  };

  static_assert(sizeof(PcToCtlFrame) == 30);
  static_assert(sizeof(CtlToPcFrame) == 35);

  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  CtlToPcFrame rx_frame_;
  PcToCtlFrame tx_frame_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_{};
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
