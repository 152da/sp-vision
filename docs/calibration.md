# 相机标定指南（内参 + 手眼标定）

本文档介绍如何使用本仓库的标定程序，完成相机内参标定和相机-云台手眼标定，并把结果写入机器人配置文件。

标定共四个程序，全部在 **Linux 实车环境**（需相机 + 云台串口）下运行：

| 程序 | 作用 | 输出 |
|---|---|---|
| `capture` | 采集图像 + 对应时刻云台四元数 | `*.jpg` + `*.txt` 文件对 |
| `calibrate_camera` | 相机内参标定 | `camera_matrix`、`distort_coeffs` |
| `calibrate_handeye` | 手眼标定 | `R_camera2gimbal`、`t_camera2gimbal` |
| `calibrate_robotworld_handeye` | 手眼标定（另算标定板位置，用于交叉验证） | 同上 + 板距离/偏角 |

**依赖顺序**：采集 → 内参 → 手眼。手眼标定依赖内参结果，顺序不能颠倒。

## 编译

在实车 Linux 环境的仓库根目录执行（编译全部代码）：

```bash
# 首次或跨平台残留缓存：清掉旧 build 目录后重新配置
rm -rf build && cmake -B build

# 编译全部目标
cmake --build build -j$(nproc)
```

> 若之前在同一台 Linux 机器上已配置过，直接 `cmake --build build -j$(nproc)` 增量编译即可，无需删 build。
> Windows 下残留的 build 缓存（VS 生成器）在 Linux 上会配置失败，所以换环境编译前先清目录。

---

## 0. 准备：配置标定板参数

编辑 `configs/calibration.yaml` 开头三行，按实际棋盘格填写：

```yaml
# 棋盘格内角点数（内角点 = 方格数 - 1，如 9x6 内角点对应 10x7 方格）
pattern_cols: 11
pattern_rows: 8
# 棋盘格方格边长，单位 mm
center_distance_mm: 60
```

⚠️ **最常见的错误**：`pattern_cols/rows` 填的是**内角点数**，不是方格数。
例如一块俗称 "12×9 格" 的棋盘，内角点是 11×8。填错则后续所有图片检测全部 `[failure]`。

当前配置对应：11×8 内角点、方格边长 60mm 的棋盘格。

同文件中还需确认云台串口参数（与实车配置一致即可，可从 `configs/standard3.yaml` 复制）：

```yaml
#####-----gimbal参数-----#####
com_port: "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3144366B3233-if00"
baud_rate: 115200
```

## 1. 数据采集（capture）

```bash
./build/capture configs/calibration.yaml -o=assets/img_with_q
```

启动后若卡在 `[Gimbal] First q received.` 之前，说明串口没有数据——检查下位机是否在持续发送 35 字节协议帧（与实车运行时相同，标定对下位机无额外要求）。

**操作方式**：窗口内按 `s` 保存当前帧（图像 + 对应四元数），按 `q` 退出。四元数由 `io::Gimbal` 从串口帧中的 yaw/pitch/roll 合成（`q = Rz·Ry·Rx`），并按图像时间戳插值对齐。

**采集要点**：

- **板子固定不动，转动云台拍摄**。采集 10~20 张，覆盖不同 yaw/pitch 姿态和距离（建议 1~3 m），姿态越丰富手眼标定越准。
- 每张图确认窗口内角点被彩色连线完整覆盖后再按 `s`；终端打印 `[success]` 才有效。
- **避免板子相对相机接近 180° 旋转**的图——棋盘格角点序会翻转，导致后续 3D-2D 错配。判断方法：与第一张图对比，角点彩色连线方向（首尾颜色）应一致。
- 利用窗口上显示的 yaw/pitch/roll 检查 `R_gimbal2imubody`：云台抬头/左转时对应欧拉角变化方向应符合坐标系约定，不对则先修 yaml 里的 `R_gimbal2imubody` 再采。

产出：`assets/img_with_q/` 下的 `1.jpg, 1.txt, 2.jpg, 2.txt, ...`（txt 内为 wxyz 顺序四元数）。

## 2. 相机内参标定（calibrate_camera）

```bash
./build/calibrate_camera assets/img_with_q -c=configs/calibration.yaml
```

程序会逐张弹窗显示检测效果（按任意键看下一张），跳过 `[failure]` 的图，最后在终端输出一段 yaml：

```yaml
# 重投影误差: 0.xxxx px
camera_matrix: [...]
distort_coeffs: [...]
```

**把这段复制进 `configs/calibration.yaml`**，供手眼标定和实车程序使用。

- 重投影误差一般应在 **0.5 px 以内**，明显偏大说明有坏图（模糊、反光、角点序翻转），剔除后重跑。
- 内参标定固定了 K3 畸变系数（视场角小，不需要 K3）。

## 3. 手眼标定

内参写入 yaml 后，二选一（输入相同）：

```bash
./build/calibrate_handeye assets/img_with_q -c=configs/calibration.yaml
# 或
./build/calibrate_robotworld_handeye assets/img_with_q -c=configs/calibration.yaml
```

输出：

```yaml
R_gimbal2imubody: [...]
# 相机同理想情况的偏角: yaw x.xx pitch x.xx roll x.xx degree
R_camera2gimbal: [...]
t_camera2gimbal: [...]   # 单位：米
```

`calibrate_robotworld_handeye` 额外输出"标定板到世界原点的水平距离"和"板偏角"注释——**拿你实际摆板的位置核对**（误差应在几 cm 内），这是最方便的交叉验证手段。

**把 `R_camera2gimbal`、`t_camera2gimbal`（连同 `R_gimbal2imubody`）复制进机器人主配置**（如 `configs/standard3.yaml` 对应字段）。

结果合理性检查：

- `t_camera2gimbal` 量级应为 0.0x m，与相机相对云台中心的实际安装位置相符；
- "相机同理想情况的偏角"应接近相机的实际安装偏角（理想安装为 yaw 0 / pitch -90° / roll 0）。

## 4. 实车验证

用 `handeye_test` 程序验证：对准已知目标，观察解算位置/姿态是否与实际一致。

```bash
./build/handeye_test configs/standard3.yaml
```

---

## 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `Is a directory` / 数据存到了名为 `true` 的目录 | OpenCV `CommandLineParser` 可选参数必须用**等号**：`-o=path`，空格分隔会把值解析成 `true` | 用 `-o=`/`-c=` 形式重跑；误存的数据从 `true/` 搬回 |
| `capture` 卡在 `Waiting for q` / `First q received` | 串口无数据 | 检查下位机是否在发帧；`ls /dev/serial/by-id/` 确认设备路径 |
| 所有图片检测 `[failure]` | `pattern_cols/rows` 与板不符（含横竖颠倒） | 数内角点重填；横竖拿反则交换两值 |
| 个别图 `[failure]` | 模糊/反光/角度过斜 | 正常，程序自动跳过 |
| 重投影误差 > 1 px | 混入坏图或板参数错 | 逐张检查检测效果，剔除后重跑 |
| 手眼结果平移明显不符 | 采集姿态多样性不足，或混入角点序翻转的图 | 换更多姿态重新采集；剔除连线方向突变的图 |
| 标定板距离/偏角与实际摆放不符 | 同上，或内参不准 | 先确认内参误差小，再重采数据 |

## 附：换标定板时改哪里

只改 `configs/calibration.yaml` 前三行（内角点数 + 方格边长），代码无需改动。检测使用 `cv::findChessboardCornersSB`（OpenCV ≥ 4.0），直接吃彩色图、内置亚像素精化。旧板（圆点板）拍的存量数据对新程序无效，需重新采集。
