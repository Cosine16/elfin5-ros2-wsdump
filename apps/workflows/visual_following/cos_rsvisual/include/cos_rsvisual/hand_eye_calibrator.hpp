// cos_rsvisual: 眼在手上手眼标定骨架(设计稿, 暂未注册为组件, 不参与编译链接)
//
// 目的: 真机阶段求解 elfin_end_link -> camera_link 的固定外参,
//       结果填到 launch 的 mount_xyz/mount_rpy(见 docs/architecture.md §4)。
//
// 推荐做法: 直接用 easy_handeye(https://github.com/IFL-CAMP/easy_handeye),
// 本骨架仅在本包需要自研轻量标定流程时启用。流程:
//   1. 机械臂末端附近固定一块标定板(ArUco/棋盘格), 对世界系静止;
//   2. 采集 N 组样本: 每组 = 末端在 world 的位姿(TF) + 标定板在相机系的位姿(检测);
//   3. 解 AX=XB(OpenCV calibrateHandEye, Tsai/Park)得 hand->camera;
//   4. 输出 mount_xyz/mount_rpy 写入 launch 默认值。
//
// TODO 列表见 docs/TODO.md。

#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace cos_rsvisual
{

// 单组标定样本
struct HandEyeSample
{
  geometry_msgs::msg::PoseStamped end_link_in_world;   // 末端在 world 系位姿(来自 TF)
  geometry_msgs::msg::PoseStamped board_in_camera;     // 标定板在相机系位姿(来自检测)
  bool valid = false;
};

class HandEyeCalibrator : public rclcpp::Node
{
public:
  explicit HandEyeCalibrator(const rclcpp::NodeOptions & options)
  : Node("hand_eye_calibrator", options)
  {
    // TODO: 参数: end_link/target_frame/board 话题/样本数下限/采样最小位姿间隔
    // TODO: 订阅标定板位姿话题(如 /aruco_single/pose), 建 TF listener
  }

private:
  // 采集一组样本(末端动过足够距离/角度后才采, 保证方程组良态)
  void collectSample()
  {
    // TODO: 查 TF 得 end_link_in_world; 取最新 board_in_camera;
    //       与上一样本的位姿差 > 阈值才收进 samples_
  }

  // 样本足够后求解 AX=XB
  bool solve()
  {
    // TODO: samples_ -> cv::calibrateHandEye(Tsai/Park), 评估重投影残差
    return false;
  }

  // 把解算结果转成 mount_xyz/mount_rpy 文本并落盘(yaml/打印)
  void publishResult()
  {
    // TODO: 输出到参数文件 + 提示用户更新 launch 默认值
  }

  std::vector<HandEyeSample> samples_;
  // TODO: TF buffer/listener、标定板订阅、参数成员
};

}  // namespace cos_rsvisual
