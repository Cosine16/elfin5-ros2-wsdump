// cos_rsvisual: 人位置跟踪滤波器骨架(设计稿, 暂未注册为组件, 不参与编译链接)
//
// 目的: 检测器输出的 /person_pose 是逐帧独立估计, 存在像素级抖动与偶发跳变,
//       直接驱动 arm_follower 会导致末端抖动。本组件做时间域滤波:
//       位置用 CV(匀速)模型卡尔曼滤波, 输出平滑后的 /person_pose_filtered。
//
// 使用方式(启用后): 检测器 -> /person_pose -> [本组件] -> /person_pose_filtered
//       -> arm_follower/obstacle_updater(follower.yaml 里改 person_topic 即可)。
//
// TODO 列表见 docs/TODO.md。

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace cos_rsvisual
{

class PersonTracker : public rclcpp::Node
{
public:
  explicit PersonTracker(const rclcpp::NodeOptions & options)
  : Node("person_tracker", options)
  {
    // TODO: 参数: in_topic(/person_pose), out_topic(/person_pose_filtered),
    //       process_noise, measurement_noise, lost_timeout,
    //       max_jump(单帧跳变超过该值视为误检, 不入滤波器)
  }

private:
  // 检测输入回调: 跳变门限检查 -> 卡尔曼量测更新
  void onPersonPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    (void)msg;
    // TODO: if (跳变 > max_jump && 已初始化) 拒绝本次量测
    // TODO: kf_.correct(msg->pose.position)
  }

  // 定时预测+发布(固定频率, 检测丢失后外推 lost_timeout 秒再停发)
  void onTimer()
  {
    // TODO: kf_.predict(); 发布 /person_pose_filtered
    // TODO: 超时未更新 -> 停发(下游 arm_follower 会因数据超时自动暂停)
  }

  // TODO: 卡尔曼状态 [x y z vx vy vz], 可以用 OpenCV cv::KalmanFilter 实现
  // cv::KalmanFilter kf_;  // 引入 <opencv2/video/tracking.hpp>
};

}  // namespace cos_rsvisual
