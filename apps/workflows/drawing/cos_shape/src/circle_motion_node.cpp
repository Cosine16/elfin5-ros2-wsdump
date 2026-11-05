// circle_motion_node.cpp
// =====================================================================
// 让机械臂末端围绕某个三维点做圆周运动（cos_shape 包）。
//
// 圆周参数：
//   - center       : 圆心（规划坐标系下的三维点）
//   - radius       : 半径 [m]
//   - inclination  : 圆平面倾角 [rad]，圆平面法线与基座 Z 轴的夹角
//                    （0 = 水平圆，pi/2 = 竖直圆）
//   - azimuth      : 倾角旋转方位 [rad]，绕 Z 轴
//   - speed        : 角速度 [rad/s]（speed_mode=angular）
//                    或线速度 [m/s]（speed_mode=linear）
//
// Topic 接口（均在节点命名空间 ~/ 下）：
//   订阅 ~/set_angular_velocity (std_msgs/Float64)  rad/s，同时切换到角速度模式
//   订阅 ~/set_linear_velocity  (std_msgs/Float64)  m/s，  同时切换到线速度模式
//   订阅 ~/set_center           (geometry_msgs/Point) 圆心，下一圈生效
//   订阅 ~/set_inclination      (std_msgs/Float64)  倾角 rad，下一圈生效
//   订阅 ~/set_azimuth          (std_msgs/Float64)  方位角 rad，下一圈生效
//   订阅 ~/set_radius           (std_msgs/Float64)  半径 m，下一圈生效
//   订阅 ~/enable               (std_msgs/Bool)     true 开始 / false 停止
//   发布 ~/state                (std_msgs/String)   每圈结束发布一次状态摘要
//
// 末端姿态：运动期间保持开始时刻的姿态不变。
// 速度通过时间参数化保证：先 computeCartesianPath 生成关节轨迹，
// 再用 IterativeParabolicTimeParameterization 加时间戳，最后整体
// 缩放到目标周期 T = 2*pi/omega（或 T = 2*pi*r/v）。
// =====================================================================

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace
{
constexpr double kTwoPi = 6.283185307179586;

using Vec3 = std::array<double, 3>;

Vec3 cross(const Vec3& a, const Vec3& b)
{
  return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
}

double norm(const Vec3& v)
{
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Vec3 normalize(const Vec3& v)
{
  const double n = norm(v);
  return { v[0] / n, v[1] / n, v[2] / n };
}

// 把时间戳整体缩放 k 倍；速度、加速度同步缩放保持轨迹一致
void scaleTrajectoryTime(trajectory_msgs::msg::JointTrajectory& jt, double k)
{
  for (auto& p : jt.points)
  {
    const int64_t ns = static_cast<int64_t>(p.time_from_start.sec) * 1000000000LL + p.time_from_start.nanosec;
    const int64_t scaled = static_cast<int64_t>(ns * k);
    p.time_from_start.sec = static_cast<int32_t>(scaled / 1000000000LL);
    p.time_from_start.nanosec = static_cast<uint32_t>(scaled % 1000000000LL);
    for (auto& v : p.velocities)
      v /= k;
    for (auto& a : p.accelerations)
      a /= (k * k);
  }
}
}  // namespace

class CircleMotion : public rclcpp::Node
{
public:
  CircleMotion() : Node("circle_motion")
  {
    // ---- 参数（可被 launch / --ros-args -p 覆盖）----
    group_name_ = declare_parameter<std::string>("group_name", "elfin_arm");
    ee_link_ = declare_parameter<std::string>("ee_link", "elfin_end_link");
    cfg_.center = declare_parameter<std::vector<double>>("center", { 0.3, 0.0, 0.3 });
    cfg_.radius = declare_parameter<double>("radius", 0.05);
    cfg_.angular_velocity = declare_parameter<double>("angular_velocity", 0.5);
    cfg_.linear_velocity = declare_parameter<double>("linear_velocity", 0.05);
    cfg_.speed_mode = declare_parameter<std::string>("speed_mode", "angular");
    cfg_.inclination = declare_parameter<double>("inclination", 0.0);
    cfg_.azimuth = declare_parameter<double>("azimuth", 0.0);
    waypoints_per_rev_ = declare_parameter<int>("waypoints_per_rev", 64);
    revolutions_ = declare_parameter<int>("revolutions", -1);  // -1 = 无限
    eef_step_ = declare_parameter<double>("eef_step", 0.005);
    autostart_ = declare_parameter<bool>("autostart", false);
    if (cfg_.center.size() != 3)
    {
      RCLCPP_WARN(get_logger(), "center 参数必须是 3 个数字，回退到默认值 [0.3, 0, 0.3]");
      cfg_.center = { 0.3, 0.0, 0.3 };
    }

    // ---- Topic 接口 ----
    sub_angular_ = create_subscription<std_msgs::msg::Float64>(
        "~/set_angular_velocity", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          cfg_.angular_velocity = msg->data;
          cfg_.speed_mode = "angular";
          RCLCPP_INFO(get_logger(), "角速度更新为 %.4f rad/s（下一圈生效）", msg->data);
        });
    sub_linear_ = create_subscription<std_msgs::msg::Float64>(
        "~/set_linear_velocity", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          cfg_.linear_velocity = msg->data;
          cfg_.speed_mode = "linear";
          RCLCPP_INFO(get_logger(), "线速度更新为 %.4f m/s（下一圈生效）", msg->data);
        });
    sub_center_ = create_subscription<geometry_msgs::msg::Point>(
        "~/set_center", 10,
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          cfg_.center = { msg->x, msg->y, msg->z };
          RCLCPP_INFO(get_logger(), "圆心更新为 [%.3f, %.3f, %.3f]（下一圈生效）", msg->x, msg->y, msg->z);
        });
    sub_inclination_ = create_subscription<std_msgs::msg::Float64>(
        "~/set_inclination", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          cfg_.inclination = msg->data;
          RCLCPP_INFO(get_logger(), "倾角更新为 %.4f rad（下一圈生效）", msg->data);
        });
    sub_azimuth_ = create_subscription<std_msgs::msg::Float64>(
        "~/set_azimuth", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          cfg_.azimuth = msg->data;
          RCLCPP_INFO(get_logger(), "方位角更新为 %.4f rad（下一圈生效）", msg->data);
        });
    sub_radius_ = create_subscription<std_msgs::msg::Float64>(
        "~/set_radius", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          cfg_.radius = msg->data;
          RCLCPP_INFO(get_logger(), "半径更新为 %.4f m（下一圈生效）", msg->data);
        });
    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
        "~/enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          enabled_.store(msg->data);
          RCLCPP_INFO(get_logger(), "%s", msg->data ? "收到启动指令" : "收到停止指令");
        });
    pub_state_ = create_publisher<std_msgs::msg::String>("~/state", 10);

    enabled_.store(autostart_);
  }

  void run()
  {
    moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), group_name_);
    mgi.setMaxVelocityScalingFactor(0.3);
    mgi.setMaxAccelerationScalingFactor(0.3);
    const std::string planning_frame = mgi.getPlanningFrame();
    RCLCPP_INFO(get_logger(), "规划坐标系: %s, 末端连杆: %s", planning_frame.c_str(), ee_link_.c_str());
    RCLCPP_INFO(get_logger(), "等待 ~/enable = true ...");

    while (rclcpp::ok())
    {
      if (!enabled_.load())
      {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      Config cfg = snapshot();
      if (cfg.radius <= 1e-6)
      {
        RCLCPP_ERROR(get_logger(), "radius 必须 > 0，停止。");
        enabled_.store(false);
        continue;
      }

      // 固定末端姿态为启动时刻姿态
      auto current_pose = mgi.getCurrentPose(ee_link_);
      const geometry_msgs::msg::Quaternion keep_orientation = current_pose.pose.orientation;

      // 1) 先走到圆周起点 p(0)
      geometry_msgs::msg::Pose start_pose;
      const Vec3 p0 = circlePoint(cfg, 0.0);
      start_pose.position.x = p0[0];
      start_pose.position.y = p0[1];
      start_pose.position.z = p0[2];
      start_pose.orientation = keep_orientation;
      mgi.setPoseTarget(start_pose, ee_link_);
      RCLCPP_INFO(get_logger(), "移动到圆周起点 [%.3f, %.3f, %.3f] ...", p0[0], p0[1], p0[2]);
      if (mgi.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "无法到达圆周起点（可能超出工作空间），停止。");
        enabled_.store(false);
        continue;
      }

      // 2) 逐圈执行
      int rev = 0;
      int consecutive_failures = 0;
      while (rclcpp::ok() && enabled_.load() && (revolutions_ < 0 || rev < revolutions_))
      {
        cfg = snapshot();
        moveit_msgs::msg::RobotTrajectory traj;
        const double fraction = planOneRevolution(mgi, cfg, keep_orientation, traj);
        if (fraction < 0.99)
        {
          RCLCPP_ERROR(get_logger(), "圆周路径仅覆盖 %.1f%%，轨迹不可达，停止。", fraction * 100.0);
          enabled_.store(false);
          break;
        }

        if (!retimeToPeriod(mgi, cfg, traj))
        {
          RCLCPP_ERROR(get_logger(), "时间参数化失败，停止。");
          enabled_.store(false);
          break;
        }

        // 异步执行，主循环轮询停止标志
        auto fut = std::async(std::launch::async, [&mgi, &traj]() { return mgi.execute(traj); });
        while (fut.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
        {
          if (!rclcpp::ok())
            return;
          if (!enabled_.load())
            mgi.stop();
        }
        const auto code = fut.get();
        if (code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          if (!enabled_.load())
            break;  // 用户主动停止
          // 控制器侧偶发中止（如仿真时钟抖动导致执行超时）：跳过本圈，重新规划下一圈
          if (++consecutive_failures >= 5)
          {
            RCLCPP_ERROR(get_logger(), "连续 %d 圈执行失败，停止。", consecutive_failures);
            enabled_.store(false);
            break;
          }
          RCLCPP_WARN(get_logger(), "本圈执行失败（返回码 %d），跳过并重新规划下一圈（连续失败 %d/5）。",
                      code.val, consecutive_failures);
          continue;
        }

        consecutive_failures = 0;
        ++rev;
        publishState(cfg, rev);
      }
      enabled_.store(false);
      RCLCPP_INFO(get_logger(), "圆周运动结束（完成 %d 圈），等待下一次 enable。", rev);
    }
  }

private:
  struct Config
  {
    std::vector<double> center;
    double radius;
    double angular_velocity;
    double linear_velocity;
    std::string speed_mode;
    double inclination;
    double azimuth;
  };

  Config snapshot()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return cfg_;
  }

  // 圆平面法线 n：Z 轴按 (azimuth, inclination) 旋转
  // 平面内正交轴 u（全局 X 在平面内的投影，退化时改用 Y）、v = n × u
  Vec3 circlePoint(const Config& cfg, double theta) const
  {
    const Vec3 n = { std::sin(cfg.inclination) * std::cos(cfg.azimuth),
                     std::sin(cfg.inclination) * std::sin(cfg.azimuth),
                     std::cos(cfg.inclination) };
    Vec3 ref = { 1.0, 0.0, 0.0 };
    if (std::fabs(n[0]) > 0.99)
      ref = { 0.0, 1.0, 0.0 };
    // u = ref 在圆平面内的投影（去掉法向分量后归一化）
    const double d = ref[0] * n[0] + ref[1] * n[1] + ref[2] * n[2];
    const Vec3 u = normalize({ ref[0] - d * n[0], ref[1] - d * n[1], ref[2] - d * n[2] });
    const Vec3 v = cross(n, u);
    return { cfg.center[0] + cfg.radius * (std::cos(theta) * u[0] + std::sin(theta) * v[0]),
             cfg.center[1] + cfg.radius * (std::cos(theta) * u[1] + std::sin(theta) * v[1]),
             cfg.center[2] + cfg.radius * (std::cos(theta) * u[2] + std::sin(theta) * v[2]) };
  }

  double planOneRevolution(moveit::planning_interface::MoveGroupInterface& mgi, const Config& cfg,
                           const geometry_msgs::msg::Quaternion& orientation,
                           moveit_msgs::msg::RobotTrajectory& traj)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(waypoints_per_rev_);
    for (int i = 1; i <= waypoints_per_rev_; ++i)
    {
      const double theta = kTwoPi * i / waypoints_per_rev_;
      const Vec3 p = circlePoint(cfg, theta);
      geometry_msgs::msg::Pose pose;
      pose.position.x = p[0];
      pose.position.y = p[1];
      pose.position.z = p[2];
      pose.orientation = orientation;
      waypoints.push_back(pose);
    }
    return mgi.computeCartesianPath(waypoints, eef_step_, 0.0 /*jump_threshold 禁用*/, traj);
  }

  bool retimeToPeriod(moveit::planning_interface::MoveGroupInterface& mgi, const Config& cfg,
                      moveit_msgs::msg::RobotTrajectory& traj)
  {
    // 目标周期
    double period;
    if (cfg.speed_mode == "linear")
    {
      if (cfg.linear_velocity <= 1e-9)
        return false;
      period = kTwoPi * cfg.radius / cfg.linear_velocity;
    }
    else
    {
      if (cfg.angular_velocity <= 1e-9)
        return false;
      period = kTwoPi / cfg.angular_velocity;
    }

    auto state = mgi.getCurrentState(2.0);
    if (!state)
      return false;

    robot_trajectory::RobotTrajectory rt(mgi.getRobotModel(), group_name_);
    rt.setRobotTrajectoryMsg(*state, traj);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if (!iptp.computeTimeStamps(rt, 1.0, 1.0))
      return false;
    rt.getRobotTrajectoryMsg(traj);

    auto& points = traj.joint_trajectory.points;
    if (points.empty())
      return false;
    const auto& last = points.back().time_from_start;
    const double t_old = static_cast<double>(last.sec) + static_cast<double>(last.nanosec) * 1e-9;
    if (t_old <= 1e-9)
      return false;

    const double k = period / t_old;
    if (k < 1.0)
    {
      RCLCPP_WARN(get_logger(),
                  "目标周期 %.2fs 快于机械臂满速能力 %.2fs，将被关节速度限制放慢。"
                  "请降低速度。",
                  period, t_old);
    }
    scaleTrajectoryTime(traj.joint_trajectory, k);
    RCLCPP_INFO(get_logger(), "本圈周期 %.2fs（模式 %s）", period, cfg.speed_mode.c_str());
    return true;
  }

  void publishState(const Config& cfg, int rev)
  {
    std_msgs::msg::String msg;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "rev=%d center=[%.3f,%.3f,%.3f] radius=%.3f mode=%s omega=%.3f v=%.3f incl=%.3f",
                  rev, cfg.center[0], cfg.center[1], cfg.center[2], cfg.radius, cfg.speed_mode.c_str(),
                  cfg.angular_velocity, cfg.linear_velocity, cfg.inclination);
    msg.data = buf;
    pub_state_->publish(msg);
  }

  // 参数
  std::string group_name_;
  std::string ee_link_;
  int waypoints_per_rev_;
  int revolutions_;
  double eef_step_;
  bool autostart_;

  Config cfg_;
  mutable std::mutex mtx_;
  std::atomic<bool> enabled_{ false };

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_angular_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_linear_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_center_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_inclination_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_azimuth_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_radius_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CircleMotion>();

  // MoveGroupInterface 依赖节点的回调（当前状态监听等），单独线程 spin
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  node->run();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
