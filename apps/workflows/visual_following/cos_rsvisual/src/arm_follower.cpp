// cos_rsvisual: 末端跟随组件
//
// 订阅 /person_pose(检测器发布的世界系人位置), 通过 tf2 读取 elfin_end_link
// 当前位姿, 满足条件时直接用内置 MoveGroupInterface 规划执行(原因: 本环境下
// elfin_basic_api_node 的 /cart_goal 订阅只能收到首条消息, 后续消息送达但
// 回调不再触发, 已用 gdb + 手动发布实证; 故不再经 /cart_goal 中转)。
//   - 末端位置保持不变;
//   - 姿态解算为 "使 elfin_end_link 的 +Z 轴(相机安装方向)对准人的方向" 的
//     最小旋转(从当前 z 轴到目标方向的最短弧四元数);
//   - 人位置较上次目标移动超过 deadband 且距上次发送超过 update_period 才发;
//   - 人距末端 < keep_distance 或 z < min_height 时跳过。
// 注意: MoveGroupInterface 需要本节点持有 robot_description 参数(launch 中已注入)。

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace cos_rsvisual
{

class ArmFollower : public rclcpp::Node
{
public:
  explicit ArmFollower(const rclcpp::NodeOptions & options)
  : Node("arm_follower", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- 参数(与 config/follower.yaml 对应) ----
    person_topic_ = declare_parameter<std::string>("person_topic", "/person_pose");
    target_frame_ = declare_parameter<std::string>("target_frame", "world");
    planning_group_ = declare_parameter<std::string>("planning_group", "elfin_arm");
    end_link_ = declare_parameter<std::string>("end_link", "elfin_end_link");
    deadband_ = declare_parameter<double>("deadband", 0.15);
    update_period_ = declare_parameter<double>("update_period", 1.5);
    min_height_ = declare_parameter<double>("min_height", 0.10);
    keep_distance_ = declare_parameter<double>("keep_distance", 0.45);
    velocity_scaling_ = declare_parameter<double>("velocity_scaling", 0.3);
    enable_follow_ = declare_parameter<bool>("enable_follow", true);
    // 初始姿态: home(全零直立)是腕部奇异位形, 姿态目标 IK 不可解。
    // 首次就绪时先做一个关节空间小幅度手腕下弯(默认 joint5=-0.5, 置空数组则跳过)。
    // 不用大摆臂的命名姿态: 大轨迹执行中途被规划场景更新(障碍物插入)打断过。
    initial_joints_ = declare_parameter<std::vector<double>>(
      "initial_joints", std::vector<double>{0.0, 0.0, 0.0, 0.0, -0.5, 0.0});

    person_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      person_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_person_ = *msg;
        has_person_ = true;
      });

    // 定时检查并决定是否发新目标(5Hz 轮询, 实际发送受 update_period 节流)
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&ArmFollower::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "arm_follower 已启动: %s -> MoveGroupInterface(%s), enable_follow=%s",
      person_topic_.c_str(), planning_group_.c_str(),
      enable_follow_ ? "true" : "false");
  }

private:
  // MoveGroupInterface 构造依赖节点已在容器中完成加载(shared_from_this)
  // 且 move_group 已就绪, 故延迟到首次使用时初始化, 失败重试
  bool ensureMoveGroup()
  {
    if (move_group_) {return true;}
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
      move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
      move_group_->setMaxAccelerationScalingFactor(velocity_scaling_);
      move_group_->setPlanningTime(5.0);
      RCLCPP_INFO(get_logger(), "MoveGroupInterface 就绪, 规划组 %s", planning_group_.c_str());

      // 初始姿态: 关节空间小幅手腕下弯, 离开 home 奇异位形
      if (initial_joints_.size() == 6 && !initial_move_sent_) {
        const std::vector<std::string> & joint_names =
          move_group_->getJointNames();  // elfin_joint1..6
        std::map<std::string, double> target;
        for (size_t i = 0; i < 6; ++i) {
          target[joint_names[i]] = initial_joints_[i];
        }
        move_group_->setJointValueTarget(target);
        move_group_->asyncMove();
        initial_move_sent_ = true;
        initial_move_time_ = this->now();
        RCLCPP_INFO(
          get_logger(), "先执行初始姿态(手腕下弯 %.2f rad, 离开 home 奇异位形)",
          initial_joints_[4]);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "MoveGroupInterface 初始化失败(5s 后重试): %s", e.what());
      return false;
    }
    return static_cast<bool>(move_group_);
  }

  void onTimer()
  {
    // enable_follow 支持运行时关闭
    enable_follow_ = get_parameter("enable_follow").as_bool();
    if (!enable_follow_ || !has_person_) {return;}

    const rclcpp::Time now = this->now();

    // 人位置太久没更新视为目标丢失(超时阈值取 2 倍更新周期)
    const rclcpp::Time person_stamp(latest_person_.header.stamp);
    if (person_stamp.nanoseconds() > 0 &&
      (now - person_stamp).seconds() > 2.0 * update_period_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "人位置数据超时, 等待检测器输出...");
      return;
    }

    // ---- 人位置转到 target_frame(检测器一般已在此系, 保险起见再转一次) ----
    // 用 TimePointZero + 零等待, 避免 foxy canTransform 阻塞轮询卡死执行器
    geometry_msgs::msg::PoseStamped person;
    try {
      if (latest_person_.header.frame_id == target_frame_) {
        person = latest_person_;
      } else {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
          target_frame_, latest_person_.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(latest_person_, person, tf);
      }
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "人位置 TF 变换失败: %s", e.what());
      return;
    }
    const tf2::Vector3 person_pos(
      person.pose.position.x, person.pose.position.y, person.pose.position.z);

    // ---- 查询末端当前位姿(TimePointZero 零等待, 理由同上) ----
    geometry_msgs::msg::TransformStamped end_tf;
    try {
      end_tf = tf_buffer_.lookupTransform(
        target_frame_, end_link_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "末端位姿查询失败: %s", e.what());
      return;
    }
    const tf2::Vector3 end_pos(
      end_tf.transform.translation.x,
      end_tf.transform.translation.y,
      end_tf.transform.translation.z);
    tf2::Quaternion end_q;
    tf2::fromMsg(end_tf.transform.rotation, end_q);

    // ---- 安全/有效性检查 ----
    const tf2::Vector3 to_person = person_pos - end_pos;
    const double dist = to_person.length();
    if (dist < keep_distance_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "人距末端 %.2fm < keep_distance %.2fm, 跳过", dist, keep_distance_);
      return;
    }
    if (person_pos.z() < min_height_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "目标高度 %.2fm < min_height %.2fm, 跳过", person_pos.z(), min_height_);
      return;
    }

    // ---- 节流: 位置变化超过 deadband 且间隔超过 update_period 才发 ----
    if (has_last_goal_ &&
      (person_pos - last_goal_person_).length() < deadband_)
    {
      return;
    }
    if ((now - last_send_time_).seconds() < update_period_) {return;}

    // ---- 目标姿态: 末端 +Z 轴最短弧旋转到指向人的方向 ----
    const tf2::Vector3 z_cur = tf2::quatRotate(end_q, tf2::Vector3(0, 0, 1));
    const tf2::Vector3 dir = to_person.normalized();
    // 退化情况: 方向与当前 z 轴(反)平行时 shortestArcQuat 不稳定
    if (std::fabs(z_cur.dot(dir)) > 0.9999) {return;}
    const tf2::Quaternion rot = tf2::shortestArcQuat(z_cur, dir);
    tf2::Quaternion goal_q = rot * end_q;
    goal_q.normalize();

    if (!ensureMoveGroup()) {return;}

    // 初始姿态运动进行中, 暂不跟随
    if (initial_move_sent_ &&
      (now - initial_move_time_).seconds() < 6.0)
    {
      return;
    }

    // ---- 规划执行: 位置不变, 只转姿态 ----
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now;
    goal.header.frame_id = target_frame_;
    goal.pose.position.x = end_pos.x();
    goal.pose.position.y = end_pos.y();
    goal.pose.position.z = end_pos.z();
    goal.pose.orientation = tf2::toMsg(goal_q);

    const bool ok_target = move_group_->setPoseTarget(goal.pose, end_link_);
    const auto move_code = move_group_->asyncMove();
    RCLCPP_INFO(
      get_logger(), "发送跟随目标: 人@(%.2f, %.2f, %.2f), 距离 %.2fm, setPoseTarget=%d, asyncMove=%d",
      person_pos.x(), person_pos.y(), person_pos.z(), dist,
      static_cast<int>(ok_target), static_cast<int>(move_code.val));

    last_goal_person_ = person_pos;
    has_last_goal_ = true;
    last_send_time_ = now;
  }

  // 参数
  std::string person_topic_, target_frame_, planning_group_, end_link_;
  std::vector<double> initial_joints_;
  double deadband_, update_period_, min_height_, keep_distance_, velocity_scaling_;
  bool enable_follow_;
  bool initial_move_sent_ = false;
  rclcpp::Time initial_move_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // 最新人位置
  geometry_msgs::msg::PoseStamped latest_person_;
  bool has_person_ = false;

  // 上次发送的目标对应的人位置与发送时间
  tf2::Vector3 last_goal_person_ = tf2::Vector3(0, 0, 0);
  bool has_last_goal_ = false;
  rclcpp::Time last_send_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr person_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cos_rsvisual

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cos_rsvisual::ArmFollower)
