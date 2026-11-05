// cos_rsvisual: 规划场景障碍物更新组件
//
// 背景: Foxy 的 MoveIt deb (2.2.3) 没有编译 PointCloudOctomapUpdater 插件
// (foxy 分支 occupancy_map_monitor 的 CMakeLists 不含 updater), 点云无法
// 进入 OctoMap。作为替代方案, 本组件订阅检测器发布的 /person_pose, 把人作为
// 一个圆柱体 collision object 通过 /planning_scene diff 话题写入规划场景,
// 使 MoveIt 规划时绕开人; 人丢失超时后自动移除。
//
// 不链接 moveit 库, 只发 moveit_msgs, 保持组件轻量。

#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace cos_rsvisual
{

class ObstacleUpdater : public rclcpp::Node
{
public:
  explicit ObstacleUpdater(const rclcpp::NodeOptions & options)
  : Node("obstacle_updater", options)
  {
    // ---- 参数(与 config/follower.yaml 对应) ----
    person_topic_ = declare_parameter<std::string>("person_topic", "/person_pose");
    target_frame_ = declare_parameter<std::string>("target_frame", "world");
    planning_scene_topic_ =
      declare_parameter<std::string>("planning_scene_topic", "/planning_scene");
    obstacle_radius_ = declare_parameter<double>("obstacle_radius", 0.30);
    obstacle_height_ = declare_parameter<double>("obstacle_height", 1.7);
    update_period_ = declare_parameter<double>("update_period", 0.5);
    move_threshold_ = declare_parameter<double>("move_threshold", 0.05);
    lost_timeout_ = declare_parameter<double>("lost_timeout", 3.0);
    enabled_ = declare_parameter<bool>("enabled", true);

    scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      planning_scene_topic_, 10);

    person_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      person_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_person_ = *msg;
        has_person_ = true;
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ObstacleUpdater::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "obstacle_updater 已启动: %s -> %s (圆柱 r=%.2f h=%.2f)",
      person_topic_.c_str(), planning_scene_topic_.c_str(),
      obstacle_radius_, obstacle_height_);
  }

private:
  void publishObject(uint8_t operation, double x, double y)
  {
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // CYLINDER dimensions: [height, radius]
    prim.dimensions = {obstacle_height_, obstacle_radius_};

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = obstacle_height_ / 2.0;  // 底部落地
    pose.orientation.w = 1.0;

    moveit_msgs::msg::CollisionObject obj;
    obj.header.stamp = this->now();
    obj.header.frame_id = target_frame_;
    obj.id = "person_obstacle";
    obj.operation = operation;
    if (operation != moveit_msgs::msg::CollisionObject::REMOVE) {
      obj.primitives.push_back(prim);
      obj.primitive_poses.push_back(pose);
    }

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.world.collision_objects.push_back(obj);
    scene_pub_->publish(scene);
  }

  void onTimer()
  {
    enabled_ = get_parameter("enabled").as_bool();
    const rclcpp::Time now = this->now();

    // ---- 人丢失超时: 移除障碍物 ----
    if (has_person_) {
      const rclcpp::Time stamp(latest_person_.header.stamp);
      if (stamp.nanoseconds() > 0 && (now - stamp).seconds() > lost_timeout_) {
        has_person_ = false;
      }
    }
    if ((!has_person_ || !enabled_) && obstacle_present_) {
      publishObject(moveit_msgs::msg::CollisionObject::REMOVE, 0.0, 0.0);
      obstacle_present_ = false;
      RCLCPP_INFO(get_logger(), "人目标丢失/功能关闭, 已移除障碍物");
      return;
    }
    if (!has_person_ || !enabled_) {return;}

    // ---- 节流: 移动超过阈值且间隔超过 update_period 才更新 ----
    const double x = latest_person_.pose.position.x;
    const double y = latest_person_.pose.position.y;
    if (obstacle_present_) {
      const double dx = x - last_x_, dy = y - last_y_;
      if (std::hypot(dx, dy) < move_threshold_) {return;}
    }
    if ((now - last_pub_time_).seconds() < update_period_) {return;}

    publishObject(moveit_msgs::msg::CollisionObject::ADD, x, y);
    obstacle_present_ = true;
    last_x_ = x;
    last_y_ = y;
    last_pub_time_ = now;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "障碍物已更新: (%.2f, %.2f)", x, y);
  }

  // 参数
  std::string person_topic_, target_frame_, planning_scene_topic_;
  double obstacle_radius_, obstacle_height_, update_period_, move_threshold_, lost_timeout_;
  bool enabled_;

  geometry_msgs::msg::PoseStamped latest_person_;
  bool has_person_ = false;
  bool obstacle_present_ = false;
  double last_x_ = 0.0, last_y_ = 0.0;
  rclcpp::Time last_pub_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr person_sub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cos_rsvisual

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cos_rsvisual::ObstacleUpdater)
