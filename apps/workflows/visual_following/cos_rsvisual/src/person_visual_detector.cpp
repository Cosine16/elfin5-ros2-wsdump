// cos_rsvisual: HOG 视觉人形检测组件(备选方案, 与 person_depth_detector 互斥)
//
// 思路: 订阅彩色图 + 深度图, 用 OpenCV 自带的 HOGDescriptor 默认行人模型做
// 人形检测(CPU 较慢, 每 detect_every_n_frames 帧检测一次), 命中最大检测框后
// 取框中心区域深度中值反投影为 3D 点, 变换到 target_frame 发 /person_pose。
//
// enable 参数默认 false; launch 中由 enable_visual_detector 控制是否加载本组件。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>

namespace cos_rsvisual
{

class PersonVisualDetector : public rclcpp::Node
{
public:
  explicit PersonVisualDetector(const rclcpp::NodeOptions & options)
  : Node("person_visual_detector", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- 参数 ----
    color_topic_ = declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/camera/depth/camera_info");
    enable_ = declare_parameter<bool>("enable", false);
    detect_every_n_frames_ = declare_parameter<int>("detect_every_n_frames", 5);
    scale_factor_ = declare_parameter<double>("scale_factor", 1.05);
    min_neighbors_ = declare_parameter<int>("min_neighbors", 3);
    min_range_ = declare_parameter<double>("min_range", 0.3);
    max_range_ = declare_parameter<double>("max_range", 2.0);
    target_frame_ = declare_parameter<std::string>("target_frame", "world");
    person_topic_ = declare_parameter<std::string>("person_topic", "/person_pose");

    // HOG 默认行人检测模型
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    person_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(person_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/person_marker", 10);

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (cam_model_initialized_) {return;}
        cam_model_.fromCameraInfo(msg);
        cam_frame_ = msg->header.frame_id;
        cam_model_initialized_ = true;
      });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PersonVisualDetector::depthCallback, this, std::placeholders::_1));

    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      color_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PersonVisualDetector::colorCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "person_visual_detector 已启动, enable=%s, 订阅 %s",
      enable_ ? "true" : "false", color_topic_.c_str());
  }

private:
  // 缓存最新深度图(转成 32FC1 米制), 供检测命中后取深度用
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
          msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptr->image.convertTo(latest_depth_, CV_32FC1, 0.001);
      } else {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
          msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.copyTo(latest_depth_);
      }
      has_depth_ = true;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "深度图转换失败: %s", e.what());
    }
  }

  void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!enable_) {return;}
    if (!cam_model_initialized_ || !has_depth_) {return;}

    // 降频检测
    if (++frame_count_ % detect_every_n_frames_ != 0) {return;}

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "彩色图转换失败: %s", e.what());
      return;
    }

    // ---- HOG 人形检测 ----
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Rect> rects;
    hog_.detectMultiScale(
      gray, rects, 0.0, cv::Size(8, 8), cv::Size(32, 32),
      scale_factor_, static_cast<double>(min_neighbors_));
    if (rects.empty()) {return;}

    // 取最大检测框
    cv::Rect best = *std::max_element(
      rects.begin(), rects.end(),
      [](const cv::Rect & a, const cv::Rect & b) {return a.area() < b.area();});

    // 框中心 50% 区域内的深度中值(边缘容易混入背景)
    cv::Rect inner(
      best.x + best.width / 4, best.y + best.height / 4,
      best.width / 2, best.height / 2);
    inner &= cv::Rect(0, 0, latest_depth_.cols, latest_depth_.rows);
    if (inner.empty()) {return;}

    std::vector<float> depths;
    for (int v = inner.y; v < inner.y + inner.height; ++v) {
      const float * dm = latest_depth_.ptr<float>(v);
      for (int u = inner.x; u < inner.x + inner.width; ++u) {
        float d = dm[u];
        if (d >= min_range_ && d <= max_range_) {depths.push_back(d);}
      }
    }
    if (depths.size() < 20) {return;}  // 有效深度太少, 不可信
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    const float depth = depths[depths.size() / 2];

    // ---- 反投影 + TF 变换 ----
    const double u = best.x + best.width / 2.0;
    const double v = best.y + best.height / 2.0;
    cv::Point3d ray = cam_model_.projectPixelTo3dRay(cv::Point2d(u, v));

    geometry_msgs::msg::PoseStamped person_cam;
    person_cam.header.stamp = msg->header.stamp;
    person_cam.header.frame_id = cam_frame_;
    person_cam.pose.position.x = ray.x * depth;
    person_cam.pose.position.y = ray.y * depth;
    person_cam.pose.position.z = ray.z * depth;
    person_cam.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped person_world;
    try {
      // TimePointZero 零等待, 避免 foxy canTransform 阻塞轮询卡死执行器
      geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
        target_frame_, cam_frame_, tf2::TimePointZero);
      tf2::doTransform(person_cam, person_world, tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF 变换失败, 丢弃本帧: %s", e.what());
      return;
    }

    person_world.header.stamp = this->now();
    person_world.pose.orientation.w = 1.0;
    person_world.pose.orientation.x = 0.0;
    person_world.pose.orientation.y = 0.0;
    person_world.pose.orientation.z = 0.0;
    person_pub_->publish(person_world);

    visualization_msgs::msg::Marker marker;
    marker.header = person_world.header;
    marker.ns = "cos_rsvisual";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = person_world.pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.15;
    marker.color.g = 1.0;
    marker.color.a = 0.9;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_pub_->publish(marker);
  }

  // 参数
  std::string color_topic_, depth_topic_, camera_info_topic_, target_frame_, person_topic_;
  bool enable_;
  int detect_every_n_frames_, min_neighbors_;
  double scale_factor_, min_range_, max_range_;

  cv::HOGDescriptor hog_;
  image_geometry::PinholeCameraModel cam_model_;
  std::string cam_frame_;
  bool cam_model_initialized_ = false;

  cv::Mat latest_depth_;
  bool has_depth_ = false;
  long frame_count_ = 0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr person_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace cos_rsvisual

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cos_rsvisual::PersonVisualDetector)
