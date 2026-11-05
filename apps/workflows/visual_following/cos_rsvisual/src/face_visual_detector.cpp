// cos_rsvisual: 人脸检测组件(真机 RealSense 彩色图 + 对齐深度图)
//
// 思路: 订阅彩色图 + (已对齐彩色的)深度图 + 彩色相机内参, 用 OpenCV Haar
// 级联分类器检测正脸(可选侧面级联兜底), 命中最大人脸框后取框中心区域深度
// 中值反投影为 3D 点, 变换到 target_frame 发 /face_pose, 供 face_follower
// (ArmFollower 的另一实例) 让末端相机对准人脸。
//
// 与 person_visual_detector 的差异:
//   - 检测目标是"脸"而非"全身", 用 Haar(CPU 实时性远好于 HOG);
//   - 深度图要求与彩色图像素对齐(realsense align_depth), 人脸框小,
//     非对齐深度在框边缘的错位会直接毁掉取距;
//   - 多发布一个 /face_debug_image 调试图(画框 + 距离标注), 便于真机调参。
//
// 仿真不适用: gazebo actor 的纹理不满足 Haar 特征, 本组件面向实相机。

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

class FaceVisualDetector : public rclcpp::Node
{
public:
  explicit FaceVisualDetector(const rclcpp::NodeOptions & options)
  : Node("face_visual_detector", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- 参数(与 config/face_follow.yaml 对应) ----
    color_topic_ = declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
    // 深度必须对齐到彩色(realsense: align_depth.enable:=true),
    // 话题 /camera/aligned_depth_to_color/image_raw (16UC1)
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera/aligned_depth_to_color/image_raw");
    camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    cascade_path_ = declare_parameter<std::string>(
      "cascade_path", "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");
    // 侧面级联兜底(正脸检测不到时尝试, 原图 + 水平镜像各跑一次), 置空禁用
    profile_cascade_path_ = declare_parameter<std::string>("profile_cascade_path", "");
    enable_ = declare_parameter<bool>("enable", true);
    detect_every_n_frames_ = declare_parameter<int>("detect_every_n_frames", 3);
    scale_factor_ = declare_parameter<double>("scale_factor", 1.1);
    min_neighbors_ = declare_parameter<int>("min_neighbors", 5);
    min_face_size_ = declare_parameter<int>("min_face_size", 48);
    min_range_ = declare_parameter<double>("min_range", 0.2);
    max_range_ = declare_parameter<double>("max_range", 2.5);
    target_frame_ = declare_parameter<std::string>("target_frame", "world");
    face_topic_ = declare_parameter<std::string>("face_topic", "/face_pose");
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
    debug_image_topic_ =
      declare_parameter<std::string>("debug_image_topic", "/face_debug_image");
    // 跳变门控: 与上一帧发布位置距离超过 jump_threshold(m) 的候选先挂起,
    // 连续 jump_confirm_frames 帧落在同一新位置才接受(实测背景纹理有
    // 固定坐标的单帧误检, 不过滤会导致镜头突然甩向背景)
    jump_threshold_ = declare_parameter<double>("jump_threshold", 0.5);
    jump_confirm_frames_ = declare_parameter<int>("jump_confirm_frames", 3);

    if (!face_cascade_.load(cascade_path_)) {
      RCLCPP_ERROR(
        get_logger(), "Haar 级联加载失败: %s (检查 cascade_path, 需要 libopencv-data)",
        cascade_path_.c_str());
    }
    if (!profile_cascade_path_.empty() && !profile_cascade_.load(profile_cascade_path_)) {
      RCLCPP_WARN(
        get_logger(), "侧面级联加载失败, 忽略: %s", profile_cascade_path_.c_str());
    }

    face_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(face_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/face_marker", 10);
    if (publish_debug_image_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 1);
    }

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (cam_model_initialized_) {return;}
        cam_model_.fromCameraInfo(msg);
        cam_frame_ = msg->header.frame_id;
        cam_model_initialized_ = true;
        RCLCPP_INFO(get_logger(), "彩色相机内参已就绪, frame=%s", cam_frame_.c_str());
      });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FaceVisualDetector::depthCallback, this, std::placeholders::_1));

    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      color_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FaceVisualDetector::colorCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "face_visual_detector 已启动, enable=%s, 订阅 %s + %s",
      enable_ ? "true" : "false", color_topic_.c_str(), depth_topic_.c_str());
  }

private:
  // 缓存最新深度图(统一转 32FC1 米制), 供检测命中后取深度用
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

  // 在灰度图上跑 Haar: 先正脸, 失败再侧面(原图 + 水平镜像, 镜像结果映射回原坐标)
  std::vector<cv::Rect> detectFaces(const cv::Mat & gray)
  {
    std::vector<cv::Rect> faces;
    if (!face_cascade_.empty()) {
      face_cascade_.detectMultiScale(
        gray, faces, scale_factor_, min_neighbors_,
        cv::CASCADE_SCALE_IMAGE, cv::Size(min_face_size_, min_face_size_));
    }
    if (!faces.empty() || profile_cascade_.empty()) {return faces;}

    profile_cascade_.detectMultiScale(
      gray, faces, scale_factor_, min_neighbors_,
      cv::CASCADE_SCALE_IMAGE, cv::Size(min_face_size_, min_face_size_));
    if (!faces.empty()) {return faces;}

    cv::Mat flipped;
    cv::flip(gray, flipped, 1);
    std::vector<cv::Rect> mirror_faces;
    profile_cascade_.detectMultiScale(
      flipped, mirror_faces, scale_factor_, min_neighbors_,
      cv::CASCADE_SCALE_IMAGE, cv::Size(min_face_size_, min_face_size_));
    for (auto & r : mirror_faces) {
      r.x = gray.cols - r.x - r.width;
      faces.push_back(r);
    }
    return faces;
  }

  void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!enable_) {return;}
    if (!cam_model_initialized_ || !has_depth_) {return;}
    if (face_cascade_.empty()) {return;}

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

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);

    const std::vector<cv::Rect> faces = detectFaces(gray);
    if (faces.empty()) {
      publishDebugImage(cv_ptr, faces, cv::Rect(), 0.0);
      return;
    }

    // 逐个框做深度门控, 在"框内有足够有效深度"的候选里取最大的一个。
    // 不能只看框面积: 实测 Haar 会在背景纹理上打出大误检框(7m 外),
    // 选最大框会把真脸(较小)盖掉, 且误检框深度超量程导致整帧被丢弃。
    cv::Rect best;
    float depth = 0.0f;
    for (const auto & r : faces) {
      // 框中心 50% 区域内的深度中值(边缘容易混入背景/头发)
      cv::Rect inner(r.x + r.width / 4, r.y + r.height / 4, r.width / 2, r.height / 2);
      inner &= cv::Rect(0, 0, latest_depth_.cols, latest_depth_.rows);
      if (inner.empty()) {continue;}

      std::vector<float> depths;
      depths.reserve(inner.area());
      for (int v = inner.y; v < inner.y + inner.height; ++v) {
        const float * dm = latest_depth_.ptr<float>(v);
        for (int u = inner.x; u < inner.x + inner.width; ++u) {
          const float d = dm[u];
          if (d >= min_range_ && d <= max_range_) {depths.push_back(d);}
        }
      }
      if (depths.size() < 20) {continue;}  // 有效深度太少(空洞/超量程), 不可信
      if (r.area() <= best.area()) {continue;}
      std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
      best = r;
      depth = depths[depths.size() / 2];
    }
    if (best.area() == 0) {  // 所有人脸框深度都不可信
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "检测到 %zu 个人脸框但有效深度都不足, 跳过", faces.size());
      publishDebugImage(cv_ptr, faces, cv::Rect(), 0.0);
      return;
    }

    // ---- 反投影 + TF 变换 ----
    const double u = best.x + best.width / 2.0;
    const double v = best.y + best.height / 2.0;
    const cv::Point3d ray = cam_model_.projectPixelTo3dRay(cv::Point2d(u, v));

    geometry_msgs::msg::PoseStamped face_cam;
    face_cam.header.stamp = msg->header.stamp;
    face_cam.header.frame_id = cam_frame_;
    face_cam.pose.position.x = ray.x * depth;
    face_cam.pose.position.y = ray.y * depth;
    face_cam.pose.position.z = ray.z * depth;
    face_cam.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped face_world;
    try {
      // TimePointZero 零等待, 避免 foxy canTransform 阻塞轮询卡死执行器
      const geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
        target_frame_, cam_frame_, tf2::TimePointZero);
      tf2::doTransform(face_cam, face_world, tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF 变换失败, 丢弃本帧: %s", e.what());
      return;
    }

    face_world.header.stamp = this->now();
    face_world.pose.orientation.w = 1.0;
    face_world.pose.orientation.x = 0.0;
    face_world.pose.orientation.y = 0.0;
    face_world.pose.orientation.z = 0.0;

    // ---- 跳变门控: 实测背景纹理偶发单帧误检且深度有效(固定坐标),
    // 与上一帧发布位置距离超过 jump_threshold 的候选先挂起,
    // 连续 jump_confirm_frames 帧都落在同一新位置才接受(防止真走动被锁死) ----
    if (has_last_pub_) {
      const double dx = face_world.pose.position.x - last_pub_.x;
      const double dy = face_world.pose.position.y - last_pub_.y;
      const double dz = face_world.pose.position.z - last_pub_.z;
      const double jump = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (jump > jump_threshold_) {
        if (has_pending_ &&
          std::abs(face_world.pose.position.x - pending_.x) < 0.2 &&
          std::abs(face_world.pose.position.y - pending_.y) < 0.2 &&
          std::abs(face_world.pose.position.z - pending_.z) < 0.2)
        {
          ++pending_count_;
        } else {
          pending_.x = face_world.pose.position.x;
          pending_.y = face_world.pose.position.y;
          pending_.z = face_world.pose.position.z;
          pending_count_ = 1;
          has_pending_ = true;
        }
        if (pending_count_ < jump_confirm_frames_) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "跳变 %.2fm 超阈值, 挂起候选(%d/%d)", jump, pending_count_, jump_confirm_frames_);
          return;
        }
        has_pending_ = false;  // 确认是真移动, 放行
      } else {
        has_pending_ = false;
      }
    }
    last_pub_.x = face_world.pose.position.x;
    last_pub_.y = face_world.pose.position.y;
    last_pub_.z = face_world.pose.position.z;
    has_last_pub_ = true;

    face_pub_->publish(face_world);

    visualization_msgs::msg::Marker marker;
    marker.header = face_world.header;
    marker.ns = "cos_rsvisual";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = face_world.pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.08;
    marker.color.r = 1.0;
    marker.color.g = 0.6;
    marker.color.a = 0.9;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_pub_->publish(marker);

    publishDebugImage(cv_ptr, faces, best, depth);
  }

  void publishDebugImage(
    const cv_bridge::CvImageConstPtr & cv_ptr, const std::vector<cv::Rect> & faces,
    const cv::Rect & best, double depth)
  {
    if (!debug_pub_ || debug_pub_->get_subscription_count() == 0) {return;}
    cv::Mat img = cv_ptr->image.clone();
    for (const auto & r : faces) {
      cv::rectangle(img, r, cv::Scalar(0, 255, 255), 1);
    }
    if (best.area() > 0) {
      cv::rectangle(img, best, cv::Scalar(0, 255, 0), 2);
      cv::drawMarker(
        img, cv::Point(best.x + best.width / 2, best.y + best.height / 2),
        cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);
      cv::putText(
        img, cv::format("face %.2fm", depth),
        cv::Point(best.x, std::max(15, best.y - 5)),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    cv_bridge::CvImage out(cv_ptr->header, sensor_msgs::image_encodings::BGR8, img);
    debug_pub_->publish(*out.toImageMsg());
  }

  // 参数
  std::string color_topic_, depth_topic_, camera_info_topic_;
  std::string cascade_path_, profile_cascade_path_;
  std::string target_frame_, face_topic_, debug_image_topic_;
  bool enable_, publish_debug_image_;
  int detect_every_n_frames_, min_neighbors_, min_face_size_, jump_confirm_frames_;
  double scale_factor_, min_range_, max_range_, jump_threshold_;

  cv::CascadeClassifier face_cascade_, profile_cascade_;
  image_geometry::PinholeCameraModel cam_model_;
  std::string cam_frame_;
  bool cam_model_initialized_ = false;

  cv::Mat latest_depth_;
  bool has_depth_ = false;
  long frame_count_ = 0;

  // 跳变门控状态
  struct Pt { double x = 0.0, y = 0.0, z = 0.0; };
  Pt last_pub_, pending_;
  bool has_last_pub_ = false, has_pending_ = false;
  int pending_count_ = 0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr face_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

}  // namespace cos_rsvisual

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cos_rsvisual::FaceVisualDetector)
