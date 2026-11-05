// cos_rsvisual: 深度图人体检测组件
//
// 思路(简洁优先): 订阅深度图 + camera_info, 在 [min_range, max_range] 距离带
// 内建掩码, 形态学开运算去噪, 取最大连通域(像素数 >= min_cluster_pixels),
// 用连通域质心像素 + 该区域深度中值, 经 image_geometry 反投影得到相机系 3D 点,
// 再用 tf2 变换到 target_frame 后发布 /person_pose。
//
// 深度图编码兼容: 仿真 gazebo_ros_camera 发 32FC1(米), 真机 realsense 发
// 16UC1(毫米), 两种都支持。

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

class PersonDepthDetector : public rclcpp::Node
{
public:
  explicit PersonDepthDetector(const rclcpp::NodeOptions & options)
  : Node("person_depth_detector", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- 参数(与 config/follower.yaml 对应) ----
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/camera/depth/camera_info");
    min_range_ = declare_parameter<double>("min_range", 0.3);
    max_range_ = declare_parameter<double>("max_range", 2.0);
    min_cluster_pixels_ = declare_parameter<int>("min_cluster_pixels", 200);
    use_roi_ = declare_parameter<bool>("use_roi", false);
    roi_x_ = declare_parameter<int>("roi_x", 0);
    roi_y_ = declare_parameter<int>("roi_y", 0);
    roi_width_ = declare_parameter<int>("roi_width", 640);
    roi_height_ = declare_parameter<int>("roi_height", 480);
    target_frame_ = declare_parameter<std::string>("target_frame", "world");
    person_topic_ = declare_parameter<std::string>("person_topic", "/person_pose");
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);

    person_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(person_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/person_marker", 10);

    // camera_info 只需取一次内参
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (cam_model_initialized_) {return;}
        cam_model_.fromCameraInfo(msg);
        cam_frame_ = msg->header.frame_id;
        cam_model_initialized_ = true;
        RCLCPP_INFO(
          get_logger(), "camera_info 已获取: frame=%s, fx=%.1f",
          cam_frame_.c_str(), cam_model_.fx());
      });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PersonDepthDetector::depthCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "person_depth_detector 已启动, 订阅 %s", depth_topic_.c_str());
  }

private:
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!cam_model_initialized_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "等待 camera_info (%s)...", camera_info_topic_.c_str());
      return;
    }

    // 处理频率限制
    const rclcpp::Time now = this->now();
    if (publish_rate_ > 0.0 &&
      (now - last_pub_time_).seconds() < 1.0 / publish_rate_)
    {
      return;
    }

    // ---- 深度图转 OpenCV, 统一成 32FC1 米制 ----
    cv::Mat depth_m;
    try {
      if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
          msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptr->image.convertTo(depth_m, CV_32FC1, 0.001);  // 毫米 -> 米
      } else {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
          msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_m = cv_ptr->image;  // 已是米; inf/nan 会在距离带判断时被排除
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "cv_bridge 转换失败: %s", e.what());
      return;
    }

    // ---- 距离带掩码 ----
    cv::Mat mask = (depth_m >= min_range_) & (depth_m <= max_range_);

    // 可选 ROI: ROI 外清零
    if (use_roi_) {
      cv::Mat roi_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
      cv::Rect roi(roi_x_, roi_y_, roi_width_, roi_height_);
      roi &= cv::Rect(0, 0, mask.cols, mask.rows);
      roi_mask(roi) = 255;
      cv::bitwise_and(mask, roi_mask, mask);
    }

    // 形态学开运算去噪
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    // ---- 连通域: 取面积最大的前景区域 ----
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids);
    int best_label = -1;
    int best_area = 0;
    for (int i = 1; i < n; ++i) {
      int area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area > best_area) {
        best_area = area;
        best_label = i;
      }
    }
    // 诊断日志(2s 节流): 距离带内像素数与最大连通域面积
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "距离带内像素 %d, 最大连通域 %d px", cv::countNonZero(mask), best_area);

    if (best_label < 0 || best_area < min_cluster_pixels_) {
      return;  // 本帧没有有效目标, 静默跳过
    }

    // 连通域像素深度中值
    std::vector<float> depths;
    depths.reserve(best_area);
    for (int v = 0; v < depth_m.rows; ++v) {
      const int * lb = labels.ptr<int>(v);
      const float * dm = depth_m.ptr<float>(v);
      for (int u = 0; u < depth_m.cols; ++u) {
        if (lb[u] == best_label) {depths.push_back(dm[u]);}
      }
    }
    if (depths.empty()) {return;}
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    const float depth = depths[depths.size() / 2];

    // ---- 反投影: 像素 -> 相机系 3D 点 ----
    const double u = centroids.at<double>(best_label, 0);
    const double v = centroids.at<double>(best_label, 1);
    cv::Point3d ray = cam_model_.projectPixelTo3dRay(cv::Point2d(u, v));  // 单位射线
    // 射线 z 分量归一为 1, 乘深度即得相机系坐标
    const double px = ray.x * depth;
    const double py = ray.y * depth;
    const double pz = ray.z * depth;

    geometry_msgs::msg::PoseStamped person_cam;
    person_cam.header.stamp = msg->header.stamp;
    person_cam.header.frame_id = cam_frame_;
    person_cam.pose.position.x = px;
    person_cam.pose.position.y = py;
    person_cam.pose.position.z = pz;
    person_cam.pose.orientation.w = 1.0;  // 朝向无意义, 用恒等四元数

    // ---- 变换到 target_frame ----
    // 用 TimePointZero 取最新可用变换, 且不做带超时的阻塞等待:
    // 带超时的 canTransform 轮询循环在 foxy 中依赖节点时钟推进,
    // 时钟域不一致(仿真/系统时间)时会把执行器永久卡死(已用 gdb 实证)。
    geometry_msgs::msg::PoseStamped person_world;
    try {
      geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
        target_frame_, cam_frame_, tf2::TimePointZero);
      tf2::doTransform(person_cam, person_world, tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF 变换失败, 丢弃本帧: %s", e.what());
      return;
    }

    person_world.header.stamp = now;
    person_world.pose.orientation.w = 1.0;
    person_world.pose.orientation.x = 0.0;
    person_world.pose.orientation.y = 0.0;
    person_world.pose.orientation.z = 0.0;
    person_pub_->publish(person_world);
    last_pub_time_ = now;

    // rviz 可视化: 红色小球
    visualization_msgs::msg::Marker marker;
    marker.header = person_world.header;
    marker.ns = "cos_rsvisual";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = person_world.pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.15;
    marker.color.r = 1.0;
    marker.color.a = 0.9;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_pub_->publish(marker);
  }

  // 参数
  std::string depth_topic_, camera_info_topic_, target_frame_, person_topic_;
  double min_range_, max_range_, publish_rate_;
  int min_cluster_pixels_;
  bool use_roi_;
  int roi_x_, roi_y_, roi_width_, roi_height_;

  // 相机模型
  image_geometry::PinholeCameraModel cam_model_;
  std::string cam_frame_;
  bool cam_model_initialized_ = false;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Time last_pub_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr person_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace cos_rsvisual

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cos_rsvisual::PersonDepthDetector)
