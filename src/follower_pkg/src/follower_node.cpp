// follower_node — 面包屑轨迹跟随(M2 实现)
//
// 职责(见 docs/follow_fly_car_design.md §4.2):
//   输入  /leader_pose(飞车位姿,车 map 系)、/follow_enable(使能)、TF map←laser_link(自身位姿)
//   输出  /target_position(Float32MultiArray [x_cm, y_cm, 0, yaw_deg],定频,移动靶,
//         与 fly_car 的单位约定一致;由 diff_drive_controller 转成差速 v/w)
//
// 面包屑逻辑:
//   - leader 每移动 ≥ breadcrumb_spacing 记一个轨迹点;
//   - 跟随目标 = 沿 [面包屑链 + leader 当前位置] 从 leader 端向回走弧长 d_follow 处(段内插值);
//   - 轨迹总弧长 < d_follow 时不发布目标(原地保持,等 leader 走远);
//   - 目标 yaw = 指向 leader 当前位置(为对接装货保持车头朝向)。
// 安全:
//   - leader 超时 → 目标点 = 当前自身位姿(原地刹停);
//   - 与 leader 直线距离 < d_min → 同上,但 yaw 仍指向 leader。

#include <cmath>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct Pose2D
{
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
};

namespace
{
double dist2D(const Pose2D & a, const Pose2D & b)
{
  return std::hypot(a.x_m - b.x_m, a.y_m - b.y_m);
}
}  // namespace

class FollowerNode : public rclcpp::Node
{
public:
  FollowerNode()
  : Node("follower_node"),
    state_(State::IDLE)
  {
    declare_parameter<double>("d_follow_cm", 80.0);           // 跟随弧长距离
    declare_parameter<double>("d_min_cm", 40.0);              // 最小安全直线距离
    declare_parameter<double>("breadcrumb_spacing_cm", 10.0); // 面包屑间距
    declare_parameter<double>("prune_margin_cm", 50.0);       // 弧长超出 d_follow+margin 的旧点丢弃
    declare_parameter<double>("leader_timeout_s", 0.5);       // 位姿超时判定
    declare_parameter<double>("publish_rate_hz", 20.0);       // 目标点发布频率

    d_follow_m_ = get_parameter("d_follow_cm").as_double() / 100.0;
    d_min_m_ = get_parameter("d_min_cm").as_double() / 100.0;
    spacing_m_ = get_parameter("breadcrumb_spacing_cm").as_double() / 100.0;
    prune_margin_m_ = get_parameter("prune_margin_cm").as_double() / 100.0;
    leader_timeout_s_ = get_parameter("leader_timeout_s").as_double();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    leader_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/leader_pose", rclcpp::QoS(10),
      std::bind(&FollowerNode::leaderPoseCallback, this, std::placeholders::_1));

    follow_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/follow_enable", rclcpp::QoS(10),
      std::bind(&FollowerNode::followEnableCallback, this, std::placeholders::_1));

    target_position_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/target_position", rclcpp::QoS(10));

    const double period_sec = 1.0 / std::max(publish_rate_hz_, 1.0);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
      std::bind(&FollowerNode::controlTimerCallback, this));

    RCLCPP_INFO(get_logger(),
      "follower_node up (d_follow=%.2fm d_min=%.2fm spacing=%.2fm timeout=%.1fs %.0fHz)",
      d_follow_m_, d_min_m_, spacing_m_, leader_timeout_s_, publish_rate_hz_);
  }

private:
  // 状态机:IDLE → FOLLOW;DOCK/LOAD 为 M4 预留
  enum class State { IDLE, FOLLOW };

  void leaderPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    leader_pose_.x_m = msg->pose.position.x;
    leader_pose_.y_m = msg->pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    leader_pose_.yaw_rad = yaw;

    last_leader_time_ = now();
    has_leader_pose_ = true;

    // 面包屑入队:leader 相对队尾移动够远才记一个点
    if (breadcrumbs_.empty() || dist2D(breadcrumbs_.back(), leader_pose_) >= spacing_m_) {
      breadcrumbs_.push_back(leader_pose_);
    }
    pruneBreadcrumbs();
  }

  // 丢弃离 leader 弧长超过 d_follow + prune_margin 的旧点,保持队列有界。
  // 队首段以外的弧长足够时,队首点已经不可能成为跟随目标。
  void pruneBreadcrumbs()
  {
    while (breadcrumbs_.size() >= 2) {
      double arc_without_front = 0.0;
      for (size_t i = 2; i < breadcrumbs_.size(); ++i) {
        arc_without_front += dist2D(breadcrumbs_[i - 1], breadcrumbs_[i]);
      }
      arc_without_front += dist2D(breadcrumbs_.back(), leader_pose_);
      if (arc_without_front >= d_follow_m_ + prune_margin_m_) {
        breadcrumbs_.pop_front();
      } else {
        break;
      }
    }
  }

  void followEnableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && state_ == State::IDLE) {
      state_ = State::FOLLOW;
      RCLCPP_INFO(get_logger(), "FOLLOW enabled");
    } else if (!msg->data && state_ == State::FOLLOW) {
      state_ = State::IDLE;
      breadcrumbs_.clear();
      RCLCPP_INFO(get_logger(), "FOLLOW disabled -> IDLE");
    }
  }

  // 自身位姿:TF map←laser_link(与 fly_car 取位姿方式一致)
  bool getCurrentPose(Pose2D & pose)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform("map", "laser_link", tf2::TimePointZero);
      pose.x_m = tf.transform.translation.x;
      pose.y_m = tf.transform.translation.y;

      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      pose.yaw_rad = yaw;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF map->laser_link unavailable: %s", ex.what());
      return false;
    }
  }

  // 沿 [面包屑链 + leader 当前位置] 从 leader 端向回走 d_follow 弧长取目标点。
  // 轨迹总弧长不足时返回 false(leader 还没走出足够远,原地保持)。
  bool computeTrailTarget(Pose2D & target) const
  {
    double remaining = d_follow_m_;
    Pose2D cursor = leader_pose_;

    for (auto it = breadcrumbs_.rbegin(); it != breadcrumbs_.rend(); ++it) {
      const double seg = dist2D(*it, cursor);
      if (seg >= remaining && seg > 1e-6) {
        const double t = remaining / seg;  // 从 cursor 向 *it 方向回退的比例
        target.x_m = cursor.x_m + (it->x_m - cursor.x_m) * t;
        target.y_m = cursor.y_m + (it->y_m - cursor.y_m) * t;
        return true;
      }
      remaining -= seg;
      cursor = *it;
    }
    return false;
  }

  void controlTimerCallback()
  {
    if (state_ != State::FOLLOW) {
      return;
    }

    Pose2D self_pose;
    if (!getCurrentPose(self_pose) || !has_leader_pose_) {
      return;  // 无自身 TF 或还没收到 leader → 不发布目标(控制器端无目标即不动)
    }

    // 安全 1:leader 位姿超时 → 目标点 = 当前自身位姿,原地刹停
    const double age_s = (now() - last_leader_time_).seconds();
    if (age_s > leader_timeout_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "leader pose stale (%.2fs) -> hold position", age_s);
      publishTarget(self_pose);
      return;
    }

    // 安全 2:离 leader 太近 → 原地保持,车头仍指向 leader,等它走远
    const double d_leader = dist2D(self_pose, leader_pose_);
    if (d_leader < d_min_m_) {
      Pose2D hold = self_pose;
      hold.yaw_rad = std::atan2(
        leader_pose_.y_m - self_pose.y_m, leader_pose_.x_m - self_pose.x_m);
      publishTarget(hold);
      return;
    }

    // 正常跟随:面包屑轨迹上取移动靶
    Pose2D target;
    if (!computeTrailTarget(target)) {
      return;  // 轨迹弧长 < d_follow:leader 没怎么动,保持不动(不发布)
    }
    target.yaw_rad = std::atan2(
      leader_pose_.y_m - target.y_m, leader_pose_.x_m - target.x_m);
    publishTarget(target);
  }

  // 发布移动靶,单位换算:内部 m/rad → 对外约定的 cm/deg
  void publishTarget(const Pose2D & target)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(4);
    msg.data[0] = static_cast<float>(target.x_m * 100.0);
    msg.data[1] = static_cast<float>(target.y_m * 100.0);
    msg.data[2] = 0.0f;
    msg.data[3] = static_cast<float>(target.yaw_rad * 180.0 / M_PI);
    target_position_pub_->publish(msg);
  }

  // 参数(内部统一用 m/rad)
  double d_follow_m_;
  double d_min_m_;
  double spacing_m_;
  double prune_margin_m_;
  double leader_timeout_s_;
  double publish_rate_hz_;

  // 状态
  State state_;
  Pose2D leader_pose_;
  bool has_leader_pose_ = false;
  rclcpp::Time last_leader_time_;
  std::deque<Pose2D> breadcrumbs_;

  // ROS 接口
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follow_enable_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_position_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerNode>());
  rclcpp::shutdown();
  return 0;
}
