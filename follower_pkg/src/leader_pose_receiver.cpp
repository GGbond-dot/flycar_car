// leader_pose_receiver — 传输层适配节点(M1 实现:UDP)
//
// 职责(见 docs/follow_fly_car_design.md §4.1):
//   1. 非阻塞 UDP 收飞车位姿包(fly_car 侧 pose_sender_pkg 发送)
//   2. 用静态变换 T_carmap←flymap(align_dx/dy/dyaw)把位姿从飞车 map 系转到车 map 系
//   3. 发布 /leader_pose(PoseStamped, frame_id=map)—— 下游唯一稳定接口
//
// 包格式(小端,定长 24 字节,与 fly_car/src/pose_sender_pkg/src/pose_sender.cpp 保持一致,
// 改一处必须同步改另一处):
//   [magic u16 = 0xFC01][seq u16][stamp_ms u32][x_m f32][y_m f32][yaw_rad f32][reserved f32]
//
// 若后续改用 domain_bridge,只需把本节点的 UDP 轮询换成话题订阅,/leader_pose 接口不变。

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
constexpr uint16_t kMagic = 0xFC01;

struct __attribute__((packed)) PosePacket
{
  uint16_t magic;
  uint16_t seq;
  uint32_t stamp_ms;
  float x_m;
  float y_m;
  float yaw_rad;
  float reserved;
};
static_assert(sizeof(PosePacket) == 24, "PosePacket must be 24 bytes");
}  // namespace

class LeaderPoseReceiver : public rclcpp::Node
{
public:
  LeaderPoseReceiver()
  : Node("leader_pose_receiver")
  {
    declare_parameter<int>("udp_port", 8888);
    declare_parameter<double>("align_dx", 0.0);    // m,飞车 map 原点在车 map 系下的 x
    declare_parameter<double>("align_dy", 0.0);    // m
    declare_parameter<double>("align_dyaw", 0.0);  // rad

    udp_port_ = get_parameter("udp_port").as_int();
    align_dx_ = get_parameter("align_dx").as_double();
    align_dy_ = get_parameter("align_dy").as_double();
    align_dyaw_ = get_parameter("align_dyaw").as_double();

    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
      throw std::runtime_error("leader_pose_receiver: failed to create UDP socket");
    }
    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(static_cast<uint16_t>(udp_port_));
    if (::bind(sock_fd_, reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr)) < 0) {
      ::close(sock_fd_);
      throw std::runtime_error(
        "leader_pose_receiver: failed to bind UDP port " + std::to_string(udp_port_));
    }
    const int flags = ::fcntl(sock_fd_, F_GETFL, 0);
    ::fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

    leader_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/leader_pose", rclcpp::QoS(10));

    // 50Hz 轮询:每次把已到达的包全部读完,只发布最新一个有效包(发送端 20Hz)
    poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&LeaderPoseReceiver::pollSocket, this));

    RCLCPP_INFO(get_logger(),
      "leader_pose_receiver: UDP :%d, align dx=%.3f dy=%.3f dyaw=%.3f",
      udp_port_, align_dx_, align_dy_, align_dyaw_);
  }

  ~LeaderPoseReceiver() override
  {
    if (sock_fd_ >= 0) {
      ::close(sock_fd_);
    }
  }

private:
  void pollSocket()
  {
    PosePacket pkt;
    bool got_new = false;
    PosePacket latest{};

    while (true) {
      const ssize_t n = ::recv(sock_fd_, &pkt, sizeof(pkt), 0);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;  // 缓冲区读空
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "leader_pose_receiver: recv failed: %s", std::strerror(errno));
        break;
      }
      if (n != static_cast<ssize_t>(sizeof(pkt)) || pkt.magic != kMagic) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "leader_pose_receiver: dropped malformed packet (%zd bytes)", n);
        continue;
      }
      // seq 用 int16 差值判新旧,自然处理 65535→0 回绕;乱序旧包丢弃
      const int16_t diff = static_cast<int16_t>(pkt.seq - last_seq_);
      if (has_received_ && diff <= 0) {
        continue;
      }
      last_seq_ = pkt.seq;
      has_received_ = true;
      latest = pkt;
      got_new = true;
    }

    if (got_new) {
      publishLeaderPose(latest.x_m, latest.y_m, latest.yaw_rad);
    }
  }

  // 输入:飞车在飞车 map 系的位姿(m/rad);输出:转到车 map 系后发布 /leader_pose
  void publishLeaderPose(double x_fly, double y_fly, double yaw_fly)
  {
    const double cos_a = std::cos(align_dyaw_);
    const double sin_a = std::sin(align_dyaw_);

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();  // 用收包本地时间,不做跨机时钟同步
    msg.header.frame_id = "map";
    msg.pose.position.x = align_dx_ + cos_a * x_fly - sin_a * y_fly;
    msg.pose.position.y = align_dy_ + sin_a * x_fly + cos_a * y_fly;
    msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_fly + align_dyaw_);
    msg.pose.orientation = tf2::toMsg(q);

    leader_pose_pub_->publish(msg);
  }

  int udp_port_;
  double align_dx_;
  double align_dy_;
  double align_dyaw_;
  int sock_fd_{-1};
  uint16_t last_seq_{0};
  bool has_received_{false};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pose_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderPoseReceiver>());
  rclcpp::shutdown();
  return 0;
}
