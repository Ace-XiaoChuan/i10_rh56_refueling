// RTDE 只订阅服务器输出；回放模式用于离线验证，使用独立话题。
#include "aubo_joint_state_bridge/joint_sample.hpp"
#include <aubo_sdk/rtde.h>
#include <arpa/inet.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>

namespace {
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;
using aubo_joint_state_bridge::kJointNames;
using aubo_joint_state_bridge::validateSample;

class JointStateBridge : public rclcpp::Node {
 public:
  JointStateBridge() : Node("aubo_joint_state_bridge") {
    source_ = declare_parameter<std::string>("source", "rtde");
    host_ = declare_parameter<std::string>("host", "");
    prefix_ = declare_parameter<std::string>("rtde_prefix", "R1");
    user_ = declare_parameter<std::string>("user", "aubo");
    password_env_ = declare_parameter<std::string>("password_env", "AUBO_RPC_PASSWORD");
    port_ = declare_parameter<int>("rtde_port", 30010);
    rate_ = declare_parameter<double>("rate_hz", 50.0);
    startup_timeout_ = declare_parameter<double>("startup_timeout_s", 3.0);
    stale_timeout_ = declare_parameter<double>("stale_timeout_s", 0.5);
    replay_path_ = declare_parameter<std::string>("replay_path", "");
    const auto topic = declare_parameter<std::string>(
        "joint_states_topic", source_ == "replay" ? "/aubo/replay/joint_states" : "/joint_states");
    if (source_ != "rtde" && source_ != "replay")
      throw std::runtime_error("source 必须为 rtde 或 replay");
    if (get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("本节点使用主机收包时间，要求 use_sim_time=false");
    if (!std::isfinite(rate_) || rate_ < 1 || rate_ > 100 ||
        !std::isfinite(startup_timeout_) || startup_timeout_ <= 0 || startup_timeout_ > 60 ||
        !std::isfinite(stale_timeout_) || stale_timeout_ <= 0 || stale_timeout_ > 60)
      throw std::runtime_error("频率须为 1～100 Hz，超时须在 (0,60] 秒内");
    if (port_ < 1 || port_ > 65535 || prefix_.empty() || user_.empty())
      throw std::runtime_error("端口、RTDE 前缀或用户名无效");
    // Reliable/volatile 与 MoveIt CurrentStateMonitor 兼容；不保存历史状态给晚到订阅者。
    publisher_ = create_publisher<sensor_msgs::msg::JointState>(topic, rclcpp::QoS(10));
    if (source_ == "replay" && std::string(publisher_->get_topic_name()) == "/joint_states")
      throw std::runtime_error("历史回放必须使用独立话题，不能发布到 /joint_states");
    if (source_ == "rtde") {
      in_addr address{};
      if (inet_pton(AF_INET, host_.c_str(), &address) != 1)
        throw std::runtime_error("启动 RTDE 必须显式提供 host:=控制柜IPv4");
      if (!replay_path_.empty()) throw std::runtime_error("RTDE 模式不能提供 replay_path");
    } else if (!host_.empty() || replay_path_.empty()) {
      throw std::runtime_error("回放必须提供 replay_path，且不能提供 host");
    }
  }

  ~JointStateBridge() override {
    // 先停止 SDK 回调，再销毁其访问的互斥量和 ROS 发布器。
    if (rtde_) {
      try { if (channel_ >= 0) rtde_->removeTopic(false, channel_); } catch (...) {}
      try { if (rtde_->hasLogined()) rtde_->logout(); } catch (...) {}
      try { if (rtde_->hasConnected()) rtde_->disconnect(); } catch (...) {}
      rtde_.reset();
    }
  }

  void start() {
    RCLCPP_INFO(get_logger(), "关节状态话题：%s，来源：%s",
                publisher_->get_topic_name(), source_.c_str());
    if (source_ == "rtde") {
      rtde_ = std::make_unique<arcs::aubo_sdk::RtdeClient>();
      // SDK 请求日志可能含认证内容，只保留本节点的阶段日志。
      rtde_->setLogHandler([](int, const char*, int, const std::string&) {});
      if (rtde_->connect(host_, port_) != 0 || !rtde_->hasConnected())
        throw std::runtime_error("RTDE 连接失败");
      const char* password = std::getenv(password_env_.c_str());
      // 未设置环境变量时沿用本地 SDK 官方示例的默认认证。
      if (rtde_->login(user_, password ? password : "123456") != 0 || !rtde_->hasLogined())
        throw std::runtime_error("RTDE 认证失败");
      const auto menu = rtde_->getOutputMaps();
      const auto position_field = prefix_ + "_actual_q";
      const auto velocity_field = prefix_ + "_actual_qd";
      if (!menu.count(position_field)) throw std::runtime_error("RTDE 缺少关节位置输出字段");
      std::vector<std::string> fields{position_field};
      const bool with_velocity = menu.count(velocity_field) != 0;
      if (with_velocity) fields.push_back(velocity_field);
      channel_ = rtde_->setTopic(false, fields, rate_, 0);
      if (channel_ < 0) throw std::runtime_error("RTDE 输出订阅失败");
      if (rtde_->subscribe(channel_, [this, with_velocity](arcs::aubo_sdk::InputParser& parser) {
            try {
              auto position = parser.popVectorDouble();
              auto velocity = with_velocity ? parser.popVectorDouble() : std::vector<double>{};
              receive(position, velocity);
            } catch (...) {
              std::lock_guard<std::mutex> lock(mutex_);
              failure_ = "RTDE 帧解析或数值校验失败";
            }
          }) != 0) throw std::runtime_error("RTDE 回调注册失败");
      RCLCPP_INFO(get_logger(), "只读 RTDE 已订阅 %s，速度字段=%s，请求频率=%.1f Hz",
                  position_field.c_str(), with_velocity ? "有" : "无", rate_);
    } else {
      replay_.open(replay_path_);
      if (!replay_) throw std::runtime_error("无法读取回放文件");
      readNextReplay();
      RCLCPP_WARN(get_logger(), "历史回放：%s；消息时间是本次回放接收时间", replay_path_.c_str());
    }
    started_ = Clock::now();
    // 回放保留帧间隔，留少量启动时间供离线订阅者完成发现。
    replay_origin_ = started_ + std::chrono::milliseconds(300);
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { tick(); });
  }

 private:
  void receive(const std::vector<double>& position, const std::vector<double>& velocity) {
    validateSample(position, velocity);
    // 同时保存 ROS 接收时间和单调时钟；后者不受系统校时影响，用于断流检查。
    const auto received = Clock::now();
    const auto stamp = now();
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.name.assign(kJointNames.begin(), kJointNames.end());
    pending_.position = position;
    pending_.velocity = velocity;
    pending_.header.stamp = stamp;
    last_received_ = received;
    ++received_count_;
  }

  void readNextReplay() {
    std::string line;
    if (!std::getline(replay_, line)) {
      replay_done_ = true;
      return;
    }
    const auto record = Json::parse(line);
    if (!record.value("parse_ok", false)) throw std::runtime_error("回放包含解析失败的帧");
    next_position_ = record.at(prefix_ + "_actual_q").get<std::vector<double>>();
    next_velocity_ = record.contains(prefix_ + "_actual_qd")
        ? record.at(prefix_ + "_actual_qd").get<std::vector<double>>() : std::vector<double>{};
    validateSample(next_position_, next_velocity_);
    const double time = record.at("host_monotonic_s").get<double>();
    if (!std::isfinite(time) || (replay_count_ && time <= replay_last_time_))
      throw std::runtime_error("回放的主机单调时间必须有限且严格递增");
    if (!replay_count_) replay_first_time_ = time;
    next_offset_s_ = time - replay_first_time_;
    replay_last_time_ = time;
    ++replay_count_;
  }

  void tick() {
    const auto current = Clock::now();
    if (source_ == "replay" && !replay_done_ &&
        std::chrono::duration<double>(current - replay_origin_).count() >= next_offset_s_) {
      receive(next_position_, next_velocity_);
      readNextReplay();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!failure_.empty()) throw std::runtime_error(failure_);
    const double age = std::chrono::duration<double>(
        current - (received_count_ ? last_received_ : started_)).count();
    if (age > (received_count_ ? stale_timeout_ : startup_timeout_))
      throw std::runtime_error(received_count_ ? "关节状态断流，停止发布并退出" : "等待首个有效关节状态超时");
    if (received_count_ != published_count_) {
      // 每帧最多发布一次，绝不靠更新时间戳重复发布旧状态。
      publisher_->publish(pending_);
      if (!published_count_)
        RCLCPP_INFO(get_logger(), "已发布首个六轴有效帧，速度项=%zu，订阅者=%zu",
                    pending_.velocity.size(), publisher_->get_subscription_count());
      published_count_ = received_count_;
    }
  }

  std::string source_, host_, prefix_, user_, password_env_, replay_path_, failure_;
  int port_ = 30010, channel_ = -1;
  double rate_ = 50, startup_timeout_ = 3, stale_timeout_ = .5;
  std::mutex mutex_;
  sensor_msgs::msg::JointState pending_;
  std::size_t received_count_ = 0, published_count_ = 0, replay_count_ = 0;
  Clock::time_point started_, last_received_, replay_origin_;
  std::ifstream replay_;
  bool replay_done_ = false;
  double replay_first_time_ = 0, replay_last_time_ = 0, next_offset_s_ = 0;
  std::vector<double> next_position_, next_velocity_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<arcs::aubo_sdk::RtdeClient> rtde_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<JointStateBridge>();
    node->start();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("aubo_joint_state_bridge"), "%s", error.what());
    code = 1;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("aubo_joint_state_bridge"), "SDK 异常，状态桥退出");
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
