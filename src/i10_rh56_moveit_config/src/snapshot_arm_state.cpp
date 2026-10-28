// 只读检查 MoveIt 是否接收到完整六轴状态，可导出一次带时间与来源的快照。
#include "scene_loader.hpp"
#include "current_arm_state.hpp"
#include <nlohmann/json.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 1;
  auto node = std::make_shared<rclcpp::Node>("snapshot_arm_state");
  try {
    const auto dir = node->declare_parameter<std::string>("config_dir", "");
    const auto topic = node->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const auto output = node->declare_parameter<std::string>("output_path", "");
    const auto timeout = node->declare_parameter<double>("state_timeout_s", 3.0);
    const auto max_age = node->declare_parameter<double>("state_max_age_s", .5);
    if (dir.empty()) throw std::runtime_error("必须提供 config_dir");
    if (node->get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("实测关节状态检查要求 use_sim_time=false");
    // 启动时先使旧快照失效，读取失败后不能误用旧结果。
    if (!output.empty()) {
      std::ofstream file(output);
      if (!file) throw std::runtime_error("无法打开快照输出路径");
      file << "{\"valid\":false,\"reason\":\"waiting_for_current_state\"}\n";
    }
    node->declare_parameter("robot_description", read(dir + "/robot.urdf"));
    node->declare_parameter("robot_description_semantic", read(dir + "/robot.srdf"));
    robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
    const auto model = loader.getModel();
    if (!model) throw std::runtime_error("无法加载机器人模型");
    const auto sample = readCurrentArmState(node, model, topic, timeout, max_age);
    const nlohmann::json result{
        {"valid", true}, {"source", "joint_states"}, {"topic", topic},
        {"timestamp_clock", "host_ros_system_receive_time"},
        {"stamp_ns", sample.stamp.nanoseconds()}, {"arm_joints", sample.names},
        {"arm_q_rad", sample.positions}, {"hand_measured", false}};
    if (!output.empty()) {
      std::ofstream file(output);
      file.exceptions(std::ios::badbit | std::ios::failbit);
      file << result.dump(2) << '\n';
    }
    std::cout << result.dump() << std::endl;
    code = 0;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
  }
  rclcpp::shutdown();
  return code;
}
