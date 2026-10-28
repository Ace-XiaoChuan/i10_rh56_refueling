#pragma once
// 使用 MoveIt 状态监视器读取六轴新状态；未测量的手指不参与完整性判断。
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

struct CurrentArmState {
  std::vector<std::string> names;
  std::vector<double> positions;
  rclcpp::Time stamp;
};

inline CurrentArmState readCurrentArmState(
    const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModelConstPtr& model,
    const std::string& topic, double timeout_s, double max_age_s) {
  if (node->get_parameter("use_sim_time").as_bool())
    throw std::runtime_error("实测状态起点要求 use_sim_time=false");
  if (!std::isfinite(timeout_s) || timeout_s <= 0 || timeout_s > 60 ||
      !std::isfinite(max_age_s) || max_age_s <= 0 || max_age_s > 60)
    throw std::runtime_error("状态等待和允许帧龄须在 (0,60] 秒内");
  const auto* arm = model->getJointModelGroup("arm");
  if (!arm || arm->getVariableCount() != 6)
    throw std::runtime_error("模型必须具有六轴 arm 组");
  // 独立节点只处理状态订阅，避免与规划器的 executor 竞争同一个节点。
  auto monitor_node = std::make_shared<rclcpp::Node>(
      "current_arm_state_reader", node->get_namespace(),
      rclcpp::NodeOptions().context(node->get_node_base_interface()->get_context())
                          .use_global_arguments(false));
  planning_scene_monitor::CurrentStateMonitor monitor(monitor_node, model, nullptr, false);
  monitor.setBoundsError(0.0);  // 不允许监视器悄悄修正实测角度。
  monitor.startStateMonitor(topic);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(monitor_node);
  const auto requested_at = monitor_node->now();
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
  const auto& names = arm->getActiveJointModelNames();
  while (rclcpp::ok(node->get_node_base_interface()->get_context()) &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::vector<std::string> missing;
    const auto oldest = monitor_node->now() - rclcpp::Duration::from_seconds(max_age_s);
    monitor.haveCompleteState(oldest, missing);
    const bool arm_complete = std::none_of(names.begin(), names.end(), [&](const auto& name) {
      return std::find(missing.begin(), missing.end(), name) != missing.end();
    });
    if (arm_complete) {
      auto [state, stamp] = monitor.getCurrentStateAndTime();
      // 等待订阅启动后的新帧，并拒绝未来时间；所有六轴还须通过上面的逐关节帧龄检查。
      if (stamp >= requested_at && stamp <= monitor_node->now() + rclcpp::Duration::from_seconds(.05)) {
        CurrentArmState result{names, {}, stamp};
        state->copyJointGroupPositions(arm, result.positions);
        if (!state->satisfiesBounds(arm, 0.0) ||
            !std::all_of(result.positions.begin(), result.positions.end(),
                         [](double value) { return std::isfinite(value); }))
          throw std::runtime_error("实测臂关节非有限或超出模型限位");
        return result;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw std::runtime_error("等待六轴完整且新鲜的关节状态超时；未使用默认或历史起点");
}
