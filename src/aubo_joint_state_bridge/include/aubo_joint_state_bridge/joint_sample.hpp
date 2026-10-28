#pragma once
// SDK 的六项数组与规划模型逐项对应；保持原角度、符号、零点和绕数。
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace aubo_joint_state_bridge {
inline const std::array<std::string, 6> kJointNames{
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint"};

inline void validateSample(const std::vector<double>& position,
                           const std::vector<double>& velocity) {
  // 位置单位 rad、速度单位 rad/s；缺少速度时留空，不补零或差分估计。
  if (position.size() != 6 || (!velocity.empty() && velocity.size() != 6))
    throw std::runtime_error("关节位置必须为六项，速度必须为空或六项");
  for (const auto* values : {&position, &velocity})
    for (double value : *values)
      if (!std::isfinite(value))
        throw std::runtime_error("关节状态包含非有限数值");
}
}  // namespace aubo_joint_state_bridge
