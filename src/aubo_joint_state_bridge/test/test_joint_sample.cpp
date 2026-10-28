// 保护真实关节数据的完整性；不将缺失速度、非有限值或绕数转换成伪测量。
#include "aubo_joint_state_bridge/joint_sample.hpp"
#include <gtest/gtest.h>
#include <limits>
using aubo_joint_state_bridge::validateSample;
TEST(JointSample, RejectsPartialAndNonFiniteMeasurements) {
  EXPECT_THROW(validateSample({1, 2}, {}), std::runtime_error);
  EXPECT_THROW(validateSample(std::vector<double>(6), {1}), std::runtime_error);
  auto values = std::vector<double>(6);
  values[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validateSample(values, {}), std::runtime_error);
  values[3] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(validateSample(std::vector<double>(6), values), std::runtime_error);
}
TEST(JointSample, AcceptsMissingVelocityAndPreservesJointWinding) {
  const std::vector<double> position{0, -1, 2, -3, 4, 7};
  EXPECT_NO_THROW(validateSample(position, {}));
  EXPECT_NO_THROW(validateSample(position, std::vector<double>(6)));
  EXPECT_DOUBLE_EQ(position.back(), 7);
}
