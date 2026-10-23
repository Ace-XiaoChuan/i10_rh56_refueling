/**
 * 检查官方 ROS2 硬件驱动引用的主要 SDK 符号能否在本地编译、链接。
 * 只取成员函数地址；没有客户端实例，也不调用这些 SDK 方法。
 * 这项检查不替代驱动完整构建和控制器运行验证。
 */
#include <iostream>

#include <aubo_sdk/rpc.h>
#include <aubo_sdk/rtde.h>

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

template <typename T>
void require_symbol(T symbol, int& count) {
  // volatile 保留符号引用，使链接器实际检查共享库是否提供该方法。
  [[maybe_unused]] T volatile reference = symbol;
  ++count;
}

int main() {
  int count = 0;
#define CHECK_SDK(method) require_symbol(&method, count)
  CHECK_SDK(RpcClient::setRequestTimeout);
  CHECK_SDK(RpcClient::connect);
  CHECK_SDK(RpcClient::login);
  CHECK_SDK(RpcClient::disconnect);
  CHECK_SDK(RtdeClient::connect);
  CHECK_SDK(RtdeClient::login);
  CHECK_SDK(RtdeClient::setTopic);
  CHECK_SDK(RtdeClient::subscribe);
  CHECK_SDK(RtdeClient::publish);
  CHECK_SDK(RtdeClient::disconnect);
  CHECK_SDK(AuboApi::getRobotNames);
  CHECK_SDK(AuboApi::getRobotInterface);
  CHECK_SDK(RobotInterface::getRobotConfig);
  CHECK_SDK(RobotInterface::getMotionControl);
  CHECK_SDK(RobotInterface::getRobotState);
  CHECK_SDK(RobotConfig::setHardwareCustomParameters);
  CHECK_SDK(MotionControl::setServoMode);
  CHECK_SDK(MotionControl::isServoModeEnabled);
  CHECK_SDK(MotionControl::servoJoint);
  CHECK_SDK(RobotState::isSteady);
  CHECK_SDK(InputParser::popVectorDouble);
  CHECK_SDK(InputParser::popRobotModeType);
  CHECK_SDK(InputParser::popSafetyModeType);
  CHECK_SDK(InputParser::popRuntimeState);
  CHECK_SDK(InputParser::popInt32);
  CHECK_SDK(InputParser::popRobotMsgVector);
#undef CHECK_SDK
  std::cout << count << " 个 SDK 接口符号已通过编译和链接检查。\n"
            << "未创建网络客户端，未调用控制器接口。\n";
}
