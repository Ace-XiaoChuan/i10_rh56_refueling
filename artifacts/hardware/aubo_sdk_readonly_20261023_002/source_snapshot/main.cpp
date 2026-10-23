/**
 * AUBO SDK 本地加载和只读通信检查。
 * 默认只构造客户端；显式提供控制柜 IPv4 后才连接并查询状态。
 * 在线认证默认采用该版本官方示例的参数，可按现场配置覆盖。
 * 关节角按 SDK 顺序输出为 rad；TCP/法兰六维位姿保留控制器原始表示，
 * 不在此解释姿态编码，也不转换成 MoveIt 的四元数。
 */
#include <arpa/inet.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <aubo_sdk/rpc.h>

using arcs::aubo_sdk::RpcClient;

namespace {

void usage() {
  std::cout
      << "用法：aubo_sdk_probe [--offline]\n"
      << "      aubo_sdk_probe --host 控制柜IPv4 [--robot 机器人名称]\n"
      << "      可选：--user RPC用户名 --ask-password\n"
      << "默认只检查本地加载；在线采用官方示例的 RPC 认证参数，端口为 30004。\n";
}

// 离开作用域时释放会话；清理异常不覆盖原来的检查错误。
struct SessionCleanup {
  RpcClient& client;
  ~SessionCleanup() {
    try {
      if (client.hasLogined()) client.logout();
    } catch (...) {
    }
    try {
      if (client.hasConnected()) client.disconnect();
    } catch (...) {
    }
  }
};

void print_vector(const char* name, const std::vector<double>& values) {
  std::cout << name << " = [";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << values[i];
  }
  std::cout << "]\n";
}

void validate_vector(const std::vector<double>& values) {
  if (values.size() != 6 ||
      !std::all_of(values.begin(), values.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error("状态不是六个有限数值");
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string host, user = "aubo", robot_name;
  bool user_given = false;
  bool ask_password = false;
  bool offline = argc == 1;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      usage();
      return 0;
    }
    if (arg == "--offline") {
      offline = true;
    } else if (arg == "--ask-password") {
      ask_password = true;
    } else if ((arg == "--host" || arg == "--user" || arg == "--robot") &&
               i + 1 < argc) {
      const std::string value = argv[++i];
      if (arg == "--host") host = value;
      if (arg == "--user") {
        user = value;
        user_given = true;
      }
      if (arg == "--robot") robot_name = value;
    } else {
      std::cerr << "参数无效或缺少参数值：" << arg << '\n';
      usage();
      return 2;
    }
  }
  if (offline && (!host.empty() || user_given || ask_password || !robot_name.empty())) {
    std::cerr << "--offline 不能与在线连接参数同时使用。\n";
    return 2;
  }
  if (!offline) {
    in_addr address{};
    if (host.empty() || user.empty() ||
        inet_pton(AF_INET, host.c_str(), &address) != 1) {
      std::cerr << "在线检查必须提供 --host 控制柜IPv4，RPC 用户名不能为空。\n";
      return 2;
    }
    if (ask_password && !isatty(STDIN_FILENO)) {
      std::cerr << "使用 --ask-password 时请在交互终端隐藏输入 RPC 密码。\n";
      return 2;
    }
  }

  // 只报告本程序的阶段和返回码，避免 SDK 调试日志包含登录请求内容。
  const char* stage = "本地客户端初始化";
  try {
    RpcClient client;
    client.setLogHandler([](int, const char*, int, const std::string&) {});
    if (offline) {
      if (client.hasConnected()) {
        throw std::runtime_error("离线客户端出现连接");
      }
      std::cout << "SDK 本地编译、链接和客户端初始化通过。\n"
                << "未发起控制柜连接。\n";
      return 0;
    }

    SessionCleanup cleanup{client};
    client.setRequestTimeout(3000);  // 本版本头文件规定单位为 ms。
    stage = "连接 RPC 服务";
    const int connected = client.connect(host, 30004);
    if (connected != 0 || !client.hasConnected()) {
      std::cerr << "RPC 连接失败，返回码=" << connected
                << "。请核对网络、IP、RPC 服务和 SDK 兼容性。\n";
      return 1;
    }

    stage = "控制器 RPC 认证";
    // 来源：随包 share/example/c++/example_state.cpp 的 login 调用。
    // 这是通信协议的默认认证参数，不需要另行注册网站或软件账户。
    char* entered_password = nullptr;
    if (ask_password) {
      entered_password = getpass("控制器 RPC 密码（隐藏输入）: ");
      if (!entered_password) throw std::runtime_error("无法读取密码");
    }
    const int logged_in = client.login(user, entered_password ? entered_password : "123456");
    if (entered_password) {
      std::fill(entered_password,
                entered_password + std::char_traits<char>::length(entered_password), '\0');
    }
    if (logged_in != 0 || !client.hasLogined()) {
      std::cerr << "控制器 RPC 认证失败，返回码=" << logged_in
                << "。若现场修改过认证参数，可使用 --user 和 --ask-password。\n";
      return 1;
    }

    stage = "获取机器人名称";
    const auto names = client.getRobotNames();
    if (names.empty()) throw std::runtime_error("没有机器人");
    if (robot_name.empty()) {
      if (names.size() != 1) {
        std::cerr << "控制器包含多个机器人，请用 --robot 指定：";
        for (const auto& name : names) std::cerr << ' ' << name;
        std::cerr << '\n';
        return 1;
      }
      robot_name = names.front();
    }
    if (std::find(names.begin(), names.end(), robot_name) == names.end()) {
      std::cerr << "控制器未返回指定的机器人名称。\n";
      return 1;
    }

    // 顺序读取一次诊断数据，不将其当成硬实时同步采集或运动完成判据。
    stage = "只读机器人状态";
    const auto robot = client.getRobotInterface(robot_name);
    const auto state = robot->getRobotState();
    const auto joints = state->getJointPositions();
    const auto tcp = state->getTcpPose();
    const auto flange = state->getToolPose();
    validate_vector(joints);
    validate_vector(tcp);
    validate_vector(flange);
    const auto mode = state->getRobotModeType();
    const auto safety = state->getSafetyModeType();
    const bool steady = state->isSteady();

    // 仅查询控制器配置，区分仿真开关与当前 TCP，不更改任何设置。
    stage = "只读机器人配置";
    const auto config = robot->getRobotConfig();
    const auto robot_type = config->getRobotType();
    const auto tcp_offset = config->getTcpOffset();
    validate_vector(tcp_offset);
    const bool simulation = robot->getRobotManage()->isSimulationEnabled();

    std::cout << std::setprecision(12) << "机器人名称 = " << robot_name << '\n';
    print_vector("关节角_rad", joints);
    print_vector("TCP位姿_控制器原值", tcp);
    print_vector("法兰位姿_控制器原值", flange);
    print_vector("TCP偏移_控制器原值", tcp_offset);
    std::cout << "运行模式 = " << mode << "\n安全模式 = " << safety
              << "\n当前静止 = " << std::boolalpha << steady
              << "\n机器人型号 = " << robot_type
              << "\n仿真模式 = " << simulation << '\n';
    return 0;
  } catch (...) {
    std::cerr << "检查失败，阶段：" << stage
              << "。请核对设备连接、SDK 兼容性及返回数据。\n";
    return 1;
  }
}
