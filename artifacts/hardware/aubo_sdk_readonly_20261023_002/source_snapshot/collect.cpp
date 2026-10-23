/**
 * 只读保存控制器配置和 RTDE 状态，便于后续核对运动学与接入 MoveIt。
 * 仅使用 getter 和客户端连接认证；RTDE 只订阅输出，不发布输入。
 * 保存主机 UTC/单调时钟及控制器启动时间戳，不假定两端时钟已同步。
 */
#include <arpa/inet.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <nlohmann/json.hpp>
#include <aubo_sdk/rpc.h>
#include <aubo_sdk/rtde.h>

using Json = nlohmann::json;
using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace {
std::string utc_now() {
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&seconds, &tm);
  const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
      now.time_since_epoch()).count() % 1000000;
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << '.'
      << std::setw(6) << std::setfill('0') << micros << 'Z';
  return out.str();
}

double monotonic_s() {
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

template <typename T> std::string label(T value) {
  std::ostringstream out;
  out << value;
  return out.str();
}

void save_json(const fs::path& path, const Json& value) {
  std::ofstream file(path);
  file.exceptions(std::ios::badbit | std::ios::failbit);
  file << value.dump(2) << '\n';
}

struct RpcCleanup {
  RpcClient& client;
  ~RpcCleanup() {
    try { if (client.hasLogined()) client.logout(); } catch (...) {}
    try { if (client.hasConnected()) client.disconnect(); } catch (...) {}
  }
};

struct RtdeCleanup {
  RtdeClient& client;
  int channel = -1;
  ~RtdeCleanup() {
    try { if (channel >= 0) client.removeTopic(false, channel); } catch (...) {}
    try { if (client.hasLogined()) client.logout(); } catch (...) {}
    try { if (client.hasConnected()) client.disconnect(); } catch (...) {}
  }
};

struct Field {
  std::string name;
  char type;  // d=double, v=double数组, r=运行模式, s=安全模式, b=bool
};
}  // namespace

int main(int argc, char** argv) {
  std::string host, output;
  double duration_s = 15.0;
  double requested_hz = 50.0;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "用法：aubo_sdk_collect --host IPv4 --output-dir 新目录 "
                  << "[--duration 15] [--hz 50]\n仅只读采集，不发送运动指令。\n";
        return 0;
      }
      if (i + 1 >= argc) throw std::runtime_error("缺少参数值");
      const std::string value = argv[++i];
      if (arg == "--host") host = value;
      else if (arg == "--output-dir") output = value;
      else if (arg == "--duration") {
        std::size_t used = 0;
        duration_s = std::stod(value, &used);
        if (used != value.size()) throw std::runtime_error("采样时长格式错误");
      } else if (arg == "--hz") {
        std::size_t used = 0;
        requested_hz = std::stod(value, &used);
        if (used != value.size()) throw std::runtime_error("采样频率格式错误");
      } else throw std::runtime_error("未知参数");
    }
    in_addr address{};
    if (inet_pton(AF_INET, host.c_str(), &address) != 1 || output.empty() ||
        !std::isfinite(duration_s) || duration_s < 1 || duration_s > 60 ||
        !std::isfinite(requested_hz) || requested_hz < 1 || requested_hz > 100) {
      throw std::runtime_error("需提供 IPv4、新输出目录，时长 1～60 秒、频率 1～100 Hz");
    }
    const fs::path directory(output);
    if (fs::exists(directory)) throw std::runtime_error("输出目录已存在，避免覆盖原数据");
    fs::create_directories(directory);

    Json metadata{{"schema_version", 1}, {"host", host}, {"rpc_port", 30004},
                  {"rtde_port", 30010}, {"sdk_version", INTERFACE_VERSION},
                  {"started_utc", utc_now()}, {"read_only", true},
                  {"authentication", "vendor_example_default"},
                  {"requested_duration_s", duration_s}, {"requested_rtde_hz", requested_hz},
                  {"clock_note", "主机 UTC 与控制器启动时间未同步；RPC 各字段顺序读取"},
                  {"pose_encoding", "保留控制器六维原值，姿态编码需核对"},
                  {"joint_order", "SDK 原顺序，尚未与 MoveIt 关节名逐项核对"}};
    save_json(directory / "metadata.json", metadata);

    RpcClient rpc;
    rpc.setLogHandler([](int, const char*, int, const std::string&) {});
    RpcCleanup rpc_cleanup{rpc};
    rpc.setRequestTimeout(2000);
    const int connect_code = rpc.connect(host, 30004);
    if (connect_code != 0 || !rpc.hasConnected()) throw std::runtime_error("RPC 连接失败");
    const int login_code = rpc.login("aubo", "123456");
    if (login_code != 0 || !rpc.hasLogined()) throw std::runtime_error("默认 RPC 认证失败");
    const auto names = rpc.getRobotNames();
    if (names.size() != 1) throw std::runtime_error("需要控制器恰好返回一个机器人");
    metadata["robot_name"] = names.front();
    const auto robot = rpc.getRobotInterface(names.front());
    const auto config = robot->getRobotConfig();
    const auto state = robot->getRobotState();
    const auto system = rpc.getSystemInfo();

    Json queries = Json::object();
    std::ofstream journal(directory / "queries.jsonl");
    journal.exceptions(std::ios::badbit | std::ios::failbit);
    int ok_count = 0, failed_count = 0;
    auto capture = [&](const std::string& name, const std::function<Json()>& fn) {
      Json result{{"method", name}, {"host_started_utc", utc_now()}};
      const double started = monotonic_s();
      try {
        result["value"] = fn();
        result["ok"] = true;
        ++ok_count;
      } catch (const std::exception& error) {
        result["ok"] = false;
        result["error"] = error.what();
        ++failed_count;
      } catch (...) {
        result["ok"] = false;
        result["error"] = "SDK 查询异常";
        ++failed_count;
      }
      result["host_duration_s"] = monotonic_s() - started;
      result["host_finished_utc"] = utc_now();
      queries[name] = result;
      journal << result.dump() << '\n';
      journal.flush();
      std::cout << (result["ok"].get<bool>() ? "已保存 " : "查询失败 ")
                << name << std::endl;
    };
#define READ_CONFIG(method) capture("RobotConfig." #method, [&]() -> Json { return config->method(); })
#define READ_STATE(method) capture("RobotState." #method, [&]() -> Json { return state->method(); })
#define READ_SYSTEM(method) capture("SystemInfo." #method, [&]() -> Json { return system->method(); })
    READ_SYSTEM(getControlSoftwareVersionCode);
    READ_SYSTEM(getInterfaceVersionCode);
    READ_SYSTEM(getControlSoftwareBuildDate);
    READ_SYSTEM(getControlSoftwareVersionHash);
    READ_CONFIG(getName);
    READ_CONFIG(getRobotType);
    READ_CONFIG(getRobotSubType);
    READ_CONFIG(getControlBoxType);
    READ_CONFIG(getDof);
    READ_CONFIG(getCycletime);
    capture("RobotConfig.getKinematicsParam(false)", [&]() -> Json { return config->getKinematicsParam(false); });
    capture("RobotConfig.getKinematicsParam(true)", [&]() -> Json { return config->getKinematicsParam(true); });
    capture("RobotConfig.getKinematicsCompensate(20)", [&]() -> Json { return config->getKinematicsCompensate(20.0); });
    READ_CONFIG(getJointMinPositions);
    READ_CONFIG(getJointMaxPositions);
    READ_CONFIG(getJointMaxSpeeds);
    READ_CONFIG(getJointMaxAccelerations);
    READ_CONFIG(getTcpMaxSpeeds);
    READ_CONFIG(getTcpMaxAccelerations);
    READ_CONFIG(getTcpOffset);
    READ_CONFIG(getMountingPose);
    READ_CONFIG(getGravity);
    capture("RobotConfig.getPayload", [&]() -> Json {
      const auto payload = config->getPayload();
      return {{"mass", std::get<0>(payload)}, {"cog", std::get<1>(payload)},
              {"aom", std::get<2>(payload)}, {"inertia", std::get<3>(payload)}};
    });
    READ_CONFIG(getHomePosition);
    READ_CONFIG(getDefaultJointAcc);
    READ_CONFIG(getDefaultJointSpeed);
    READ_CONFIG(getDefaultToolAcc);
    READ_CONFIG(getDefaultToolSpeed);
    READ_CONFIG(getCollisionLevel);
    READ_CONFIG(getCollisionStopType);
    READ_CONFIG(hasTcpForceSensor);
    READ_CONFIG(getTcpForceSensorNames);
    READ_CONFIG(getTcpForceSensorPose);
    READ_CONFIG(getTcpForceOffset);
    READ_CONFIG(hasBaseForceSensor);
    READ_CONFIG(getBaseForceSensorNames);
    READ_CONFIG(getBaseForceOffset);
    capture("RobotManage.isSimulationEnabled", [&]() -> Json { return robot->getRobotManage()->isSimulationEnabled(); });
    READ_STATE(getJointPositions);
    READ_STATE(getJointSpeeds);
    READ_STATE(getJointAccelerations);
    READ_STATE(getJointCurrents);
    READ_STATE(getJointVoltages);
    READ_STATE(getJointTemperatures);
    READ_STATE(getJointTorqueSensors);
    READ_STATE(getTcpPose);
    READ_STATE(getToolPose);
    READ_STATE(getTcpSpeed);
    READ_STATE(getTcpForce);
    READ_STATE(getTcpForceSensors);
    READ_STATE(getBaseForce);
    READ_STATE(getBaseForceSensor);
    READ_STATE(getJointUniqueIds);
    READ_STATE(getJointFirmwareVersions);
    READ_STATE(getJointHardwareVersions);
    READ_STATE(getMasterBoardUniqueId);
    READ_STATE(getMasterBoardFirmwareVersion);
    READ_STATE(getMasterBoardHardwareVersion);
    READ_STATE(getToolUniqueId);
    READ_STATE(getToolFirmwareVersion);
    READ_STATE(getToolHardwareVersion);
    READ_STATE(getControlBoxTemperature);
    READ_STATE(getMainVoltage);
    READ_STATE(getRobotVoltage);
    READ_STATE(getRobotCurrent);
    READ_STATE(isPowerOn);
    READ_STATE(isSteady);
    READ_STATE(isWithinSafetyLimits);
    READ_STATE(isCollisionOccurred);
    capture("RobotState.getRobotModeType", [&]() -> Json { return label(state->getRobotModeType()); });
    capture("RobotState.getSafetyModeType", [&]() -> Json { return label(state->getSafetyModeType()); });

    // 只请求正运动学计算，不将这些关节角发送给 MotionControl。
    // 这些是假设构型的模型输出，不能充当机械臂实际走过的标定样本。
    capture("RobotAlgorithm.forwardToolKinematics_model_probes", [&]() -> Json {
      const auto q = state->getJointPositions();
      if (q.size() != 6) throw std::runtime_error("实际关节角维度错误");
      Json probes = Json::array();
      for (int axis = -1; axis < 6; ++axis) {
        for (int sign : {-1, 1}) {
          if (axis == -1 && sign == 1) continue;
          auto input = q;
          if (axis >= 0) input[axis] += sign * 0.1;
          const auto result = robot->getRobotAlgorithm()->forwardToolKinematics(input);
          probes.push_back({{"q_rad", input}, {"changed_axis", axis},
                            {"delta_rad", axis < 0 ? 0.0 : sign * 0.1},
                            {"flange_pose_raw", std::get<0>(result)},
                            {"result_code", std::get<1>(result)},
                            {"source", "controller_model_computation_not_physical_motion"}});
        }
      }
      return probes;
    });
    capture("Math.poseTrans_flange_basis_computation", [&]() -> Json {
      const auto pose = state->getToolPose();
      Json probes = Json::array();
      for (int axis = 0; axis < 3; ++axis) {
        std::vector<double> local(6, 0.0);
        local[axis] = 0.01;
        const auto result = rpc.getMath()->poseTrans(pose, local);
        probes.push_back({{"flange_pose_raw", pose}, {"local_translation_pose", local},
                          {"transformed_pose_raw", result},
                          {"source", "SDK_math_computation_not_physical_motion"}});
      }
      return probes;
    });
#undef READ_CONFIG
#undef READ_STATE
#undef READ_SYSTEM
    save_json(directory / "configuration.json", queries);
    metadata["rpc_query_successes"] = ok_count;
    metadata["rpc_query_failures"] = failed_count;
    save_json(directory / "metadata.json", metadata);

    // 回调只读取 RTDE 输出。共享对象先于客户端创建，保证析构时回调资源有效。
    std::mutex stream_mutex;
    std::ofstream stream(directory / "rtde_samples.jsonl");
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    std::atomic<std::size_t> samples{0}, parse_errors{0}, invalid_samples{0};
    const std::vector<Field> candidates{
        {"timestamp", 'd'}, {"R1_actual_q", 'v'}, {"R1_actual_qd", 'v'},
        {"R1_actual_TCP_pose", 'v'}, {"R1_actual_TCP_speed", 'v'},
        {"R1_actual_TCP_force", 'v'}, {"R1_actual_current", 'v'},
        {"R1_joint_temperatures", 'v'}, {"R1_target_q", 'v'},
        {"R1_joint_torque_sensor", 'v'}, {"R1_robot_mode", 'r'},
        {"R1_safety_mode", 's'}, {"R1_is_simulation_enabled", 'b'}};
    std::vector<Field> fields;
    std::vector<std::string> requested_names;
    Json rtde_result{{"requested_hz", requested_hz}, {"requested_duration_s", duration_s}};
    const double rtde_started = monotonic_s();
    try {
      RtdeClient rtde;
      rtde.setLogHandler([](int, const char*, int, const std::string&) {});
      RtdeCleanup rtde_cleanup{rtde};
      if (rtde.connect(host, 30010) != 0 || !rtde.hasConnected()) throw std::runtime_error("RTDE 连接失败");
      if (rtde.login("aubo", "123456") != 0 || !rtde.hasLogined()) throw std::runtime_error("RTDE 认证失败");
      const auto menu = rtde.getOutputMaps();
      save_json(directory / "rtde_output_menu.json", menu);
      rtde_result["protocol_version"] = rtde.getProtocolVersion();
      for (const auto& field : candidates) {
        if (menu.count(field.name)) {
          fields.push_back(field);
          requested_names.push_back(field.name);
        }
      }
      if (std::find(requested_names.begin(), requested_names.end(), "R1_actual_q") == requested_names.end()) {
        throw std::runtime_error("RTDE 菜单未包含 R1_actual_q");
      }
      rtde_result["selected_fields"] = requested_names;
      const int channel = rtde.setTopic(false, requested_names, requested_hz, 0);
      if (channel < 0) throw std::runtime_error("RTDE 输出订阅失败");
      rtde_cleanup.channel = channel;
      Json recipes = Json::object();
      for (const auto& entry : rtde.getRegisteredOutputRecipe()) {
        const auto& recipe = entry.second;
        recipes[std::to_string(entry.first)] = {
            {"to_server", recipe.to_server}, {"channel", recipe.chanel},
            {"frequency", recipe.frequency}, {"trigger", recipe.trigger},
            {"segments", recipe.segments}};
      }
      save_json(directory / "rtde_registered_recipes.json", recipes);
      const int subscribe_code = rtde.subscribe(channel, [&](InputParser& parser) {
        Json sample{{"host_received_utc", utc_now()}, {"host_monotonic_s", monotonic_s()}};
        try {
          for (const auto& field : fields) {
            if (field.type == 'd') {
              const double value = parser.popDouble();
              sample[field.name] = value;
              if (!std::isfinite(value)) sample["invalid_fields"][field.name] = "non_finite";
            }
            if (field.type == 'v') {
              const auto values = parser.popVectorDouble();
              sample[field.name] = values;
              if (values.empty()) sample["empty_fields"].push_back(field.name);
              if (!std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); })) {
                sample["invalid_fields"][field.name] = "contains_non_finite";
              }
            }
            if (field.type == 'r') sample[field.name] = label(parser.popRobotModeType());
            if (field.type == 's') sample[field.name] = label(parser.popSafetyModeType());
            if (field.type == 'b') sample[field.name] = parser.popBool();
          }
          sample["parse_ok"] = true;
          if (sample.contains("invalid_fields")) ++invalid_samples;
        } catch (...) {
          sample["parse_ok"] = false;
          ++parse_errors;
        }
        std::lock_guard<std::mutex> lock(stream_mutex);
        stream << sample.dump() << '\n';
        ++samples;
        if (samples % 25 == 0) stream.flush();
      });
      if (subscribe_code != 0) throw std::runtime_error("RTDE 回调注册失败");
      std::cout << "开始只读 RTDE 采集，时长 " << duration_s << " 秒" << std::endl;
      std::this_thread::sleep_for(std::chrono::duration<double>(duration_s));
      rtde_result["ok"] = true;
    } catch (const std::exception& error) {
      rtde_result["ok"] = false;
      rtde_result["error"] = error.what();
    } catch (...) {
      rtde_result["ok"] = false;
      rtde_result["error"] = "RTDE 查询异常";
    }
    stream.flush();
    rtde_result["samples"] = samples.load();
    rtde_result["parse_errors"] = parse_errors.load();
    rtde_result["samples_with_non_finite_fields"] = invalid_samples.load();
    rtde_result["host_elapsed_s"] = monotonic_s() - rtde_started;
    metadata["rtde"] = rtde_result;
    metadata["finished_utc"] = utc_now();
    save_json(directory / "metadata.json", metadata);
    std::cout << "资料已保存到 " << directory << "；RPC 成功 " << ok_count
              << "、失败 " << failed_count << "，RTDE 样本 " << samples
              << "，解析异常 " << parse_errors << std::endl;
    return samples > 0 && parse_errors == 0 ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "采集未完成：" << error.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "采集未完成：SDK 异常，已写入的资料保留。\n";
    return 1;
  }
}
