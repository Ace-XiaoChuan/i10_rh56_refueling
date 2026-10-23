cmake_minimum_required(VERSION 3.16)
project(aubo_sdk_probe LANGUAGES CXX)

# 单独构建只读检查工具，不改动厂家开发包和现有 MoveIt 规划工程。
set(AUBO_SDK_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../sdk/aubo_sdk-0.23.1-Linux_x86_64+71831e1"
    CACHE PATH "AUBO SDK 开发包目录")
find_package(aubo_sdk 0.23.1 EXACT CONFIG REQUIRED
    PATHS "${AUBO_SDK_ROOT}/lib/cmake/aubo_sdk" NO_DEFAULT_PATH)

add_executable(aubo_sdk_probe main.cpp)
target_compile_features(aubo_sdk_probe PRIVATE cxx_std_17)
target_compile_options(aubo_sdk_probe PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(aubo_sdk_probe PRIVATE aubo_sdk::aubo_sdk)

# 仅引用官方驱动所需的 SDK 符号，不构造网络客户端，也不调用控制接口。
add_executable(aubo_sdk_api_check api_check.cpp)
target_compile_features(aubo_sdk_api_check PRIVATE cxx_std_17)
target_compile_options(aubo_sdk_api_check PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(aubo_sdk_api_check PRIVATE aubo_sdk::aubo_sdk)

# JSON 为本机已有的头文件库，采集工具不依赖 ROS 或 Python SDK。
add_executable(aubo_sdk_collect collect.cpp)
target_compile_features(aubo_sdk_collect PRIVATE cxx_std_17)
target_compile_options(aubo_sdk_collect PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(aubo_sdk_collect PRIVATE aubo_sdk::aubo_sdk)
