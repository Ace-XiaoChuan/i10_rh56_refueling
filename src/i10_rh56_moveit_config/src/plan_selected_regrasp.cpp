// 指定原始关节构型的重新接近规划；不执行轨迹或生成硬件时序。
#include "scene_loader.hpp"
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <nlohmann/json.hpp>
namespace mtc = moveit::task_constructor;
using Json = nlohmann::json;

// 从 MTC 的解树中提取真实规划路点，并加密复核整台机器人的碰撞和关节限位。
// 加密只用于检查已规划的路径，不能把任意插值当作规划成功。
void collect(const mtc::SolutionBase &solution, Json &stages,
             const planning_scene::PlanningScenePtr &scene,
             const moveit::core::JointModelGroup *arm) {
  if (auto seq = dynamic_cast<const mtc::SolutionSequence *>(&solution)) {
    for (auto s : seq->solutions())
      collect(*s, stages, scene, arm);
    return;
  }
  auto sub = dynamic_cast<const mtc::SubTrajectory *>(&solution);
  if (!sub || !sub->trajectory() || !sub->trajectory()->getWayPointCount())
    return;
  auto traj = sub->trajectory();
  Json stage;
  stage["name"] = solution.creator()->name();
  stage["points"] = Json::array();
  std::vector<double> previous;
  for (size_t i = 0; i < traj->getWayPointCount(); ++i) {
    auto state = traj->getWayPoint(i);
    std::vector<double> q;
    state.copyJointGroupPositions(arm, q);
    // q 为六个臂关节角（rad）；按变化最大的关节确定插值份数。
    // 使每个关节相邻检查点间隔不超过合同中的 0.02 rad。
    int steps = 1;
    if (!previous.empty())
      for (size_t k = 0; k < q.size(); ++k)
        steps =
            std::max(steps, int(std::ceil(std::abs(q[k] - previous[k]) / .02)));
    for (int step = 1; step <= steps; ++step) {
      auto dense = q;
      if (!previous.empty())
        for (size_t k = 0; k < q.size(); ++k)
          dense[k] = previous[k] + (q[k] - previous[k]) * step / steps;
      state.setJointGroupPositions(arm, dense);
      state.update();
      // 空组名检查全机器人，而非只检查 arm；手与安装件也参与碰撞检测。
      if (!state.satisfiesBounds(1e-8) ||
          scene->isStateColliding(state, "", false))
        throw std::runtime_error("Dense trajectory validation failed");
      stage["points"].push_back(dense);
    }
    previous = q;
  }
  stages.push_back(stage);
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int code = 1;
  auto node = std::make_shared<rclcpp::Node>(
      "i10_mtc_selected_regrasp",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  try {
    const auto dir = node->get_parameter("config_dir").as_string();
    const auto output = node->get_parameter("output_path").as_string();
    auto request =
        Json::parse(read(node->get_parameter("request_path").as_string()));
    node->declare_parameter("robot_description", read(dir + "/robot.urdf"));
    node->declare_parameter("robot_description_semantic",
                            read(dir + "/robot.srdf"));
    robot_model_loader::RobotModelLoader loader(node, "robot_description",
                                                true);
    auto model = loader.getModel();
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    // 随机场景使用请求中的障碍快照；普通 P1 请求使用固定基线场景。
    loadWorld(*scene,
              request.contains("planning_scene")
                  ? YAML::Load(request["planning_scene"].dump())
                  : YAML::LoadFile(dir + "/scene.json"),
              false);
    auto arm = model->getJointModelGroup("arm");
    if (arm->getActiveJointModelNames() !=
        request["arm_joints"].get<std::vector<std::string>>())
      throw std::runtime_error("Joint order mismatch");
    auto &state = scene->getCurrentStateNonConst();
    // 同时设置臂起点和手预型；起点不合法就停止，不移动起点来规避失败。
    state.setToDefaultValues();
    for (auto it = request["initial_joint_positions"].begin();
         it != request["initial_joint_positions"].end(); ++it)
      state.setVariablePosition(it.key(), it.value().get<double>());
    state.update();
    if (!state.satisfiesBounds(1e-8) ||
        scene->isStateColliding(state, "", true))
      throw std::runtime_error("Exact initial state invalid; stopping");
    mtc::Task task("", false);
    task.setRobotModel(model);
    task.setName("i10 selected regrasp configuration");
    task.setTimeout(30.);
    // 阶段一：FixedState 固定已确认的完整起始状态，供后续阶段向前规划。
    auto fixed =
        std::make_unique<mtc::stages::FixedState>("confirmed initial state");
    fixed->setState(scene);
    task.add(std::move(fixed));
    // 重新抓握请求先沿盖轴退开；盖保持末段姿态，手使用接近预型。
    // 这只规划释放后的无接触臂运动，不代表松手已实际执行。
    if (request.contains("regrasp")) {
      auto retreat_solver = std::make_shared<mtc::solvers::CartesianPath>();
      retreat_solver->setStepSize(.002);
      retreat_solver->setMinFraction(1.);
      retreat_solver->setJumpThreshold(1.5);
      retreat_solver->setTimeParameterization(nullptr);
      auto retreat = std::make_unique<mtc::stages::MoveRelative>("axial retreat", retreat_solver);
      retreat->setGroup("arm");
      retreat->setIKFrame("base");
      geometry_msgs::msg::Vector3Stamped outward;
      outward.header.frame_id = "world";
      auto axis = request["outward_axis_world"];
      outward.vector.x = axis[0].get<double>();
      outward.vector.y = axis[1].get<double>();
      outward.vector.z = axis[2].get<double>();
      retreat->setDirection(outward);
      double distance = request["regrasp"]["retreat_distance_m"];
      if (!std::isfinite(distance) || distance <= 0.)
        throw std::runtime_error("Invalid retreat distance");
      retreat->setMinMaxDistance(distance, distance);
      task.add(std::move(retreat));
    }
    // 阶段二：MoveTo 用 OMPL/RRTConnect 到达选定预抓取关节构型。
    // 只生成几何路径；nullptr 关闭执行用的时间参数化。
    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    pipeline->setPlannerId("RRTConnectkConfigDefault");
    pipeline->setTimeParameterization(nullptr);
    auto move =
        std::make_unique<mtc::stages::MoveTo>("move to pregrasp", pipeline);
    move->setGroup("arm");
    // "base" 是手根 link：要求这个坐标系到达目标，不是要求臂基座移动。
    move->setIKFrame("base");
    move->setTimeout(15.);
    // 关节目标保留绕数；仅指定手根位姿会让 IK 返回另一个构型。
    if (!request.contains("regrasp") || !request.contains("configuration_selection"))
      throw std::runtime_error("Selected regrasp request required");
    auto selection = request["configuration_selection"];
    auto names = request["arm_joints"].get<std::vector<std::string>>();
    auto pre = selection["pregrasp_q_rad"].get<std::vector<double>>();
    auto selected = selection["grasp_q_rad"].get<std::vector<double>>();
    double joint_tolerance = selection["endpoint_joint_tolerance_rad"];
    if (pre.size() != names.size() || selected.size() != names.size() ||
        !std::isfinite(joint_tolerance) || joint_tolerance <= 0.)
      throw std::runtime_error("Invalid selected joint target");
    std::map<std::string, double> goal;
    for (size_t i = 0; i < names.size(); ++i) {
      if (!std::isfinite(pre[i]) || !std::isfinite(selected[i]))
        throw std::runtime_error("Nonfinite selected joint target");
      goal[names[i]] = pre[i];
    }
    auto pre_state = state;
    pre_state.setJointGroupPositions(arm, pre);
    pre_state.update();
    if (!pre_state.satisfiesBounds(1e-8) || scene->isStateColliding(pre_state, "", false)) {
      std::ofstream rejected(output);
      if (!rejected)
        throw std::runtime_error("Cannot save rejected pregrasp");
      rejected << Json{{"mode", "mtc_selected_regrasp_planning_rejected_no_replay"},
                       {"request", request}, {"reason", "selected_pregrasp_invalid"}}.dump(2) << std::endl;
      throw std::runtime_error("Selected pregrasp invalid");
    }
    auto q = request["hand_orientation_xyzw"];
    move->setGoal(goal);
    task.add(std::move(move));
    // 阶段三：MoveRelative 使用 CartesianPath，手根沿盖轴直线接近。
    // 采样步长单位 m；要求整段完成，并启用相对关节跳变检查。
    auto cart = std::make_shared<mtc::solvers::CartesianPath>();
    cart->setStepSize(.002);
    cart->setMinFraction(1.);
    cart->setJumpThreshold(1.5);
    cart->setTimeParameterization(nullptr);
    auto approach =
        std::make_unique<mtc::stages::MoveRelative>("axial approach", cart);
    approach->setGroup("arm");
    approach->setIKFrame("base");
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "world";
    auto axis = request["outward_axis_world"];
    // 盖轴朝外，取反后得到朝盖体接近的方向；该方向在世界系中表达。
    direction.vector.x = -axis[0].get<double>();
    direction.vector.y = -axis[1].get<double>();
    direction.vector.z = -axis[2].get<double>();
    approach->setDirection(direction);
    double distance = request["approach_distance_m"];
    // 最小/最大距离设为同一值，要求完成指定距离（m），不能只走部分就通过。
    approach->setMinMaxDistance(distance, distance);
    task.add(std::move(approach));
    task.init();
    // 最多取得一个完整解；这不是执行命令，也不是调用失败后自动重试。
    auto result = task.plan(1);
    task.printState();
    if (!result || task.solutions().empty()) {
      task.explainFailure();
      // 已进入真实 MTC 求解且明确无完整路径；与 ROS 启动故障区分。
      std::ofstream rejected(output);
      if (!rejected)
        throw std::runtime_error("Cannot save rejected planning");
      rejected << Json{{"mode", "mtc_selected_regrasp_planning_rejected_no_replay"},
                       {"request", request}, {"reason", "no_complete_mtc_solution"}}.dump(2) << std::endl;
      throw std::runtime_error("MTC found no complete approach solution");
    }
    Json report;
    report["schema_version"] = 1;
    report["mode"] = "mtc_geometric_approach_not_execution";
    report["request"] = request;
    report["stages"] = Json::array();
    collect(**task.solutions().begin(), report["stages"], scene, arm);
    if (report["stages"].size() != (request.contains("regrasp") ? 3u : 2u))
      throw std::runtime_error("Unexpected motion stage count");
    const auto endpoint =
        report["stages"].back()["points"].back().get<std::vector<double>>();
    auto final = state;
    final.setJointGroupPositions(arm, endpoint);
    final.update();
    // 用末态 FK 核对手根是否真到目标；误差分别是 m 和 rad。
    Eigen::Vector3d target;
    for (int i = 0; i < 3; ++i)
      target[i] = request["hand_position"][i];
    auto actual = final.getGlobalLinkTransform("base");
    double error = (actual.translation() - target).norm();
    Eigen::Quaterniond wanted(q[3].get<double>(), q[0].get<double>(),
                              q[1].get<double>(), q[2].get<double>());
    double rotation =
        Eigen::Quaterniond(actual.rotation()).angularDistance(wanted);
    report["endpoint_position_error_m"] = error;
    report["endpoint_orientation_error_rad"] = rotation;
    // 本轮请求显式携带位姿容差；历史请求保持原有验收值。
    double position_tolerance = 1e-4, orientation_tolerance = .002;
    if (request.contains("endpoint_tolerances")) {
      position_tolerance = request["endpoint_tolerances"]["position_m"];
      orientation_tolerance = request["endpoint_tolerances"]["orientation_rad"];
    }
    if (!std::isfinite(position_tolerance) || position_tolerance <= 0. ||
        !std::isfinite(orientation_tolerance) || orientation_tolerance <= 0.)
      throw std::runtime_error("Invalid endpoint tolerances");
    // 朝向相同不保证绕数相同：直接比较原始六关节角，禁止取模。
    double joint_error = 0.;
    for (size_t i = 0; i < endpoint.size(); ++i)
      joint_error = std::max(joint_error, std::abs(endpoint[i] - selected[i]));
    report["endpoint_joint_error_rad"] = joint_error;
    report["selected_configuration_check_passed"] = joint_error <= joint_tolerance;
    if (error > position_tolerance || rotation > orientation_tolerance ||
        joint_error > joint_tolerance) {
      // 保留真实拒绝路点和误差供诊断；独立模式确保不能当作成功轨迹回放。
      report["mode"] = "mtc_endpoint_rejected_no_replay";
      report["dense_fcl_check_passed"] = true;
      report["endpoint_check_passed"] = false;
      std::ofstream rejected(output);
      if (!rejected)
        throw std::runtime_error("Cannot save rejected endpoint");
      rejected << report.dump(2) << std::endl;
      RCLCPP_ERROR(node->get_logger(), "Endpoint error: %.9g m, %.9g rad", error, rotation);
      throw std::runtime_error("Endpoint pose mismatch");
    }
    report["dense_fcl_check_passed"] = true;
    std::ofstream file(output);
    if (!file)
      throw std::runtime_error("Cannot open output");
    file << report.dump(2) << std::endl;
    code = 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
  }
  rclcpp::shutdown();
  return code;
}
