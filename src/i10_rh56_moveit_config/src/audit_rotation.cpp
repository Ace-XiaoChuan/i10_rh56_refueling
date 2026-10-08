// 复核已记录的旋转诊断前缀；只检查几何，不规划、不执行或放宽碰撞规则。
#include "scene_loader.hpp"
#include <nlohmann/json.hpp>
using Json = nlohmann::json;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 1;
  auto node = std::make_shared<rclcpp::Node>("i10_rotation_audit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try {
    const auto dir = node->get_parameter("config_dir").as_string();
    auto input = Json::parse(read(node->get_parameter("diagnostic_path").as_string()));
    node->declare_parameter("robot_description", read(dir+"/robot.urdf"));
    node->declare_parameter("robot_description_semantic", read(dir+"/robot.srdf"));
    robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
    auto model = loader.getModel();
    auto arm = model->getJointModelGroup("arm");
    if (arm->getActiveJointModelNames() != input["arm_joints"].get<std::vector<std::string>>())
      throw std::runtime_error("Joint order mismatch");
    planning_scene::PlanningScene scene(model);
    loadWorld(scene, YAML::Load(input["planning_scene"].dump()), false);
    auto& state = scene.getCurrentStateNonConst();
    state.setToDefaultValues();
    for (auto it=input["initial_joint_positions"].begin(); it!=input["initial_joint_positions"].end(); ++it)
      state.setVariablePosition(it.key(), it.value().get<double>());
    Json report = {{"mode", "full_fcl_rotation_prefix_audit_not_execution"},
                   {"diagnostic_sha256", node->get_parameter("diagnostic_sha256").as_string()},
                   {"checked_samples", 0}, {"passed_samples", 0}, {"clear_prefix_angle_deg", 0.},
                   {"failure", nullptr}, {"completed_input_check", false}};
    // 盖体随角度改变位姿；机器人始终保留接近预型，不添加手盖接触豁免。
    for (const auto& sample : input["samples"]) {
      Json cap_world = {{"frame_id", "world"}, {"objects", sample["cap_objects"]}, {"allowed_world_contacts", Json::array()}};
      loadWorld(scene, YAML::Load(cap_world.dump()), false);
      state.setJointGroupPositions(arm, sample["arm_qpos_rad"].get<std::vector<double>>());
      state.update();
      int checked = report["checked_samples"].get<int>();
      report["checked_samples"] = checked+1;
      collision_detection::CollisionRequest request;
      request.contacts = true; request.max_contacts = 10000; request.max_contacts_per_pair = 100;
      collision_detection::CollisionResult result;
      scene.checkCollision(request, result, state);
      if (!state.satisfiesBounds(1e-8) || result.collision) {
        Json failure = {{"angle_deg", sample["angle_deg"]},
                        {"kind", result.collision ? "fcl_robot_collision" : "joint_bounds"}, {"contacts", Json::array()}};
        for (const auto& pair : result.contacts) {
          double depth = 0.;
          for (const auto& contact : pair.second) depth = std::max(depth, contact.depth);
          failure["contacts"].push_back({{"a", pair.first.first}, {"b", pair.first.second}, {"depth_m", depth}});
        }
        report["failure"] = failure;
        break;
      }
      report["passed_samples"] = checked+1;
      report["clear_prefix_angle_deg"] = sample["angle_deg"];
    }
    report["completed_input_check"] = report["failure"].is_null();
    std::ofstream output(node->get_parameter("output_path").as_string());
    if (!output) throw std::runtime_error("Cannot write rotation audit");
    output << report.dump(2) << std::endl;
    std::cout << report.dump(2) << std::endl;
    code = 0;  // 检出几何限制也是有效诊断结果；故障或输入错误才返回非零。
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
  }
  rclcpp::shutdown();
  return code;
}
