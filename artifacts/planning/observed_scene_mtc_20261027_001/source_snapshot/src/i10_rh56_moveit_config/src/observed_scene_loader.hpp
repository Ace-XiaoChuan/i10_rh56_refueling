#pragma once
// MTC、碰撞对照与 RViz 共用实测场景入口；不会把实测障碍叠在旧油箱上。
#include "scene_loader.hpp"
#include <nlohmann/json.hpp>
#include <octomap/OcTree.h>
#include <openssl/evp.h>
#include <filesystem>
#include <iomanip>

using SceneJson = nlohmann::json;

inline std::string fileSha256(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("Cannot read: " + path.string());
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA256 initialization failed");
  std::array<char, 65536> buffer;
  while (input) {
    input.read(buffer.data(), buffer.size());
    if (EVP_DigestUpdate(context.get(), buffer.data(), input.gcount()) != 1)
      throw std::runtime_error("SHA256 update failed");
  }
  if (!input.eof()) throw std::runtime_error("File read failed: " + path.string());
  unsigned char hash[EVP_MAX_MD_SIZE]; unsigned size = 0;
  if (EVP_DigestFinal_ex(context.get(), hash, &size) != 1) throw std::runtime_error("SHA256 failed");
  std::ostringstream text;
  for (unsigned i = 0; i < size; ++i) text << std::hex << std::setw(2) << std::setfill('0') << int(hash[i]);
  return text.str();
}

inline Eigen::Isometry3d rigidJson(const SceneJson& value) {
  if (!value.is_array() || value.size() != 4) throw std::runtime_error("Invalid rigid matrix shape");
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 4; ++r) {
    if (!value[r].is_array() || value[r].size() != 4) throw std::runtime_error("Invalid matrix row");
    for (int c = 0; c < 4; ++c) result.matrix()(r,c) = value[r][c].get<double>();
  }
  if (!result.matrix().allFinite() ||
      (result.matrix().row(3)-Eigen::RowVector4d(0,0,0,1)).norm() > 1e-10 ||
      (result.linear().transpose()*result.linear()-Eigen::Matrix3d::Identity()).norm() > 1e-8 ||
      std::abs(result.linear().determinant()-1.) > 1e-8)
    throw std::runtime_error("Invalid right-handed rigid matrix");
  return result;
}

inline geometry_msgs::msg::Pose poseJson(const SceneJson& value) {
  geometry_msgs::msg::Pose result;
  auto position = value.value("position", SceneJson::object());
  result.position.x = position.value("x", 0.); result.position.y = position.value("y", 0.);
  result.position.z = position.value("z", 0.);
  auto q = value.at("orientation");
  Eigen::Quaterniond quaternion(q.value("w",0.), q.value("x",0.), q.value("y",0.), q.value("z",0.));
  if (!Eigen::Vector3d(result.position.x,result.position.y,result.position.z).allFinite() ||
      !quaternion.coeffs().allFinite() || std::abs(quaternion.norm()-1.) > 1e-8)
    throw std::runtime_error("Invalid collision object pose");
  result.orientation.x=quaternion.x();result.orientation.y=quaternion.y();
  result.orientation.z=quaternion.z();result.orientation.w=quaternion.w();
  return result;
}

struct ObservedWorld {
  SceneJson manifest, summary;
  std::shared_ptr<octomap::OcTree> octree;
  Eigen::Isometry3d world_from_octomap;
};

inline ObservedWorld loadObservedWorld(planning_scene::PlanningScene& scene, const YAML::Node& baseline,
                                       const SceneJson& binding) {
  const std::filesystem::path manifest_path = binding.at("manifest_path").get<std::string>();
  if (fileSha256(manifest_path) != binding.at("manifest_sha256").get<std::string>())
    throw std::runtime_error("Observed scene manifest hash mismatch");
  ObservedWorld result;
  result.manifest = SceneJson::parse(read(manifest_path.string()));
  const auto& manifest = result.manifest;
  if (manifest.at("scene_representation") != "environment_octomap_and_cap_collision_object" ||
      manifest.at("planning_frame").get<std::string>() != scene.getPlanningFrame() ||
      manifest.at("valid_for_hardware").get<bool>() || !manifest.at("allowed_collision_pairs").empty())
    throw std::runtime_error("Observed scene frame, representation or contact rules mismatch");
  const std::set<std::string> replace{"fuel_tank_body","cap_body","fuel_door"};
  if (binding.at("replace_source_bodies").get<std::set<std::string>>() != replace)
    throw std::runtime_error("Unexpected legacy replacement list");
  YAML::Node kept = YAML::Clone(baseline);
  kept["objects"] = YAML::Node(YAML::NodeType::Sequence);
  kept["allowed_world_contacts"] = YAML::Node(YAML::NodeType::Sequence);
  std::set<std::string> retained;
  SceneJson removed = SceneJson::array();
  for (const auto& object : baseline["objects"]) {
    if (replace.count(object["source_body"].as<std::string>())) removed.push_back(object["id"].as<std::string>());
    else {kept["objects"].push_back(object);retained.insert(object["id"].as<std::string>());}
  }
  for (const auto& pair : baseline["allowed_world_contacts"])
    if (retained.count(pair["object"].as<std::string>())) kept["allowed_world_contacts"].push_back(pair);
  loadWorld(scene, kept, false);
  const auto base = manifest_path.parent_path();
  const auto map_path = base/manifest.at("map_path").get<std::string>();
  const auto cap_path = base/manifest.at("cap_object_path").get<std::string>();
  if (fileSha256(map_path) != manifest.at("map_sha256") || fileSha256(cap_path) != manifest.at("cap_object_sha256"))
    throw std::runtime_error("Observed geometry hash mismatch");
  result.octree = std::make_shared<octomap::OcTree>(manifest.at("resolution_m").get<double>());
  if (!result.octree->readBinary(map_path.string()) || result.octree->size() == 0 ||
      std::abs(result.octree->getResolution()-manifest.at("resolution_m").get<double>()) > 1e-12)
    throw std::runtime_error("Observed OctoMap loading failed");
  result.world_from_octomap = rigidJson(manifest.at("world_from_octomap"));
  // 同一棵局部八叉树整体变换，不旋转中心后重新分格。
  scene.processOctomapPtr(result.octree, result.world_from_octomap);
  const auto raw = SceneJson::parse(read(cap_path.string()));
  moveit_msgs::msg::CollisionObject cap;
  cap.header.frame_id = raw.at("header").at("frame_id");
  cap.id = raw.at("id"); cap.operation = cap.ADD;
  if (cap.id != manifest.at("cap_object_id") || cap.header.frame_id != scene.getPlanningFrame() ||
      raw.at("operation") != SceneJson::array({0}) || raw.at("primitives").size() != raw.at("primitive_poses").size())
    throw std::runtime_error("Invalid independent cap object");
  cap.pose = poseJson(raw.at("pose"));
  for (std::size_t i = 0; i < raw.at("primitives").size(); ++i) {
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape.BOX;
    for(const auto& value:raw["primitives"][i]["dimensions"]) shape.dimensions.push_back(value.get<double>());
    if (raw["primitives"][i]["type"] != shape.BOX || shape.dimensions.size() != 3)
      throw std::runtime_error("Cap requires original voxel boxes");
    for (double size : shape.dimensions)
      if (!std::isfinite(size) || std::abs(size-result.octree->getResolution()) > 1e-12)
        throw std::runtime_error("Cap voxel dimensions changed");
    cap.primitives.push_back(shape);cap.primitive_poses.push_back(poseJson(raw["primitive_poses"][i]));
  }
  if (cap.primitives.size() != manifest.at("validation").at("cap_object_voxel_count") ||
      !scene.processCollisionObjectMsg(cap) || !scene.getWorld()->hasObject(planning_scene::PlanningScene::OCTOMAP_NS) ||
      !scene.getWorld()->hasObject(cap.id)) throw std::runtime_error("Observed scene insertion failed");
  for (const auto& id : removed)
    if (scene.getWorld()->hasObject(id.get<std::string>())) throw std::runtime_error("Legacy obstacle remains");
  // 接近阶段逐对禁止与实测障碍碰撞，避免 SRDF 默认值或其他请求意外放行。
  for (const auto* link : scene.getRobotModel()->getLinkModelsWithCollisionGeometry()) {
    scene.getAllowedCollisionMatrixNonConst().setEntry(cap.id, link->getName(), false);
    scene.getAllowedCollisionMatrixNonConst().setEntry(planning_scene::PlanningScene::OCTOMAP_NS, link->getName(), false);
  }
  result.summary = {{"manifest_path",manifest_path.string()}, {"manifest_sha256",fileSha256(manifest_path)},
                    {"removed_legacy_object_ids",removed}, {"retained_legacy_object_ids",retained},
                    {"world_object_ids",scene.getWorld()->getObjectIds()}, {"cap_object_id",cap.id},
                    {"cap_box_count",cap.primitives.size()}, {"resolution_m",result.octree->getResolution()},
                    {"environment_base_cell_count",manifest.at("validation").at("environment_voxel_count")},
                    {"whole_robot_contact_exemptions",false}, {"placement_mode",manifest.at("mode")},
                    {"valid_for_hardware",false}};
  return result;
}

inline SceneJson collisionDetails(const planning_scene::PlanningScene& scene, const moveit::core::RobotState& state) {
  collision_detection::CollisionRequest request;
  request.contacts=true; request.max_contacts=10000;request.max_contacts_per_pair=1;
  collision_detection::CollisionResult result;scene.checkCollision(request,result,state);
  SceneJson pairs=SceneJson::array();
  for (const auto& pair : result.contacts)
    pairs.push_back({{"a",pair.first.first},{"b",pair.first.second},{"depth_m",pair.second.front().depth}});
  return {{"collision",result.collision},{"pairs",pairs},{"satisfies_bounds",state.satisfiesBounds(1e-8)}};
}
