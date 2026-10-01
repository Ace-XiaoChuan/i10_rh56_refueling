#pragma once
// Shared model/world loading for audit and MTC.
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <set>
#include <map>
#include <algorithm>

inline std::string read(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot read " + path);
  std::stringstream buffer; buffer << file.rdbuf(); return buffer.str();
}
inline Eigen::Isometry3d transform(const YAML::Node& rows) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  for (int r=0;r<4;++r) for (int c=0;c<4;++c) out.matrix()(r,c)=rows[r][c].as<double>();
  return out;
}
inline geometry_msgs::msg::Pose pose(const YAML::Node& object) {
  geometry_msgs::msg::Pose out;
  out.position.x=object["position"][0].as<double>();
  out.position.y=object["position"][1].as<double>();
  out.position.z=object["position"][2].as<double>();
  Eigen::Matrix3d rot;
  for (int r=0;r<3;++r) for (int c=0;c<3;++c) rot(r,c)=object["rotation_matrix"][r][c].as<double>();
  Eigen::Quaterniond q(rot); out.orientation.x=q.x();out.orientation.y=q.y();out.orientation.z=q.z();out.orientation.w=q.w();
  return out;
}
inline void loadWorld(planning_scene::PlanningScene& scene,const YAML::Node& config,bool source) {
  for (const auto& entry:config["objects"]) {
    if (source && entry["added_collision"].as<bool>()) continue;
    moveit_msgs::msg::CollisionObject msg;
    msg.header.frame_id=config["frame_id"].as<std::string>();
    msg.id=entry["id"].as<std::string>();msg.operation=msg.ADD;
    auto p=pose(entry);auto shape=entry["shape"];auto type=shape["type"].as<std::string>();
    if (type=="mesh") {
      std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(shape["uri"].as<std::string>()));
      if (!mesh) throw std::runtime_error("Mesh load failed: "+msg.id);
      shapes::ShapeMsg mesh_msg;
      if (!shapes::constructMsgFromShape(mesh.get(),mesh_msg)) throw std::runtime_error("Mesh conversion failed");
      msg.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));msg.mesh_poses.push_back(p);
    } else if (type=="plane") {
      shape_msgs::msg::Plane plane;
      for (int i=0;i<4;++i) plane.coef[i]=shape["coefficients"][i].as<double>();
      msg.planes.push_back(plane);msg.plane_poses.push_back(p);
    } else {
      shape_msgs::msg::SolidPrimitive primitive;
      if (type=="box") {
        primitive.type=primitive.BOX;
        for (const auto& v:shape["size"]) primitive.dimensions.push_back(v.as<double>());
      } else if(type=="cylinder") {
        primitive.type=primitive.CYLINDER;primitive.dimensions={shape["length"].as<double>(),shape["radius"].as<double>()};
      } else if(type=="sphere") {
        primitive.type=primitive.SPHERE;primitive.dimensions={shape["radius"].as<double>()};
      } else throw std::runtime_error("Unsupported primitive");
      msg.primitives.push_back(primitive);msg.primitive_poses.push_back(p);
    }
    if (!scene.processCollisionObjectMsg(msg)) throw std::runtime_error("World insertion failed: "+msg.id);
  }
  if (!source) for (const auto& entry:config["allowed_world_contacts"])
    scene.getAllowedCollisionMatrixNonConst().setEntry(entry["object"].as<std::string>(),entry["link"].as<std::string>(),true);
}
