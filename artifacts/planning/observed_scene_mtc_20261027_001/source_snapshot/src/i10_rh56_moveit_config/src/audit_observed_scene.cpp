// 对同一实测场景执行 FCL 相交/分离检查、全臂手几何参与检查与目标 IK 诊断。
#include "observed_scene_loader.hpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <chrono>
#include <thread>
#include <tf2_ros/static_transform_broadcaster.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

using Json = nlohmann::json;

inline void setRequestState(moveit::core::RobotState& state,const Json& request) {
  state.setToDefaultValues();
  for(auto it=request.at("initial_joint_positions").begin();it!=request.at("initial_joint_positions").end();++it)
    state.setVariablePosition(it.key(),it.value().get<double>());
  state.update();
}

Json fixtureTests(const YAML::Node& baseline,const Json& binding) {
  const std::string urdf=R"(<robot name="occupancy_probe"><link name="world"/>
    <link name="probe"><collision><geometry><sphere radius="0.00025"/></geometry></collision></link>
    <joint name="probe_fixed" type="fixed"><parent link="world"/><child link="probe"/></joint></robot>)";
  auto parsed=urdf::parseURDF(urdf);
  auto semantic=std::make_shared<srdf::Model>();
  if(!parsed || !semantic->initString(*parsed,"<robot name='occupancy_probe'/>") )
    throw std::runtime_error("Cannot construct probe fixture");
  auto model=std::make_shared<moveit::core::RobotModel>(parsed,semantic);
  planning_scene::PlanningScene scene(model);
  const auto loaded=loadObservedWorld(scene,baseline,binding);
  moveit::core::RobotState state(model);state.setToDefaultValues();state.update();
  auto test=[&](const Eigen::Vector3d& position,const std::string& expected) {
    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation()=position;
    // 几何探针不用于规划；直接放置球体后只更新其碰撞变换，保留指定探针位置。
    state.updateStateWithLinkAt("probe",pose);state.updateCollisionBodyTransforms();
    auto result=collisionDetails(scene,state);
    bool detected=false;
    for(const auto& pair:result["pairs"])
      if((pair["a"]=="probe" && pair["b"]==expected) || (pair["b"]=="probe" && pair["a"]==expected)) detected=true;
    if(expected.empty() ? result["collision"].get<bool>() : !detected)
      throw std::runtime_error("FCL occupancy probe did not match expected result: "+result.dump());
    result["position_world_m"]={position.x(),position.y(),position.z()};return result;
  };
  auto leaf=loaded.octree->begin_leafs();
  const auto p=leaf.getCoordinate();
  Eigen::Vector3d occupied=loaded.world_from_octomap*Eigen::Vector3d(p.x(),p.y(),p.z());
  const auto cap=scene.getWorld()->getObject(loaded.manifest.at("cap_object_id").get<std::string>());
  return {{"environment_intersection",test(occupied,planning_scene::PlanningScene::OCTOMAP_NS)},
          {"cap_intersection",test(cap->shape_poses_.front().translation(),cap->id_)},
          {"separated",test(Eigen::Vector3d(20,20,20),"")}};
}

Json geometryParticipation(const planning_scene::PlanningScenePtr& scene) {
  Json results=Json::object();
  auto state=scene->getCurrentState();
  for(const auto* link:scene->getRobotModel()->getLinkModelsWithCollisionGeometry()) {
    auto copy=scene->diff();
    const auto shape=link->getShapes().front();
    Eigen::Vector3d local=Eigen::Vector3d::Zero();
    if(shape->type==shapes::MESH) {
      const auto* mesh=static_cast<const shapes::Mesh*>(shape.get());
      // 三角网格按表面参与 FCL 检查；探针跨过实际网格顶点，避免整个探针落在壳内。
      if(!mesh->vertex_count)throw std::runtime_error("Empty collision mesh");
      local=Eigen::Vector3d(mesh->vertices[0],mesh->vertices[1],mesh->vertices[2]);
    }
    auto center=state.getCollisionBodyTransform(link,0)*local;
    moveit_msgs::msg::CollisionObject probe;probe.header.frame_id=scene->getPlanningFrame();
    probe.id="geometry_participation_probe";probe.operation=probe.ADD;
    shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={.08,.08,.08};
    geometry_msgs::msg::Pose pose;pose.orientation.w=1.;
    pose.position.x=center.x();pose.position.y=center.y();pose.position.z=center.z();
    probe.primitives.push_back(box);probe.primitive_poses.push_back(pose);
    if(!copy->processCollisionObjectMsg(probe))throw std::runtime_error("Geometry probe insertion failed");
    auto collision=collisionDetails(*copy,state);bool hit=false;
    for(const auto& pair:collision["pairs"])
      if((pair["a"]==link->getName() && pair["b"]==probe.id) ||
         (pair["b"]==link->getName() && pair["a"]==probe.id)) hit=true;
    if(!hit)throw std::runtime_error("Collision geometry not active for link: "+link->getName()+" "+collision.dump());
    results[link->getName()]=true;
  }
  return results;
}

Json endpointDiagnostics(const planning_scene::PlanningScenePtr& scene,const Json& request) {
  const auto* arm=scene->getRobotModel()->getJointModelGroup("arm");
  Json output;
  for(const auto& field:{"pregrasp_position","hand_position"}) {
    Eigen::Isometry3d goal=Eigen::Isometry3d::Identity();
    for(int axis=0;axis<3;++axis)goal.translation()[axis]=request.at(field)[axis];
    const auto q=request.at("hand_orientation_xyzw");
    goal.linear()=Eigen::Quaterniond(q[3].get<double>(),q[0].get<double>(),q[1].get<double>(),q[2].get<double>()).toRotationMatrix();
    Json solutions=Json::array();bool collision_free=false;
    for(int seed=0;seed<16 && !collision_free;++seed) {
      auto state=scene->getCurrentState();
      if(seed) {
        std::vector<double> positions;
        state.copyJointGroupPositions(arm,positions);
        for(size_t i=0;i<positions.size();++i) positions[i]=2.4*std::sin((seed+1)*(i+1)*1.618);
        state.setJointGroupPositions(arm,positions);state.enforceBounds(arm);state.update();
      }
      if(!state.setFromIK(arm,goal,"base",.1))continue;
      state.update();auto check=collisionDetails(*scene,state);
      std::vector<double> positions;state.copyJointGroupPositions(arm,positions);
      const auto actual=state.getGlobalLinkTransform("base");
      check["position_error_m"]=(actual.translation()-goal.translation()).norm();
      check["orientation_error_rad"]=Eigen::Quaterniond(actual.rotation()).angularDistance(Eigen::Quaterniond(goal.rotation()));
      check["arm_positions_rad"]=positions;
      collision_free=!check["collision"].get<bool>() && check["satisfies_bounds"].get<bool>();
      solutions.push_back(check);
    }
    output[field]={{"collision_free_ik_found",collision_free},{"sampled_ik_solutions",solutions}};
  }
  return output;
}

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);int code=1;
  auto node=std::make_shared<rclcpp::Node>("observed_scene_audit",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  std::string output;Json report={{"mode","observed_scene_audit_failed"}};
  try {
    const auto dir=node->get_parameter("config_dir").as_string();
    output=node->get_parameter("output_path").as_string();
    const auto request=Json::parse(read(node->get_parameter("request_path").as_string()));
    node->declare_parameter("robot_description",read(dir+"/robot.urdf"));
    node->declare_parameter("robot_description_semantic",read(dir+"/robot.srdf"));
    robot_model_loader::RobotModelLoader loader(node,"robot_description",true);
    if(!loader.getModel())throw std::runtime_error("Robot model loading failed");
    auto scene=std::make_shared<planning_scene::PlanningScene>(loader.getModel());
    const auto baseline=YAML::LoadFile(dir+"/scene.json");
    report["scene"]=loadObservedWorld(*scene,baseline,request.at("observed_scene")).summary;
    setRequestState(scene->getCurrentStateNonConst(),request);
    report["initial_state"]=collisionDetails(*scene,scene->getCurrentState());
    report["intersection_separation_tests"]=fixtureTests(baseline,request.at("observed_scene"));
    report["robot_collision_geometry_participation"]=geometryParticipation(scene);
    report["endpoint_diagnostics"]=endpointDiagnostics(scene,request);
    report["candidate_id"]=request.at("observed_candidate_id");
    report["mode"]="observed_scene_audit_completed";
    std::ofstream file(output);if(!file)throw std::runtime_error("Cannot save observed scene audit");
    file<<report.dump(2)<<std::endl;
    std::cout<<"Observed scene audit saved: "<<output<<std::endl;
    const double seconds=node->has_parameter("publish_seconds")?node->get_parameter("publish_seconds").as_double():0.;
    if(seconds>0) {
      auto service=node->create_service<moveit_msgs::srv::GetPlanningScene>("get_planning_scene",
          [scene](const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request> request,
                  std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response> response) {
            scene->getPlanningSceneMsg(response->scene,request->components);
          });
      auto broadcaster=std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
      geometry_msgs::msg::TransformStamped frame;
      frame.header.frame_id=scene->getPlanningFrame();frame.child_frame_id="base_link";frame.header.stamp=node->now();
      const auto base=scene->getCurrentState().getGlobalLinkTransform("base_link");
      frame.transform.translation.x=base.translation().x();frame.transform.translation.y=base.translation().y();frame.transform.translation.z=base.translation().z();
      const Eigen::Quaterniond rotation(base.rotation());
      frame.transform.rotation.x=rotation.x();frame.transform.rotation.y=rotation.y();
      frame.transform.rotation.z=rotation.z();frame.transform.rotation.w=rotation.w();broadcaster->sendTransform(frame);
      auto publisher=node->create_publisher<moveit_msgs::msg::PlanningScene>("/i10_observed_planning_scene",rclcpp::QoS(1).transient_local());
      auto path_publisher=node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",rclcpp::QoS(1).transient_local());
      moveit_msgs::msg::DisplayTrajectory display;
      if(node->has_parameter("trajectory_path") && !node->get_parameter("trajectory_path").as_string().empty()) {
        const auto path=Json::parse(read(node->get_parameter("trajectory_path").as_string()));
        if(path.at("mode")!="mtc_geometric_approach_not_execution" || !path.at("dense_fcl_check_passed").get<bool>() ||
           path.at("request").at("observed_scene")!=request.at("observed_scene") ||
           path.at("request").at("observed_candidate_id")!=request.at("observed_candidate_id"))
          throw std::runtime_error("Display trajectory and observed scene request mismatch");
        display.model_id=scene->getRobotModel()->getName();
        moveit::core::robotStateToRobotStateMsg(scene->getCurrentState(),display.trajectory_start);
        moveit_msgs::msg::RobotTrajectory trajectory;
        trajectory.joint_trajectory.joint_names=request.at("arm_joints").get<std::vector<std::string>>();
        std::vector<std::vector<double>> points;
        for(const auto& stage:path.at("stages"))for(const auto& point:stage.at("points"))points.push_back(point.get<std::vector<double>>());
        const size_t stride=std::max<size_t>(1,points.size()/200);
        // 只为显示抽取已验证路点；固定 10 秒是动画时长，不是可执行的时间参数化。
        for(size_t i=0;i<points.size();++i)if(i%stride==0 || i+1==points.size()) {
          trajectory_msgs::msg::JointTrajectoryPoint point;point.positions=points[i];
          double time=10.*i/std::max<size_t>(1,points.size()-1);
          point.time_from_start.sec=int(time);point.time_from_start.nanosec=uint32_t((time-int(time))*1e9);
          trajectory.joint_trajectory.points.push_back(point);
        }
        display.trajectory.push_back(trajectory);
      }
      bool path_sent=false;
      moveit_msgs::msg::PlanningScene message;scene->getPlanningSceneMsg(message);
      message.name="Measured environment and independent fuel_cap";
      moveit_msgs::msg::ObjectColor color;color.id="fuel_cap";color.color.r=1.;color.color.a=1.;
      message.object_colors.push_back(color);
      auto started=std::chrono::steady_clock::now();
      while(rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<seconds) {
        publisher->publish(message);rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if(!path_sent && !display.trajectory.empty() && path_publisher->get_subscription_count()) {
          path_publisher->publish(display);path_sent=true;
        }
      }
    }
    code=0;
  }catch(const std::exception& error) {
    report["error"]=error.what();if(!output.empty()){std::ofstream file(output);file<<report.dump(2)<<std::endl;}
    RCLCPP_ERROR(node->get_logger(),"%s",error.what());
  }
  rclcpp::shutdown();return code;
}
