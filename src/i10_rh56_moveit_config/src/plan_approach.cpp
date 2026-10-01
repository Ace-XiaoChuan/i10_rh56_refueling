// P1 geometric MTC approach, no execution or hardware timing.
#include "scene_loader.hpp"
#include <nlohmann/json.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
namespace mtc=moveit::task_constructor;
using Json=nlohmann::json;
void collect(const mtc::SolutionBase& solution, Json& stages, const planning_scene::PlanningScenePtr& scene,
             const moveit::core::JointModelGroup* arm) {
  if(auto seq=dynamic_cast<const mtc::SolutionSequence*>(&solution)) {
    for(auto s:seq->solutions())collect(*s,stages,scene,arm);
    return;
  }
  auto sub=dynamic_cast<const mtc::SubTrajectory*>(&solution);
  if(!sub || !sub->trajectory() || !sub->trajectory()->getWayPointCount())return;
  auto traj=sub->trajectory(); Json stage;stage["name"]=solution.creator()->name();stage["points"]=Json::array();
  std::vector<double> previous;
  for(size_t i=0;i<traj->getWayPointCount();++i) {
    auto state=traj->getWayPoint(i);std::vector<double> q;state.copyJointGroupPositions(arm,q);
    int steps=1;
    if(!previous.empty())for(size_t k=0;k<q.size();++k)steps=std::max(steps,int(std::ceil(std::abs(q[k]-previous[k])/.02)));
    for(int step=1;step<=steps;++step) {
      auto dense=q;
      if(!previous.empty())for(size_t k=0;k<q.size();++k)dense[k]=previous[k]+(q[k]-previous[k])*step/steps;
      state.setJointGroupPositions(arm,dense);state.update();
      if(!state.satisfiesBounds(1e-8) || scene->isStateColliding(state,"",false))throw std::runtime_error("Dense trajectory validation failed");
      stage["points"].push_back(dense);
    }
    previous=q;
  }
  stages.push_back(stage);
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);int code=1;
  auto node=std::make_shared<rclcpp::Node>("i10_mtc_approach",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try {
    const auto dir=node->get_parameter("config_dir").as_string();
    const auto output=node->get_parameter("output_path").as_string();
    auto request=Json::parse(read(node->get_parameter("request_path").as_string()));
    node->declare_parameter("robot_description",read(dir+"/robot.urdf"));
    node->declare_parameter("robot_description_semantic",read(dir+"/robot.srdf"));
    robot_model_loader::RobotModelLoader loader(node,"robot_description",true);auto model=loader.getModel();
    auto scene=std::make_shared<planning_scene::PlanningScene>(model);loadWorld(*scene,YAML::LoadFile(dir+"/scene.json"),false);
    auto arm=model->getJointModelGroup("arm");
    if(arm->getActiveJointModelNames()!=request["arm_joints"].get<std::vector<std::string>>())throw std::runtime_error("Joint order mismatch");
    auto& state=scene->getCurrentStateNonConst();state.setToDefaultValues();
    for(auto it=request["initial_joint_positions"].begin();it!=request["initial_joint_positions"].end();++it)state.setVariablePosition(it.key(),it.value().get<double>());
    state.update();
    if(!state.satisfiesBounds(1e-8) || scene->isStateColliding(state,"",true))throw std::runtime_error("Exact initial state invalid; stopping");
    mtc::Task task("",false);task.setRobotModel(model);task.setName("i10 RH56 approach");task.setTimeout(30.);
    auto fixed=std::make_unique<mtc::stages::FixedState>("confirmed initial state");fixed->setState(scene);task.add(std::move(fixed));
    auto pipeline=std::make_shared<mtc::solvers::PipelinePlanner>(node);pipeline->setPlannerId("RRTConnectkConfigDefault");pipeline->setTimeParameterization(nullptr);
    auto move=std::make_unique<mtc::stages::MoveTo>("move to pregrasp",pipeline);move->setGroup("arm");move->setIKFrame("base");move->setTimeout(15.);
    geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="world";
    auto p=request["pregrasp_position"],q=request["hand_orientation_xyzw"];
    goal.pose.position.x=p[0];goal.pose.position.y=p[1];goal.pose.position.z=p[2];
    goal.pose.orientation.x=q[0];goal.pose.orientation.y=q[1];goal.pose.orientation.z=q[2];goal.pose.orientation.w=q[3];move->setGoal(goal);task.add(std::move(move));
    auto cart=std::make_shared<mtc::solvers::CartesianPath>();cart->setStepSize(.002);cart->setMinFraction(1.);cart->setJumpThreshold(1.5);cart->setTimeParameterization(nullptr);
    auto approach=std::make_unique<mtc::stages::MoveRelative>("axial approach",cart);approach->setGroup("arm");approach->setIKFrame("base");
    geometry_msgs::msg::Vector3Stamped direction;direction.header.frame_id="world";auto axis=request["outward_axis_world"];
    direction.vector.x=-axis[0].get<double>();direction.vector.y=-axis[1].get<double>();direction.vector.z=-axis[2].get<double>();approach->setDirection(direction);
    double distance=request["approach_distance_m"];approach->setMinMaxDistance(distance,distance);task.add(std::move(approach));
    task.init();auto result=task.plan(1);task.printState();
    if(!result || task.solutions().empty()){task.explainFailure();throw std::runtime_error("MTC found no complete approach solution");}
    Json report;report["schema_version"]=1;report["mode"]="mtc_geometric_approach_not_execution";report["request"]=request;report["stages"]=Json::array();
    collect(**task.solutions().begin(),report["stages"],scene,arm);
    if(report["stages"].size()!=2)throw std::runtime_error("Expected two motion stages");
    const auto endpoint=report["stages"].back()["points"].back().get<std::vector<double>>();
    auto final=state;final.setJointGroupPositions(arm,endpoint);final.update();
    Eigen::Vector3d target;for(int i=0;i<3;++i)target[i]=request["hand_position"][i];
    auto actual=final.getGlobalLinkTransform("base");double error=(actual.translation()-target).norm();
    Eigen::Quaterniond wanted(q[3].get<double>(),q[0].get<double>(),q[1].get<double>(),q[2].get<double>());
    double rotation=Eigen::Quaterniond(actual.rotation()).angularDistance(wanted);
    if(error>1e-4 || rotation>0.002)throw std::runtime_error("Endpoint pose mismatch");
    report["endpoint_position_error_m"]=error;report["endpoint_orientation_error_rad"]=rotation;report["dense_fcl_check_passed"]=true;
    std::ofstream file(output);if(!file)throw std::runtime_error("Cannot open output");file<<report.dump(2)<<std::endl;
    code=0;
  } catch(const std::exception& e){RCLCPP_ERROR(node->get_logger(),"%s",e.what());}
  rclcpp::shutdown();return code;
}
