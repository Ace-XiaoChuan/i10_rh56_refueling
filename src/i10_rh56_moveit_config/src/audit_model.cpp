#include "scene_loader.hpp"
using Pair = std::pair<std::string,std::string>;
Pair sortedPair(std::string a,std::string b) {if(b<a)std::swap(a,b);return {a,b};}
YAML::Node pairList(const std::set<Pair>& pairs) {
  YAML::Node out(YAML::NodeType::Sequence);
  for(const auto& p:pairs) {YAML::Node v;v.push_back(p.first);v.push_back(p.second);out.push_back(v);}return out;
}

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("i10_rh56_model_audit",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try {
    const auto dir=node->get_parameter("config_dir").as_string();
    const auto output=node->get_parameter("output_path").as_string();
    const auto profile=node->get_parameter("coverage").as_string();
    if(profile!="source" && profile!="planning")throw std::runtime_error("coverage must be source or planning");
    const bool source=profile=="source";
    const std::string suffix=source?"_source_coverage":"";
    node->declare_parameter("robot_description",read(dir+"/robot"+suffix+".urdf"));
    node->declare_parameter("robot_description_semantic",read(dir+"/robot"+suffix+".srdf"));
    robot_model_loader::RobotModelLoader loader(node,"robot_description",true);
    auto model=loader.getModel();
    if(!model)throw std::runtime_error("RobotModel load failed");
    auto ref=YAML::LoadFile(dir+"/audit_reference.json");
    auto world=YAML::LoadFile(dir+"/scene.json");
    auto arm=model->getJointModelGroup("arm");
    if(!arm || arm->getActiveJointModelNames()!=ref["arm_joints"].as<std::vector<std::string>>())
      throw std::runtime_error("Arm group/order mismatch");
    if(model->getMimicJointModels().size()!=ref["mimic_count"].as<size_t>())throw std::runtime_error("Mimic count mismatch");
    if(!arm->getSolverInstance())throw std::runtime_error("KDL solver not loaded");
    for(const auto& entry:ref["joint_bounds"]) {
      const auto name=entry.first.as<std::string>();const auto& bound=model->getVariableBounds(name);
      if(!bound.position_bounded_ || std::abs(bound.min_position_-entry.second[0].as<double>())>1e-12 ||
         std::abs(bound.max_position_-entry.second[1].as<double>())>1e-12)throw std::runtime_error("Limit mismatch: "+name);
    }
    planning_scene::PlanningScene scene(model);loadWorld(scene,world,source);
    std::map<std::string,std::string> world_names;
    for(const auto& obj:world["objects"])world_names[obj["id"].as<std::string>()]=obj["source_body"].as<std::string>();
    YAML::Node report;report["mode"]="moveit_model_audit_not_execution";report["coverage"]=profile;
    report["arm_joints"]=arm->getActiveJointModelNames();report["mimic_count"]=model->getMimicJointModels().size();
    report["world_object_count"]=scene.getWorld()->size();
    double maximum_error=0;int mismatch_cases=0, ik_passes=0;
    for(const auto& sample:ref["cases"]) {
      moveit::core::RobotState state(model);state.setToDefaultValues();
      for(const auto& q:sample["joint_positions"])state.setVariablePosition(q.first.as<std::string>(),q.second.as<double>());
      state.update();
      double err=0;
      for(const auto& link:sample["links"]) {
        auto name=link.first.as<std::string>();
        if(!model->hasLinkModel(name))throw std::runtime_error("Missing link "+name);
        err=std::max(err,(state.getGlobalLinkTransform(name).matrix()-transform(link.second).matrix()).cwiseAbs().maxCoeff());
      }
      maximum_error=std::max(maximum_error,err);
      if(err>ref["fk_tolerance"].as<double>())throw std::runtime_error("FK mismatch: "+sample["name"].as<std::string>());
      if(!state.satisfiesBounds(1e-9))throw std::runtime_error("Reference sample violates bounds");
      collision_detection::CollisionRequest req;req.contacts=true;req.max_contacts=10000;req.max_contacts_per_pair=100;
      collision_detection::CollisionResult result;scene.checkCollision(req,result,state);
      YAML::Node row;row["name"]=sample["name"].as<std::string>();row["fk_max_abs_error"]=err;row["collision"]=result.collision;
      row["contacts"]=YAML::Node(YAML::NodeType::Sequence);
      std::set<Pair> actual,expected;
      const double threshold=ref["penetration_diagnostic_m"].as<double>();
      for(const auto& contacts:result.contacts) {
        double depth=0;
        for(const auto& c:contacts.second)depth=std::max(depth,c.depth);
        YAML::Node contact;contact["a"]=contacts.first.first;contact["b"]=contacts.first.second;contact["max_depth_m"]=depth;
        row["contacts"].push_back(contact);
        if(depth>threshold) {
          auto a=contacts.first.first,b=contacts.first.second;
          if(world_names.count(a))a=world_names.at(a);
          if(world_names.count(b))b=world_names.at(b);
          actual.insert(sortedPair(a,b));
        }
      }
      for(const auto& contact:sample["mujoco_contacts"])if(contact["depth_m"].as<double>()>threshold)
        expected.insert(sortedPair(contact["pair"][0].as<std::string>(),contact["pair"][1].as<std::string>()));
      row["moveit_diagnostic_pairs"]=pairList(actual);row["mujoco_diagnostic_pairs"]=pairList(expected);
      row["source_pair_match"]=actual==expected;
      if(actual!=expected)++mismatch_cases;
      // Exercise the plugin from a nearby (not exact solution) seed.
      auto target=state.getGlobalLinkTransform("base");
      moveit::core::RobotState solved(state);std::vector<double> q;solved.copyJointGroupPositions(arm,q);
      for(auto& v:q)v+=0.01;
      solved.setJointGroupPositions(arm,q);solved.enforceBounds(arm);solved.update();
      const bool found=solved.setFromIK(arm,target,"base",0.05);
      double ik_error=found?(solved.getGlobalLinkTransform("base").matrix()-target.matrix()).cwiseAbs().maxCoeff():-1;
      row["kdl_known_target_found"]=found;row["kdl_target_max_abs_error"]=ik_error;
      if(found && ik_error<1e-4)++ik_passes;
      report["cases"].push_back(row);
    }
    // Verify the newly enabled geometry really participates in world collision.
    if(!source) {
      auto& state=scene.getCurrentStateNonConst();state.setToDefaultValues();
      for(const auto& q:ref["cases"][1]["joint_positions"])
        state.setVariablePosition(q.first.as<std::string>(),q.second.as<double>());
      state.update();
      for(const std::string link:{"sensor_link","adapter"}) {
        moveit_msgs::msg::CollisionObject obstacle;
        obstacle.header.frame_id="world";obstacle.id="audit_probe";obstacle.operation=obstacle.ADD;
        shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={0.08,0.08,0.08};
        geometry_msgs::msg::Pose at;at.orientation.w=1;
        auto center=state.getGlobalLinkTransform(link).translation();
        at.position.x=center.x();at.position.y=center.y();at.position.z=center.z();
        obstacle.primitives.push_back(box);obstacle.primitive_poses.push_back(at);
        if(!scene.processCollisionObjectMsg(obstacle))throw std::runtime_error("Probe insertion failed");
        collision_detection::CollisionRequest request;request.contacts=true;request.max_contacts=10000;
        collision_detection::CollisionResult result;scene.checkCollision(request,result,state);
        bool detected=false;
        for(const auto& pair:result.contacts)
          if(sortedPair(pair.first.first,pair.first.second)==sortedPair(link,"audit_probe"))detected=true;
        if(!detected)throw std::runtime_error("Added geometry does not detect probe: "+link);
        report["added_geometry_obstacle_tests"][link]=true;
        obstacle.operation=obstacle.REMOVE;scene.processCollisionObjectMsg(obstacle);
      }
    }
    // A deliberately invalid state must be rejected by real MoveIt bounds.
    moveit::core::RobotState invalid(model);invalid.setToDefaultValues();
    auto first=arm->getActiveJointModelNames().front();
    invalid.setVariablePosition(first,model->getVariableBounds(first).max_position_+0.1);
    if(invalid.satisfiesBounds(arm))throw std::runtime_error("Invalid joint position accepted");
    if(ik_passes==0)throw std::runtime_error("KDL could not recover any known target");
    report["fk_max_abs_error"]=maximum_error;report["fk_pass"]=true;report["bounds_and_mimic_pass"]=true;
    report["kdl_recovered_cases"]=ik_passes;report["source_pair_mismatch_cases"]=mismatch_cases;
    report["path_verified"]=false;report["dynamics_verified"]=false;
    report["sources"]=YAML::LoadFile(dir+"/provenance.json")["sources"];
    report["generated_files"]=YAML::LoadFile(dir+"/generated_hashes.json");
    std::ofstream out(output);if(!out)throw std::runtime_error("Cannot write report");out<<report<<"\n";
    std::cout<<"AUDIT "<<profile<<": "<<ref["cases"].size()<<" FK cases, max error "<<maximum_error
             <<", KDL recovered "<<ik_passes<<", collision pair differences "<<mismatch_cases<<"\n";
  }catch(const std::exception& error) {std::cerr<<"AUDIT FAILED: "<<error.what()<<"\n";rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
