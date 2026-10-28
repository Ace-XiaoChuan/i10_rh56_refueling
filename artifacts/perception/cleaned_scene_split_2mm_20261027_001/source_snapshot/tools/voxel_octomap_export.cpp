// 使用本机 OctoMap 库保存已有占据格；输入是整数 XYZ 格号与米制分辨率。
// 地图局部原点在原体素网格原点，camera_from_octomap 由 Python 另行记录。
#include <octomap/OcTree.h>
#include <nlohmann/json.hpp>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>

using Json = nlohmann::json;
using Index = std::array<int, 3>;

// 展开剪枝后的叶子并核对全部格号，既检查漏格，也检查意外填充与空闲格。
Json verify(const octomap::OcTree& tree, const std::set<Index>& expected) {
  std::set<Index> actual;
  std::size_t leaves = 0;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (!tree.isNodeOccupied(*it))
      throw std::runtime_error("Unexpected free leaf");
    ++leaves;
    const unsigned difference = tree.getTreeDepth() - it.getDepth();
    if (difference > 20) throw std::runtime_error("Unexpected oversized leaf");
    const int span = 1 << difference;
    if (std::uint64_t(span)*span*span > expected.size())
      throw std::runtime_error("Leaf covers more cells than input");
    const Index lower = {
        int(std::llround(it.getX()/tree.getResolution() - span/2.0)),
        int(std::llround(it.getY()/tree.getResolution() - span/2.0)),
        int(std::llround(it.getZ()/tree.getResolution() - span/2.0))};
    for (int x = 0; x < span; ++x)
      for (int y = 0; y < span; ++y)
        for (int z = 0; z < span; ++z) {
          Index cell = {lower[0]+x, lower[1]+y, lower[2]+z};
          if (!expected.count(cell)) throw std::runtime_error("Extra occupied cell");
          actual.insert(cell);
        }
  }
  if (actual != expected) throw std::runtime_error("Occupied cell set changed");
  return {{"occupied_leaf_count", leaves}, {"occupied_base_cell_count", actual.size()},
          {"free_leaf_count", 0}, {"exact_occupied_cell_set", true}};
}

int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::runtime_error("Usage: voxel_octomap_export input.json output_dir");
    const std::filesystem::path output(argv[2]);
    for (const auto& name : {"scene.bt", "octomap_validation.json"})
      if (std::filesystem::exists(output/name)) throw std::runtime_error("Output already exists");
    std::ifstream stream(argv[1]);
    if (!stream) throw std::runtime_error("Cannot read input");
    const auto input = Json::parse(stream);
    const double resolution = input.at("voxel_size_m");
    if (!std::isfinite(resolution) || resolution <= 0)
      throw std::runtime_error("Resolution must be finite and positive");
    octomap::OcTree tree(resolution);
    const int offset = 1 << (tree.getTreeDepth()-1);
    std::set<Index> expected;
    for (const auto& row : input.at("indices_xyz")) {
      if (!row.is_array() || row.size() != 3) throw std::runtime_error("Invalid index shape");
      Index index;
      octomap::OcTreeKey key;
      for (int axis = 0; axis < 3; ++axis) {
        if (!row[axis].is_number_integer()) throw std::runtime_error("Noninteger index");
        const auto value = row[axis].get<std::int64_t>();
        if (value < -offset || value >= offset) throw std::runtime_error("Index outside OctoMap key range");
        index[axis] = int(value);
        key[axis] = octomap::key_type(value+offset);
      }
      if (!expected.insert(index).second) throw std::runtime_error("Duplicate index");
      // 直接设置已观测占据格，不调用 insertPointCloud，避免射线把未知空间改为空闲。
      if (!tree.setNodeValue(key, tree.getClampingThresMaxLog(), true))
        throw std::runtime_error("OctoMap insertion failed");
    }
    if (expected.empty()) throw std::runtime_error("Empty occupied grid");
    tree.updateInnerOccupancy();
    if (!tree.writeBinary((output/"scene.bt").string()))
      throw std::runtime_error("OctoMap write failed");
    // .bt 写出时会无损合并全占据子节点；读取后展开核对原分辨率的完整格号集合。
    octomap::OcTree restored(resolution);
    if (!restored.readBinary((output/"scene.bt").string()) || restored.getResolution() != resolution)
      throw std::runtime_error("OctoMap read failed or resolution changed");
    auto report = verify(restored, expected);
    report["resolution_m"] = resolution;
    report["tree_type"] = restored.getTreeType();
    report["roundtrip_from_disk"] = true;
    report["raycasting_performed"] = false;
    std::ofstream result(output/"octomap_validation.json");
    result << report.dump(2) << '\n';
    if (!result) throw std::runtime_error("Cannot write validation");
    std::cout << report.dump() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
