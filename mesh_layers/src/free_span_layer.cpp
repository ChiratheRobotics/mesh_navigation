/*
 *  FreeSpanLayer — free-space-span traversability (SE(2) NavMesh style, no yaw).
 *  See free_span_layer.h for the idea. Math:
 *    column (cx,cy) = (floor(x/s_xy), floor(y/s_xy)); z-bin = floor((z-zmin)/s_z)
 *    solid span = run of occupied bins (merging gaps <= merge_gap)
 *    walking surface = top of a solid span; headroom = gap to next solid span
 *    walkable  <=> headroom >= robot_height
 *    edge(s,s') <=> |z_surf(s)-z_surf(s')| <= step_height  (4-neighbour columns)
 *    traversable <=> walkable surface in the largest connected component
 */
#include "mesh_layers/free_span_layer.h"
#include <mesh_map/mesh_map.h>

#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <functional>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mesh_layers::FreeSpanLayer, mesh_map::AbstractLayer)

namespace mesh_layers
{
// pack/unpack a (cx,cy) column into one int64 (assumes |c| < 2^20 cells)
static inline int64_t packKey(int cx, int cy)
{
  return (static_cast<int64_t>(cx + (1 << 20)) << 21) | static_cast<int64_t>(cy + (1 << 20));
}

bool FreeSpanLayer::readLayer()
{
  const auto map = map_ptr_.lock();
  auto opt = map->meshIO()->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(layer_name_);
  if (opt)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Read free_span traversability from map file.");
    costs_ = opt.get();
    return computeLethals();
  }
  return false;
}

bool FreeSpanLayer::writeLayer()
{
  const auto map = map_ptr_.lock();
  if (map->meshIO()->addDenseAttributeMap(costs_, layer_name_))
  {
    RCLCPP_INFO_STREAM(get_logger(), "Saved free_span traversability to map file.");
    return true;
  }
  RCLCPP_ERROR_STREAM(get_logger(), "Could not save free_span to map file!");
  return false;
}

float FreeSpanLayer::threshold()
{
  return config_.threshold;
}

bool FreeSpanLayer::computeLethals()
{
  lethal_vertices_.clear();
  for (auto vH : costs_)
  {
    if (costs_[vH] > config_.threshold)
      lethal_vertices_.insert(vH);
  }
  RCLCPP_INFO_STREAM(get_logger(), "FreeSpanLayer: " << lethal_vertices_.size() << " lethal vertices ("
                                   << costs_.numValues() << " total).");
  return true;
}

const lvr2::VertexMap<float>& FreeSpanLayer::costs()
{
  return costs_;
}

bool FreeSpanLayer::computeLayer()
{
  RCLCPP_INFO_STREAM(get_logger(), "FreeSpanLayer: computing free-space-span traversability (s_xy="
      << config_.cell_size << " s_z=" << config_.voxel_height << " robot_h=" << config_.robot_height
      << " step=" << config_.step_height << ")");
  const auto map = map_ptr_.lock();
  const auto mesh = map->mesh();
  const double sxy = config_.cell_size, sz = config_.voxel_height;
  const int merge_gap = std::max(0, config_.merge_gap);

  // 1) gather vertices + zmin
  std::vector<lvr2::VertexHandle> vhs;
  std::vector<float> vx, vy, vz;
  vhs.reserve(mesh->numVertices());
  double zmin = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < mesh->nextVertexIndex(); i++)
  {
    auto vH = lvr2::VertexHandle(i);
    if (!mesh->containsVertex(vH)) continue;
    const auto p = mesh->getVertexPosition(vH);
    vhs.push_back(vH); vx.push_back(p.x); vy.push_back(p.y); vz.push_back(p.z);
    if (p.z < zmin) zmin = p.z;
  }
  const size_t N = vhs.size();

  // 2) occupied z-bins per column
  std::unordered_map<int64_t, std::set<int>> col_bins;
  col_bins.reserve(N / 4 + 16);
  for (size_t i = 0; i < N; i++)
  {
    int cx = (int)std::floor(vx[i] / sxy);
    int cy = (int)std::floor(vy[i] / sxy);
    int b  = (int)std::floor((vz[i] - zmin) / sz);
    col_bins[packKey(cx, cy)].insert(b);
  }

  // 3) walkable surfaces per column (top of solid span with enough headroom)
  std::vector<float> surf_z;          // height of each walkable surface
  std::vector<int64_t> surf_key;      // its column
  std::unordered_map<int64_t, std::vector<int>> col_surfs;  // key -> surface ids
  const double robot_h = config_.robot_height;
  for (const auto& kv : col_bins)
  {
    const auto& s = kv.second;
    std::vector<int> bins(s.begin(), s.end());  // sorted
    // merge into solid spans [lo,hi]
    std::vector<std::pair<int,int>> spans;
    int lo = bins[0], hi = bins[0];
    for (size_t j = 1; j < bins.size(); j++)
    {
      if (bins[j] - hi <= merge_gap + 1) hi = bins[j];
      else { spans.emplace_back(lo, hi); lo = hi = bins[j]; }
    }
    spans.emplace_back(lo, hi);
    // Group spans into CLIMB-RUNS: a run is a maximal sequence of consecutive
    // spans whose vertical gap is <= step_height. These are stairs the robot
    // CLIMBS, not a floor+ceiling: a span above the surface is a true overhead
    // obstacle ("ceiling") only if it is MORE than step_height up; otherwise it
    // is simply the next step. So headroom for every tread in a run is the
    // clearance from the RUN TOP to the first span above the run. Crucially we
    // still register EACH tread in the run as its own walkable surface, so the
    // union-find connects adjacent columns tread->tread by step_height up the
    // flight -- instead of merging the whole flight into one surface (which would
    // then sit >step_height above the lower floor and disconnect it).
    const int step_bins = std::max(1, (int)std::ceil(config_.step_height / sz));
    size_t k = 0;
    while (k < spans.size())
    {
      size_t e = k;  // extend the run while the next span is a climbable step
      while (e + 1 < spans.size() && (spans[e+1].first - spans[e].second) <= step_bins) e++;
      double headroom = (e + 1 < spans.size()) ? (spans[e+1].first - spans[e].second) * sz : 1e3;
      if (headroom >= robot_h)
      {
        for (size_t m = k; m <= e; m++)
        {
          double z_surf = spans[m].second * sz + zmin;
          int id = (int)surf_z.size();
          surf_z.push_back((float)z_surf); surf_key.push_back(kv.first);
          col_surfs[kv.first].push_back(id);
        }
      }
      k = e + 1;
    }
  }
  const size_t M = surf_z.size();

  // 4) union-find: connect adjacent-column surfaces with step <= step_height
  std::vector<int> par(M);
  for (size_t i = 0; i < M; i++) par[i] = (int)i;
  std::function<int(int)> find = [&](int a){ while (par[a]!=a){ par[a]=par[par[a]]; a=par[a]; } return a; };
  const double step = config_.step_height;
  const int dxs[4] = {1,0,-1,0}, dys[4] = {0,1,0,-1};
  for (const auto& kv : col_surfs)
  {
    int cx = (int)((kv.first >> 21) - (1 << 20));
    int cy = (int)((kv.first & ((1 << 21) - 1)) - (1 << 20));
    for (int d = 0; d < 4; d++)
    {
      auto it = col_surfs.find(packKey(cx + dxs[d], cy + dys[d]));
      if (it == col_surfs.end()) continue;
      for (int s : kv.second)
        for (int t : it->second)
          if (std::fabs(surf_z[s] - surf_z[t]) <= step)
          { int ra = find(s), rb = find(t); if (ra != rb) par[rb] = ra; }
    }
  }

  // 5) largest component = main traversable region; floor-connectivity diagnostic
  std::unordered_map<int,int> comp_size;
  for (size_t i = 0; i < M; i++) comp_size[find((int)i)]++;
  int main_root = -1, best = -1;
  for (auto& c : comp_size) if (c.second > best) { best = c.second; main_root = c.first; }
  bool f1 = false, f2 = false;
  for (size_t i = 0; i < M; i++) if (find((int)i) == main_root)
  { if (surf_z[i] > -0.6 && surf_z[i] < 0.1) f1 = true; if (surf_z[i] > 1.9 && surf_z[i] < 2.6) f2 = true; }
  RCLCPP_INFO_STREAM(get_logger(), "FreeSpanLayer: " << M << " walkable surfaces, " << comp_size.size()
      << " components, main=" << best << " cells. floor1+floor2 in main (stairs connected): "
      << ((f1 && f2) ? "YES" : "NO"));

  // 5b) lateral-clearance SOFT cost on the main component (inflation on the cell
  // graph, not the mesh): adjacency among main-component surfaces; a surface is a
  // boundary if it lacks a walkable 4-neighbour; multi-source BFS from boundaries
  // gives distance-to-edge -> standard inflation cost (center cheap, edges high).
  std::unordered_map<int64_t, std::vector<int>> main_col;
  for (size_t i = 0; i < M; i++) if (find((int)i) == main_root) main_col[surf_key[i]].push_back((int)i);
  std::vector<std::vector<int>> adjm(M);
  std::vector<char> is_bnd(M, 0);
  for (const auto& kv : main_col)
  {
    int cx = (int)((kv.first >> 21) - (1 << 20));
    int cy = (int)((kv.first & ((1 << 21) - 1)) - (1 << 20));
    for (int sid : kv.second)
    {
      float zs = surf_z[sid];
      for (int d = 0; d < 4; d++)
      {
        auto it = main_col.find(packKey(cx + dxs[d], cy + dys[d]));
        bool found = false;
        if (it != main_col.end())
          for (int t : it->second) if (std::fabs(surf_z[t] - zs) <= step) { adjm[sid].push_back(t); found = true; }
        if (!found) is_bnd[sid] = 1;
      }
    }
  }
  std::vector<int> hops(M, -1);
  std::queue<int> bfsq;
  for (const auto& kv : main_col) for (int sid : kv.second) if (is_bnd[sid]) { hops[sid] = 0; bfsq.push(sid); }
  while (!bfsq.empty())
  {
    int u = bfsq.front(); bfsq.pop();
    for (int v : adjm[u]) if (hops[v] < 0) { hops[v] = hops[u] + 1; bfsq.push(v); }
  }
  const double insc = config_.inscribed_radius, infl = config_.inflation_radius;
  const double scal = config_.cost_scaling, iv = config_.inscribed_value;
  std::unordered_map<int64_t, std::vector<std::pair<float,float>>> main_surf;  // key -> (z, cost)
  for (const auto& kv : main_col) for (int sid : kv.second)
  {
    double clr = (hops[sid] < 0) ? infl : hops[sid] * sxy;  // distance-to-edge (m)
    double cost = (clr < insc) ? iv : (clr < infl ? iv * std::exp(-scal * (clr - insc)) : 0.0);
    main_surf[surf_key[sid]].push_back({surf_z[sid], (float)cost});
  }

  // 6) per-vertex cost: nearest main-component surface's soft cost; else lethal(1)
  costs_.clear(); costs_.reserve(mesh->nextVertexIndex());
  const double tol = config_.connect_tol;
  size_t trav = 0;
  for (size_t i = 0; i < N; i++)
  {
    int cx = (int)std::floor(vx[i] / sxy);
    int cy = (int)std::floor(vy[i] / sxy);
    float c = 1.0f;  // lethal (wall / low-headroom / isolated)
    auto it = main_surf.find(packKey(cx, cy));
    if (it != main_surf.end())
    {
      float bestdz = 1e9f;
      for (const auto& zc : it->second)
      {
        float dz = std::fabs(zc.first - (float)vz[i]);
        if (dz <= tol && dz < bestdz) { bestdz = dz; c = zc.second; }
      }
      if (c < 1.0f) trav++;
    }
    costs_.insert(vhs[i], c);
  }
  RCLCPP_INFO_STREAM(get_logger(), "FreeSpanLayer: traversable vertices "
      << (100.0 * trav / std::max<size_t>(1,N)) << "% (soft clearance gradient applied)");

  return computeLethals();
}

rcl_interfaces::msg::SetParametersResult FreeSpanLayer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  const std::string pfx = mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".";
  for (auto p : parameters)
  {
    if (p.get_name() == pfx + "robot_height") config_.robot_height = p.as_double();
    else if (p.get_name() == pfx + "step_height") config_.step_height = p.as_double();
    else if (p.get_name() == pfx + "cell_size") config_.cell_size = p.as_double();
    else if (p.get_name() == pfx + "threshold") config_.threshold = p.as_double();
  }
  return result;
}

bool FreeSpanLayer::initialize()
{
  const std::string pfx = mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".";
  config_.cell_size    = node_->declare_parameter(pfx + "cell_size",    config_.cell_size);
  config_.voxel_height = node_->declare_parameter(pfx + "voxel_height", config_.voxel_height);
  config_.robot_height = node_->declare_parameter(pfx + "robot_height", config_.robot_height);
  config_.step_height  = node_->declare_parameter(pfx + "step_height",  config_.step_height);
  config_.merge_gap    = node_->declare_parameter(pfx + "merge_gap",    config_.merge_gap);
  config_.connect_tol  = node_->declare_parameter(pfx + "connect_tol",  config_.connect_tol);
  config_.inscribed_radius = node_->declare_parameter(pfx + "inscribed_radius", config_.inscribed_radius);
  config_.inflation_radius = node_->declare_parameter(pfx + "inflation_radius", config_.inflation_radius);
  config_.cost_scaling     = node_->declare_parameter(pfx + "cost_scaling",     config_.cost_scaling);
  config_.inscribed_value  = node_->declare_parameter(pfx + "inscribed_value",  config_.inscribed_value);
  config_.threshold    = node_->declare_parameter(pfx + "threshold",    config_.threshold);
  dyn_params_handler_ = node_->add_on_set_parameters_callback(
      std::bind(&FreeSpanLayer::reconfigureCallback, this, std::placeholders::_1));
  return true;
}

} /* namespace mesh_layers */
