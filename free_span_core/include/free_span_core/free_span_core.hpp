/*
 *  free_span_core — shared free-space-span cell graph (SE(2) NavMesh style, no yaw).
 *
 *  Header-only, dependency-free (plain float arrays in, graph out). Used by BOTH
 *  mesh_layers/FreeSpanLayer (per-vertex costs) and se2_mesh_planner (A* on the
 *  cell graph). The construction mirrors the layer exactly:
 *    column (cx,cy) = (floor(x/s_xy), floor(y/s_xy)); z-bin = floor((z-zmin)/s_z)
 *    solid span = run of occupied bins (merge gaps <= merge_gap)
 *    walking surface = top of a solid span with headroom >= robot_height
 *    edge(s,s') <=> 4-neighbour columns AND |z_surf(s)-z_surf(s')| <= step_height
 *    main component = largest union-find component (floor1<->floor2 live here)
 *    clearance(s)  = s_xy * (BFS hops from the main-component boundary)
 */
#ifndef FREE_SPAN_CORE__FREE_SPAN_CORE_HPP
#define FREE_SPAN_CORE__FREE_SPAN_CORE_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

namespace free_span_core
{

struct Params
{
  double cell_size = 0.10;    // s_xy horizontal column size (m)
  double voxel_height = 0.05; // s_z vertical voxel size (m)
  double robot_height = 0.50; // required headroom (m)
  double step_height = 0.20;  // max climbable step between adjacent columns (m)
  int    merge_gap = 1;       // merge solid bins separated by <= this many empties
  int    fill_iters = 2;      // morphological-close passes to fill walkable holes (stairs -> solid)
};

// Cost shaping for the A* search (centering gradient on the cell graph).
struct PlanParams
{
  double inscribed_radius = 0.15; // r_in: full centering penalty at/under this clearance (m)
  double inflation_radius = 0.50; // r_infl: penalty decays to 0 by this clearance (m)
  double alpha = 4.0;             // centering weight (0 = shortest path, large = hug the centre)
};

struct Node
{
  int cx = 0, cy = 0;     // column index
  float z = 0.0f;         // walking-surface height (m)
  float clearance = 0.0f; // lateral distance to the main-component boundary (m)
  int comp = -1;          // union-find component root
};

struct Graph
{
  std::vector<Node> nodes;
  std::vector<std::vector<int>> adj;              // 4-neighbour edges (step<=h_step); for clearance/union-find
  std::vector<std::vector<int>> move_adj;         // 8-neighbour no-corner-cut edges; for planner A*
  std::unordered_map<int64_t, std::vector<int>> col_nodes; // column key -> node ids
  int main_comp = -1;
  double cell_size = 0.10;

  static inline int64_t key(int cx, int cy)
  {
    return (static_cast<int64_t>(cx + (1 << 20)) << 21) | static_cast<int64_t>(cy + (1 << 20));
  }
  inline double nodeX(int id) const { return (nodes[id].cx + 0.5) * cell_size; }
  inline double nodeY(int id) const { return (nodes[id].cy + 0.5) * cell_size; }
  inline double nodeZ(int id) const { return nodes[id].z; }

  // nearest node in a column whose height is within `band` of z (optionally in a component); -1 if none
  int nodeInColumn(int cx, int cy, double z, double band, int comp_filter) const
  {
    auto it = col_nodes.find(key(cx, cy));
    if (it == col_nodes.end()) return -1;
    int best = -1; double bd = band;
    for (int id : it->second)
    {
      if (comp_filter >= 0 && nodes[id].comp != comp_filter) continue;
      double dz = std::fabs(nodes[id].z - z);
      if (dz <= bd) { bd = dz; best = id; }
    }
    return best;
  }

  // Snap a world (x,y,z) to the nearest main-component node (horiz dist + |dz|); -1 if none nearby.
  int nearestMain(double x, double y, double z, int win = 3) const
  {
    int cx = static_cast<int>(std::floor(x / cell_size));
    int cy = static_cast<int>(std::floor(y / cell_size));
    int best = -1; double bestd = std::numeric_limits<double>::infinity();
    for (int dx = -win; dx <= win; dx++)
      for (int dy = -win; dy <= win; dy++)
      {
        auto it = col_nodes.find(key(cx + dx, cy + dy));
        if (it == col_nodes.end()) continue;
        for (int id : it->second)
        {
          if (nodes[id].comp != main_comp) continue;
          double ex = nodeX(id) - x, ey = nodeY(id) - y, ez = nodes[id].z - z;
          double d = std::sqrt(ex * ex + ey * ey) + std::fabs(ez);
          if (d < bestd) { bestd = d; best = id; }
        }
      }
    return best;
  }
};

// Build the free-span cell graph from plain vertex arrays.
inline Graph build(const std::vector<float>& vx, const std::vector<float>& vy,
                   const std::vector<float>& vz, const Params& p)
{
  Graph g; g.cell_size = p.cell_size;
  const double sxy = p.cell_size, sz = p.voxel_height;
  const int merge_gap = std::max(0, p.merge_gap);
  const size_t N = vx.size();
  if (N == 0) return g;

  double zmin = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < N; i++) if (vz[i] < zmin) zmin = vz[i];

  // occupied z-bins per column
  std::unordered_map<int64_t, std::set<int>> col_bins;
  col_bins.reserve(N / 4 + 16);
  for (size_t i = 0; i < N; i++)
  {
    int cx = (int)std::floor(vx[i] / sxy);
    int cy = (int)std::floor(vy[i] / sxy);
    int b  = (int)std::floor((vz[i] - zmin) / sz);
    col_bins[Graph::key(cx, cy)].insert(b);
  }

  // walkable surfaces per column (top of a solid span with headroom >= robot_height)
  std::vector<int64_t> surf_key;
  std::unordered_map<int64_t, std::vector<int>> col_surfs;
  const double robot_h = p.robot_height;
  auto unkey = [](int64_t k, int& cx, int& cy) {
    cx = (int)((k >> 21) - (1 << 20));
    cy = (int)((k & ((1 << 21) - 1)) - (1 << 20));
  };
  for (const auto& kv : col_bins)
  {
    std::vector<int> bins(kv.second.begin(), kv.second.end());  // sorted
    std::vector<std::pair<int,int>> spans;
    int lo = bins[0], hi = bins[0];
    for (size_t j = 1; j < bins.size(); j++)
    {
      if (bins[j] - hi <= merge_gap + 1) hi = bins[j];
      else { spans.emplace_back(lo, hi); lo = hi = bins[j]; }
    }
    spans.emplace_back(lo, hi);
    int cx, cy; unkey(kv.first, cx, cy);
    // CLIMB-RUNS: group consecutive spans whose vertical gap is <= step_height
    // (stairs the robot climbs, not floor+ceiling). A span above the surface is
    // a true overhead obstacle only if it is MORE than step_height up; within
    // step_height it is the next step. Headroom for every tread in a run is the
    // clearance from the RUN TOP to the first span above the run; each tread is
    // still registered as its own node so adjacent columns connect tread->tread
    // up the flight (not merged into one surface that steps >step_height down to
    // the lower floor). Mirrors mesh_layers/free_span_layer.cpp.
    const int step_bins = std::max(1, (int)std::ceil(p.step_height / sz));
    size_t k = 0;
    while (k < spans.size())
    {
      size_t e = k;
      while (e + 1 < spans.size() && (spans[e+1].first - spans[e].second) <= step_bins) e++;
      double headroom = (e + 1 < spans.size()) ? (spans[e+1].first - spans[e].second) * sz : 1e3;
      if (headroom >= robot_h)
      {
        for (size_t m = k; m <= e; m++)
        {
          double z_surf = spans[m].second * sz + zmin;
          int id = (int)g.nodes.size();
          Node n; n.cx = cx; n.cy = cy; n.z = (float)z_surf; g.nodes.push_back(n);
          surf_key.push_back(kv.first);
          col_surfs[kv.first].push_back(id);
        }
      }
      k = e + 1;
    }
  }
  const size_t M = g.nodes.size();
  g.adj.assign(M, {});

  // union-find: connect 4-neighbour surfaces with step <= step_height, build adjacency
  std::vector<int> par(M);
  for (size_t i = 0; i < M; i++) par[i] = (int)i;
  std::function<int(int)> find = [&](int a){ while (par[a]!=a){ par[a]=par[par[a]]; a=par[a]; } return a; };
  const double step = p.step_height;
  const int dxs[4] = {1,0,-1,0}, dys[4] = {0,1,0,-1};
  for (const auto& kv : col_surfs)
  {
    int cx, cy; unkey(kv.first, cx, cy);
    for (int d = 0; d < 4; d++)
    {
      auto it = col_surfs.find(Graph::key(cx + dxs[d], cy + dys[d]));
      if (it == col_surfs.end()) continue;
      for (int s : kv.second)
        for (int t : it->second)
          if (std::fabs(g.nodes[s].z - g.nodes[t].z) <= step)
          {
            g.adj[s].push_back(t);                 // directed add; mirror comes from neighbour's own pass
            int ra = find(s), rb = find(t); if (ra != rb) par[rb] = ra;
          }
    }
  }

  // components; main = largest
  std::unordered_map<int,int> comp_size;
  for (size_t i = 0; i < M; i++) { int r = find((int)i); g.nodes[i].comp = r; comp_size[r]++; }
  int main_root = -1, best = -1;
  for (auto& c : comp_size) if (c.second > best) { best = c.second; main_root = c.first; }
  g.main_comp = main_root;

  // Hole-fill (morphological close) on the MAIN component: a legged robot should
  // see the staircase as ONE solid traversable area, but mesh holes leave gaps in
  // the walkable band. Fill an empty column surrounded by >=5 main-component
  // surfaces of consistent height -> add a synthetic surface there, in main.
  {
    const int f8x[8] = {1,-1,0,0, 1,1,-1,-1}, f8y[8] = {0,0,1,-1, 1,-1,1,-1};
    for (int iter = 0; iter < std::max(0, p.fill_iters); iter++)
    {
      std::set<int64_t> cand;
      size_t cur = g.nodes.size();
      for (size_t i = 0; i < cur; i++)
      {
        if (g.nodes[i].comp != main_root) continue;
        int cx = g.nodes[i].cx, cy = g.nodes[i].cy;
        for (int d = 0; d < 8; d++)
          if (col_surfs.find(Graph::key(cx + f8x[d], cy + f8y[d])) == col_surfs.end())
            cand.insert(Graph::key(cx + f8x[d], cy + f8y[d]));
      }
      std::vector<std::pair<int64_t,double>> toadd;
      for (int64_t nk : cand)
      {
        int cx, cy; unkey(nk, cx, cy);
        std::vector<double> zs;
        for (int d = 0; d < 8; d++)
        {
          auto it = col_surfs.find(Graph::key(cx + f8x[d], cy + f8y[d]));
          if (it == col_surfs.end()) continue;
          for (int t : it->second) if (g.nodes[t].comp == main_root) zs.push_back(g.nodes[t].z);
        }
        if ((int)zs.size() < 5) continue;
        std::sort(zs.begin(), zs.end());
        if (zs.back() - zs.front() > step) continue;   // not a flat-ish hole -> don't bridge a wall/drop
        toadd.push_back({nk, zs[zs.size()/2]});
      }
      for (auto& pr : toadd)
      {
        int cx, cy; unkey(pr.first, cx, cy);
        int id = (int)g.nodes.size();
        Node n; n.cx = cx; n.cy = cy; n.z = (float)pr.second; n.comp = main_root; g.nodes.push_back(n);
        surf_key.push_back(pr.first);
        col_surfs[pr.first].push_back(id);
        g.adj.push_back({});
        for (int d = 0; d < 4; d++)
        {
          auto it = col_surfs.find(Graph::key(cx + dxs[d], cy + dys[d]));
          if (it == col_surfs.end()) continue;
          for (int t : it->second)
            if (g.nodes[t].comp == main_root && std::fabs(g.nodes[t].z - n.z) <= step)
            { g.adj[id].push_back(t); g.adj[t].push_back(id); }
        }
      }
    }
  }
  const size_t Mn = g.nodes.size();

  // clearance = s_xy * BFS hops from the main-component boundary (a main node that
  // lacks a walkable 4-neighbour in main). Non-main nodes keep clearance 0.
  std::vector<int> hops(Mn, -1);
  std::vector<char> is_bnd(Mn, 0);
  for (size_t i = 0; i < Mn; i++)
  {
    if (g.nodes[i].comp != main_root) continue;
    int cx = g.nodes[i].cx, cy = g.nodes[i].cy; bool full = true;
    for (int d = 0; d < 4; d++)
    {
      bool found = false;
      auto it = col_surfs.find(Graph::key(cx + dxs[d], cy + dys[d]));
      if (it != col_surfs.end())
        for (int t : it->second)
          if (g.nodes[t].comp == main_root && std::fabs(g.nodes[t].z - g.nodes[i].z) <= step) { found = true; break; }
      if (!found) { full = false; break; }
    }
    if (!full) is_bnd[i] = 1;
  }
  std::queue<int> bfsq;
  for (size_t i = 0; i < Mn; i++) if (is_bnd[i]) { hops[i] = 0; bfsq.push((int)i); }
  while (!bfsq.empty())
  {
    int u = bfsq.front(); bfsq.pop();
    for (int v : g.adj[u])
      if (g.nodes[v].comp == main_root && hops[v] < 0) { hops[v] = hops[u] + 1; bfsq.push(v); }
  }
  for (size_t i = 0; i < Mn; i++)
    g.nodes[i].clearance = (g.nodes[i].comp == main_root && hops[i] >= 0) ? (float)(hops[i] * sxy) : 0.0f;

  for (size_t i = 0; i < Mn; i++) g.col_nodes[Graph::key(g.nodes[i].cx, g.nodes[i].cy)].push_back((int)i);

  // 8-connected no-corner-cut MOVEMENT graph for the planner: 4 orthogonal + 4
  // diagonal neighbours (step<=h_step, same main component). A diagonal is allowed
  // only if BOTH shared-corner orthogonal cells are also walkable, so a long body
  // cannot slip through a diagonal wall-pinch. (union-find/clearance stay 4-conn.)
  g.move_adj.assign(Mn, {});
  const int mdx[8] = {1,0,-1,0, 1,1,-1,-1}, mdy[8] = {0,1,0,-1, 1,-1,1,-1};
  for (size_t i = 0; i < Mn; i++)
  {
    if (g.nodes[i].comp != main_root) continue;
    int cx = g.nodes[i].cx, cy = g.nodes[i].cy; double zi = g.nodes[i].z;
    for (int d = 0; d < 8; d++)
    {
      int t = g.nodeInColumn(cx + mdx[d], cy + mdy[d], zi, step, main_root);
      if (t < 0) continue;
      if (d >= 4)  // diagonal: forbid corner cutting
      {
        if (g.nodeInColumn(cx + mdx[d], cy, zi, step, main_root) < 0) continue;
        if (g.nodeInColumn(cx, cy + mdy[d], zi, step, main_root) < 0) continue;
      }
      g.move_adj[i].push_back(t);
    }
  }
  return g;
}

// A* on the main component. Edge weight = 3D length * centering multiplier(clearance(v)).
// Returns node-id path (start..goal) or empty on failure; fills out_length (geometric, m)
// and out_cost (weighted). Heuristic = Euclidean-to-goal (admissible: multiplier >= 1).
inline std::vector<int> astar(const Graph& g, int start, int goal, const PlanParams& pp,
                              double& out_length, double& out_cost)
{
  out_length = 0.0; out_cost = 0.0;
  const size_t M = g.nodes.size();
  if (start < 0 || goal < 0 || (size_t)start >= M || (size_t)goal >= M) return {};
  const double r_in = pp.inscribed_radius, r_infl = pp.inflation_radius, alpha = pp.alpha;
  const double denom = std::max(1e-6, r_infl - r_in);
  auto cmul = [&](double clr) {
    if (clr >= r_infl) return 1.0;
    double t = (r_infl - std::max(0.0, clr)) / denom;   // 0 at r_infl, >1 below r_in
    return 1.0 + alpha * std::min(t, r_infl / denom);   // finite even at clr=0 (never lethal)
  };
  auto dist3 = [&](int a, int b) {
    double dx = g.nodeX(a) - g.nodeX(b), dy = g.nodeY(a) - g.nodeY(b), dz = g.nodes[a].z - g.nodes[b].z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  };
  auto heur = [&](int a) { return dist3(a, goal); };

  std::vector<double> gscore(M, std::numeric_limits<double>::infinity());
  std::vector<int> came(M, -1);
  using QI = std::pair<double,int>;
  std::priority_queue<QI, std::vector<QI>, std::greater<QI>> open;
  gscore[start] = 0.0; open.push({heur(start), start});
  std::vector<char> closed(M, 0);
  while (!open.empty())
  {
    int u = open.top().second; open.pop();
    if (closed[u]) continue; closed[u] = 1;
    if (u == goal) break;
    for (int v : g.move_adj[u])
    {
      if (g.nodes[v].comp != g.main_comp) continue;
      double w = dist3(u, v) * cmul(g.nodes[v].clearance);
      double ng = gscore[u] + w;
      if (ng < gscore[v]) { gscore[v] = ng; came[v] = u; open.push({ng + heur(v), v}); }
    }
  }
  if (!std::isfinite(gscore[goal])) return {};
  std::vector<int> path;
  for (int c = goal; c != -1; c = came[c]) path.push_back(c);
  std::reverse(path.begin(), path.end());
  out_cost = gscore[goal];
  for (size_t i = 1; i < path.size(); i++) out_length += dist3(path[i-1], path[i]);
  return path;
}

// Line-of-sight on the walkable surface: every cell the 3D segment a->b crosses
// must have a main-component surface within `band` of the linearly-interpolated
// segment height AND lateral clearance >= min_clear (so a shortcut cannot dip out
// of the central corridor). Used by string pulling (ASA stage 2).
inline bool lineOfSight(const Graph& g, int a, int b, double band, double min_clear)
{
  double ax = g.nodeX(a), ay = g.nodeY(a), az = g.nodes[a].z;
  double bx = g.nodeX(b), by = g.nodeY(b), bz = g.nodes[b].z;
  double d = std::sqrt((bx-ax)*(bx-ax) + (by-ay)*(by-ay));
  int n = std::max(2, (int)(d / (g.cell_size * 0.5)) + 1);
  for (int k = 0; k <= n; k++)
  {
    double t = (double)k / n;
    double x = ax + t*(bx-ax), y = ay + t*(by-ay), z = az + t*(bz-az);
    int cx = (int)std::floor(x / g.cell_size), cy = (int)std::floor(y / g.cell_size);
    int id = g.nodeInColumn(cx, cy, z, band, g.main_comp);
    if (id < 0) return false;
    if (g.nodes[id].clearance + 1e-6 < min_clear) return false;
  }
  return true;
}

// String pulling (ASA stage 2): greedily replace sub-paths with straight segments
// that keep line-of-sight on the walkable surface -> straight diagonal runs. A
// shortcut is only taken if it stays in the clearance corridor of the sub-path it
// replaces (min clearance along [i..j] minus clear_tol), so centering is kept.
inline std::vector<int> stringPull(const Graph& g, const std::vector<int>& path,
                                   double band, double clear_tol)
{
  if (path.size() < 3) return path;
  std::vector<int> out; out.push_back(path.front());
  size_t i = 0;
  while (i < path.size() - 1)
  {
    size_t j = path.size() - 1; bool found = false;
    for (; j > i + 1; j--)
    {
      double req = std::numeric_limits<double>::infinity();
      for (size_t k = i; k <= j; k++) req = std::min(req, (double)g.nodes[path[k]].clearance);
      req -= clear_tol;
      if (lineOfSight(g, path[i], path[j], band, req)) { found = true; break; }
    }
    if (!found) j = i + 1;
    out.push_back(path[j]); i = j;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Stage 2b: SE(2) yaw lattice + footprint-at-yaw feasibility.
// ---------------------------------------------------------------------------
struct Se2Params
{
  double length = 0.60;   // body length L (along heading), m
  double width  = 0.30;   // body width W (lateral), m
  int    n_yaw  = 16;     // yaw channels
  double v_long = 1.0;    // forward speed (m/s)
  double v_lat  = 0.3;    // lateral speed (m/s) -> crab-walking penalized
  double omega  = 1.0;    // turn rate (rad/s)
  double height_tol = 0.25; // allowed |dz| of a footprint cell vs the body centre (covers slope)
  double max_unsupported_frac = 0.25; // tolerate this fraction of footprint cells missing (mesh holes)
};

// Is the body rectangle at (node, yaw) supported by main-component surface?
// Tolerates up to max_unsupported_frac missing cells (scattered mesh holes); a
// real wall removes a large contiguous fraction and still fails.
inline bool footprintFeasible(const Graph& g, int node, double yaw, const Se2Params& sp)
{
  const double cs = g.cell_size;
  const double x0 = g.nodeX(node), y0 = g.nodeY(node), z0 = g.nodes[node].z;
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double ha = sp.length * 0.5, hb = sp.width * 0.5;
  const double da = cs * 0.75, db = cs * 0.75;  // sample spacing (<cell so no gap skipped)
  int total = 0, bad = 0;
  for (double a = -ha; a <= ha + 1e-9; a += da)
    for (double b = -hb; b <= hb + 1e-9; b += db)
    {
      double wx = x0 + a * c - b * s;
      double wy = y0 + a * s + b * c;
      int cx = (int)std::floor(wx / cs), cy = (int)std::floor(wy / cs);
      total++;
      if (g.nodeInColumn(cx, cy, z0, sp.height_tol, g.main_comp) < 0) bad++;
    }
  return total > 0 && (double)bad <= sp.max_unsupported_frac * total;
}

// Precompute feasibility[node*n_yaw + i] for all main-component nodes.
inline std::vector<char> buildFeasibility(const Graph& g, const Se2Params& sp)
{
  const int ny = sp.n_yaw;
  std::vector<char> feas((size_t)g.nodes.size() * ny, 0);
  for (size_t u = 0; u < g.nodes.size(); u++)
  {
    if (g.nodes[u].comp != g.main_comp) continue;
    // rectangle is symmetric under +pi: compute first half, mirror
    for (int i = 0; i < ny; i++)
    {
      int im = i % (ny / 2 == 0 ? ny : ny / 2);
      if (i >= ny / 2 && ny % 2 == 0) { feas[u*ny + i] = feas[u*ny + im]; continue; }
      feas[u*ny + i] = footprintFeasible(g, (int)u, 2.0 * M_PI * i / ny, sp) ? 1 : 0;
    }
  }
  return feas;
}

struct Se2Result { std::vector<int> nodes; std::vector<int> yaws; double time = 0.0; };

// A* on the (node, yaw) lattice. start_yaw in [0,n_yaw) is the preferred start
// channel; goal_yaw < 0 = any. Returns empty nodes on failure.
inline Se2Result se2astar(const Graph& g, const std::vector<char>& feas, const Se2Params& sp,
                          int start, int start_yaw, int goal, int goal_yaw)
{
  Se2Result R;
  const int ny = sp.n_yaw;
  const size_t M = g.nodes.size();
  if (start < 0 || goal < 0) return R;
  const double dpsi = 2.0 * M_PI / ny, rot_cost = dpsi / sp.omega;
  const double vmax = std::max(sp.v_long, sp.v_lat);
  auto yawVal = [&](int i){ return i * dpsi; };
  auto stateId = [&](int node, int i){ return (size_t)node * ny + i; };

  auto feasAt = [&](int node, int i){ return feas[stateId(node, i)] != 0; };
  auto firstFeasYaw = [&](int node, int pref) {
    for (int d = 0; d < ny; d++)
    {
      int i1 = ((pref + d) % ny + ny) % ny, i2 = ((pref - d) % ny + ny) % ny;
      if (feasAt(node, i1)) return i1;
      if (feasAt(node, i2)) return i2;
    }
    return -1;
  };
  // Snap start/goal to the NEAREST node that is footprint-feasible at some yaw
  // (the requested pose may land on a sparse spot where the body can't be placed).
  auto nearestFeasible = [&](int from, int pref, int& out_yaw) {
    std::queue<int> q; std::set<int> seen; q.push(from); seen.insert(from);
    int lim = 0;
    while (!q.empty() && lim < 20000)
    {
      int u = q.front(); q.pop(); lim++;
      int fy = firstFeasYaw(u, pref);
      if (fy >= 0) { out_yaw = fy; return u; }
      for (int v : g.move_adj[u]) if (!seen.count(v)) { seen.insert(v); q.push(v); }
    }
    return -1;
  };
  int s_yaw = -1, g_yaw = -1;
  start = nearestFeasible(start, start_yaw, s_yaw);
  goal  = nearestFeasible(goal, (goal_yaw >= 0 ? goal_yaw : 0), g_yaw);
  if (start < 0 || goal < 0) return R;  // body fits nowhere reachable near start/goal

  auto transCost = [&](int u, int v, int i){
    double dx = g.nodeX(v) - g.nodeX(u), dy = g.nodeY(v) - g.nodeY(u);
    double c = std::cos(yawVal(i)), s = std::sin(yawVal(i));
    double lo = std::fabs(dx * c + dy * s), la = std::fabs(-dx * s + dy * c);
    return lo / sp.v_long + la / sp.v_lat;
  };
  auto heur = [&](int node, int i){
    double dx = g.nodeX(goal) - g.nodeX(node), dy = g.nodeY(goal) - g.nodeY(node);
    double h = std::sqrt(dx*dx + dy*dy) / vmax;
    if (goal_yaw >= 0)
    {
      double a = std::fabs(yawVal(i) - yawVal(goal_yaw));
      a = std::min(a, 2*M_PI - a);
      h += a / sp.omega;
    }
    return h;
  };

  std::unordered_map<size_t,double> gsc;
  std::unordered_map<size_t,size_t> came;
  using QI = std::pair<double,size_t>;
  std::priority_queue<QI, std::vector<QI>, std::greater<QI>> open;
  size_t s0 = stateId(start, s_yaw);
  gsc[s0] = 0.0; open.push({heur(start, s_yaw), s0});
  std::unordered_map<size_t,char> closed;
  size_t goal_state = SIZE_MAX;
  while (!open.empty())
  {
    size_t u = open.top().second; open.pop();
    if (closed[u]) continue; closed[u] = 1;
    int un = (int)(u / ny), ui = (int)(u % ny);
    if (un == goal && (goal_yaw < 0 || ui == goal_yaw)) { goal_state = u; break; }
    double gu = gsc[u];
    // translational
    for (int v : g.move_adj[un])
    {
      if (!feasAt(v, ui)) continue;
      size_t sv = stateId(v, ui);
      double ng = gu + transCost(un, v, ui);
      auto it = gsc.find(sv);
      if (it == gsc.end() || ng < it->second) { gsc[sv] = ng; came[sv] = u; open.push({ng + heur(v, ui), sv}); }
    }
    // rotational
    for (int dd = -1; dd <= 1; dd += 2)
    {
      int vi = ((ui + dd) % ny + ny) % ny;
      if (!feasAt(un, vi)) continue;
      size_t sv = stateId(un, vi);
      double ng = gu + rot_cost;
      auto it = gsc.find(sv);
      if (it == gsc.end() || ng < it->second) { gsc[sv] = ng; came[sv] = u; open.push({ng + heur(un, vi), sv}); }
    }
  }
  if (goal_state == SIZE_MAX) return R;
  R.time = gsc[goal_state];
  std::vector<size_t> st;
  for (size_t c = goal_state; ; )
  {
    st.push_back(c);
    auto it = came.find(c);
    if (it == came.end()) break;
    c = it->second;
  }
  std::reverse(st.begin(), st.end());
  for (size_t c : st) { R.nodes.push_back((int)(c / ny)); R.yaws.push_back((int)(c % ny)); }
  return R;
}

// Footprint-aware line-of-sight: straight segment a->b, body at the segment's
// heading channel must be feasible in every crossed cell.
inline bool lineOfSightFootprint(const Graph& g, const std::vector<char>& feas, const Se2Params& sp,
                                 int a, int b, double band)
{
  double ax = g.nodeX(a), ay = g.nodeY(a), az = g.nodes[a].z;
  double bx = g.nodeX(b), by = g.nodeY(b), bz = g.nodes[b].z;
  double hd = std::atan2(by - ay, bx - ax);
  int yi = ((int)std::llround(hd / (2*M_PI / sp.n_yaw)) % sp.n_yaw + sp.n_yaw) % sp.n_yaw;
  double d = std::sqrt((bx-ax)*(bx-ax) + (by-ay)*(by-ay));
  int n = std::max(2, (int)(d / (g.cell_size * 0.5)) + 1);
  for (int k = 0; k <= n; k++)
  {
    double t = (double)k / n;
    double x = ax + t*(bx-ax), y = ay + t*(by-ay), z = az + t*(bz-az);
    int cx = (int)std::floor(x / g.cell_size), cy = (int)std::floor(y / g.cell_size);
    int id = g.nodeInColumn(cx, cy, z, band, g.main_comp);
    if (id < 0) return false;
    if (feas[(size_t)id * sp.n_yaw + yi] == 0) return false;  // body wouldn't fit here at this heading
  }
  return true;
}

// String pull a node path keeping footprint feasibility along each straight run.
inline std::vector<int> stringPullFootprint(const Graph& g, const std::vector<char>& feas,
                                            const Se2Params& sp, const std::vector<int>& path, double band)
{
  if (path.size() < 3) return path;
  std::vector<int> out; out.push_back(path.front());
  size_t i = 0;
  while (i < path.size() - 1)
  {
    size_t j = path.size() - 1; bool found = false;
    for (; j > i + 1; j--)
      if (lineOfSightFootprint(g, feas, sp, path[i], path[j], band)) { found = true; break; }
    if (!found) j = i + 1;
    out.push_back(path[j]); i = j;
  }
  return out;
}

}  // namespace free_span_core

#endif  // FREE_SPAN_CORE__FREE_SPAN_CORE_HPP
