/*
 *  se2_mesh_planner — Stage 1 (cell-graph A* on free_span_core). See header.
 */
#include "se2_mesh_planner/se2_mesh_planner.h"

#include <cmath>
#include <mbf_msgs/action/get_path.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(se2_mesh_planner::SE2MeshPlanner, mbf_mesh_core::MeshPlanner)

namespace se2_mesh_planner
{
using GetPathResult = mbf_msgs::action::GetPath::Result;

SE2MeshPlanner::SE2MeshPlanner() {}
SE2MeshPlanner::~SE2MeshPlanner() {}

bool SE2MeshPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}

void SE2MeshPlanner::buildGraph()
{
  const auto mesh = mesh_map_->mesh();
  std::vector<float> vx, vy, vz;
  vx.reserve(mesh->numVertices()); vy.reserve(mesh->numVertices()); vz.reserve(mesh->numVertices());
  for (size_t i = 0; i < mesh->nextVertexIndex(); i++)
  {
    auto vH = lvr2::VertexHandle(i);
    if (!mesh->containsVertex(vH)) continue;
    const auto p = mesh->getVertexPosition(vH);
    vx.push_back(p.x); vy.push_back(p.y); vz.push_back(p.z);
  }
  free_span_core::Params p;
  p.cell_size = config_.cell_size; p.voxel_height = config_.voxel_height;
  p.robot_height = config_.robot_height; p.step_height = config_.step_height;
  p.merge_gap = config_.merge_gap;
  graph_ = free_span_core::build(vx, vy, vz, p);

  if (config_.n_yaw > 0)  // Stage 2b: precompute footprint feasibility
  {
    se2p_.length = config_.body_length; se2p_.width = config_.body_width;
    se2p_.n_yaw = config_.n_yaw; se2p_.v_long = config_.v_long;
    se2p_.v_lat = config_.v_lat; se2p_.omega = config_.omega; se2p_.height_tol = config_.height_tol;
    se2p_.max_unsupported_frac = config_.footprint_slack;
    feas_ = free_span_core::buildFeasibility(graph_, se2p_);
    size_t nf = 0; for (char c : feas_) nf += (c != 0);
    RCLCPP_INFO_STREAM(node_->get_logger(), name_ << ": SE(2) footprint feasibility built — "
        << nf << " feasible (node,yaw) states (L=" << se2p_.length << " W=" << se2p_.width
        << " n_yaw=" << se2p_.n_yaw << ")");
  }
  graph_ready_ = true;

  // floor connectivity diagnostic (same bands as the layer)
  size_t main_cells = 0; bool f1 = false, f2 = false;
  for (const auto& n : graph_.nodes)
    if (n.comp == graph_.main_comp)
    {
      main_cells++;
      if (n.z > -0.6f && n.z < 0.1f) f1 = true;
      if (n.z > 1.9f && n.z < 2.6f) f2 = true;
    }
  RCLCPP_INFO_STREAM(node_->get_logger(), name_ << ": free_span graph built — " << graph_.nodes.size()
      << " surfaces, main=" << main_cells << " cells, floor1+floor2 in main: " << ((f1 && f2) ? "YES" : "NO"));
}

uint32_t SE2MeshPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                  const geometry_msgs::msg::PoseStamped& goal, double /*tolerance*/,
                                  std::vector<geometry_msgs::msg::PoseStamped>& plan, double& cost,
                                  std::string& message)
{
  cancel_planning_ = false;
  geometry_msgs::msg::PoseStamped start_in_map, goal_in_map;
  try
  {
    start_in_map = mesh_map_->transformToMapFrame(start);
    goal_in_map = mesh_map_->transformToMapFrame(goal);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), name_ << ": TF to map frame failed: " << ex.what());
    message = "tf error";
    return GetPathResult::TF_ERROR;
  }

  if (!graph_ready_) buildGraph();
  if (graph_.main_comp < 0 || graph_.nodes.empty())
  {
    message = "empty free_span graph";
    return GetPathResult::NO_PATH_FOUND;
  }

  const auto& sp = start_in_map.pose.position;
  const auto& gp = goal_in_map.pose.position;
  int si = graph_.nearestMain(sp.x, sp.y, sp.z);
  int gi = graph_.nearestMain(gp.x, gp.y, gp.z);
  if (si < 0) { message = "start not on traversable surface"; return GetPathResult::INVALID_START; }
  if (gi < 0) { message = "goal not on traversable surface"; return GetPathResult::INVALID_GOAL; }

  std::vector<int> ids;
  size_t raw_count = 0;
  const char* mode = "2a/position";
  if (config_.n_yaw > 0)
  {
    // Stage 2b: SE(2) yaw-lattice A* + footprint-aware string pull
    double syaw = 2.0 * std::atan2(start_in_map.pose.orientation.z, start_in_map.pose.orientation.w);
    int syi = ((int)std::llround(syaw / (2.0 * M_PI / config_.n_yaw)) % config_.n_yaw + config_.n_yaw) % config_.n_yaw;
    free_span_core::Se2Result r = free_span_core::se2astar(graph_, feas_, se2p_, si, syi, gi, -1);
    if (r.nodes.empty())
    {
      message = "no SE(2) path (body footprint may not fit)";
      return GetPathResult::NO_PATH_FOUND;
    }
    raw_count = r.nodes.size();
    ids = free_span_core::stringPullFootprint(graph_, feas_, se2p_, r.nodes, config_.step_height);
    mode = "2b/SE2";
  }
  else
  {
    free_span_core::PlanParams pp;
    pp.inscribed_radius = config_.inscribed_radius;
    pp.inflation_radius = config_.inflation_radius;
    pp.alpha = config_.centering_alpha;
    double length = 0.0, wcost = 0.0;
    std::vector<int> raw = free_span_core::astar(graph_, si, gi, pp, length, wcost);
    if (raw.empty())
    {
      message = "no path on cell graph";
      return GetPathResult::NO_PATH_FOUND;
    }
    raw_count = raw.size();
    ids = free_span_core::stringPull(graph_, raw, config_.step_height, config_.string_pull_tol);
  }
  double pulled_len = 0.0;
  for (size_t k = 1; k < ids.size(); k++)
  {
    double dx = graph_.nodeX(ids[k]) - graph_.nodeX(ids[k-1]);
    double dy = graph_.nodeY(ids[k]) - graph_.nodeY(ids[k-1]);
    double dz = graph_.nodes[ids[k]].z - graph_.nodes[ids[k-1]].z;
    pulled_len += std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  std_msgs::msg::Header header;
  header.stamp = node_->now();
  header.frame_id = map_frame_;

  plan.clear(); plan.reserve(ids.size());
  for (size_t k = 0; k < ids.size(); k++)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = graph_.nodeX(ids[k]);
    pose.pose.position.y = graph_.nodeY(ids[k]);
    pose.pose.position.z = graph_.nodes[ids[k]].z;
    // yaw from tangent (Stage 2 replaces with SE(2) yaw)
    double yaw = 0.0;
    int a = (k + 1 < ids.size()) ? (int)k : (int)k - 1;
    int b = a + 1;
    if (a >= 0 && b < (int)ids.size())
      yaw = std::atan2(graph_.nodeY(ids[b]) - graph_.nodeY(ids[a]),
                       graph_.nodeX(ids[b]) - graph_.nodeX(ids[a]));
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    plan.push_back(pose);
  }
  cost = pulled_len;

  nav_msgs::msg::Path path_msg;
  path_msg.header = header;
  path_msg.poses = plan;
  path_pub_->publish(path_msg);

  RCLCPP_INFO_STREAM(node_->get_logger(), name_ << " [" << mode << "]: " << raw_count
      << " -> string-pulled " << plan.size() << " poses, length " << pulled_len << " m");
  return GetPathResult::SUCCESS;
}

bool SE2MeshPlanner::initialize(const std::string& plugin_name,
                                const std::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr,
                                const rclcpp::Node::SharedPtr& node)
{
  mesh_map_ = mesh_map_ptr;
  name_ = plugin_name;
  node_ = node;
  map_frame_ = mesh_map_->mapFrame();
  cancel_planning_ = false;

  config_.cell_size       = node_->declare_parameter(name_ + ".cell_size", config_.cell_size);
  config_.voxel_height    = node_->declare_parameter(name_ + ".voxel_height", config_.voxel_height);
  config_.robot_height    = node_->declare_parameter(name_ + ".robot_height", config_.robot_height);
  config_.step_height     = node_->declare_parameter(name_ + ".step_height", config_.step_height);
  config_.merge_gap       = node_->declare_parameter(name_ + ".merge_gap", config_.merge_gap);
  config_.inscribed_radius= node_->declare_parameter(name_ + ".inscribed_radius", config_.inscribed_radius);
  config_.inflation_radius= node_->declare_parameter(name_ + ".inflation_radius", config_.inflation_radius);
  config_.centering_alpha = node_->declare_parameter(name_ + ".centering_alpha", config_.centering_alpha);
  config_.string_pull_tol = node_->declare_parameter(name_ + ".string_pull_tol", config_.string_pull_tol);
  config_.n_yaw       = node_->declare_parameter(name_ + ".n_yaw", config_.n_yaw);
  config_.body_length = node_->declare_parameter(name_ + ".body_length", config_.body_length);
  config_.body_width  = node_->declare_parameter(name_ + ".body_width", config_.body_width);
  config_.v_long      = node_->declare_parameter(name_ + ".v_long", config_.v_long);
  config_.v_lat       = node_->declare_parameter(name_ + ".v_lat", config_.v_lat);
  config_.omega       = node_->declare_parameter(name_ + ".omega", config_.omega);
  config_.height_tol  = node_->declare_parameter(name_ + ".height_tol", config_.height_tol);
  config_.footprint_slack = node_->declare_parameter(name_ + ".footprint_slack", config_.footprint_slack);

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&SE2MeshPlanner::reconfigureCallback, this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(node_->get_logger(), name_ << ": initialized SE(2) Stage-1 cell-graph A* planner.");
  return true;
}

rcl_interfaces::msg::SetParametersResult
SE2MeshPlanner::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool rebuild = false;
  for (auto p : parameters)
  {
    const std::string n = p.get_name();
    if (n == name_ + ".cell_size")            { config_.cell_size = p.as_double(); rebuild = true; }
    else if (n == name_ + ".voxel_height")    { config_.voxel_height = p.as_double(); rebuild = true; }
    else if (n == name_ + ".robot_height")    { config_.robot_height = p.as_double(); rebuild = true; }
    else if (n == name_ + ".step_height")     { config_.step_height = p.as_double(); rebuild = true; }
    else if (n == name_ + ".merge_gap")       { config_.merge_gap = p.as_int(); rebuild = true; }
    else if (n == name_ + ".inscribed_radius"){ config_.inscribed_radius = p.as_double(); }
    else if (n == name_ + ".inflation_radius"){ config_.inflation_radius = p.as_double(); }
    else if (n == name_ + ".centering_alpha") { config_.centering_alpha = p.as_double(); }
    else if (n == name_ + ".string_pull_tol") { config_.string_pull_tol = p.as_double(); }
    else if (n == name_ + ".n_yaw")       { config_.n_yaw = p.as_int(); rebuild = true; }
    else if (n == name_ + ".body_length") { config_.body_length = p.as_double(); rebuild = true; }
    else if (n == name_ + ".body_width")  { config_.body_width = p.as_double(); rebuild = true; }
    else if (n == name_ + ".v_long")      { config_.v_long = p.as_double(); }
    else if (n == name_ + ".v_lat")       { config_.v_lat = p.as_double(); }
    else if (n == name_ + ".omega")       { config_.omega = p.as_double(); }
    else if (n == name_ + ".height_tol")  { config_.height_tol = p.as_double(); rebuild = true; }
    else if (n == name_ + ".footprint_slack") { config_.footprint_slack = p.as_double(); rebuild = true; }
  }
  if (rebuild) graph_ready_ = false;  // graph params changed -> rebuild on next makePlan
  return result;
}

}  // namespace se2_mesh_planner
