/*
 *  se2_mesh_planner — Stage 1 of SE(2) NavMesh for mesh_navigation.
 *
 *  Plans A* on the free_span_core CELL graph (NOT the triangle mesh), using a
 *  lateral-clearance centering multiplier as edge cost. Position-only (yaw taken
 *  from the path tangent); Stage 2 adds a yaw lattice + footprint feasibility.
 *  Drop-in mbf_mesh_core::MeshPlanner plugin, selected by name — existing
 *  dijkstra / cvp planners are untouched.
 */
#ifndef SE2_MESH_PLANNER__SE2_MESH_PLANNER_H
#define SE2_MESH_PLANNER__SE2_MESH_PLANNER_H

#include <atomic>
#include <mbf_mesh_core/mesh_planner.h>
#include <mesh_map/mesh_map.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <free_span_core/free_span_core.hpp>

namespace se2_mesh_planner
{
class SE2MeshPlanner : public mbf_mesh_core::MeshPlanner
{
public:
  typedef std::shared_ptr<SE2MeshPlanner> Ptr;

  SE2MeshPlanner();
  virtual ~SE2MeshPlanner();

  virtual uint32_t makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                            double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan, double& cost,
                            std::string& message) override;

  virtual bool cancel() override;

  virtual bool initialize(const std::string& plugin_name, const std::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr,
                          const rclcpp::Node::SharedPtr& node) override;

protected:
  rcl_interfaces::msg::SetParametersResult reconfigureCallback(std::vector<rclcpp::Parameter> parameters);
  void buildGraph();

private:
  mesh_map::MeshMap::Ptr mesh_map_;
  std::string name_;
  rclcpp::Node::SharedPtr node_;
  std::string map_frame_;
  std::atomic_bool cancel_planning_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr reconfiguration_callback_handle_;

  free_span_core::Graph graph_;
  bool graph_ready_ = false;
  std::vector<char> feas_;              // Stage-2b footprint feasibility [node*n_yaw+i]
  free_span_core::Se2Params se2p_;

  struct {
    double cell_size = 0.10;
    double voxel_height = 0.05;
    double robot_height = 0.50;
    double step_height = 0.20;
    int    merge_gap = 1;
    double inscribed_radius = 0.15;
    double inflation_radius = 0.50;
    double centering_alpha = 4.0;
    double string_pull_tol = 0.05; // allowed clearance dip when straightening (m); 0 = strict corridor (keep centering), large = straighter
    // Stage 2b (SE(2) yaw lattice + footprint). n_yaw = 0 -> Stage-2a (position-only).
    int    n_yaw = 0;
    double body_length = 0.60;
    double body_width = 0.30;
    double v_long = 1.0;
    double v_lat = 0.3;
    double omega = 1.0;
    double height_tol = 0.25;
    double footprint_slack = 0.25; // max fraction of footprint cells allowed missing (mesh holes)
  } config_;
};

}  // namespace se2_mesh_planner

#endif  // SE2_MESH_PLANNER__SE2_MESH_PLANNER_H
