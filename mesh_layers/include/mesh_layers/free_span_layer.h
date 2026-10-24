/*
 *  FreeSpanLayer — free-space-span traversability (SE(2) NavMesh style, no yaw).
 *
 *  Robustness idea: instead of reading per-vertex surface normals/geometry
 *  (fragile to holes, noise and inconsistent normals), voxelize the surface into
 *  vertical COLUMNS, find solid/free spans, derive walkable surfaces by
 *  headroom >= robot_height, connect adjacent surfaces by step <= step_height,
 *  and keep only the main connected component. Columns are independent, so a hole
 *  in one place cannot disconnect a floor elsewhere, and no normals are used.
 *
 *  Cost: 0 = traversable (in main component), 1 = lethal (wall / low-headroom /
 *  isolated). Lethal when cost > threshold.
 */
#ifndef MESH_LAYERS__FREE_SPAN_LAYER_H
#define MESH_LAYERS__FREE_SPAN_LAYER_H

#include <mesh_map/abstract_layer.h>
#include <rclcpp/rclcpp.hpp>

namespace mesh_layers
{
class FreeSpanLayer : public mesh_map::AbstractLayer
{
  virtual bool readLayer() override;
  virtual bool writeLayer() override;
  virtual float threshold() override;

  virtual float defaultValue() override
  {
    return std::numeric_limits<float>::infinity();
  }

  virtual bool computeLayer() override;
  bool computeLethals();

  virtual const lvr2::VertexMap<float>& costs() override;

  virtual const std::set<lvr2::VertexHandle>& lethals() override
  {
    return lethal_vertices_;
  }

  virtual bool initialize() override;

  rcl_interfaces::msg::SetParametersResult reconfigureCallback(std::vector<rclcpp::Parameter> parameters);

  // per-vertex cost: 0 traversable, 1 lethal
  lvr2::DenseVertexMap<float> costs_;
  std::set<lvr2::VertexHandle> lethal_vertices_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  struct {
    double cell_size = 0.10;     // horizontal column size s_xy (m)
    double voxel_height = 0.05;  // vertical voxel size s_z (m)
    double robot_height = 0.50;  // required headroom (m)
    double step_height = 0.20;   // max climbable step between cells (m)
    int    merge_gap = 1;        // merge solid voxels separated by <= this many empty bins
    double connect_tol = 0.25;   // vertex-to-surface z tolerance (m)
    // soft lateral-clearance cost on the cell graph (inflation, done robustly):
    double inscribed_radius = 0.15; // full cost within this lateral distance (m)
    double inflation_radius = 0.5;  // cost decays to 0 by this distance (m)
    double cost_scaling = 2.0;      // exponential decay rate beyond inscribed
    double inscribed_value = 0.9;   // cost at/under inscribed_radius (< threshold)
    double threshold = 0.95;        // lethal when cost > threshold (only cost==1)
  } config_;
};

} /* namespace mesh_layers */

#endif  // MESH_LAYERS__FREE_SPAN_LAYER_H
