-- carto.lua

include "cartographer_ros/configuration_files/backpack_2d.lua"
-- Ensure global tables are defined
MAP_BUILDER = MAP_BUILDER or {}
TRAJECTORY_BUILDER = TRAJECTORY_BUILDER or {}
POSE_GRAPH = POSE_GRAPH or {}


options = {
  map_frame = "map",
  tracking_frame = "map", -- base frame
  published_frame = "velodyne", -- aser or point cloud frame

  odom_frame = "odom", -- If you have odometry
  provide_odom_frame = false, -- no odom for kitti seq

  publish_frame_projected_to_2d = true,

  scan_topic = "/kitti/velodyne", -- PointCloud2 topic
  rangefinder_sampling_rate = 1.,

  use_image_data = true,
  image_topic = "/kitti/image", -- image topic
  image_sampling_rate = 1.,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,

  pose_constraints = {
    adaptive_odometry_pose_constraints = {
      enabled = true
    }
  },

  trajectory_builder_2d = {
    min_range = 0.5,
    max_range = 10.,
    use_intensities = false,
    adaptive_voxel_filter = {
      enabled = true,
      leaf_size = 0.2,
    },
    real_time_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      translation_delta_cost_weight = 1e-1,
      rotation_delta_cost_weight = 1e-1,
    },
    ceres_scan_matcher = {
      translation_weight = 10.,
      rotation_weight = 1e+4,
    },
    loop_closure = {
      loop_closure_adaptive_voxel_filter = {
        enabled = true,
        leaf_size = 0.3,
      },
      loop_closure_translation_weight = 1e+5,
      loop_closure_rotation_weight = 1e+10,
    },
  },

  mapper_2d = {
    use_grid_probabilities = false,
    resolution = 0.05,
  },
}

MAP_BUILDER.use_trajectory_builder_2d(options.trajectory_builder_2d)
MAP_BUILDER.use_laser_scan_data(options)

if options.use_image_data then
  MAP_BUILDER.use_image_data(options)
end

POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.adaptive_vocabulary_pose_constraints.loop_closure_frequency = 10.
POSE_GRAPH.constraint_builder.log_matches_locally = true
POSE_GRAPH.constraint_builder.log_matches_globally = true