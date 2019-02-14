#include <mapper_emvs/data_loading.hpp>
#include <mapper_emvs/mapper_emvs.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>

// Input parameters
DEFINE_string(bag_filename, "input.bag", "Path to the rosbag");
DEFINE_string(event_topic, "/dvs/events", "Name of the event topic (default: /dvs/events)");
DEFINE_string(pose_topic, "/optitrack/davis", "Name of the pose topic (default: /optitrack/davis)");
DEFINE_string(camera_info_topic, "/dvs/camera_info", "Name of the camera info topic (default: /dvs/camera_info)");
DEFINE_double(start_time_s, 0.0, "Start time in seconds (default: 0.0)");
DEFINE_double(stop_time_s, 1000.0, "Stop time in seconds (default: 1000.0)");

// Disparity Space Image (DSI) parameters. Section 5.2 in the IJCV paper.
DEFINE_int32(dimX, 0, "X dimension of the voxel grid (if 0, will use the X dim of the event camera) (default: 0)");
DEFINE_int32(dimY, 0, "Y dimension of the voxel grid (if 0, will use the Y dim of the event camera) (default: 0)");
DEFINE_int32(dimZ, 100, "Z dimension of the voxel grid (default: 100) must be <= 256");
DEFINE_double(fov_deg, 0.0, "Field of view of the DSI, in degrees (if < 10, will use the FoV of the event camera) (default: 0.0)");
DEFINE_double(min_depth, 0.3, "Min depth, in meters (default: 0.3)");
DEFINE_double(max_depth, 5.0, "Max depth, in meters (default: 5.0)");

// Depth map parameters (selection and noise removal). Section 5.2.3 in the IJCV paper.
DEFINE_int32(adaptive_threshold_kernel_size, 5, "Size of the Gaussian kernel used for adaptive thresholding. (default: 5)");
DEFINE_double(adaptive_threshold_c, 5., "A value in [0, 255]. The smaller the noisier and more dense reconstruction (default: 5.)");
DEFINE_int32(median_filter_size, 5, "Size of the median filter used to clean the depth map. (default: 5)");

// Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
DEFINE_double(radius_search, 0.05, "Size of the radius filter. (default: 0.05)");
DEFINE_int32(min_num_neighbors, 3, "Minimum number of points for the radius filter. (default: 3)");


/*
 * Load a set of events and poses from a rosbag,
 * compute the disparity space image (DSI),
 * extract a depth map (and point cloud) from the DSI,
 * and save to disk.
 */
int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Load events, poses, and camera intrinsics from the rosbag
  sensor_msgs::CameraInfo camera_info_msg;
  std::vector<dvs_msgs::Event> events;
  std::map<ros::Time, geometry_utils::Transformation> poses;
  data_loading::parse_rosbag(FLAGS_bag_filename, events, poses, camera_info_msg,
                             FLAGS_event_topic, FLAGS_camera_info_topic, FLAGS_pose_topic, FLAGS_start_time_s, FLAGS_stop_time_s);

  // Create a camera object from the loaded intrinsic parameters
  image_geometry::PinholeCameraModel cam;
  cam.fromCameraInfo(camera_info_msg);

  // Use linear interpolation to compute the camera pose for each event
  LinearTrajectory trajectory = LinearTrajectory(poses);

  // Set the position of the reference view in the middle of the trajectory
  geometry_utils::Transformation T0_, T1_;
  ros::Time t0_, t1_;
  trajectory.getFirstControlPose(&T0_, &t0_);
  trajectory.getLastControlPose(&T1_, &t1_);
  geometry_utils::Transformation T_w_rv;
  trajectory.getPoseAt(ros::Time(0.5 * (t0_.toSec() + t1_.toSec())), T_w_rv);
  geometry_utils::Transformation T_rv_w = T_w_rv.inverse();

  // Initialize the DSI
  CHECK_LE(FLAGS_dimZ, 256) << "Number of depth planes should be <= 256";
  EMVS::ShapeDSI dsi_shape(FLAGS_dimX, FLAGS_dimY, FLAGS_dimZ,
                           FLAGS_min_depth, FLAGS_max_depth,
                           FLAGS_fov_deg);

  // Initialize mapper
  EMVS::MapperEMVS mapper(cam, dsi_shape);

  // 1. Back-project events into the DSI
  std::chrono::high_resolution_clock::time_point t_start_dsi = std::chrono::high_resolution_clock::now();
  mapper.evaluateDSI(events, trajectory, T_rv_w);
  std::chrono::high_resolution_clock::time_point t_end_dsi = std::chrono::high_resolution_clock::now();
  auto duration_dsi = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_dsi - t_start_dsi ).count();
  LOG(INFO) << "Time to evaluate DSI: " << duration_dsi << " milliseconds";
  LOG(INFO) << "Number of events processed: " << events.size() << " events";
  LOG(INFO) << "Number of events processed per second: " << static_cast<float>(events.size()) / (1000.f * static_cast<float>(duration_dsi)) << " Mev/s";

  LOG(INFO) << "Mean square = " << mapper.dsi_.computeMeanSquare();

  // Write the DSI (3D voxel grid) to disk
  mapper.dsi_.writeGridNpy("dsi.npy");


  // 2. Extract semi-dense depth map from DSI
  EMVS::OptionsDepthMap opts_depth_map;
  opts_depth_map.adaptive_threshold_kernel_size_ = FLAGS_adaptive_threshold_kernel_size;
  opts_depth_map.adaptive_threshold_c_ = FLAGS_adaptive_threshold_c;
  opts_depth_map.median_filter_size_ = FLAGS_median_filter_size;
  cv::Mat depth_map, confidence_map, semidense_mask;
  mapper.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
  
  // Save depth map, confidence map and semi-dense mask

  // Save semi-dense mask as an image
  cv::imwrite("semidense_mask.png", 255 * semidense_mask);

  // Save confidence map as an 8-bit image
  cv::Mat confidence_map_255;
  cv::normalize(confidence_map, confidence_map_255, 0, 255.0, cv::NORM_MINMAX, CV_32FC1);
  cv::imwrite("confidence_map.png", confidence_map_255);

  // Normalize depth map using given min and max depth values
  cv::Mat depth_map_255 = (depth_map - dsi_shape.min_depth_) * (255.0 / (dsi_shape.max_depth_ - dsi_shape.min_depth_));
  cv::imwrite("depth_map.png", depth_map_255);

  // Save pseudo-colored depth map on white canvas
  cv::Mat depthmap_8bit, depthmap_color;
  depth_map_255.convertTo(depthmap_8bit, CV_8U);
  cv::applyColorMap(depthmap_8bit, depthmap_color, cv::COLORMAP_RAINBOW);
  cv::Mat depth_on_canvas = cv::Mat(depth_map.rows, depth_map.cols, CV_8UC3, cv::Scalar(1,1,1)*255);
  depthmap_color.copyTo(depth_on_canvas, semidense_mask);
  cv::imwrite("depth_colored.png", depth_on_canvas);


  // 3. Convert semi-dense depth map to point cloud
  EMVS::OptionsPointCloud opts_pc;
  opts_pc.radius_search_ = FLAGS_radius_search;
  opts_pc.min_num_neighbors_ = FLAGS_min_num_neighbors;
  
  EMVS::PointCloud::Ptr pc (new EMVS::PointCloud);
  mapper.getPointcloud(depth_map, semidense_mask, opts_pc, pc);
  
  // Save point cloud to disk
  pcl::io::savePCDFileASCII ("pointcloud.pcd", *pc);
  LOG(INFO) << "Saved " << pc->points.size () << " data points to pointcloud.pcd";
  
  return 0;
}
