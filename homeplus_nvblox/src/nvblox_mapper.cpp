// Copyright 2026 HomePlus contributors. SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <cuda_runtime_api.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <nvblox/mapper/mapper.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

using Apply = moveit_msgs::srv::ApplyPlanningScene;
using GetScene = moveit_msgs::srv::GetPlanningScene;
using Trigger = std_srvs::srv::Trigger;

class NvbloxMapper : public rclcpp::Node {
 public:
  NvbloxMapper() : Node("nvblox_mapper") {
    declare_parameter("robot_description", "");
    declare_parameter("robot_description_semantic", "");
    frame_ = declare_parameter("world_frame", "world");
    voxel_ = declare_parameter("voxel_size_m", 0.02);
    stride_ = declare_parameter("depth_downsample", 4);
    max_depth_ = declare_parameter("max_depth_m", 2.0);
    max_age_ = declare_parameter("max_cloud_age_sec", 2.0);
    min_frames_ = declare_parameter("min_integration_frames", 3);
    padding_ = declare_parameter("self_filter_padding_m", 0.01);
    max_triangles_ = declare_parameter("max_mesh_triangles", 200000);
    max_blocks_ = declare_parameter("max_tsdf_blocks", 30000);
    timeout_ = declare_parameter("scene_timeout_sec", 5.0);
    cloud_topic_ = declare_parameter("cloud_topic", "/obstacle_cloud");
    info_topic_ = declare_parameter("camera_info_topic", "/cameras/d405/color/camera_info");
    if (voxel_ <= 0 || stride_ < 1 || min_frames_ < 1 || max_depth_ <= 0 ||
        max_age_ <= 0 || padding_ < 0 || max_triangles_ < 1 || max_blocks_ < 1 || timeout_ <= 0)
      throw std::invalid_argument("Invalid nvblox mapping parameters");
    int devices = 0;
    const auto status = cudaGetDeviceCount(&devices);
    if (status != cudaSuccess || devices == 0)
      throw std::runtime_error(std::string("Nvblox requires a CUDA GPU: ") + cudaGetErrorString(status));
    stream_ = std::make_shared<nvblox::CudaStreamOwning>();
    reset_mapper();
  }

  void init() {
    loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description", false);
    model_ = loader_->getModel();
    if (!model_) throw std::runtime_error("Robot model is required for depth self-filtering");
    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_, shared_from_this(), true);
    scene_ = create_client<Apply>("/apply_planning_scene");
    scene_names_ = create_client<GetScene>("/get_planning_scene");
    ready_ = create_publisher<std_msgs::msg::String>("/vision/planning_map_ready", 10);
    mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>("/nvblox/mesh", rclcpp::QoS(1).transient_local());
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(info_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { info_ = msg; });
    motion_sub_ = create_subscription<std_msgs::msg::Bool>("/arduino/status", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (settled_ != msg->data) { cutoff_ = now().nanoseconds(); pending_cloud_.reset(); }
        settled_ = msg->data;
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, rclcpp::QoS(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (settled_ && !reset_header_ && !pending_cloud_) pending_cloud_ = msg;
      });
    // Deferred response: the clear service succeeds only after MoveIt removes our mesh.
    clear_ = create_service<Trigger>("/nvblox/clear",
      [this](std::shared_ptr<rmw_request_id_t> header, Trigger::Request::SharedPtr) {
        if (reset_header_) {
          Trigger::Response response; response.message = "A map reset is already pending";
          clear_->send_response(*header, response); return;
        }
        reset_header_ = header;
        pending_cloud_.reset();
        cutoff_ = now().nanoseconds();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Nvblox TSDF %.3f m; MoveIt collision meshes; readiness requires %d frames and scene acceptance",
                voxel_, min_frames_);
  }

 private:
  void reset_mapper() {
    mapper_ = std::make_unique<nvblox::Mapper>(voxel_, nvblox::BlockMemoryPoolParams(),
                                             nvblox::ProjectiveLayerType::kTsdf, stream_);
    mapper_->tsdf_integrator().max_integration_distance_m(max_depth_);
    mapper_->tsdf_integrator().truncation_distance_vox(3.0f);
    mapper_->tsdf_integrator().max_weight(20.0f);
    mapper_->tsdf_integrator().weighting_function_type(nvblox::WeightingFunctionType::kConstantWeight);
    mapper_->color_mesh_integrator().min_weight(2.0f);
    frames_ = 0; last_stamp_ = 0;
  }

  void tick() {
    if (request_pending_) {
      if ((now() - request_time_).seconds() > timeout_) {
        if (checking_scene_) scene_names_->remove_pending_request(request_id_);
        else scene_->remove_pending_request(request_id_);
        checking_scene_ = false;
        request_pending_ = false; uncertain_ = true;
        ++scene_serial_;  // Invalidate any response already queued before timeout.
        finish_reset(false, "MoveIt scene update timed out");
        RCLCPP_ERROR(get_logger(), "MoveIt scene update timed out; reset required before planning");
      }
      return;
    }
    if (!scene_->service_is_ready()) return;
    if (reset_header_) {
      if (!scene_names_->service_is_ready()) return;
      auto request = std::make_shared<GetScene::Request>();
      request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;
      request_pending_ = checking_scene_ = true;
      request_time_ = now();
      const auto serial = ++scene_serial_;
      auto future = scene_names_->async_send_request(request, [this, serial](rclcpp::Client<GetScene>::SharedFuture result) {
        if (serial != scene_serial_) return;
        request_pending_ = checking_scene_ = false;
        has_map_ = false;
        for (const auto& object : result.get()->scene.world.collision_objects)
          if (object.id == "nvblox_environment") has_map_ = true;
        reset_mapper();
        submit(shape_msgs::msg::Mesh(), 0, true);
      });
      request_id_ = future.request_id;
      return;
    }
    if (uncertain_ || !settled_ || !info_ || !pending_cloud_) return;
    auto cloud = pending_cloud_;
    pending_cloud_.reset();
    const auto stamp = rclcpp::Time(cloud->header.stamp).nanoseconds();
    const double age = (now().nanoseconds() - stamp) / 1e9;
    if (stamp <= cutoff_ || stamp <= last_stamp_ || age < 0 || age > max_age_) return;
    try {
      integrate(*cloud);
      last_stamp_ = stamp; ++frames_;
      if (mapper_->tsdf_layer().numAllocatedBlocks() > static_cast<size_t>(max_blocks_))
        throw std::runtime_error("TSDF block budget exceeded; reset required");
      if (frames_ < min_frames_) return;
      mapper_->updateColorMesh();
      auto mesh = extract_mesh();
      if (!mesh.triangles.empty()) submit(std::move(mesh), stamp, false);
    } catch (const tf2::TransformException& error) {
      // Keep acquisition time fixed while joint feedback/TF catches up.
      // The age gate above bounds how long this observation can be retried.
      pending_cloud_ = cloud;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Image-time TF missing: %s", error.what());
    } catch (const std::exception& error) {
      uncertain_ = true;
      RCLCPP_ERROR(get_logger(), "Mapping stopped; reset required: %s", error.what());
    }
  }

  void integrate(const sensor_msgs::msg::PointCloud2& cloud) {
    if (info_->header.frame_id != cloud.header.frame_id || info_->k[0] <= 0 || info_->k[4] <= 0 ||
        info_->width == 0 || info_->height == 0 || info_->width % stride_ || info_->height % stride_)
      throw std::runtime_error("Cloud must retain the calibrated optical frame; camera dimensions must divide by depth_downsample");
    const auto stamp = rclcpp::Time(cloud.header.stamp);
    const Eigen::Isometry3d world_camera = tf2::transformToEigen(tf_->lookupTransform(frame_, cloud.header.frame_id, stamp));
    std::vector<std::unique_ptr<bodies::Body>> bodies;
    for (const auto* link : model_->getLinkModelsWithCollisionGeometry()) {
      const Eigen::Isometry3d camera_link = tf2::transformToEigen(tf_->lookupTransform(cloud.header.frame_id, link->getName(), stamp));
      const auto& shapes = link->getShapes();
      for (size_t i = 0; i < shapes.size(); ++i) {
        std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shapes[i].get()));
        if (!body) throw std::runtime_error("Could not build robot collision body");
        body->setPadding(padding_);
        body->setPose(camera_link * link->getCollisionOriginTransforms()[i]);
        bodies.push_back(std::move(body));
      }
    }
    const int width = info_->width / stride_, height = info_->height / stride_;
    const float fx = info_->k[0] / stride_, fy = info_->k[4] / stride_;
    // Cloud sampling picks pixels 0,stride,...: no half-pixel shift.
    const float cx = info_->k[2] / stride_, cy = info_->k[5] / stride_;
    std::vector<float> depth(width * height, 0.0f);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    size_t valid = 0;
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) || *z <= 0 || *z > max_depth_) continue;
      Eigen::Vector3d point(*x, *y, *z);
      if (std::any_of(bodies.begin(), bodies.end(), [&](const auto& body) { return body->containsPoint(point); })) continue;
      const int u = std::lround(fx * *x / *z + cx), v = std::lround(fy * *y / *z + cy);
      if (u < 0 || v < 0 || u >= width || v >= height) continue;
      auto& d = depth[v * width + u];
      if (d == 0 || *z < d) d = *z;
      ++valid;
    }
    if (!valid) throw std::runtime_error("No valid depth after robot self-filtering");
    nvblox::DepthImage image(nvblox::MemoryType::kDevice);
    image.copyFrom(height, width, depth.data());
    mapper_->integrateDepth(image, world_camera.cast<float>(), nvblox::Camera(fx, fy, cx, cy, width, height));
    stream_->synchronize();
  }

  shape_msgs::msg::Mesh extract_mesh() {
    shape_msgs::msg::Mesh result;
    const auto& layer = mapper_->color_mesh_layer();
    for (const auto& index : layer.getAllBlockIndices()) {
      const auto block = layer.getBlockAtIndex(index);
      auto vertices = block->vertices.toVectorAsync(*stream_);
      auto triangles = block->triangles.toVectorAsync(*stream_);
      stream_->synchronize();
      if (result.triangles.size() + triangles.size() / 3 > static_cast<size_t>(max_triangles_))
        throw std::runtime_error("Collision mesh triangle budget exceeded");
      const size_t offset = result.vertices.size();
      for (const auto& v : vertices) {
        geometry_msgs::msg::Point point; point.x = v.x(); point.y = v.y(); point.z = v.z();
        result.vertices.push_back(point);
      }
      for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
        shape_msgs::msg::MeshTriangle triangle;
        for (size_t j = 0; j < 3; ++j) triangle.vertex_indices[j] = offset + triangles[i+j];
        result.triangles.push_back(triangle);
      }
    }
    return result;
  }

  void submit(shape_msgs::msg::Mesh mesh, int64_t stamp, bool removing) {
    auto request = std::make_shared<Apply::Request>();
    request->scene.is_diff = true;
    request->scene.robot_state.is_diff = true;
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_; object.id = "nvblox_environment";
    object.operation = removing ? object.REMOVE : object.ADD;
    if (!removing) {
      object.meshes.push_back(mesh);
      geometry_msgs::msg::Pose pose; pose.orientation.w = 1;
      object.mesh_poses.push_back(pose);
    }
    // MoveIt may reject removing a missing object. An empty scene diff still
    // provides acknowledgement when the queried scene had no mapping mesh.
    if (!removing || has_map_) request->scene.world.collision_objects.push_back(object);
    request_pending_ = true; request_time_ = now();
    const auto serial = ++scene_serial_;
    auto future = scene_->async_send_request(request,
      [this, mesh = std::move(mesh), stamp, removing, serial](rclcpp::Client<Apply>::SharedFuture result) {
        if (serial != scene_serial_) return;
        request_pending_ = false;
        const bool success = result.get()->success;
        if (success) has_map_ = !removing;
        if (removing) {
          if (success) { ++generation_; uncertain_ = false; cutoff_ = now().nanoseconds(); }
          finish_reset(success, success ? std::to_string(generation_) : "MoveIt rejected map removal");
        }
        if (!success) { uncertain_ = true; RCLCPP_ERROR(get_logger(), "MoveIt rejected nvblox collision scene"); return; }
        publish_mesh(mesh, removing);
        if (!removing && settled_ && !reset_header_ && stamp > cutoff_) {
          std_msgs::msg::String ready;
          ready.data = "{\"stamp_ns\":" + std::to_string(stamp) + ",\"generation\":" + std::to_string(generation_) +
            ",\"frames\":" + std::to_string(frames_) + ",\"triangles\":" + std::to_string(mesh.triangles.size()) + "}";
          ready_->publish(ready);
        }
      });
    request_id_ = future.request_id;
  }

  void finish_reset(bool success, const std::string& message) {
    if (!reset_header_) return;
    Trigger::Response response; response.success = success; response.message = message;
    clear_->send_response(*reset_header_, response); reset_header_.reset();
  }

  void publish_mesh(const shape_msgs::msg::Mesh& mesh, bool removing) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_; marker.header.stamp = now();
    marker.ns = "nvblox"; marker.id = 0; marker.type = marker.TRIANGLE_LIST;
    marker.action = removing ? marker.DELETE : marker.ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = marker.scale.y = marker.scale.z = 1;
    marker.color.r = 0.25; marker.color.g = 0.65; marker.color.b = 1; marker.color.a = 0.7;
    for (const auto& triangle : mesh.triangles)
      for (const auto index : triangle.vertex_indices) marker.points.push_back(mesh.vertices.at(index));
    mesh_pub_->publish(marker);
  }

  std::string frame_, cloud_topic_, info_topic_;
  double voxel_, max_depth_, max_age_, padding_, timeout_;
  int stride_, min_frames_, max_triangles_, max_blocks_, frames_ = 0, generation_ = 0;
  int64_t last_stamp_ = 0, cutoff_ = 0, request_id_ = 0;
  uint64_t scene_serial_ = 0;
  bool settled_ = false, request_pending_ = false, uncertain_ = false;
  bool checking_scene_ = false, has_map_ = false;
  rclcpp::Time request_time_{0, 0, RCL_ROS_TIME};
  std::shared_ptr<rmw_request_id_t> reset_header_;
  std::shared_ptr<nvblox::CudaStream> stream_;
  std::unique_ptr<nvblox::Mapper> mapper_;
  std::unique_ptr<robot_model_loader::RobotModelLoader> loader_;
  moveit::core::RobotModelPtr model_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  rclcpp::Client<Apply>::SharedPtr scene_;
  rclcpp::Client<GetScene>::SharedPtr scene_names_;
  rclcpp::Service<Trigger>::SharedPtr clear_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ready_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<NvbloxMapper>(); node->init(); rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("nvblox_mapper"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
