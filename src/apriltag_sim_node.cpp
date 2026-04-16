#include "apriltag_sim/apriltag_sim_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

namespace apriltag_sim
{

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;

tf2::Transform makeTagTransform(double x, double y, double z, double yaw_face)
{
  // Tag axes in map:
  //   +z = direction the tag faces (away from the wall, toward observer)
  //   +y = world up
  //   +x = y cross z
  const double c = std::cos(yaw_face);
  const double s = std::sin(yaw_face);
  tf2::Matrix3x3 R;
  R.setValue(
    -s, 0.0, c,
     c, 0.0, s,
    0.0, 1.0, 0.0);
  tf2::Quaternion q;
  R.getRotation(q);
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(x, y, z));
  T.setRotation(q);
  return T;
}
}  // namespace

AprilTagSimNode::AprilTagSimNode(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  pose_topic_ = node_->declare_parameter<std::string>("pose_topic", "/sim_truth/pose");
  base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_footprint");
  const double publish_rate_hz = node_->declare_parameter<double>("publish_rate_hz", 20.0);

  use_sim_world_ = node_->declare_parameter<bool>("use_sim_world", false);

  loadCameras();
  if (!use_sim_world_) {
    loadTags();
  }
  createEnableServices();

  // sim_world integration (optional)
  if (use_sim_world_) {
    auto qos = rclcpp::QoS(1)
      .reliable()
      .transient_local();
    sub_sim_world_tags_ = node_->create_subscription<sim_world_msgs::msg::SimTagArray>(
      "/sim_world/tags", qos,
      std::bind(&AprilTagSimNode::onSimWorldTags, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),
      "use_sim_world=true, subscribing to /sim_world/tags");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  const auto pose_qos = rclcpp::QoS(10).reliable();
  sub_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, pose_qos,
    std::bind(&AprilTagSimNode::onPose, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(
    publish_rate_hz > 0.0 ? 1.0 / publish_rate_hz : 0.05);
  timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&AprilTagSimNode::tick, this));

  RCLCPP_INFO(node_->get_logger(),
    "[apriltag_sim] Ready. pose=%s base=%s cameras=%zu tags=%zu rate=%.1fHz",
    pose_topic_.c_str(), base_frame_.c_str(),
    cameras_.size(), tags_.size(), publish_rate_hz);
}

void AprilTagSimNode::loadCameras()
{
  const auto names = node_->declare_parameter<std::vector<std::string>>(
    "cameras.names", std::vector<std::string>{});
  for (const auto & name : names) {
    CameraCfg cam;
    cam.name = name;
    const std::string p = "cameras." + name + ".";
    cam.optical_frame = node_->declare_parameter<std::string>(
      p + "optical_frame", name + "_realsense_color_optical_frame");
    cam.hfov_rad = node_->declare_parameter<double>(p + "hfov_deg", 69.4) * kDeg2Rad;
    cam.vfov_rad = node_->declare_parameter<double>(p + "vfov_deg", 42.5) * kDeg2Rad;
    cam.max_range_m = node_->declare_parameter<double>(p + "max_range", 3.0);
    cam.min_range_m = node_->declare_parameter<double>(p + "min_range", 0.1);
    cameras_.push_back(std::move(cam));
  }
}

void AprilTagSimNode::loadTags()
{
  const auto names = node_->declare_parameter<std::vector<std::string>>(
    "tag_names", std::vector<std::string>{});
  const auto xs = node_->declare_parameter<std::vector<double>>(
    "tag_xs", std::vector<double>{});
  const auto ys = node_->declare_parameter<std::vector<double>>(
    "tag_ys", std::vector<double>{});
  const auto zs = node_->declare_parameter<std::vector<double>>(
    "tag_zs", std::vector<double>{});
  const auto yaws = node_->declare_parameter<std::vector<double>>(
    "tag_yaws_face", std::vector<double>{});

  const size_t n = names.size();
  if (xs.size() != n || ys.size() != n || zs.size() != n || yaws.size() != n) {
    RCLCPP_ERROR(node_->get_logger(),
      "[apriltag_sim] tag arrays length mismatch: names=%zu xs=%zu ys=%zu zs=%zu yaws=%zu",
      names.size(), xs.size(), ys.size(), zs.size(), yaws.size());
    return;
  }
  tags_.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    TagCfg t;
    t.name = names[i];
    t.T_map_tag = makeTagTransform(xs[i], ys[i], zs[i], yaws[i]);
    tags_.push_back(std::move(t));
  }
}

void AprilTagSimNode::createEnableServices()
{
  const auto service_names = node_->declare_parameter<std::vector<std::string>>(
    "enable_service_names", std::vector<std::string>{});
  for (const auto & name : service_names) {
    auto srv = node_->create_service<std_srvs::srv::SetBool>(
      name,
      [this, name](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res)
      {
        (void)req;
        res->success = true;
        res->message = "sim noop";
        RCLCPP_INFO(node_->get_logger(),
          "[apriltag_sim] %s called (data=%s) → noop",
          name.c_str(), req->data ? "true" : "false");
      });
    enable_services_.push_back(srv);
    RCLCPP_INFO(node_->get_logger(),
      "[apriltag_sim] Advertising enable service: %s", name.c_str());
  }

  // DetectParameter shim services (amr_ex_action calls /apriltag/set_parameter).
  // When called, store the target_id → alias frame mapping so that tick()
  // publishes the matching tag's TF with the alias as child_frame_id
  // (e.g. "reference_target"), mimicking production apriltag-detector behaviour.
  const auto detect_param_names = node_->declare_parameter<std::vector<std::string>>(
    "detect_param_service_names", std::vector<std::string>{});
  for (const auto & name : detect_param_names) {
    auto srv = node_->create_service<amr_ex_action::srv::DetectParameter>(
      name,
      [this, name](
        const std::shared_ptr<amr_ex_action::srv::DetectParameter::Request> req,
        std::shared_ptr<amr_ex_action::srv::DetectParameter::Response> res)
      {
        {
          std::lock_guard<std::mutex> lk(alias_mtx_);
          active_alias_.target_id = req->target_id;
          active_alias_.frame_name = req->name;
        }
        res->success = true;
        res->message = "sim alias set";
        RCLCPP_INFO(node_->get_logger(),
          "[apriltag_sim] %s called: target_id=%d alias=%s",
          name.c_str(), req->target_id, req->name.c_str());
      });
    detect_param_services_.push_back(srv);
    RCLCPP_INFO(node_->get_logger(),
      "[apriltag_sim] Advertising detect_param service: %s", name.c_str());
  }
}

bool AprilTagSimNode::ensureCameraTf(CameraCfg & cam)
{
  if (cam.tf_cached) return true;
  try {
    const auto tf = tf_buffer_->lookupTransform(
      base_frame_, cam.optical_frame, tf2::TimePointZero);
    cam.T_base_opt.setOrigin(tf2::Vector3(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z));
    cam.T_base_opt.setRotation(tf2::Quaternion(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w));
    cam.tf_cached = true;
    RCLCPP_INFO(node_->get_logger(),
      "[apriltag_sim] Cached TF %s -> %s",
      base_frame_.c_str(), cam.optical_frame.c_str());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[apriltag_sim] TF %s -> %s not yet available: %s",
      base_frame_.c_str(), cam.optical_frame.c_str(), ex.what());
    return false;
  }
  return true;
}

void AprilTagSimNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  const tf2::Quaternion q(
    msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w);
  T.setRotation(q);
  const double yaw = tf2::getYaw(q);

  std::lock_guard<std::mutex> lk(mtx_);
  T_map_base_ = T;
  const bool first = !has_pose_;
  has_pose_ = true;
  if (first) {
    RCLCPP_INFO(node_->get_logger(),
      "[apriltag_sim] first pose: (%.3f, %.3f, yaw=%.3f)",
      msg->pose.position.x, msg->pose.position.y, yaw);
  }
}

void AprilTagSimNode::tick()
{
  tf2::Transform T_map_base;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_pose_ || tags_.empty() || cameras_.empty()) return;
    T_map_base = T_map_base_;
  }

  // For each tag, find the best camera that can see it (smallest range among visible),
  // then publish optical_frame -> tag_name.
  const rclcpp::Time now = node_->now();
  std::vector<geometry_msgs::msg::TransformStamped> to_send;
  to_send.reserve(tags_.size());

  for (const auto & tag : tags_) {
    const CameraCfg * best_cam = nullptr;
    double best_range = std::numeric_limits<double>::infinity();
    tf2::Transform best_T_opt_tag;

    for (auto & cam : cameras_) {
      if (!ensureCameraTf(cam)) continue;

      const tf2::Transform T_map_opt = T_map_base * cam.T_base_opt;
      const tf2::Transform T_opt_map = T_map_opt.inverse();
      const tf2::Transform T_opt_tag = T_opt_map * tag.T_map_tag;

      const tf2::Vector3 p = T_opt_tag.getOrigin();
      // In optical frame convention: +z is forward (camera view direction).
      if (p.z() <= 0.0) continue;
      const double range = p.length();
      if (range < cam.min_range_m || range > cam.max_range_m) continue;
      const double ang_h = std::atan2(std::fabs(p.x()), p.z());
      const double ang_v = std::atan2(std::fabs(p.y()), p.z());
      if (ang_h > cam.hfov_rad * 0.5 || ang_v > cam.vfov_rad * 0.5) continue;

      // Back-face: the camera must be on the +z side of the tag (i.e. in front of it).
      // camera position in tag frame:
      const tf2::Transform T_tag_map = tag.T_map_tag.inverse();
      const tf2::Vector3 cam_in_tag = T_tag_map * T_map_opt.getOrigin();
      if (cam_in_tag.z() <= 0.0) continue;

      if (range < best_range) {
        best_range = range;
        best_cam = &cam;
        best_T_opt_tag = T_opt_tag;
      }
    }

    if (best_cam) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = best_cam->optical_frame;
      t.child_frame_id = tag.name;
      t.transform.translation.x = best_T_opt_tag.getOrigin().x();
      t.transform.translation.y = best_T_opt_tag.getOrigin().y();
      t.transform.translation.z = best_T_opt_tag.getOrigin().z();
      const auto q = best_T_opt_tag.getRotation();
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      to_send.push_back(std::move(t));
    } else {
      // Fallback: tag outside all camera FOVs (common in sim where camera
      // mounting geometry makes close-range tags exceed the vertical FOV).
      // Publish base_footprint -> tag_name unconditionally so tf_util::get_tfs
      // can discover the tag for home detection and reference movement.
      const tf2::Transform T_base_tag = T_map_base.inverse() * tag.T_map_tag;

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = base_frame_;
      t.child_frame_id = tag.name;
      t.transform.translation.x = T_base_tag.getOrigin().x();
      t.transform.translation.y = T_base_tag.getOrigin().y();
      t.transform.translation.z = T_base_tag.getOrigin().z();
      const auto q = T_base_tag.getRotation();
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      to_send.push_back(std::move(t));
    }
  }

  // If an alias is active (set via /apriltag/set_parameter), publish an
  // additional TF for the matching tag using the alias frame name (e.g.
  // "reference_target"). This mimics production apriltag-detector behaviour
  // where set_parameter tells the detector to publish the detected tag pose
  // under a specific child_frame_id.
  {
    std::lock_guard<std::mutex> lk(alias_mtx_);
    if (active_alias_.target_id >= 0 && !active_alias_.frame_name.empty()) {
      // Build the expected tag name from the alias target_id.
      // Tag naming convention: home_<id>, e.g. home_11 for target_id=11.
      // Search to_send for a TF whose original tag name contains the id.
      for (const auto & tag : tags_) {
        // Extract numeric suffix from tag name (e.g. "home_11" → 11)
        const auto upos = tag.name.rfind('_');
        if (upos == std::string::npos) continue;
        int tag_id = -1;
        try { tag_id = std::stoi(tag.name.substr(upos + 1)); } catch (...) { continue; }
        if (tag_id != active_alias_.target_id) continue;

        // Found matching tag. Publish base_footprint -> alias with same transform.
        // No range gating for alias: the alias is used by amr_ex_action's PID
        // controller and must remain available as long as the AMR is moving.
        const tf2::Transform T_base_tag = T_map_base.inverse() * tag.T_map_tag;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = base_frame_;
        t.child_frame_id = active_alias_.frame_name;
        t.transform.translation.x = T_base_tag.getOrigin().x();
        t.transform.translation.y = T_base_tag.getOrigin().y();
        t.transform.translation.z = T_base_tag.getOrigin().z();
        const auto q = T_base_tag.getRotation();
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        to_send.push_back(std::move(t));
        break;
      }
    }
  }

  if (!to_send.empty()) {
    tf_broadcaster_->sendTransform(to_send);
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[apriltag_sim] published %zu tag TF(s)", to_send.size());
  }
}

void AprilTagSimNode::onSimWorldTags(
  const sim_world_msgs::msg::SimTagArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  tags_.clear();
  tags_.reserve(msg->tags.size());
  for (const auto & t : msg->tags) {
    TagCfg cfg;
    cfg.name = t.name;
    cfg.T_map_tag = makeTagTransform(t.x, t.y, t.z, t.yaw_face);
    tags_.push_back(std::move(cfg));
  }
  RCLCPP_DEBUG(node_->get_logger(),
    "[sim_world] received %zu tags", msg->tags.size());
}

}  // namespace apriltag_sim
