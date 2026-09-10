#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

#include "asr_summer_school/frontier_detection.h"

class FrontierDetectionNode : public rclcpp::Node
{
public:
  explicit FrontierDetectionNode(const rclcpp::NodeOptions & options)
  : Node("frontier_detection_node", options)
  {
    declare_parameter("epsilon", 0.5);
    declare_parameter("min_points", 3);
    declare_parameter("min_frontier_size", 5);
    declare_parameter("active_area_radius", 5.0);
    declare_parameter("map_topic", std::string("map"));
    declare_parameter("pose_topic", std::string("amcl_pose"));

    declare_parameter("use_tf_pose", true);
    declare_parameter("base_frame", std::string("base_link"));
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    params_.epsilon             = get_parameter("epsilon").as_double();
    params_.min_points          = get_parameter("min_points").as_int();
    params_.min_frontier_size   = get_parameter("min_frontier_size").as_int();
    params_.active_area_radius  = get_parameter("active_area_radius").as_double();

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "frontier_centroids", rclcpp::QoS(1).transient_local());

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("pose_topic").as_string(), rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
      });

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(), rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (get_parameter("use_tf_pose").as_bool()) {
          try {
            const auto transform = buffer_->lookupTransform(
              msg->header.frame_id, get_parameter("base_frame").as_string(), tf2::TimePointZero);
            if (std::abs((now() - rclcpp::Time(transform.header.stamp)).seconds()) > 3.0) {
              return;
            }
            robot_x_ = transform.transform.translation.x;
            robot_y_ = transform.transform.translation.y;
          } catch (const tf2::TransformException & error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
            return;
          }
        }
        auto centroids = frontier_detection::detect_frontiers(*msg, params_, robot_x_, robot_y_);
        frontier_detection::publish_frontiers_marker(
          marker_pub_, centroids, msg->header.frame_id, get_clock());
      });
  }

private:
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  frontier_detection::Params params_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(FrontierDetectionNode)
