#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;
class Visualizer {
 public:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  std::string reference_frame_;
  double plane_size_;

  void draw_scene(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
                      constraint_planes,
                  Eigen::Vector3d target, Eigen::Vector3d filtered_target,
                  rclcpp::Time stamp, double radius) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = reference_frame_;
    marker.header.stamp = stamp;
    marker.id = 1;
    marker.ns = "filtered_target";
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = filtered_target(0);
    marker.pose.position.y = filtered_target(1);
    marker.pose.position.z = filtered_target(2);
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = radius * 2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);

    marker.ns = "target_pose";
    marker.id = 2;
    marker.pose.position.x = target(0);
    marker.pose.position.y = target(1);
    marker.pose.position.z = target(2);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker_array.markers.push_back(marker);

    marker.ns = "constraint_planes";
    // reset planes
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = plane_size_;
    marker.scale.y = plane_size_;
    marker.scale.z = 0.0001;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    for (size_t i = 0; i < constraint_planes.size(); i++) {
      auto n = constraint_planes[i].first;
      n = n.normalized();
      auto p = constraint_planes[i].second;
      auto rotation_axis = z_axis.cross(n);
      auto rotation_angle = std::acos(z_axis.dot(n));
      Eigen::Quaterniond q;
      q = Eigen::AngleAxisd(rotation_angle, rotation_axis);
      marker.id = i;
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();
      marker.pose.position.x = p(0);
      marker.pose.position.y = p(1);
      marker.pose.position.z = p(2);
      marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
  }
  Visualizer(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
                 publisher,
             std::string reference_frame, double plane_size)
      : marker_pub_(publisher),
        reference_frame_(reference_frame),
        plane_size_(plane_size) {}
};

#endif  // VISUALIZATION_HPP