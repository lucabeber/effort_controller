#ifndef EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED
#define EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED

#include <effort_controller_base/effort_controller_base.h>

#include <controller_interface/controller_interface.hpp>

#include "cbf_qp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
namespace cbf_cartesian_impedance_controller {

class CBFCartesianImpedanceController
    : public virtual effort_controller_base::EffortControllerBase {
 public:
  CBFCartesianImpedanceController();

  virtual LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

  ctrl::VectorND computeTorque();

  using Base = effort_controller_base::EffortControllerBase;

  ctrl::Matrix6D m_cartesian_stiffness;
  ctrl::Matrix6D m_cartesian_damping;
  double m_null_space_stiffness;
  double m_null_space_damping;
  ctrl::Vector6D m_target_wrench;

 private:
  ctrl::Vector6D compensateGravity();

  void targetWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void targetFrameCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr target);
  ctrl::Vector6D computeMotionError();

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_target_wrench_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_frame_subscriber;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      m_logger_publisher;
  KDL::Frame m_target_frame, m_old_target_frame, m_filtered_target;
  ctrl::Vector6D m_ft_sensor_wrench;
  std::string m_ft_sensor_ref_link;
  KDL::Frame m_ft_sensor_transform;

  KDL::JntArray m_null_space;
  KDL::Frame m_current_frame;

  ctrl::MatrixND m_identity;
  ctrl::VectorND m_q_starting_pose;
  ctrl::VectorND m_tau_old;

  ctrl::Vector3D m_old_rot_error;
  ctrl::VectorND m_old_vel_error;
  double const m_alpha = 0.3;
  rclcpp::Time m_last_time;
};

}  // namespace cbf_cartesian_impedance_controller

#endif