#ifndef EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED
#define EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED

#include <effort_controller_base/effort_controller_base.h>

#include <controller_interface/controller_interface.hpp>

#include "controller_interface/controller_interface.hpp"
#include "debug_msg/msg/debug.hpp"
#include "effort_controller_base/Utility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#if LOGGING
#include <lbr_fri_idl/msg/lbr_state.hpp>
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#endif
#define DEBUG 0

namespace kuka_cartesian_impedance_controller {

/**
 * @brief A ROS2-control controller for Effort force control
 *
 * This controller implements 6-dimensional end effector force control for
 * robots with a wrist force-torque sensor.  Users command
 * geometry_msgs::msg::WrenchStamped targets to steer the robot in task space.
 * The controller additionally listens to the specified force-torque sensor
 * signals and computes the superposition with the target wrench.
 *
 * The underlying solver maps this remaining wrench to joint motion.
 * Users can steer their robot with this control in free space. The speed of
 * the end effector motion is set with PD gains on each Effort axes.
 * In contact, the controller regulates the net force of the two wrenches to
 * zero.
 *
 * Note that during free motion, users can generally set higher control gains
 * for faster motion.  In contact with the environment, however, normally lower
 * gains are required to maintain stability.  The ranges to operate in mainly
 * depend on the stiffness of the environment and the controller cycle of the
 * real hardware, such that some experiments might be required for each use
 * case.
 *
 */
class KukaCartesianImpedanceController
    : public virtual effort_controller_base::EffortControllerBase {
public:
  KukaCartesianImpedanceController();

  virtual LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  ctrl::VectorND computeTorque();
  void computeTargetPos();

  using Base = effort_controller_base::EffortControllerBase;

  ctrl::Matrix6D m_cartesian_stiffness;
  //   ctrl::Matrix6D m_cartesian_damping;
  double m_null_space_stiffness;
  double m_null_space_damping;
  ctrl::Vector6D m_target_wrench;

private:
  ctrl::Vector6D compensateGravity();

  void
  targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target);
  ctrl::Vector6D computeMotionError();

#if LOGGING
  void stateCallback(const lbr_fri_idl::msg::LBRState::SharedPtr state);
  lbr_fri_idl::msg::LBRState m_state;
#endif
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_frame_subscriber;
  rclcpp::Publisher<debug_msg::msg::Debug>::SharedPtr m_data_publisher;
  KDL::Frame m_target_frame;
  ctrl::VectorND m_target_joint_position;
#if LOGGING
  XBot::MatLogger2::Ptr m_logger;
  XBot::MatAppender::Ptr m_logger_appender;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRState>::SharedPtr
      m_state_subscriber;
#endif
  KDL::JntArray m_null_space;
  KDL::Frame m_current_frame;

  ctrl::MatrixND m_identity;
  ctrl::VectorND m_q_starting_pose;

  double m_vel_old = 0.0;
  double current_acc_j0 = 0.0;
  bool m_compensate_dJdq = false;

  double m_last_time;

  /**
   * Allow users to choose whether to specify their target wrenches in the
   * end-effector frame (= True) or the base frame (= False). The first one
   * is easier for explicit task programming, while the second one is more
   * intuitive for tele-manipulation.
   */
  bool m_hand_frame_control;

};

} // namespace cartesian_impedance_controller

#endif