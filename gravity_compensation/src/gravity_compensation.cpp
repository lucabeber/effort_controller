#include "effort_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include <gravity_compensation/gravity_compensation.h>


namespace gravity_compensation
{

GravityCompensation::GravityCompensation()
: Base::EffortControllerBase()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GravityCompensation::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GravityCompensation::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Finished Gravity Compensation on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GravityCompensation::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);

  // Update joint states
  Base::updateJointStates();
  
  RCLCPP_INFO(get_node()->get_logger(), "Finished Gravity Compensation on_activate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GravityCompensation::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  // Stop drifting by sending zero joint velocities
  Base::computeJointEffortCmds(ctrl::VectorND::Zero(m_joint_number));
  Base::writeJointEffortCmds();
  Base::on_deactivate(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Gravity Compensation on_deactivate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensation::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
{
  // Update joint states
  Base::updateJointStates();
  
  // Send 0 torque to the joints
  ctrl::VectorND tau_tot = ctrl::VectorND::Zero(m_joint_number);

  // Saturation of the torque
  Base::computeJointEffortCmds(tau_tot);

  // Write final commands to the hardware interface
  Base::writeJointEffortCmds();

  return controller_interface::return_type::OK;
}

}


// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(gravity_compensation::GravityCompensation, controller_interface::ControllerInterface)
