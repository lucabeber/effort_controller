#ifndef EFFORT_CONTROLLER_BASE_H_INCLUDED
#define EFFORT_CONTROLLER_BASE_H_INCLUDED

#include <effort_controller_base/Utility.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>

#include <cmath>
#include <controller_interface/controller_interface.hpp>
#include <functional>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/solveri.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pseudo_inversion.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace effort_controller_base {

class RobotDescriptionListener : public rclcpp::Node {
public:
  RobotDescriptionListener(std::shared_ptr<std::string> robot_description_ptr,
                           const std::string &topic_name);
  bool m_description_received_ = false;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_description_sub_;
  std::shared_ptr<std::string> m_robot_description_ptr_;
};

/**
 * @brief Base class for each effort controller
 *
 * This class implements a common forward dynamics based solver for Effort
 * end effector error correction. Different child class controllers will define
 * what this error represents and should call \ref computeJointControlCmds with
 * that error.  The control commands are sent to the hardware with \ref
 * writeJointControlCmds.
 *
 */
class EffortControllerBase : public controller_interface::ControllerInterface {
public:
  EffortControllerBase();
  virtual ~EffortControllerBase(){};

  virtual controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  virtual controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  virtual LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  /**
   * @brief Write joint control commands to the real hardware
   *
   * Depending on the hardware interface used, this is either joint positions
   * or velocities.
   */
  void writeJointEffortCmds(ctrl::VectorND &target_joint_positions);

  /**
   * @brief Compute one control step using forward dynamics simulation
   *
   * Check \ref ForwardDynamicsSolver for details.
   *
   * @param error The error to minimize
   * @param period The period for this control cycle
   */
  void computeJointEffortCmds(const ctrl::VectorND &error);

  /**
   * @brief Display the given vector in the given robot base link
   *
   * @param vector The quantity to transform
   * @param from The reference frame where the quantity was formulated
   *
   * @return The quantity in the robot base frame
   */
  ctrl::Vector6D displayInBaseLink(const ctrl::Vector6D &vector,
                                   const std::string &from);

  /**
   * @brief Display the given tensor in the robot base frame
   *
   * @param tensor The quantity to transform
   * @param from The reference frame where the quantity was formulated
   *
   * @return The quantity in the robot base frame
   */
  ctrl::Matrix6D displayInBaseLink(const ctrl::Matrix6D &tensor,
                                   const std::string &from);

  /**
   * @brief Display a given vector in a new reference frame
   *
   * The vector is assumed to be given in the robot base frame.
   *
   * @param vector The quantity to transform
   * @param to The reference frame in which to formulate the quantity
   *
   * @return The quantity in the new frame
   */
  ctrl::Vector6D displayInTipLink(const ctrl::Vector6D &vector,
                                  const std::string &to);

  void updateJointStates();

  /**
   * @brief Check if specified links are part of the robot chain
   *
   * @param s Link to check for existence
   *
   * @return True if existent, false otherwise
   */
  bool robotChainContains(const std::string &s) {
    for (const auto &segment : this->m_robot_chain.segments) {
      if (segment.getName() == s) {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Computes the Inverse Kinematics (IK) solution for the given desired
   * pose.
   *
   * This function calculates the joint positions required to achieve the
   * specified end-effector pose using the Kinematics and Dynamics Library
   * (KDL).
   *
   * @param desired_pose The desired end-effector pose represented as a
   * KDL::Frame.
   */
  void computeIKSolution(const KDL::Frame &desired_pose,
                         ctrl::VectorND &simulated_joint_positions);

  KDL::Chain m_robot_chain;
  KDL::Jacobian m_jacobian; // Jacobian

  std::shared_ptr<KDL::ChainJntToJacSolver> m_jnt_to_jac_solver;
  std::shared_ptr<KDL::TreeFkSolverPos_recursive> m_forward_kinematics_solver;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> m_fk_solver;
  std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> m_ik_solver;

  std::shared_ptr<KDL::ChainIkSolverVel_pinv_nso> m_ik_solver_vel_nso;

  std::shared_ptr<KDL::ChainIkSolverVel_pinv> m_ik_solver_vel;
  std::shared_ptr<KDL::ChainDynParam> m_dyn_solver;

  /**
   * @brief Allow users to choose the IK solver type on startup
   */
  // std::shared_ptr<pluginlib::ClassLoader<IKSolver> > m_solver_loader;
  // std::shared_ptr<IKSolver> m_ik_solver;

  // Dynamic parameters
  std::string m_end_effector_link;
  std::string m_compliance_ref_link;
  std::string m_robot_base_link;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      m_joint_state_pos_handles;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      m_joint_state_vel_handles;
  size_t m_joint_number;

  KDL::JntArray m_joint_positions;
  KDL::JntArray m_joint_velocities;
  KDL::JntArray m_old_joint_velocities;
  KDL::JntArray m_simulated_joint_motion;

private:
  std::vector<std::string> m_cmd_interface_types;
  std::vector<std::string> m_state_interface_types;
  std::vector<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      m_joint_cmd_eff_handles;
  std::vector<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      m_joint_cmd_pos_handles;

  std::vector<std::string> m_joint_names;
  ctrl::VectorND m_efforts;
  std::string m_controller_name;

  // Against multi initialization in multi inheritance scenarios
  bool m_initialized = {false};
  bool m_configured = {false};
  bool m_active = {false};
  // joint velocity filter
  const double m_dotq_alpha = 0.3;
  // Dynamic parameters
  std::string m_robot_description;

  // Effort limits
  KDL::JntArray m_joint_effort_limits;
  double m_delta_tau_max;

  bool m_kuka_hw;
};

} // namespace effort_controller_base

#endif
