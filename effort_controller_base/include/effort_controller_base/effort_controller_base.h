////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    effort_controller_base.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef EFFORT_CONTROLLER_BASE_H_INCLUDED
#define EFFORT_CONTROLLER_BASE_H_INCLUDED

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <effort_controller_base/IKSolver.h>
#include <effort_controller_base/SpatialPDController.h>
#include <effort_controller_base/Utility.h>
#include <controller_interface/controller_interface.hpp>
#include <functional>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/solveri.hpp>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>

namespace effort_controller_base
{

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
class EffortControllerBase : public controller_interface::ControllerInterface
{
  public:
    EffortControllerBase();
    virtual ~EffortControllerBase(){};

    virtual controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    virtual controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    virtual LifecycleNodeInterface::CallbackReturn on_init() override;


    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

  protected:
    /**
     * @brief Write joint control commands to the real hardware
     *
     * Depending on the hardware interface used, this is either joint positions
     * or velocities.
     */
    void writeJointEffortCmds();

    /**
     * @brief Compute one control step using forward dynamics simulation
     *
     * Check \ref ForwardDynamicsSolver for details.
     *
     * @param error The error to minimize
     * @param period The period for this control cycle
     */
    void computeJointEffortCmds(const ctrl::VectorND& error);

    /**
     * @brief Display the given vector in the given robot base link
     *
     * @param vector The quantity to transform
     * @param from The reference frame where the quantity was formulated
     *
     * @return The quantity in the robot base frame
     */
    ctrl::Vector6D displayInBaseLink(const ctrl::Vector6D& vector, const std::string& from);

    /**
     * @brief Display the given tensor in the robot base frame
     *
     * @param tensor The quantity to transform
     * @param from The reference frame where the quantity was formulated
     *
     * @return The quantity in the robot base frame
     */
    ctrl::Matrix6D displayInBaseLink(const ctrl::Matrix6D& tensor, const std::string& from);

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
    ctrl::Vector6D displayInTipLink(const ctrl::Vector6D& vector, const std::string& to);

    void updateJointStates();
    /**
     * @brief Check if specified links are part of the robot chain
     *
     * @param s Link to check for existence
     *
     * @return True if existent, false otherwise
     */
    bool robotChainContains(const std::string& s)
    {
      for (const auto& segment : this->m_robot_chain.segments)
      {
        if (segment.getName() == s)
        {
          return true;
        }
      }
      return false;
    }

    void computeNullSpace(const KDL::Frame& desired_pose);

    KDL::Chain m_robot_chain;
    KDL::Jacobian  m_jacobian;            // Jacobian

    std::shared_ptr<KDL::ChainJntToJacSolver> m_jnt_to_jac_solver;
    std::shared_ptr<KDL::TreeFkSolverPos_recursive> m_forward_kinematics_solver;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive>  m_fk_solver;
    std::shared_ptr<KDL::ChainIkSolverPos_NR_JL>  m_ik_solver;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv>  m_ik_solver_vel;
    std::shared_ptr<KDL::ChainDynParam>  m_dyn_solver;

    /**
     * @brief Allow users to choose the IK solver type on startup
     */
    // std::shared_ptr<pluginlib::ClassLoader<IKSolver> > m_solver_loader;
    // std::shared_ptr<IKSolver> m_ik_solver;

    // Dynamic parameters
    std::string m_end_effector_link;
    std::string m_robot_base_link;
    int m_iterations;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
      m_joint_state_pos_handles;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
      m_joint_state_vel_handles;

    size_t m_joint_number;

    KDL::JntArray                                     m_joint_positions;
    KDL::JntArray                                     m_joint_velocities;
    KDL::JntArray                                     m_simulated_joint_motion;

  private:
    std::vector<std::string> m_cmd_interface_types;
    std::vector<std::string> m_state_interface_types;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> m_joint_cmd_eff_handles;

    std::vector<std::string>                          m_joint_names;
    SpatialPDController                               m_spatial_controller;
    ctrl::VectorND                                    m_efforts;
    std::string                                       m_controller_name;
    

    // Against multi initialization in multi inheritance scenarios
    bool m_initialized = {false};
    bool m_configured = {false};
    bool m_active = {false};

    // Dynamic parameters
    double m_error_scale;
    std::string m_robot_description;

    // Effort limits
    KDL::JntArray m_joint_effort_limits;
    double m_delta_tau_max;
    
};

}

#endif
