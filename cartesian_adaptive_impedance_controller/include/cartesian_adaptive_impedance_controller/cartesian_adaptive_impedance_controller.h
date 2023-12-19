#ifndef CARTESIAN_COMPLIANCE_CONTROLLER_H_INCLUDED
#define CARTESIAN_COMPLIANCE_CONTROLLER_H_INCLUDED

#include <effort_controller_base/effort_controller_base.h>
#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include <controller_interface/controller_interface.hpp>
#include <cartesian_adaptive_impedance_controller/qpOASES.hpp>

namespace cartesian_adaptive_impedance_controller
{

/**
 * @brief A ROS2-control controller for Cartesian compliance control
 *
 * This controller is the combination of the \ref CartesianMotionController and
 * the \ref CartesianImpedanceController.  Users can use this controller to track
 * Cartesian end effector motion that involves contact with the environment.
 * During operation, both interfaces can be used to command target poses
 * and target wrenches in parallel.
 * While the PD gains determine the controllers responsiveness, users can
 * additionally set a 6-dimensional stiffness for this controller, relating
 * the target pose offset to reaction forces with the environment.
 *
 * Note that the target wrench is superimposed with this stiffness, meaning that
 * the target wrench is fully compensated at some point by the virtual stiffness.
 * A common application is the tracking of a moving target in close proximity
 * to a surface, and applying an additional force profile to that surface.
 * To compensate for bigger offsets, users can set a low stiffness for the axes
 * where the additional forces are applied.
 *
 */
class CartesianAdaptiveImpedanceController
: public cartesian_impedance_controller::CartesianImpedanceController
{
  public:
    CartesianAdaptiveImpedanceController();

    virtual LifecycleNodeInterface::CallbackReturn on_init() override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    // void updateMinimizationVariables();

    void computeDesiredStiffness();

    using Base = effort_controller_base::EffortControllerBase;
    using ImpedanceBase = cartesian_impedance_controller::CartesianImpedanceController;

  private:
    /**
     * @brief Compute the net force of target wrench and stiffness-related pose offset
     *
     * @return The remaining error wrench, given in robot base frame
     */
    ctrl::Vector6D        computeComplianceError();

    ctrl::Vector3D        m_stiffness_min;
    ctrl::Vector3D        m_stiffness_max;
    ctrl::Vector3D        m_Q_matrix;
    ctrl::Vector3D        m_R_matrix;
    ctrl::Vector3D        m_force_max;

    size_t                m_window_length;
    std::string           m_compliance_ref_link;

    std::vector<ctrl::Vector3D>   m_external_forces;
    std::vector<ctrl::Vector3D>   m_desired_forces;
    std::vector<ctrl::Matrix3D>   m_desired_stiffness;
};

}

#endif
