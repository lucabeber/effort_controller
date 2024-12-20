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
/*!\file    ForwardDynamicsSolver.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2016/02/14
 *
 */
//-----------------------------------------------------------------------------

#ifndef FORWARD_DYNAMICS_SOLVER_H_INCLUDED
#define FORWARD_DYNAMICS_SOLVER_H_INCLUDED

#include "rclcpp/node.hpp"
#include <effort_controller_base/IKSolver.h>
#include <effort_controller_base/Utility.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <memory>
#include <vector>

namespace effort_controller_base {

template <class T> class LowPassFilter {
public:
  LowPassFilter();
  ~LowPassFilter();

  /**
   * @brief Initializes the filter with the given parameters.
   *
   * @param nh Shared pointer to the lifecycle node.
   * @param alpha Smoothing factor for the filter, must be in the range [0, 1].
   * @param step Time step for the filter.
   * @return true if initialization is successful, false otherwise.
   */
  bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh, double alpha,
            double step) {
    if (!setAlpha(alpha)) {
      RCLCPP_ERROR(nh->get_logger(), "Alpha value must be in the range [0, 1]");
      return false;
    }
    m_time_step = step;
    m_current_value = 0.0;
  }

  /**
   * @brief Sets the alpha value for the filter.
   *
   * This function sets the alpha value, which must be between 0.0 and 1.0
   * inclusive. The alpha value is used to control the filter's behavior.
   *
   * @param alpha The alpha value to set. Must be between 0.0 and 1.0.
   * @return true if the alpha value is valid and has been set, false otherwise.
   */
  bool setAlpha(double alpha) {
    if (alpha < 0.0 || alpha > 1.0) {
      return false;
    }

    m_alpha = alpha;
    return true;
  }

  /**
   * @brief Updates the filter with a new value.
   *
   * This function updates the filter with a new value and returns the filtered
   * value.
   *
   * @param new_value The new value to update the filter with.
   * @return The filtered value.
   */
  T update(T new_value) {
    m_current_value = m_alpha * new_value + (1 - m_alpha) * m_current_value;
    return m_current_value;
  }

private:
  double m_alpha;
  T m_current_value;
  double m_time_step;
};

} // namespace effort_controller_base

#endif
