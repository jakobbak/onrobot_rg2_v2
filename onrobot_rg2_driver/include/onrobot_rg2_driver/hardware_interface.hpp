// Copyright 2022 Polarworks AS.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
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

#ifndef ONROBOT_RG2_DRIVER__HARDWARE_INTERFACE_H_
#define ONROBOT_RG2_DRIVER__HARDWARE_INTERFACE_H_

#include "rclcpp/macros.hpp"

#include "hardware_interface/actuator_interface.hpp"
#include "onrobot_rg2_driver/visibility_control.h"

namespace onrobot_rg2_driver
{
class OnRobotRG2GripperHardwareInterface : public hardware_interface::ActuatorInterface
{

  // copied and modified from https://github.com/PickNikRobotics/ros2_robotiq_gripper/blob/b9483bbf354fc5b5d209abc092cb2fa6e03b2426/robotiq_driver/include/robotiq_driver/hardware_interface.hpp
  // additionally compared to https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html visited on 2022-10-13
  // any possible extra member definitions (compared to the robotiq_driver header file) are
  // based on instructions in the above link - especially the phrase:
  //    5. Add a constructor without parameters and the following public methods implementing 
  //        LifecycleNodeInterface: 
  //            on_configure, 
  //            on_cleanup, 
  //            on_shutdown, 
  //            on_activate, 
  //            on_deactivate, 
  //            on_error; 
  //    and overriding [definition of]
  //        $InterfaceType$Interface: 
  //            on_init, 
  //            export_state_interfaces, 
  //            export_command_interfaces, 
  //            prepare_command_mode_switch (optional), 
  //            perform_command_mode_switch (optional), 
  //            read, 
  //            write.
  //        [where $InterfaceType$ can be 'Actuator', 'Sensor' or 'System' depending on the type of hardware you are using]
  // at time of writing it looks like the 'robotiq_driver' header file only implemented 
  // the non-optional virtual member functions of `hardware_interface::ActuatorInterface` 
  // plus an override of its `on_init()`. 
  // From `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface` they only implement
  // `on_activate` and `on_deactivate`. According to https://docs.ros2.org/latest/api/rclcpp_lifecycle/classrclcpp__lifecycle_1_1node__interfaces_1_1LifecycleNodeInterface.html (visited 2022-10-13)
  // all of the lifecycle `LifecycleNodeInterface` member functions are optional.
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OnRobotRG2GripperHardwareInterface)

  ONROBOT_RG2_DRIVER_PUBLIC
  OnRobotRG2GripperHardwareInterface();

  ONROBOT_RG2_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  ONROBOT_RG2_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ONROBOT_RG2_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ONROBOT_RG2_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  ONROBOT_RG2_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  ONROBOT_RG2_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  ONROBOT_RG2_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

};

}  // namespace onrobot_rg2_driver

#endif // ONROBOT_RG2_DRIVER__HARDWARE_INTERFACE_H_
