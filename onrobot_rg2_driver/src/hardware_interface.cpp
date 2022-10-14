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

#include "onrobot_rg2_driver/hardware_interface.hpp"

namespace onrobot_rg2_driver
{

OnRobotRG2GripperHardwareInterface::OnRobotRG2GripperHardwareInterface()
{
    // TODO: figure out actual implementation
}

hardware_interface::CallbackReturn OnRobotRG2GripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
    return hardware_interface::CallbackReturn::SUCCESS; // TODO: figure out actual implementation
}

std::vector<hardware_interface::StateInterface> OnRobotRG2GripperHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    return state_interfaces; // TODO: figure out actual implementation
}

std::vector<hardware_interface::CommandInterface> OnRobotRG2GripperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  return command_interfaces; // TODO: figure out actual implementation
}

hardware_interface::CallbackReturn OnRobotRG2GripperHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS; // TODO: figure out actual implementation
}

hardware_interface::CallbackReturn OnRobotRG2GripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS; // TODO: figure out actual implementation
}

hardware_interface::return_type OnRobotRG2GripperHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    return hardware_interface::return_type::OK; // TODO: figure out actual implementation
}

hardware_interface::return_type OnRobotRG2GripperHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    return hardware_interface::return_type::OK; // TODO: figure out actual implementation
}

} // namespace onrobot_rg2_driver
