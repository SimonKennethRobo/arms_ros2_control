//
// ros2_control API compatibility shim.
//
// Foxy ships ros2_control 0.11, whose handle API predates two changes made on the
// way to Jazzy:
//
//   * reading   - Jazzy:  std::optional<double> get_optional() const
//                 Foxy:   double                get_value()    const   (throws on nullptr)
//   * writing   - Jazzy:  bool set_value(double)
//                 Foxy:   void set_value(double)
//
// The helpers below present the Jazzy-shaped reading API on top of Foxy so the
// call sites keep their std::optional handling. Foxy's get_value() cannot report
// "no value" the way Jazzy's does, so a read is always engaged; the has_value()
// checks at the call sites simply never fail on this distro.
//
#pragma once

#include <optional>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace arms_controller_common::compat
{
    inline std::optional<double> get_optional(const hardware_interface::LoanedStateInterface& interface)
    {
        return interface.get_value();
    }

    inline std::optional<double> get_optional(const hardware_interface::LoanedCommandInterface& interface)
    {
        return interface.get_value();
    }
}
