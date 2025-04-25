#pragma once
#include <string>
#include "Version.h"
#include "CommonTypes.h"

namespace mmind {
namespace eye {
/**
 * @brief Describes the laser profiler information.
 */
struct ProfilerInfo
{
    std::string model{};        ///< The laser profiler model.
    std::string controllerSN{}; ///< The controller serial number.
    std::string sensorSN{};     ///< The sensor serial number.
    Version hardwareVersion{};  ///< The version of the hardware. The hardware cannot be upgraded.
    Version firmwareVersion{};  ///< The version of the firmware. The firmware can be upgraded.
    std::string ipAddress{};    ///< The IP address of the laser profiler.
    std::string subnetMask{"255.255.255.0"}; ///< The IP subnet mask of the laser profiler.
    IpAssignmentMethod
        ipAssignmentMethod{}; ///< The IP address assignment method of the laser profiler.
    uint16_t port{};          ///< The laser profiler port.
};
} // namespace eye
} // namespace mmind
