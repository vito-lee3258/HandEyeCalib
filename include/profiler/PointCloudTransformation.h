#pragma once
#include "CommonTypes.h"
#include "profiler/Profiler.h"
#include "api_global.h"
namespace mmind {
namespace eye {
/**
 * @brief Obtains the rigid body transformations of the custom reference frame of a laser profiler.
 * The custom reference frame can be adjusted using the "Custom Reference Frame" tool in Mech-Eye
 * Viewer. The rigid body transformations are automatically calculated after the settings in this
 * tool have been applied. The "Custom Reference Frame" tool is recommended as the GUI allows you to
 * adjust the reference frame easily and conveniently.
 * Alternatively, you can use the rotation and translation methods in @ref FrameTransformation to
 * define the transformations manually.
 * @param [in] profiler The @ref profiler handle.
 * @return The rigid body transformations from the laser profiler reference frame to the custom
 * reference frame. Refer to @ref FrameTransformation for more details.
 */
MMIND_API_EXPORT FrameTransformation getTransformationParams(Profiler& profiler);
} // namespace eye
} // namespace mmind
