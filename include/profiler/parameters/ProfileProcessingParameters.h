#pragma once
#include "Parameter.h"

namespace mmind {
namespace eye {
namespace profile_processing {

class Filter
{
public:
    static constexpr const char* name = "Filter";

    static constexpr const char* description =
        "Set the type of filters. Filtering the profile can reduce noise and smooth the profile.\n"
        "None: does not perform filtering. Select this option when the profile does not contain "
        "noticeable noise.\nMean (edge preserving): performs mean filtering with edge "
        "preservation. Features with abrupt depth variations (such as object edges) are well "
        "preserved, but the smoothing effect around object edges is slightly worse. Suitable for "
        "objects that have features with abrupt depth variations. When selecting this optioin, set "
        "\"MeanFilterWindowSize\".\nMedian: performs median filtering, suitable for reducing noise "
        "with depth values significantly different from surrounding points. When selecting this "
        "option, set \"MedianFilterWindowSize\".";

    static constexpr Parameter::Type type = Parameter::Type::_Enum;

    enum struct Value {
        None,
        Mean,
        Median,
    };
};

class MeanFilterWindowSize
{
public:
    static constexpr const char* name = "MeanFilterWindowSize";

    static constexpr const char* description =
        "Set the window size of the mean filter.\nLarger window size results in higher intensity "
        "of smoothing but may also distort object features.\n\nNote:\n* \"MeanFilterWindowSize\" "
        "is unavailable when \"Filter\" is not set to \"Mean\".";

    static constexpr Parameter::Type type = Parameter::Type::_Enum;

    enum struct Value {
        WindowSize_2,
        WindowSize_4,
        WindowSize_8,
        WindowSize_16,
        WindowSize_32,
    };
};

class MedianFilterWindowSize
{
public:
    static constexpr const char* name = "MedianFilterWindowSize";

    static constexpr const char* description =
        "Set the window size of the median filter.\nLarger window size removes more "
        "noise.\n\nNote:\n* \"MedianFilterWindowSize\" is unavailable when \"Filter\" is not set "
        "to \"Median\".";

    static constexpr Parameter::Type type = Parameter::Type::_Enum;

    enum struct Value {
        WindowSize_3,
        WindowSize_5,
        WindowSize_7,
        WindowSize_9,
    };
};

class GapFilling
{
public:
    static constexpr const char* name = "GapFilling";

    static constexpr const char* description =
        "Set the size of the gaps that can be filled in the profile.\nWhen the number of "
        "consecutive data points in a gap in the profile is no greater than this value, this gap "
        "will be filled. The data used for filling is calculated based on the difference between "
        "the two neighboring points (that is, based on linear interpolation).";

    static constexpr Parameter::Type type = Parameter::Type::_Int;
};

class GapFillingEdgePreservation
{
public:
    static constexpr const char* name = "GapFillingEdgePreservation";

    static constexpr const char* description =
        "Set the degree of preservation of object edges when filling gaps.\n\nIf you need to "
        "preserve features with abrupt depth variations, such as object edges, you can increase "
        "this parameter, but the amount of gaps being filled will decrease.";

    static constexpr Parameter::Type type = Parameter::Type::_Int;
};

class Resampling
{
public:
    static constexpr const char* name = "Resampling";

    static constexpr const char* description =
        "Select the point to be retained during resampling. \nMultiple points with different Z "
        "values may exist at the same location on the X-axis. This parameter is used to select the "
        "point to be retained in such a situation.\n\nNearest: retains the point closest to the "
        "laser profiler.\nFarthest: retains the point farthest from the laser profiler.\nIf the "
        "needed feature is at the bottom of the target object (such as the inner bottom of a "
        "cylindrical container), you can select \"Farthest\"";

    static constexpr Parameter::Type type = Parameter::Type::_Enum;

    enum struct Value {
        Nearest,
        Farthest,
    };
};

class ResamplingEdgePreservation
{
public:
    static constexpr const char* name = "ResamplingEdgePreservation";

    static constexpr const char* description =
        "Set the degree of preservation of object edges during resampling. \nIf you need to "
        "preserve features with abrupt depth variations, such as object edges, you can increase "
        "this parameter.";

    static constexpr Parameter::Type type = Parameter::Type::_Int;
};

} // namespace profile_processing

namespace profile_alignment {

class EnableZAxisAlignment
{
public:
    static constexpr const char* name = "EnableZAxisAlignment";

    static constexpr const char* description =
        "Set this parameter to apply the Z-axis profile alignment settings. Acquire data again "
        "to see the effect. \nNOTE: The profile alignment function can only be applied after all "
        "profiles have been retrieved. Therefore, the profile data must be retrieved with a "
        "callback function instead of polling.";
    static constexpr Parameter::Type type = Parameter::Type::_Bool;
};

class EnableXAxisAlignment
{
public:
    static constexpr const char* name = "EnableXAxisAlignment";

    static constexpr const char* description =
        "Set this parameter to apply the X-axis profile alignment settings. Acquire data again "
        "to see the effect. \nNOTE: The profile alignment function can only be applied after all "
        "profiles have been retrieved. Therefore, the profile data must be retrieved with a "
        "callback function instead of polling.";
    static constexpr Parameter::Type type = Parameter::Type::_Bool;
};

} // namespace profile_alignment

namespace filters {
class EnableBlindSpotFiltering
{
public:
    static constexpr const char* name = "EnableBlindSpotFiltering";

    static constexpr const char* description =
        "Check this parameter to apply the blind spot filtering settings. Acquire data again to "
        "see the effect. \nNOTE: The blind spot filtering function can only be applied after all "
        "profiles have been retrieved. Therefore, the profile data must be retrieved with a "
        "callback function instead of polling.";

    static constexpr Parameter::Type type = Parameter::Type::_Bool;
};

class EnableNoiseRemoval
{
public:
    static constexpr const char* name = "EnableNoiseRemoval";

    static constexpr const char* description =
        "Check this parameter to adjust and apply the noise removal setting. Acquire data again to "
        "see the effect. \nNOTE: The noise removal function can only be applied after all "
        "profiles have been retrieved. Therefore, the profile data must be retrieved with a "
        "callback function instead of polling.";

    static constexpr Parameter::Type type = Parameter::Type::_Bool;
};

class NoiseRemovalIntensity
{
public:
    static constexpr const char* name = "NoiseRemovalIntensity";

    static constexpr const char* description =
        "Sets the intensity of noise removal.\n\nHigher intensity removes more noise but may also "
        "remove some object features.";

    static constexpr Parameter::Type type = Parameter::Type::_Enum;

    enum struct Value { Low, Medium, High };
};

} // namespace filters
} // namespace eye
} // namespace mmind
