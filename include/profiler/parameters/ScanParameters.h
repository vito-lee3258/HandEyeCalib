#pragma once
#include "Parameter.h"

namespace mmind {

namespace eye {

namespace trigger_settings {

class DataAcquisitionTriggerSource
{
public:
    static constexpr const char* name = "DataAcquisitionTriggerSource";

    static constexpr const char* description =
        "Select the source of the signals that trigger the scan of a single frame.\nIf you use "
        "external input signal to trigger scanning, select \"External\".\nIf you need to trigger "
        "scanning with software, select \"Software\".";

    static constexpr Parameter::Type type = Parameter::Type::_Enum;

    enum struct Value {
        Software,
        External,
    };
};

class LineScanTriggerSource
{
public:
    static constexpr const char* name = "LineScanTriggerSource";

    static constexpr const char* description =
        "Select the source of the signals that trigger the scan of a single line.\nIf you use "
        "encoder to trigger scanning, select \"Encoder\".\nIf you need to trigger scanning at a "
        "fixed rate, select \"FixedRate\" and adjust \"SoftwareTriggerRate\".";

    static constexpr Parameter::Type type = Parameter::Type::_Enum;

    enum struct Value {
        FixedRate,
        Encoder,
    };
};

class SoftwareTriggerRate
{
public:
    static constexpr const char* name = "SoftwareTriggerRate";

    static constexpr const char* description =
        "When \"LineScanTriggerSource\" is set to \"FixedRate\", set the fixed rate at which "
        "the laser profiler is triggered to scan.\n\nThe maximum value of this parameter is the "
        "current \"MaxScanRate\". \"SoftwareTriggerRate\" is unavailable when "
        "\"LineScanTriggerSource\" is set to \"Encoder\".";

    static constexpr Parameter::Type type = Parameter::Type::_Float;

    static constexpr const char* unit = "Hz";
};

class MaxScanRate
{
public:
    static constexpr const char* name = "MaxScanRate";

    static constexpr const char* description =
        "The maximum scan rate that the laser profiler can reach. The maximum scan rate is "
        "affected by the following parameters: \"ExposureTime\", \"ZDirectionRoi\", and "
        "\"TriggerDelay\"";

    static constexpr Parameter::Type type = Parameter::Type::_Float;

    static constexpr const char* unit = "Hz";
};

class TriggerDelay
{
public:
    static constexpr const char* name = "TriggerDelay";

    static constexpr const char* description =
        "This parameter is only effective for firmware 2.4.0 and above.\n"
        "Set the delay time between receiving a line scan trigger signal and emitting laser "
        "light.\n\n"
        "Note:\n"
        "* Only adjust this parameter in the following situation: Multiple laser profilers are "
        "used to scan the same target object, and their FOVs overlap. The laser profilers will "
        "interfere with each other if they emit laser light at the same time.\n"
        "* Increasing this parameter will reduce the max scan rate.\n"
        "* The laser profiler starts exposure 10 μs after emitting laser light to ensure stable "
        "brightness of the laser lines in the raw image.";

    static constexpr Parameter::Type type = Parameter::Type::_Int;

    static constexpr const char* unit = "us";
};

class EncoderTriggerDirection
{
public:
    static constexpr const char* name = "EncoderTriggerDirection";

    static constexpr const char* description =
        "Select the encoder motion direction that triggers scanning.\nChannelALeading: Scanning "
        "is triggered when channel A is leading.\nChannelBLeading: Scanning is triggered when "
        "channel B is leading. \nBoth: Scanning is triggered when either channel A or channel B is "
        "leading.";

    static constexpr Parameter::Type type = Parameter::Type::_Enum;

    enum struct Value {
        ChannelALeading,
        ChannelBLeading,
        Both,
    };
};

class EncoderTriggerSignalCountingMode
{
public:
    static constexpr const char* name = "EncoderTriggerSignalCountingMode";

    static constexpr const char* description =
        "Set the number of signals to be counted in an encoder cycle. Counted signals are used to "
        "trigger scanning.\nNote: This parameter affects the adjustment of "
        "\"EncoderTriggerInterval\".\n\nMultiple_1: counts 1 signal in an encoder "
        "cycle.\nMULTIPLE_2: counts 2 signals in an encoder cycle.\nMULTIPLE_4: counts 3 signals "
        "in an encoder cycle.";

    static constexpr Parameter::Type type = Parameter::Type::_Enum;

    enum struct Value {
        Multiple_1,
        Multiple_2,
        Multiple_4,
    };
};

class EncoderTriggerInterval
{
public:
    static constexpr const char* name = "EncoderTriggerInterval";

    static constexpr const char* description =
        "Set the number of trigger signals needed for scanning one line.";

    static constexpr Parameter::Type type = Parameter::Type::_Int;
};
} // namespace trigger_settings

namespace scan_settings {

class ScanLineCount
{
public:
    static constexpr const char* name = "ScanLineCount";

    static constexpr const char* description =
        "Set the number of profiles needed to generate one intensity image/depth map.\nMake sure "
        "that the set value can cover one target object completely.";

    static constexpr Parameter::Type type = Parameter::Type::_Int;
};

class DataPointsPerProfile
{
public:
    static constexpr const char* name = "DataPointsPerProfile";

    static constexpr const char* description = "The number of data points in a profile.";

    static constexpr Parameter::Type type = Parameter::Type::_Int;

    static constexpr const char* unit = "";
};

class ExposureDelay
{
public:
    static constexpr const char* name = "ExposureDelay";

    static constexpr const char* description =
        "This parameter is only effective for firmware 2.3.4 and below.\nSet the delay time "
        "between laser emission and start of exposure.\n\nLarger exposure delay "
        "results in more stable brightness of the laser lines in the raw image, thus more stable "
        "quality of the intensity image and depth map. However, the \"MaxScanRate\" will be "
        "reduced.";

    static constexpr Parameter::Type type = Parameter::Type::_Int;

    static constexpr const char* unit = "us";
};

class BatchRetrievalTimeout
{
public:
    static constexpr const char* name = "BatchRetrievalTimeout";

    static constexpr const char* description =
        "Set the timeout period for retrieving a batch of data. If no batches are available for "
        "retrieval within the set timeout period, the current round of data acquisition is "
        "automatically stopped.\nThis timeout period should be no shorter than the amount of time "
        "needed for scanning 16 lines. If line scan is triggered at a slow rate, increase this "
        "parameter.";

    static constexpr Parameter::Type type = Parameter::Type::_Int;

    static constexpr const char* unit = "ms";
};

class CallbackRetrievalTimeout
{
public:
    static constexpr const char* name = "CallbackRetrievalTimeout";

    static constexpr const char* description =
        "Set the timeout period for retrieving data when using a callback function. If none or "
        "only some of the data is retrieved within the set timeout period, the current round of "
        "data acquisition is automatically stopped. The amount of data to be retrieved is "
        "determined by the \"ScanLineCount\" parameter.\nA value of 0 or -1 corresponds to an "
        "infinite timeout period.";

    static constexpr Parameter::Type type = Parameter::Type::_Int;

    static constexpr const char* unit = "ms";
};

} // namespace scan_settings

namespace point_cloud_resolutions {
class XAxisResolution
{
public:
    static constexpr const char* name = "XAxisResolution";

    static constexpr const char* description =
        "Sets the scan data resolution in the X direction, which is the distance between two "
        "neighboring points along the direction of the laser line.";

    static constexpr Parameter::Type type = Parameter::Type::_Float;

    static constexpr const char* unit = "um";
};

class YResolution
{
public:
    static constexpr const char* name = "YResolution";

    static constexpr const char* description =
        "Sets the point cloud resolution in the Y-axis direction, which is the distance between "
        "two neighboring points along the travel direction of the target object.";

    static constexpr Parameter::Type type = Parameter::Type::_Float;

    static constexpr const char* unit = "um";
};
} // namespace point_cloud_resolutions

namespace correction {

class TiltCorrectionAngle
{
public:
    static constexpr const char* name = "TiltCorrectionAngle";

    static constexpr const char* description = "Correct the tilt of the profile around the Y-axis.";

    static constexpr Parameter::Type type = Parameter::Type::_Float;

    static constexpr const char* unit = "degree";
};

class HeightCorrectionRatio
{
public:
    static constexpr const char* name = "HeightCorrectionRatio";

    static constexpr const char* description = "Correct the Z values of the profile.";

    static constexpr Parameter::Type type = Parameter::Type::_Float;
};
} // namespace correction

namespace transformation {
class CoordinateTransformation
{
public:
    static constexpr const char* name = "CoordinateTransformation";

    static constexpr const char* description =
        "The CoordinateTransformation, which represents the transformation matrix from the "
        "camera coordinate system to a custom coordinate system. It can change the xyz values of "
        "the point cloud.";

    static constexpr Parameter::Type type = Parameter::Type::_FloatArray;
};
} // namespace transformation

} // namespace eye
} // namespace mmind
