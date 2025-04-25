#pragma once
#include <memory>
#include <functional>
#include "api_global.h"
#include "ProfilerInfo.h"
#include "ProfileData.h"
#include "UserSetManager.h"
#include "UserSet.h"

namespace mmind {

namespace eye {

class ProfilerImpl;
/**
 * @brief Describes the types of output lines of GPIO.
 */
enum class OutputLineGPIO {
    Line21 = 0,
    Line22,
    Line23,
    Line24,
    Line25,
    Line26,
    Line27,
    Line28,
};

/**
 * @brief Describes the types of output levels of GPIO.
 */
enum class OutputLevel { Low, High };
/**
 * @brief Describes the types of acquisition status.
 */
enum class AcquisitionStatus {
    AcquisitionTriggerWait, ///< @ref Profiler::startAcquisition not called.
    AcquisitionActive,      ///< @ref Profiler::startAcquisition called.
    FrameTriggerWait,       ///< @ref Profiler::triggerSoftware not called or GPIO input frame start
                            ///< signal not received.
    FrameActive, ///< @ref Profiler::triggerSoftware called or GPIO input frame start signal
                 ///< received.
};

/**
 * @brief Describes the laser profiler temperatures.
 */
struct ProfilerTemperature
{
    float controllerCpuTemperature{}; ///< The temperature (in °C) of the controller CPU.
    float sensorCpuTemperature{};          ///< The temperature (in °C) of the FPGA.
};

/**
 * @brief Describes the laser profiler's statuses.
 */
struct ProfilerStatus
{
    ProfilerTemperature temperature;
};

/**
 * @brief Operates the laser profiler.
 * Use @ref Profiler.connect to connect an available laser profiler, retrieve profile data,
 * configure parameters and so on.
 */
class MMIND_API_EXPORT Profiler
{
public:
    /**
     * @brief The type of callback function.
     * @param [out] batch The retrieved data of multiple profiles. See @ref ProfileBatch for
     * details.
     * @param [out] pUser The user data pointer.
     */
    using AcquisitionCallback = std::function<void(const ProfileBatch& batch, void* pUser)>;
    /**
     * @brief Constructor.
     */
    Profiler();

    /**
     * @brief Destructor.
     */
    ~Profiler();

    /**
     * @brief Copy constructor.
     */
    Profiler(const Profiler& other) noexcept;

    /**
     * @brief Copy assignment.
     */
    Profiler& operator=(const Profiler& other) noexcept;

    /**
     * @brief Discovers all available laser profilers, and returns the laser profiler
     * information list. If a laser profiler is not successfully discovered,
     * check the running status and network connection of the laser profiler.
     * @return The available laser profiler information list.
     */
    static std::vector<ProfilerInfo> discoverProfilers();

    /**
     * @brief Connects to a laser profiler via @ref ProfilerInfo.
     * @param [in] info Laser profiler information. Use @ref Profiler.discoverProfilers to find
     * all available laser profilers.
     * @param [in] timeoutMs The timeout for connecting a laser profiler(ms). If the execution
     * time of the connecting process is greater than the timeout, the function will immediately
     * return @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_INPUT_ERROR IP address format error.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE IP address not corresponding to an available
     *  device.\n
     *  @ref ErrorStatus.MMIND_STATUS_NO_SUPPORT_ERROR Laser profiler model or
     *  firmware version not supported.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     */
    ErrorStatus connect(const ProfilerInfo& info, unsigned int timeoutMs = 5000);

    /**
     * @brief  Saves the acquired @ref ProfileBatch data collected, @ref Parameter s, and @ref
     * ProfilerInfo in an MRAW format file that can be loaded as a @ref VirtualProfiler.
     *
     * @param data The acquired @ref ProfileBatch data to be saved.
     * @param filePath The path where the MRAW file will be saved.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid laser profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Laser profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     *  @ref ErrorStatus.MMIND_STATUS_RESPONSE_PARSE_ERROR Parse response error.\n
     *  @ref ErrorStatus.MMIND_STATUS_REPLY_WITH_ERROR There are errors in reply.\n
     *  @ref ErrorStatus.MMIND_STATUS_FILE_IO_ERROR Fails to save the virtual data.\n
     */
    ErrorStatus saveVirtualDeviceFile(const ProfileBatch& data, const std::string& filePath);

    /**
     * @brief Connects to a laser profiler via IP address.
     * @param [in] ipAddress Valid IP address of the laser profiler, e.g. in "100.100.1.1" format.
     * @param [in] timeoutMs The timeout for connecting a laser profiler (ms). If the execution
     * time of the connecting process is greater than the timeout, the function will immediately
     * return @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_INPUT_ERROR IP address format error.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE IP address not corresponding to an
     *  available device.\n
     *  @ref ErrorStatus.MMIND_STATUS_NO_SUPPORT_ERROR Laser profiler model or
     *  firmware version not supported.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     */
    ErrorStatus connect(const std::string& ipAddress, unsigned int timeoutMs = 5000);

    /**
     * @brief Disconnects from the current laser profiler and releases the associated resources.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid laser profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_RESPONSE_PARSE_ERROR Parse Response error.\n
     *  @ref ErrorStatus.MMIND_STATUS_REPLY_WITH_ERROR There are errors in reply.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Laser profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     */
    ErrorStatus disconnect();

    /**
     * @brief Sets the time interval at which the client sends periodic heartbeat messages to the
     * profiler side. The default time interval is 10s.
     * @param [in] timeIntervalMs The time interval for periodic sending heartbeat messages in
     * milliseconds. The valid setting range is from 1s to 3600s.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_OUT_OF_RANGE_ERROR Invalid parameter input.\n
     */
    ErrorStatus setHeartbeatInterval(unsigned int intervalMs);

    /**
     * @brief Gets the basic information of the laser profiler, such as model, serial number,
     * firmware version, and IP setting, etc.
     * @param [out] See @ref ProfilerInfo for details.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid laser profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Laser profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     */
    ErrorStatus getProfilerInfo(ProfilerInfo& info) const;

    /**
     * @brief Gets various statuses of the laser profiler.
     * @param [out] See @ref ProfilerStatus for details.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Profiler disconnected.\n
     */
    ErrorStatus getProfilerStatus(ProfilerStatus& status) const;

    /**
     * @brief Gets the @ref UserSetManager of the laser profiler. @ref UserSetManager provides
     * various operations to manage all user set saved in the laser profiler, including adding and
     * deleting the user set and selecting the user set currently in effect. It is also available to
     * save all user set details to a json file and read a json file to load user set details.
     * @return See @ref UserSetManager for details.
     */
    UserSetManager& userSetManager();

    /**
     * @brief Gets the @ref UserSet currently in effect of the laser profiler. @ref UserSet can
     * access all available parameters of the laser profiler related to profile data acquisition.
     * @ref UserSet can also directly set and get parameters instead of using @ref Parameter
     * interface.
     * @return See @ref UserSet for details.
     */
    UserSet& currentUserSet();

    /**
     * @brief Retrieves a batch of the profiles. There are two ways to retrieve profile
     * data, by polling or callback. This method is only used with the polling method.
     * The number of profiles contained in a batch varies depending on the scan rate and the rate at
     * which this method is called.
     * @param [out] batch The retrieving result, it contains information including profile index,
     * encoder value intensity image, and depth data. See @ref ProfileBatch for details.
     * @param [in] timeoutMs The timeout for capturing in milliseconds. If the execution time of
     * the capturing process is greater than the timeout, the function will immediately return
     * @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid laser profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Laser profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_NO_DATA_ERROR No profile data obtained. Some error may have
     *  occurred on the laser profiler.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     *  @ref ErrorStatus.MMIND_STATUS_ACQUISITION_TRIGGER_WAIT Acquisition not started or stopped.\n
     */
    ErrorStatus retrieveBatchData(ProfileBatch& batch, int timeoutMs = 4000) const;

    /**
     * @brief Sends a software signal to trigger data acquisition. This method is used when no
     * external signals are input to trigger data acquisition and must be called after @ref
     * startAcquisition.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid laser profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Laser profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     *  @ref ErrorStatus.MMIND_STATUS_RESPONSE_PARSE_ERROR Parse response error.\n
     *  @ref ErrorStatus.MMIND_STATUS_REPLY_WITH_ERROR There are errors in reply.\n
     */
    ErrorStatus triggerSoftware();

    /**
     * @brief Registers the callback function for data acquisition. There are two ways to retrieve
     * profile data, by polling or callback. This method is only used with the callback method. This
     * method must be called after @ref connect and before @ref startAcquisition. If the laser
     * profiler is in acquisition ready status, @ref stopAcquisition should be called before
     * registering a different callback function.
     * @param [in] func The callback function to be registered. When the number of retrieved
     * profiles equals to the set value of @ref ScanLineCount, the callback function will be
     * executed. See
     * @ref AcquisitionCallback for details.
     * @param [out] pUser The user data pointer.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid laser profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_CALLBACKFUNC Invalid callback function.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_BUSY The callback function registration is executed
     *  again before data acquisition is stopped.\n
     */
    ErrorStatus registerAcquisitionCallback(const Profiler::AcquisitionCallback& func, void* pUser);

    /**
     * @brief Enters the laser profiler into the acquisition ready status, where it can accept
     * trigger signals for scanning.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid laser profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Laser profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     *  @ref ErrorStatus.MMIND_STATUS_RESPONSE_PARSE_ERROR Parse response error.\n
     *  @ref ErrorStatus.MMIND_STATUS_REPLY_WITH_ERROR There are errors in reply.\n
     */
    ErrorStatus startAcquisition();

    /**
     * @brief Exits the laser profiler from the acquisition ready status to avoid accidental
     * triggering of scanning. If a callback function is being executed when this method is called,
     * this method is not executed until the execution of the callback function is finished.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid laser profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Laser profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     *  @ref ErrorStatus.MMIND_STATUS_RESPONSE_PARSE_ERROR Parse response error.\n
     *  @ref ErrorStatus.MMIND_STATUS_REPLY_WITH_ERROR There are errors in reply.\n
     */
    ErrorStatus stopAcquisition();

    /**
     * @brief Sets controller GPIO output value.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid laser profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Laser profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     *  @ref ErrorStatus.MMIND_STATUS_RESPONSE_PARSE_ERROR Parse response error.\n
     *  @ref ErrorStatus.MMIND_STATUS_REPLY_WITH_ERROR There are errors in reply.\n
     */
    ErrorStatus setOutputForGPIO(OutputLineGPIO outputLine, OutputLevel value);

    /**
     * @brief Gets the acquisition status.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid laser profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Laser profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_TIMEOUT_ERROR Timeout error.\n
     *  @ref ErrorStatus.MMIND_STATUS_RESPONSE_PARSE_ERROR Parse response error.\n
     *  @ref ErrorStatus.MMIND_STATUS_REPLY_WITH_ERROR: There are errors in reply.\n
     */
    ErrorStatus getAcquisitionStatus(AcquisitionStatus& status);

private:
    friend class ProfilerEvent;
    friend class InternalInterfaces;
    std::shared_ptr<ProfilerImpl> _d;
};

} // namespace eye

} // namespace mmind
