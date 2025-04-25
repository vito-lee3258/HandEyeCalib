#pragma once
#include <memory>
#include <functional>
#include "api_global.h"
#include "ProfilerInfo.h"
#include "ProfileData.h"
#include "VirtualUserSet.h"

namespace mmind {

namespace eye {

class VirtualProfilerImpl;

class MMIND_API_EXPORT VirtualProfiler
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
     * @brief Constructor
     *
     * @param filePath The path of the virtual laser profiler file (.mraw) to be loaded.
     * @throw
     *  @ref ErrorStatus.MMIND_STATUS_FILE_IO_ERROR Fails to load the virtual data.
     */
    VirtualProfiler(const std::string& filePath);

    /**
     * @brief Destructor.
     */
    ~VirtualProfiler();

    /**
     * @brief Copy constructor.
     */
    VirtualProfiler(const VirtualProfiler& other) noexcept;

    /**
     * @brief Copy assignment.
     */
    VirtualProfiler& operator=(const VirtualProfiler& other) noexcept;

    /**
     * @brief Gets the basic information of the virtual laser profiler, such as model, serial
     * number, and firmware version.
     * @param [out] See @ref ProfilerInfo for details.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     */
    ErrorStatus getProfilerInfo(ProfilerInfo& info) const;

    /**
     * @brief Gets the parameter information used when the virtual device was saved.
     * @return An object of @ref
     * VirtualUserSet. Through @ref
     * VirtualUserSet, you can access all available parameters of the virtual device. See @ref
     * VirtualUserSet for details.
     */
    VirtualUserSet& currentUserSet();

    /**
     * @brief Retrieves a batch of the profiles. There are two ways to retrieve profile
     * data, by polling or callback. This method is only used with the polling method.
     * @param [out] batch The retrieved data of multiple profiles. See @ref ProfileBatch for
     * details.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_ACQUISITION_TRIGGER_WAIT Acquisition not started or stopped.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_INPUT_ERROR The @ref ProfileBatch batch input is
     * invalid.\n
     */
    ErrorStatus retrieveBatchData(ProfileBatch& batch) const;

    /**
     * @brief Registers the callback function for data acquisition. There are two ways to retrieve
     * profile data, by polling or callback. This method is only used with the callback method. This
     * method must be called after @ref connect and before @ref startAcquisition. If the virtual
     * device is in acquisition ready status, @ref stopAcquisition should be called before
     * registering a different callback function.
     * @param [in] func The callback function to be registered. When the number of retrieved
     * profiles equals to the set value of @ref ScanLineCount, the callback function will be
     * executed. See
     * @ref AcquisitionCallback for details.
     * @param [out] pUser The user data pointer.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_CALLBACKFUNC Invalid callback function.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_BUSY The callback function registration is executed
     *  again before data acquisition is stopped.\n
     */
    ErrorStatus registerAcquisitionCallback(const VirtualProfiler::AcquisitionCallback& func,
                                            void* pUser);

    /**
     * @brief Enters the virtual device into the acquisition ready status, where it can accept
     * trigger signals for scanning.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     */
    ErrorStatus startAcquisition();

    /**
     * @brief Exits the virtual profiler from the acquisition ready status to avoid accidental
     * triggering of scanning. If a callback function is being executed when this method is called,
     * this method is not executed until the execution of the callback function is finished.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     */
    ErrorStatus stopAcquisition();

private:
    std::shared_ptr<VirtualProfilerImpl> _d;
};

} // namespace eye

} // namespace mmind
