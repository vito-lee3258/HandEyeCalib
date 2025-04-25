#pragma once
#include "api_global.h"
#include "BatchArray.h"
#include "CommonTypes.h"
#include "ErrorStatus.h"

namespace mmind {
namespace eye {

class ProfileBatchImpl;

/**
 * @brief Describes a single profile.
 */
struct Profile
{
    unsigned int profileIndex{0};            ///< Index of the profile.
    unsigned int encoder{0};                 ///< Corresponding encoder value of the profile.
    const unsigned char* intensity{nullptr}; ///< Pointer to the intensity values of the profile.
    const float* depth{nullptr}; ///< Pointer to the depth values of the profile. The unit of depth
                                 ///< data is millimeter.
};

/**
 * @brief Represents a batch of profiles, which can be obtained by calling @ref
 * Profiler.retrieveBatchData(). It contains four elements of profile index, encoder value,
 * intensity image, and depth map.
 */
class MMIND_API_EXPORT ProfileBatch
{
public:
    /**
     * @brief Describes the status of the ProfileBatch object.
     */
    enum class BatchFlag {
        Success =
            0, ///< All profiles in the ProfileBatch object contain valid intensity and depth data.
        Incomplete = 0x1, ///< Some profiles in the ProfileBatch object do not contain valid
                          ///< intensity and depth data.
    };

    using ProfileIndexArray = BatchArray<unsigned int>;
    using EncoderArray = BatchArray<unsigned int>;
    using IntensityImage = BatchArray<unsigned char>;
    using DepthMap = BatchArray<float>;

    /**
     * @brief Constructor.
     */
    ProfileBatch(size_t width);

    /**
     * @brief Default destructor.
     */
    ~ProfileBatch() = default;

    /**
     * @brief Returns the width of the ProfileBatch object (the number of data points per profile).
     */
    size_t width() const;

    /**
     * @brief Returns the height of the ProfileBatch object (the number of profiles in the batch).
     */
    size_t height() const;

    /**
     * @brief Returns the valid height of the ProfileBatch object (the number of profiles with
     * valid intensity and depth data in the batch).
     */
    size_t validHeight() const;

    /**
     * @brief Checks if the ProfileBatch object has no elements.
     */
    bool isEmpty() const;

    /**
     * @brief Reserves the input height for the ProfileBatch object.
     */
    void reserve(size_t height);

    /**
     * @brief Appends the data of one ProfileBatch object to another.
     */
    bool append(const ProfileBatch& batch);

    /**
     * @brief Clears the data in the ProfileBatch object.
     */
    void clear();

    /**
     * @brief Gets a profile in the batch by inputting the index of the profile.
     */
    Profile getProfile(size_t profileIndex) const;

    /**
     * @brief Gets an array of indices of all profiles in the batch. Each profile data corresponds
     * to an index.
     */
    ProfileIndexArray getProfileIndexArray() const;

    /**
     * @brief Gets an array of encoder values of all profiles in the batch. Each profile data
     * corresponds to an encoder value.
     */
    EncoderArray getEncoderArray() const;

    /**
     * @brief Gets the intensity image data in the batch. The invalid data of intensity image is 0.
     */
    IntensityImage getIntensityImage() const;

    /**
     * @brief Gets the depth map data in the batch. Each point in DepthMap contains the Z
     * information in the laser profiler coordinate system. The depth data unit is mm, and invalid
     * data is nan.
     */
    DepthMap getDepthMap() const;
    /**
     * @brief Gets the error code and description of the function.
     */
    ErrorStatus getErrorStatus() const;

    /**
     * @brief Gets the flags of the ProfileBatch object. See @ref BatchFlag for details.
     */
    int getFlag() const;

    /**
     * @brief Checks if the @ref BatchFlag value of the ProfileBatch object matches the input value.
     */
    bool checkFlag(BatchFlag flag) const;

private:
    std::shared_ptr<ProfileBatchImpl> _impl;
    friend class ProfilerImpl;
    friend class VirtualProfilerImpl;
};
} // namespace eye
} // namespace mmind
