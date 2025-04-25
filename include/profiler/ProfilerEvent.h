/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2016-2024, Mech-Mind Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Info:  https://www.mech-mind.com/
 *
 ******************************************************************************/

#pragma once
#include <functional>
#include "profiler/Profiler.h"

namespace mmind {

namespace eye {

/**
 * @brief Describes the event of profiler.
 * Use @ref ProfilerEvent.registerProfilerEventCallback to register an event of interest.
 */
class MMIND_API_EXPORT ProfilerEvent
{
public:
    /**
     * @brief Describes the types of Event.
     */
    enum Event {
        PROFILER_EVENT_DISCONNECTED = 0x0001,
        PROFILER_EVENT_ALL = 0xFFFF,
    };

    using EventCallback = std::function<void(Event event, void* pUser)>;

    /**
     * @brief Registers a profiler event of interest.
     * @param [in] profiler The profiler handle.
     * @param [in] callback The callback function for responding to profiler events.
     * @param [in] pUser Pointer used by the user.
     * @param [in] events The profiler event. See @ref ProfilerEvent.Event for details.
     * @return
     *  @ref ErrorStatus.MMIND_STATUS_SUCCESS Success.\n
     *  @ref ErrorStatus.MMIND_STATUS_INVALID_DEVICE Invalid profiler handle.\n
     *  @ref ErrorStatus.MMIND_STATUS_DEVICE_OFFLINE Profiler disconnected.\n
     *  @ref ErrorStatus.MMIND_STATUS_NO_SUPPORT_ERROR Not supported profiler model or firmware
     *  version.\n
     */
    static ErrorStatus registerProfilerEventCallback(Profiler& profiler, EventCallback callback,
                                                     void* pUser, unsigned int events);
};

} // namespace eye

} // namespace mmind
