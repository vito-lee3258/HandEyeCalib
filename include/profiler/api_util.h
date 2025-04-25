/*******************************************************************************
 *BSD 3-Clause License
 *
 *Copyright (c) 2016-2024, Mech-Mind Robotics
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 *3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#pragma once
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <set>
#include <utility>
#include "ErrorStatus.h"
#include "CommonTypes.h"
#include "profiler/parameters/ScanParameters.h"
#include "Profiler.h"
#include "ProfilerInfo.h"

constexpr double kPitch = 1e-3;
constexpr long long kInitEncoderValue = 0x0FFFFFFF;

/**
 * @brief Prints the data in the ProfilerInfo object.
 */
inline void printProfilerInfo(const mmind::eye::ProfilerInfo& profilerInfo)
{
    std::cout << "........................................." << std::endl;
    std::cout << "Profiler Model Name:           " << profilerInfo.model << std::endl;
    std::cout << "Controller Serial Number:      " << profilerInfo.controllerSN << std::endl;
    std::cout << "Sensor Serial Number:          " << profilerInfo.sensorSN << std::endl;
    std::cout << "Profiler IP Address:           " << profilerInfo.ipAddress << std::endl;
    std::cout << "Profiler IP Subnet Mask:       " << profilerInfo.subnetMask << std::endl;
    std::cout << "Profiler IP Assignment Method: "
              << mmind::eye::ipAssignmentMethodToString(profilerInfo.ipAssignmentMethod)
              << std::endl;
    std::cout << "Hardware Version:              "
              << "V" << profilerInfo.hardwareVersion.toString() << std::endl;
    std::cout << "Firmware Version:              "
              << "V" << profilerInfo.firmwareVersion.toString() << std::endl;
    std::cout << "........................................." << std::endl;
    std::cout << std::endl;
}

inline void printProfilerStatus(const mmind::eye::ProfilerStatus& profilerStatus)
{
    std::cout << ".....Profiler temperatures....." << std::endl;
    std::cout << "Controller CPU: " << std::setprecision(4)
              << profilerStatus.temperature.controllerCpuTemperature << "°C" << std::endl;
    std::cout << "Sensor CPU:     " << std::setprecision(4)
              << profilerStatus.temperature.sensorCpuTemperature << "°C" << std::endl;
    std::cout << "..............................." << std::endl;
    std::cout << std::endl;
}

/**
 * @brief Discovers all available laser profilers and allows the user to connect to a laser profiler
 * by inputting the device index.
 */
inline bool findAndConnect(mmind::eye::Profiler& profiler)
{
    std::cout << "Find Mech-Eye 3D Laser Profilers..." << std::endl;
    std::vector<mmind::eye::ProfilerInfo> profilerInfoList =
        mmind::eye::Profiler::discoverProfilers();

    if (profilerInfoList.empty()) {
        std::cout << "No Mech-Eye 3D Laser Profiler found." << std::endl;
        return false;
    }

    for (int i = 0; i < profilerInfoList.size(); i++) {
        std::cout << "Mech-Eye 3D Laser profiler index : " << i << std::endl;
        printProfilerInfo(profilerInfoList[i]);
    }

    std::cout << "Please enter the profiler index you want to connect: ";
    unsigned inputIndex = 0;

    while (true) {
        std::string str;
        std::cin >> str;
        if (std::regex_match(str.begin(), str.end(), std::regex{"[0-9]+"}) &&
            atoi(str.c_str()) < profilerInfoList.size()) {
            inputIndex = atoi(str.c_str());
            break;
        }
        std::cout << "Input invalid! Please enter the profiler index you want to connect: ";
    }

    mmind::eye::ErrorStatus status;
    status = profiler.connect(profilerInfoList[inputIndex]);

    if (!status.isOK()) {
        showError(status);
        return false;
    }

    std::cout << "Connect Mech-Eye 3D Laser Profiler Successfully." << std::endl;
    return true;
}

inline std::vector<mmind::eye::Profiler> findAndConnectMultiProfiler()
{
    std::cout << "Find Mech-Eye 3D Laser Profilers..." << std::endl;
    std::vector<mmind::eye::ProfilerInfo> profilerInfoList =
        mmind::eye::Profiler::discoverProfilers();

    if (profilerInfoList.empty()) {
        std::cout << "No Mech-Eye 3D Laser Profilers found." << std::endl;
        return {};
    }

    for (int i = 0; i < profilerInfoList.size(); i++) {
        std::cout << "Mech-Eye 3D Laser Profiler index : " << i << std::endl;
        printProfilerInfo(profilerInfoList[i]);
    }

    std::string str;
    std::set<unsigned> indices;

    while (true) {
        std::cout << "Please enter the device index you want to connect: " << std::endl;
        std::cout << "Enter the character 'c' to terminate adding devices" << std::endl;

        std::cin >> str;
        if (str == "c")
            break;
        if (std::regex_match(str.begin(), str.end(), std::regex{"[0-9]+"}) &&
            atoi(str.c_str()) < profilerInfoList.size())
            indices.insert(atoi(str.c_str()));
        else
            std::cout << "Input invalid. Please enter the device index you want to connect: ";
    }

    std::vector<mmind::eye::Profiler> profilerList{};

    auto iter = indices.cbegin();
    for (int i = 0; i < indices.size(); ++i, ++iter) {
        mmind::eye::Profiler profiler;
        auto status = profiler.connect(profilerInfoList[*iter]);
        if (status.isOK())
            profilerList.push_back(profiler);
        else
            showError(status);
    }

    return profilerList;
}

inline bool confirmCapture()
{
    std::cout << "Do you want the profiler to capture image? Please input y/n to confirm: "
              << std::endl;
    while (true) {
        std::string confirmStr;
        std::cin >> confirmStr;
        if (confirmStr == "y") {
            return true;
        } else if (confirmStr == "n") {
            std::cout << "program ends!" << std::endl;
            return false;
        } else {
            std::cout << "Please input y/n again!" << std::endl;
        }
    }
}

int shiftEncoderValsAroundZero(unsigned int oriVal, long long initValue = kInitEncoderValue)
{
    return static_cast<int>(oriVal - initValue);
}

bool saveDataToPly(float* data, int* yValues, int captureLineCount, int dataWidth, float xUnit,
                   float yUnit, const std::string& fileName, bool isOrganized)
{
    FILE* fp = fopen(fileName.c_str(), "w");

    if (!fp)
        return false;

    unsigned validPointCount{0};
    if (!isOrganized) {
        for (int y = 0; y < captureLineCount; ++y) {
            for (int x = 0; x < dataWidth; ++x) {
                if (!std::isnan(data[y * dataWidth + x]))
                    validPointCount++;
            }
        }
    }

    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "comment File generated\n");
    fprintf(fp, "comment x y z data unit in mm\n");
    fprintf(fp, "element vertex %u\n",
            isOrganized ? static_cast<unsigned>(captureLineCount * dataWidth) : validPointCount);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "end_header\n");

    for (int y = 0; y < captureLineCount; ++y) {
        for (int x = 0; x < dataWidth; ++x) {
            if (!std::isnan(data[y * dataWidth + x]))
                fprintf(fp, "%f %f %f\n", static_cast<float>(x * xUnit * kPitch),
                        static_cast<float>(yValues[y] * yUnit * kPitch), data[y * dataWidth + x]);
            else if (isOrganized)
                fprintf(fp, "nan nan nan\n");
        }
    }

    fclose(fp);
    return true;
}

bool saveDataToCsv(float* data, int* yValues, int captureLineCount, int dataWidth, float xUnit,
                   float yUnit, const std::string& fileName, bool isOrganized)
{
    FILE* fp = fopen(fileName.c_str(), "w");

    if (!fp)
        return false;

    fprintf(fp, "X,Y,Z\n");

    for (int y = 0; y < captureLineCount; ++y) {
        for (int x = 0; x < dataWidth; ++x) {
            if (!std::isnan(data[y * dataWidth + x]))
                fprintf(fp, "%f,%f,%f\n", static_cast<float>(x * xUnit * kPitch),
                        static_cast<float>(yValues[y] * yUnit * kPitch), data[y * dataWidth + x]);
            else if (isOrganized)
                fprintf(fp, "nan,nan,nan\n");
        }
    }

    fclose(fp);
    return true;
}

void savePointCloud(const mmind::eye::ProfileBatch& batch, const mmind::eye::UserSet& userSet,
                    bool savePLY = true, bool saveCSV = true, bool isOrganized = true)
{
    if (batch.isEmpty())
        return;

    // Get the X-axis resolution
    double xUnit{};
    auto status =
        userSet.getFloatValue(mmind::eye::point_cloud_resolutions::XAxisResolution::name, xUnit);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    // Get the Y resolution
    double yUnit{};
    status = userSet.getFloatValue(mmind::eye::point_cloud_resolutions::YResolution::name, yUnit);
    if (!status.isOK()) {
        showError(status);
        return;
    }
    // // Uncomment the following lines for custom Y Unit
    // // Prompt to enter the desired encoder resolution, which is the travel distance corresponding
    // // to
    // // one quadrature signal.
    // std::cout << "Please enter the desired encoder resolution (integer, unit: μm, min: "
    //  "1, max: 65535): ";
    // while (true) {
    //     std::string str;
    //     std::cin >> str;
    //     if (std::regex_match(str.begin(), str.end(), std::regex{"[0-9]+"})) {
    //         yUnit = atoi(str.c_str());
    //         break;
    //     }
    //     std::cout << "Input invalid! Please enter the desired encoder resolution (integer, unit:
    //     "
    //                  "μm, min: 1, max: 65535): ";
    // }

    int lineScanTriggerSource{};
    status = userSet.getEnumValue(mmind::eye::trigger_settings::LineScanTriggerSource::name,
                                  lineScanTriggerSource);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    bool useEncoderValues =
        lineScanTriggerSource ==
        static_cast<int>(mmind::eye::trigger_settings::LineScanTriggerSource::Value::Encoder);

    int triggerInterval{};
    status = userSet.getIntValue(mmind::eye::trigger_settings::EncoderTriggerInterval::name,
                                 triggerInterval);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    // Shift the encoder values around zero
    std::vector<int> encoderVals;
    encoderVals.reserve(batch.height());
    auto encoder = batch.getEncoderArray();
    for (int r = 0; r < batch.height(); ++r)
        encoderVals.push_back(
            useEncoderValues ? shiftEncoderValsAroundZero(encoder[r], encoder[0]) / triggerInterval
                             : r);

    std::cout << "Save the point cloud." << std::endl;
    if (saveCSV)
        saveDataToCsv(batch.getDepthMap().data(), encoderVals.data(), batch.height(), batch.width(),
                      xUnit, yUnit, "PointCloud.csv", isOrganized);
    if (savePLY)
        saveDataToPly(batch.getDepthMap().data(), encoderVals.data(), batch.height(), batch.width(),
                      xUnit, yUnit, "PointCloud.ply", isOrganized);
}