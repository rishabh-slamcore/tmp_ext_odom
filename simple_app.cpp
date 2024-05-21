/******************************************************************************
 *
 * Slamcore Confidential
 * ---------------------
 *
 * Slamcore Limited
 * All Rights Reserved.
 * (C) Copyright 2021
 *
 * NOTICE:
 *
 * All information contained herein is, and remains the property of Slamcore
 * Limited and its suppliers, if any. The intellectual and technical concepts
 * contained herein are proprietary to Slamcore Limited and its suppliers and
 * may be covered by patents in process, and are protected by trade secret or
 * copyright law. Dissemination of this information or reproduction of this
 * material is strictly forbidden unless prior written permission is obtained
 * from Slamcore Limited.
 *
 ******************************************************************************/

/**
 * @file
 * @ingroup slamcore_sdk_examples
 * @brief API example to set up and run a SLAM system, enabling all streams.
 *
 * Demonstrates the following:
 *
 *   - Create and interact with the SLAM system
 * (slamcore::SLAMSystemCallbackInterface::open, slamcore::SLAMSystemCallbackInterface::start,
 * slamcore::SLAMSystemCallbackInterface::close etc.)
 *   - Enable/use data streams.
 *   - Toggling of slamcore::Property(s).
 *   - Receive and display data in a loop.
 */

#include <slamcore/errors.hpp>
#include <slamcore/slam/slam_create.hpp>
#include <slamcore/objects/pose.hpp>
#include <slamcore/sensor_source_interface.hpp>

#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <thread>

int main(int /*argc*/, char* /*argv*/[])
{
  // ******************************************************************
  // Initialise Slamcore API
  // ******************************************************************
  slamcore::slamcoreInit(slamcore::LogSeverity::Info,
                         [](const slamcore::LogMessageInterface& message)
                         {
                           const time_t time = slamcore::host_clock::to_time_t(message.getTimestamp());
                           struct tm tm;
                           localtime_r(&time, &tm);

                           std::cerr << "[" << message.getSeverity() << " "
                                     << std::put_time(&tm, "%FT%T%z") << "] "
                                     << message.getMessage() << "\n";
                         });

  // ******************************************************************
  // Create/Connect SLAM System
  // ******************************************************************
  slamcore::v0::SystemConfiguration sysCfg;
  std::unique_ptr<slamcore::SLAMSystemCallbackInterface> slam = slamcore::createSLAMSystem(sysCfg);
  if (!slam)
  {
    std::cerr << "Error creating SLAM system!" << '\n';
    slamcore::slamcoreDeinit();
    return -1;
  }

  std::cout << "Starting SLAM..." << '\n';

  // ******************************************************************
  // Open the device
  // ******************************************************************
  slam->open();

  slamcore::EmptyTag t;

  // ******************************************************************
  // Print versions
  // ******************************************************************
  const std::string slam_version = slam->getProperty<std::string>(slamcore::Property::FirmwareVersion);
  const std::string slam_build_ver =
    slam->getProperty<std::string>(slamcore::Property::FirmwareBuildVersion);
  const std::string slam_build_type =
    slam->getProperty<std::string>(slamcore::Property::FirmwareBuildType);

  std::cout << "Client Version: " << slamcore::getVersion() << "/" << slamcore::getBuildVersion()
            << "/" << slamcore::getBuildType() << '\n';
  std::cout << "SLAM Version: " << slam_version << "/" << slam_build_ver << "/" << slam_build_type
            << '\n';

  // ******************************************************************
  // Enable all the streams
  // ******************************************************************
  slam->setStreamEnabled(slamcore::Stream::Pose, true);
  slam->setStreamEnabled(slamcore::Stream::Video, true);
  slam->setStreamEnabled(slamcore::Stream::IMU, true);
  slam->setStreamEnabled(slamcore::Stream::ActiveMap, true);
  slam->setStreamEnabled(slamcore::Stream::Velocity, true);
  slam->setStreamEnabled(slamcore::Stream::MetaData, true);
  slam->setStreamEnabled(slamcore::Stream::SLAMStatus, true);
  slam->setStreamEnabled(slamcore::Stream::LocalPointCloud, true);

  // *****************************************************************
  // Register callbacks!
  // *****************************************************************

  slam->registerCallback<slamcore::Stream::ErrorCode>(
    [](const slamcore::ErrorCodeInterface::CPtr& errorObj)
    {
      const auto rc = errorObj->getValue();
      std::cout << "Received: ErrorCode" << '\n';
      std::cout << "\t" << rc.message() << " / " << rc.value() << " / " << rc.category().name()
                << '\n';
    });

  slam->registerCallback<slamcore::Stream::Pose>(
    [](const slamcore::PoseMeasurement<slamcore::camera_clock>::CPtr& poseObj)
    {
      std::cout << "Received: Pose" << '\n';
      std::cout << "\t" << poseObj->getTranslation().x() << "," << poseObj->getTranslation().y()
                << "," << poseObj->getTranslation().z() << '\n';
    });

  slam->registerCallback<slamcore::Stream::Video>(
    [](const slamcore::MultiFrameInterface::CPtr& multiFrameObj)
    {
      std::cout << "Received: MultiFrame" << '\n';
      std::cout << "\t" << multiFrameObj->size() << " "
                << (multiFrameObj->isKeyFrame() ? "KF" : "NON-KF") << '\n';
      for (const auto& frame : *multiFrameObj)
      {
        const auto& img = frame.image();
        std::cout << "\t\t" << img.getWidth() << " x " << img.getHeight() << " at "
                  << std::chrono::duration_cast<std::chrono::nanoseconds>(
                       img.getHWTimestamp().time_since_epoch())
                       .count()
                  << " [ns]" << '\n';
      }
    });

  slam->registerCallback<slamcore::Stream::FrameSync>(
    [](const slamcore::FrameSyncInterface::CPtr&) { std::cout << "Received: FrameSync" << '\n'; });

  // ******************************************************************
  // Start streaming
  // ******************************************************************
  slam->start();

  // ******************************************************************
  // Main receiving loop
  // ******************************************************************
  while (slam->spin())
  {
    ;
  }

  // ******************************************************************
  // Stop SLAM
  // ******************************************************************
  slam->stop();

  // ******************************************************************
  // Disconnect/Close SLAM
  // ******************************************************************
  slam->close();

  slam.reset();

  // ******************************************************************
  // Deinitialise Slamcore API
  // ******************************************************************
  slamcore::slamcoreDeinit();

  std::cout << "We're Done Here." << '\n';

  return 0;
}
