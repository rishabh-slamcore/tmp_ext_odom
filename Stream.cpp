/******************************************************************************
 *
 * Slamcore Confidential
 * ---------------------
 *
 * Slamcore Limited
 * All Rights Reserved.
 * (C) Copyright 2018
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

#include <slamcore/sensor_source_interface.hpp>
#include <slamcore/types/sensor_id.hpp>
#include <slamcore/types/reference_frame.hpp>
#include <slamcore/objects/static_pose.hpp>

class OdomSource : public slamcore::SensorSourceInterface
{
public:
  virtual ~OdomSource() = default;
  /**
   * @return The unique serial identifier associated with this sensor source.
   *
   * @note Must be one of the device(s) identifiers agreed upon with Slamcore in design phase.
   */
  virtual std::string getSerial() const = 0;

  /**
   * @return A lists all the sensors this source is able to provide.
   *
   * @note
   *   - Slamcore can invoke this method at any point during the sensor source
   *     instance lifecycle.
   *
   * For example, a sensor with an infrared stereo pair and an IMU might return:
   *
   *   {
   *     {SensorType::Infrared, 0},
   *     {SensorType::Infrared, 1},
   *     {SensorType::Accelerometer, 0},
   *     {SensorType::Gyroscope, 0},
   *   }
   */
  virtual std::vector<slamcore::SensorIDT> listSensors() const = 0;

  /**
   * Provides the reference frame for a given sensor.
   *
   * @param[in] sensor One of the `listSensors` entries.
   * @return The reference frame.
   *
   * @note
   *   - Implementers may throw an exception if the requested sensor does not exist.
   *   - Slamcore can only invoke this method between `open` and `close`, with reference
   *     to the lifecycle diagram above.
   *
   * For example, a sensor with an infrared stereo pair and an IMU might return:
   *
   *   {SensorType::Infrared, 0}       -->  {ReferenceFrameCategory::Camera, 0}
   *   {SensorType::Infrared, 1}       -->  {ReferenceFrameCategory::Camera, 1}
   *   {SensorType::Accelerometer, 0}  -->  {ReferenceFrameCategory::IMU, 0}
   *   {SensorType::Gyroscope, 0}      -->  {ReferenceFrameCategory::IMU, 0}
   */
  virtual slamcore::ReferenceFrame getSensorReferenceFrame(slamcore::SensorIDT sensor) const = 0;

  virtual slamcore::StaticPoseMeasurement
  getStaticTransform(const slamcore::ReferenceFrame& to, const slamcore::ReferenceFrame& from) const = 0;

  /**
   * Provides the time offset between the given destination and source sensors
   * available on the system, such that:
   *
   *   t_destination = t_source + offset
   *
   * @param[in] source One of the sensors listed in `SensorSourceInterface::listSensors`,
   * @param[in] destination Another sensor listed in `SensorSourceInterface::listSensors`.
   * @return The offset in nanoseconds from the given source to the destination sensor.
   *
   * @note
   *    - Implementers may throw an error if the offset is not known. In this case it is expected
   *      that the offset will be provided through calibration file.
   *    - Slamcore can only invoke this method between `open` and `close`, with reference
   *      to the lifecycle diagram above.
   */
  virtual std::chrono::nanoseconds getTimeOffset(slamcore::SensorIDT source, slamcore::SensorIDT destination) const = 0;

  virtual std::error_code open() = 0;

  virtual std::error_code close() = 0;

  virtual std::error_code start() = 0;

  virtual std::error_code stop() = 0;

  virtual bool isOpen() const = 0;


  virtual bool isRunning() const = 0;
};
