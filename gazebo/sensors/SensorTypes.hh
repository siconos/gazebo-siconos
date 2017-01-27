/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GAZEBO_SENSORS_SENSORTYPES_HH_
#define _GAZEBO_SENSORS_SENSORTYPES_HH_

#include <vector>
#include <memory>
#include "gazebo/util/system.hh"
#include "gazebo/common/EnumIface.hh"

/// \file
/// \ingroup gazebo_sensors
/// \brief Forward declarations and typedefs for sensors
namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    class AltimeterSensor;
    class Sensor;
    class RaySensor;
    class CameraSensor;
    class LogicalCameraSensor;
    class MagnetometerSensor;
    class MultiCameraSensor;
    class DepthCameraSensor;
    class ContactSensor;
    class ImuSensor;
    class GpuRaySensor;
    class RFIDSensor;
    class RFIDTag;
    class SonarSensor;
    class ForceTorqueSensor;
    class GpsSensor;
    class Noise;
    class GaussianNoiseModel;
    class ImageGaussianNoiseModel;
    class WideAngleCameraSensor;
    class WirelessTransceiver;
    class WirelessTransmitter;
    class WirelessReceiver;

    /// \def AltimeterSensorPtr
    /// \brief Shared pointer to AltimeterSensor
    typedef std::shared_ptr<AltimeterSensor> AltimeterSensorPtr;

    /// \def SensorPtr
    /// \brief Shared pointer to Sensor
    typedef std::shared_ptr<Sensor> SensorPtr;

    /// \def RaySensorPtr
    /// \brief Shared pointer to RaySensor
    typedef std::shared_ptr<RaySensor> RaySensorPtr;

    /// \def CameraSensorPtr
    /// \brief Shared pointer to CameraSensor
    typedef std::shared_ptr<CameraSensor> CameraSensorPtr;

    /// \def MagnetometerSensorPtr
    /// \brief Shared pointer to MagnetometerSensor
    typedef std::shared_ptr<MagnetometerSensor> MagnetometerSensorPtr;

    /// \def MultiCameraSensorPtr
    /// \brief Shared pointer to MultiCameraSensor
    typedef std::shared_ptr<MultiCameraSensor> MultiCameraSensorPtr;

    /// \def DepthCameraSensorPtr
    /// \brief Shared pointer to DepthCameraSensor
    typedef std::shared_ptr<DepthCameraSensor> DepthCameraSensorPtr;

    /// \def WideAngleCameraSensorPtr
    /// \brief Shared pointer to WideAngleCameraSensor
    typedef std::shared_ptr<WideAngleCameraSensor> WideAngleCameraSensorPtr;

    /// \def ContactSensorPtr
    /// \brief Shared pointer to ContactSensor
    typedef std::shared_ptr<ContactSensor> ContactSensorPtr;

    /// \def ImuSensorPtr
    /// \brief Shared pointer to ImuSensor
    typedef std::shared_ptr<ImuSensor> ImuSensorPtr;

    /// \def GpuRaySensorPtr
    /// \brief Shared pointer to GpuRaySensor
    typedef std::shared_ptr<GpuRaySensor> GpuRaySensorPtr;

    /// \def RFIDSensorPtr
    /// \brief Shared pointer to RFIDSensor
    typedef std::shared_ptr<RFIDSensor> RFIDSensorPtr;

    /// \def RFIDTagPtr
    /// \brief Shared pointer to RFIDTag
    typedef std::shared_ptr<RFIDTag> RFIDTagPtr;

    /// \def SonarSensorPtr
    /// \brief Shared pointer to SonarSensor
    typedef std::shared_ptr<SonarSensor> SonarSensorPtr;

    /// \def ForceTorqueSensorPtr
    /// \brief Shared pointer to ForceTorqueSensor
    typedef std::shared_ptr<ForceTorqueSensor> ForceTorqueSensorPtr;

    /// \def GpsSensorPtr
    /// \brief Shared pointer to GpsSensor
    typedef std::shared_ptr<GpsSensor> GpsSensorPtr;

    /// \def NoisePtr
    /// \brief Shared pointer to Noise
    typedef std::shared_ptr<Noise> NoisePtr;

    /// \def GaussianNoisePtr
    /// \brief Shared pointer to Noise
    typedef std::shared_ptr<GaussianNoiseModel> GaussianNoiseModelPtr;

    /// \brief Shared pointer to Noise
    typedef std::shared_ptr<ImageGaussianNoiseModel>
        ImageGaussianNoiseModelPtr;

    /// \def WirelessTransceiverPtr
    /// \brief Shared pointer to WirelessTransceiver
    typedef std::shared_ptr<WirelessTransceiver> WirelessTransceiverPtr;

    /// \def WirelessTransmitterPtr
    /// \brief Shared pointer to WirelessTransmitter
    typedef std::shared_ptr<WirelessTransmitter> WirelessTransmitterPtr;

    /// \def WirelessReceiverPtr
    /// \brief Shared pointer to WirelessReceiver
    typedef std::shared_ptr<WirelessReceiver> WirelessReceiverPtr;

    /// \def AltimeterSensor_V
    /// \brief Vector of AltimeterSensor shared pointers
    typedef std::vector<AltimeterSensor> AltimeterSensor_V;

    /// \def Sensor_V
    /// \brief Vector of Sensor shared pointers
    typedef std::vector<SensorPtr> Sensor_V;

    /// \def RaySensor_V
    /// \brief Vector of RaySensor shared pointers
    typedef std::vector<RaySensorPtr> RaySensor_V;

    /// \def CameraSensor_V
    /// \brief Vector of CameraSensor shared pointers
    typedef std::vector<CameraSensorPtr> CameraSensor_V;

    /// \def MultiCameraSensor_V
    /// \brief Vector of MultiCameraSensor shared pointers
    typedef std::vector<MultiCameraSensorPtr> MultiCameraSensor_V;

    /// \def DepthCameraSensor_V
    /// \brief Vector of DepthCameraSensor shared pointers
    typedef std::vector<DepthCameraSensorPtr> DepthCameraSensor_V;

    /// \def ContactSensor_V
    /// \brief Vector of ContactSensor shared pointers
    typedef std::vector<ContactSensorPtr> ContactSensor_V;

    /// \def ImuSensor_V
    /// \brief Vector of ImuSensor shared pointers
    typedef std::vector<ImuSensorPtr> ImuSensor_V;

    /// \def GpuRaySensor_V
    /// \brief Vector of GpuRaySensor shared pointers
    typedef std::vector<GpuRaySensorPtr> GpuRaySensor_V;

    /// \def RFIDSensor_V
    /// \brief Vector of RFIDSensors
    typedef std::vector<RFIDSensor> RFIDSensor_V;

    /// \def RFIDTag_V
    /// \brief Vector of RFIDTags
    typedef std::vector<RFIDTag> RFIDTag_V;

    /// \def WirelessTransceiver_V
    /// \brief Vector of WirelessTransceiver
    typedef std::vector<WirelessTransceiver> WirelessTransceiver_V;

    /// \def WirelessTransmitter_V
    /// \brief Vector of WirelessTransmitter
    typedef std::vector<WirelessTransmitter> WirelessTransmitter_V;

    /// \def WirelessReceiver_V
    /// \brief Vector of WirelessReceiver
    typedef std::vector<WirelessReceiver> WirelessReceiver_V;

    /// \def LogicalCameraSensorPtr
    /// \brief Shared pointer to LogicalCameraSensor
    typedef std::shared_ptr<LogicalCameraSensor> LogicalCameraSensorPtr;

    /// \def SensorNoiseType
    /// \brief Eumeration of all sensor noise types
    enum SensorNoiseType
    {
      /// \internal
      /// \brief Indicator used to create an iterator over the enum. Do not
      /// use this.
      SENSOR_NOISE_TYPE_BEGIN = 0,

      /// \brief Noise streams for the Camera sensor
      /// \sa CameraSensor
      NO_NOISE = SENSOR_NOISE_TYPE_BEGIN,

      /// \brief Noise streams for the Camera sensor
      /// \sa CameraSensor
      CAMERA_NOISE = 1,

      /// \brief Noise streams for the GPU ray sensor
      /// \sa GpuRaySensor
      GPU_RAY_NOISE = 2,

      /// \brief GPS position latitude noise streams
      /// \sa GpsSensor
      GPS_POSITION_LATITUDE_NOISE_METERS = 3,

      /// \brief GPS position longitude noise streams
      /// \sa GpsSensor
      GPS_POSITION_LONGITUDE_NOISE_METERS = 4,

      /// \brief GPS position altitude noise streams
      /// \sa GpsSensor
      GPS_POSITION_ALTITUDE_NOISE_METERS = 5,

      /// \brief GPS velocity latitude noise streams
      /// \sa GpsSensor
      GPS_VELOCITY_LATITUDE_NOISE_METERS = 6,

      /// \brief GPS velocity longitude noise streams
      /// \sa GpsSensor
      GPS_VELOCITY_LONGITUDE_NOISE_METERS = 7,

      /// \brief GPS velocity altitude noise streams
      /// \sa GpsSensor
      GPS_VELOCITY_ALTITUDE_NOISE_METERS = 8,

      /// \brief Noise streams for the ray sensor
      /// \sa RaySensor
      RAY_NOISE = 9,

      /// \brief Magnetometer body-frame X axis noise in Tesla
      /// \sa MagnetometerSensor
      MAGNETOMETER_X_NOISE_TESLA = 10,

      /// \brief Magnetometer body-frame Y axis noise in Tesla
      /// \sa MagnetometerSensor
      MAGNETOMETER_Y_NOISE_TESLA = 11,

      /// \brief Magnetometer body-frame Z axis noise in Tesla
      /// \sa MagnetometerSensor
      MAGNETOMETER_Z_NOISE_TESLA = 12,

      /// \brief Vertical noise stream for the altimeter sensor
      /// \sa AltimeterSensor
      ALTIMETER_POSITION_NOISE_METERS = 13,

      /// \brief Velocity noise streams for the altimeter sensor
      /// \sa AltimeterSensor
      ALTIMETER_VELOCITY_NOISE_METERS_PER_S = 14,

      /// \brief IMU angular velocity X noise stream
      /// \sa ImuSensor
      IMU_ANGVEL_X_NOISE_RADIANS_PER_S = 15,

      /// \brief IMU angular velocity Y noise stream
      /// \sa ImuSensor
      IMU_ANGVEL_Y_NOISE_RADIANS_PER_S = 16,

      /// \brief IMU angular velocity Z noise stream
      /// \sa ImuSensor
      IMU_ANGVEL_Z_NOISE_RADIANS_PER_S = 17,

      /// \brief IMU linear acceleration X noise stream
      /// \sa ImuSensor
      IMU_LINACC_X_NOISE_METERS_PER_S_SQR = 18,

      /// \brief IMU linear acceleration Y noise stream
      /// \sa ImuSensor
      IMU_LINACC_Y_NOISE_METERS_PER_S_SQR = 19,

      /// \brief IMU linear acceleration Z noise stream
      /// \sa ImuSensor
      IMU_LINACC_Z_NOISE_METERS_PER_S_SQR = 20,

      /// \internal
      /// \brief Indicator used to create an iterator over the enum. Do not
      /// use this.
      SENSOR_NOISE_TYPE_END
    };
    /// \}

    /// \brief SensorCategory is used to categorize sensors. This is used to
    /// put sensors into different threads.
    enum SensorCategory
    {
      // IMAGE must be the first element, and it must start with 0. Do not
      // change this! See SensorManager::sensorContainers for reference.
      /// \brief Image based sensor class. This type requires the rendering
      /// engine.
      IMAGE = 0,

      /// \brief Ray based sensor class.
      RAY = 1,

      /// \brief A type of sensor is not a RAY or IMAGE sensor.
      OTHER = 2,

      /// \brief Number of Sensor Categories
      CATEGORY_COUNT = 3
    };
  }
}
#endif
