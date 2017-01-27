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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/algorithm/string.hpp>

#include "gazebo/sensors/SensorFactory.hh"

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/sensors/GpsSensorPrivate.hh"
#include "gazebo/sensors/GpsSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("gps", GpsSensor)

/////////////////////////////////////////////////
GpsSensor::GpsSensor()
: Sensor(sensors::OTHER),
  dataPtr(new GpsSensorPrivate)
{
}

/////////////////////////////////////////////////
GpsSensor::~GpsSensor()
{
}

/////////////////////////////////////////////////
void GpsSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void GpsSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  physics::EntityPtr parentEntity =
    this->world->EntityByName(this->ParentName());
  this->dataPtr->parentLink =
    boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  this->dataPtr->lastGpsMsg.set_link_name(this->ParentName());

  this->dataPtr->topicName = "~/" + this->ParentName() + '/' + this->Name();
  if (this->sdf->HasElement("topic"))
  {
    this->dataPtr->topicName += '/' +
      this->sdf->Get<std::string>("topic");
  }
  boost::replace_all(this->dataPtr->topicName, "::", "/");

  this->dataPtr->gpsPub =
    this->node->Advertise<msgs::GPS>(this->dataPtr->topicName, 50);

  // Parse sdf noise parameters
  sdf::ElementPtr gpsElem = this->sdf->GetElement("gps");

  // Load position noise parameters
  {
    sdf::ElementPtr posElem = gpsElem->GetElement("position_sensing");
    this->noises[GPS_POSITION_LATITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          posElem->GetElement("horizontal")->GetElement("noise"));
    this->noises[GPS_POSITION_LONGITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          posElem->GetElement("horizontal")->GetElement("noise"));
    this->noises[GPS_POSITION_ALTITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          posElem->GetElement("vertical")->GetElement("noise"));
  }

  // Load velocity noise parameters
  {
    sdf::ElementPtr velElem = gpsElem->GetElement("velocity_sensing");
    this->noises[GPS_VELOCITY_LATITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          velElem->GetElement("horizontal")->GetElement("noise"));
    this->noises[GPS_VELOCITY_LONGITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          velElem->GetElement("horizontal")->GetElement("noise"));
    this->noises[GPS_VELOCITY_ALTITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          velElem->GetElement("vertical")->GetElement("noise"));
  }
}

/////////////////////////////////////////////////
void GpsSensor::Fini()
{
  Sensor::Fini();
  this->dataPtr->parentLink.reset();
  this->dataPtr->sphericalCoordinates.reset();
}

//////////////////////////////////////////////////
void GpsSensor::Init()
{
  Sensor::Init();

  this->dataPtr->sphericalCoordinates = this->world->SphericalCoords();
}

//////////////////////////////////////////////////
bool GpsSensor::UpdateImpl(const bool /*_force*/)
{
  // Get latest pose information
  if (this->dataPtr->parentLink)
  {
    // Measure position and apply noise
    {
      // Get postion in Cartesian gazebo frame
      ignition::math::Pose3d gpsPose = this->pose +
        this->dataPtr->parentLink->WorldPose();

      // Apply position noise before converting to global frame
      gpsPose.Pos().X(
        this->noises[GPS_POSITION_LATITUDE_NOISE_METERS]->Apply(
          gpsPose.Pos().X()));
      gpsPose.Pos().Y(
        this->noises[GPS_POSITION_LONGITUDE_NOISE_METERS]->Apply(
          gpsPose.Pos().Y()));
      gpsPose.Pos().Z(
        this->noises[GPS_POSITION_ALTITUDE_NOISE_METERS]->Apply(
          gpsPose.Pos().Z()));

      // Convert to global frames
      ignition::math::Vector3d spherical = this->dataPtr->sphericalCoordinates->
        SphericalFromLocal(gpsPose.Pos());
      this->dataPtr->lastGpsMsg.set_latitude_deg(spherical.X());
      this->dataPtr->lastGpsMsg.set_longitude_deg(spherical.Y());
      this->dataPtr->lastGpsMsg.set_altitude(spherical.Z());
    }

    // Measure velocity and apply noise
    {
      ignition::math::Vector3d gpsVelocity =
        this->dataPtr->parentLink->WorldLinearVel(this->pose.Pos());

      // Convert to global frame
      gpsVelocity =
        this->dataPtr->sphericalCoordinates->GlobalFromLocal(gpsVelocity);

      // Apply noise after converting to global frame
      gpsVelocity.X(
        this->noises[GPS_VELOCITY_LATITUDE_NOISE_METERS]->Apply(
          gpsVelocity.X()));
      gpsVelocity.Y(
        this->noises[GPS_VELOCITY_LONGITUDE_NOISE_METERS]->Apply(
          gpsVelocity.Y()));
      gpsVelocity.Z(
        this->noises[GPS_VELOCITY_ALTITUDE_NOISE_METERS]->Apply(
          gpsVelocity.Z()));

      this->dataPtr->lastGpsMsg.set_velocity_east(gpsVelocity.X());
      this->dataPtr->lastGpsMsg.set_velocity_north(gpsVelocity.Y());
      this->dataPtr->lastGpsMsg.set_velocity_up(gpsVelocity.Z());
    }
  }
  this->lastMeasurementTime = this->world->SimTime();
  msgs::Set(this->dataPtr->lastGpsMsg.mutable_time(),
      this->lastMeasurementTime);

  if (this->dataPtr->gpsPub)
    this->dataPtr->gpsPub->Publish(this->dataPtr->lastGpsMsg);

  return true;
}

//////////////////////////////////////////////////
ignition::math::Angle GpsSensor::Longitude() const
{
  ignition::math::Angle angle;
  angle.Degree(this->dataPtr->lastGpsMsg.longitude_deg());
  return angle;
}

//////////////////////////////////////////////////
ignition::math::Angle GpsSensor::Latitude() const
{
  ignition::math::Angle angle;
  angle.Degree(this->dataPtr->lastGpsMsg.latitude_deg());
  return angle;
}

//////////////////////////////////////////////////
double GpsSensor::Altitude() const
{
  return this->dataPtr->lastGpsMsg.altitude();
}
