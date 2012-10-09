/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Contact sensor
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
*/

#ifndef CONTACTSENSOR_HH
#define CONTACTSENSOR_HH

#include <stdint.h>
#include <vector>
#include <map>
#include <string>

#include "math/Angle.hh"
#include "sensors/Sensor.hh"
#include "physics/Contact.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
  
    /// \class Contact ContactSensor.hh sensors/sensors.hh
    /// \brief Contact class
    /// \TODO: nate fill in
    class Contact;

    /// \class ContactSensor ContactSensor.hh sensors/sensors.hh
    /// \addtogroup gazebo_sensors
    /// \{
    /// \brief Contact sensor.
    ///
    /// This sensor detects and reports contacts between objects
    class ContactSensor: public Sensor
    {
      /// \brief Constructor
      /// \param body The underlying collision test uses an ODE collision, so
      ///             ray sensors must be attached to a body.
      public: ContactSensor();

      /// \brief Destructor
      public: virtual ~ContactSensor();

      /// \brief Load the sensor with SDF parameters
      /// \param[in] _sdf SDF Sensor parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      /// \brief Load the sensor with default parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the sensor
      public: virtual void Init();

      /// \brief Update the sensor information
      /// \param[in] _force True if update is forced, false if not
      protected: virtual void UpdateImpl(bool _force);

      /// \brief Finalize the sensor
      protected: virtual void Fini();

      /// \brief Get the number of collisions that the sensor is observing
      /// \return Number of collisions
      public: unsigned int GetCollisionCount() const;

      /// \brief Get a collision name at index _index
      /// \param[in] _index Index of collision in collection of collisions
      /// \return name of collision
      public: std::string GetCollisionName(unsigned int _index) const;

      /// \brief Return the number of contacts for an observed collision
      /// \param[in] _collisionName The name of the observed collision
      /// \return The collision contact count
      public: unsigned int GetCollisionContactCount(
                  const std::string &_collisionName) const;

      /// \brief Get a contact for a collision by index
      /// \param[in] _collisionName Name of collision contact
      /// \param[in] _index Index of collision to get
      /// \return Contact object
      /// \TODO: nate check
      public: physics::Contact GetCollisionContact(
                  const std::string &_collisionName, unsigned int _index) const;

      /// \brief Gets contacts of a collision
      /// \param[in] _collisionName Name of collision
      /// \return Container of contacts
      /// \TODO: nate check
      public: std::map<std::string, physics::Contact> GetContacts(
                  const std::string &_collisionName);

      /// \brief Function callback that occurs during contact event
      /// \param[in] _collisionName Name of collision
      /// \param[in] _contact Contact undergoing event
      /// \TODO: nate check
      private: void OnContact(const std::string &_collisionName,
                              const physics::Contact &_contact);

      private: std::vector<physics::CollisionPtr> collisions;

      private: typedef std::map<std::string,
               std::map<std::string, physics::Contact> > Contact_M;

      private: Contact_M contacts;

      private: transport::NodePtr node;
      private: transport::PublisherPtr contactsPub;

      private: boost::mutex *mutex;

      /// \brief Gets the mutex for the sensor
      /// \return The mutex for the sensor
      /// \TODO: nate check
      public: boost::mutex* GetUpdateMutex() const
              {return this->mutex;}
    };
    /// \}
  }
}

#endif


