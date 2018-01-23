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

#ifndef _GEARBOXJOINT_HH_
#define _GEARBOXJOINT_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class GearboxJoint GearboxJoint.hh physics/physics.hh
    /// \brief A double axis gearbox joint
    template<class T>
    class GZ_PHYSICS_VISIBLE GearboxJoint : public T
    {
      /// \brief Constructor
      /// \param[in] _parent Parent link
      public: explicit GearboxJoint(BasePtr _parent)
              : T(_parent), gearRatio(1.0)
              { this->AddType(Base::GEARBOX_JOINT); }
      /// \brief Destructor
      public: virtual ~GearboxJoint()
              { }

      // Documentation inherited.
      public: virtual unsigned int DOF() const
              {return 2;}

      /// \brief Load joint
      /// \param[in] _sdf Pointer to SDF element
      public: virtual void Load(sdf::ElementPtr _sdf)
              {
                T::Load(_sdf);
                if (_sdf->HasElement("gearbox_ratio"))
                {
                  this->gearRatio =
                    _sdf->Get<double>("gearbox_ratio");
                }
                else
                {
                  gzerr << "gearbox_ratio_not_specified, set to 1.\n";
                  this->gearRatio = 1.0;
                  /* below should bring in default values for sdf 1.4+
                  this->gearRatio =
                    _sdf->Get<double>("gearbox_ratio");
                  */
                }

                if (_sdf->HasElement("gearbox_reference_body"))
                {
                  this->referenceBody =
                    _sdf->Get<std::string>("gearbox_reference_body");
                }
                else
                {
                  gzerr << "Gearbox joint missing reference body.\n";
                }
              }

      /// \brief Initialize joint
      protected: virtual void Init()
                 {
                   T::Init();
                 }

      /// \brief Get gearbox joint gear ratio.
      /// \return Gear ratio value.
      public: virtual double GetGearboxRatio() const
              { return this->gearRatio; }

      /// \brief Set gearbox joint gear ratio.
      ///
      /// This must be implemented in a child class
      /// \param[in] _index Index of the axis.
      /// \param[in] _gearRatio Gear ratio value.
      public: virtual void SetGearboxRatio(double _gearRatio) = 0;

      // Documentation inherited
      public: virtual void FillMsg(msgs::Joint &_msg)
              {
                Joint::FillMsg(_msg);
                msgs::Joint::Gearbox *gearboxMsg = _msg.mutable_gearbox();
                gearboxMsg->set_gearbox_reference_body(this->referenceBody);
                gearboxMsg->set_gearbox_ratio(this->gearRatio);
              }

      /// \brief Gearbox gearRatio
      protected: double gearRatio;

      /// \brief reference link/body for computing joint angles
      protected: std::string referenceBody;
    };
    /// \}
  }
}
#endif
