/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
/* Desc: The Siconos physics engine wrapper
 * Author: Stephen Sinclair <stephen.sinclair@inria.cl>
 * Date: 01-01-2016
 */

#ifndef SICONOSPHYSICS_HH
#define SICONOSPHYSICS_HH
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"

#include <SiconosFwd.hpp>
#include <MechanicsFwd.hpp>

class SiconosWorld;
namespace SP { typedef std11::shared_ptr<::SiconosWorld> SiconosWorld; }

namespace gazebo
{
  namespace physics
  {
    class Entity;
    class XMLConfigNode;
    class Mass;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_siconos Siconos Physics
    /// \{

    /// \brief Siconos physics engine
    class GZ_PHYSICS_VISIBLE SiconosPhysics : public PhysicsEngine
    {
      /// \brief Constructor
      public: SiconosPhysics(WorldPtr _world);

      /// \brief Destructor
      public: virtual ~SiconosPhysics();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Reset();

      // Documentation inherited
      public: virtual void InitForThread();

      // Documentation inherited
      public: virtual void UpdateCollision();

      // Documentation inherited
      public: virtual void UpdatePhysics();

      // Documentation inherited
      public: virtual std::string GetType() const
                      { return "siconos"; }

      // Documentation inherited
      public: virtual void SetSeed(uint32_t _seed);

      // Documentation inherited
      public: virtual LinkPtr CreateLink(ModelPtr _parent);

      // Documentation inherited
      public: virtual CollisionPtr CreateCollision(const std::string &_type,
                                                   LinkPtr _parent);

      // Documentation inherited
      public: virtual JointPtr CreateJoint(const std::string &_type,
                                           ModelPtr _parent = ModelPtr());

      // Documentation inherited
      public: virtual ShapePtr CreateShape(const std::string &_shapeType,
                                           CollisionPtr _collision);

      // Documentation inherited
      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      // Documentation inherited
      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      // Documentation inherited
      public: virtual void SetGravity(
                  const ignition::math::Vector3d &_gravity);

      /// \brief Return the composite dynamical system (world)
      public: SP::SiconosWorld GetSiconosWorld() const
        {return this->siconosWorld;}

      // Documentation inherited
      public: virtual void DebugPrint() const;

      /// Documentation inherited
      public: virtual boost::any GetParam(const std::string &_key) const;

      /// Documentation inherited
      public: virtual bool GetParam(const std::string &_key,
          boost::any &_value) const;

      private: SP::SiconosWorld siconosWorld;

      private: common::Time lastUpdateTime;

      /// \brief The type of the solver.
      private: std::string solverType;

    /// \}
    };

  /// \}
  }
}
#endif
