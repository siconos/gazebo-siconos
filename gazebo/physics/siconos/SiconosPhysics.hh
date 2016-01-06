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

namespace gazebo
{
  namespace physics
  {
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

      /// \brief Initialize the physics engine.
      public: virtual void Init();

      /// \brief Init the engine for threads.
      public: virtual void InitForThread();

      /// \brief Update the physics engine collision.
      public: virtual void UpdateCollision();

      /// \brief Return the physics engine type (ode|bullet|dart|simbody).
      /// \return Type of the physics engine.
      public: virtual std::string GetType() const
        { return "siconos"; }

      /// \brief Set the random number seed for the physics engine.
      /// \param[in] _seed The random number seed.
      public: virtual void SetSeed(uint32_t _seed);

      /// \brief Create a new body.
      /// \param[in] _parent Parent model for the link.
      public: virtual LinkPtr CreateLink(ModelPtr _parent);

      /// \brief Create a collision.
      /// \param[in] _shapeType Type of collision to create.
      /// \param[in] _link Parent link.
      public: virtual CollisionPtr CreateCollision(
                  const std::string &_shapeType, LinkPtr _parent);

      /// \brief Create a physics::Shape object.
      /// \param[in] _shapeType Type of shape to create.
      /// \param[in] _collision Collision parent.
      public: virtual ShapePtr CreateShape(const std::string &_shapeType,
                                           CollisionPtr _collision);

      /// \brief Create a new joint.
      /// \param[in] _type Type of joint to create.
      /// \param[in] _parent Model parent.
      public: virtual JointPtr CreateJoint(const std::string &_type,
                                           ModelPtr _parent = ModelPtr());

      /// \brief Set the gravity vector.
      /// \param[in] _gravity New gravity vector.
      public: virtual void SetGravity(
                  const gazebo::math::Vector3 &_gravity);

      /// \brief Debug print out of the physic engine state.
      public: virtual void DebugPrint() const;

      /// \brief Return the composite dynamical system (world)
      public: SP::Model GetDynamicsWorld() const
        {return this->dynamicsWorld;}

      private: SP::Model dynamicsWorld;

      private: common::Time lastUpdateTime;

      /// \brief The type of the solver.
      private: std::string solverType;

    /// \}
    };

  /// \}
  }
}
#endif
