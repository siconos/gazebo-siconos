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
#ifndef _PHYSICSENGINE_HH_
#define _PHYSICSENGINE_HH_

#include <boost/thread/recursive_mutex.hpp>
#include <string>
#include <ignition/transport/Node.hh>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class ContactManager;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class PhysicsEngine PhysicsEngine.hh physics/physics.hh
    /// \brief Base class for a physics engine.
    class GZ_PHYSICS_VISIBLE PhysicsEngine
    {
      /// \brief Default constructor.
      /// \param[in] _world Pointer to the world.
      public: explicit PhysicsEngine(WorldPtr _world);

      /// \brief Destructor.
      public: virtual ~PhysicsEngine();

      /// \brief Load the physics engine.
      /// \param[in] _sdf Pointer to the SDF parameters.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the physics engine.
      public: virtual void Init() = 0;

      /// \brief Finilize the physics engine.
      public: virtual void Fini();

      /// \brief Rest the physics engine.
      public: virtual void Reset() {}

      /// \brief Init the engine for threads.
      public: virtual void InitForThread() = 0;

      /// \brief Update the physics engine collision.
      /// This function works in tandem with PhysicsEngine::UpdatePhysics()
      /// to update the world. This function will be called even
      /// if the physics is disabled (when World::PhysicsEnabled())
      /// returns false).
      /// Which updates are done in which of the two functions
      /// PhysicsEngine::UpdateCollision() and PhysicsEngine::UpdatePhysics()
      /// is to some extent left to the implementing physics engine.
      /// The intention is that PhysicsEngine::UpdateCollision() will update
      /// the collision states of the world, including contact information,
      /// and PhysicsEngine::UpdatePhysics() will update the dynamics of
      /// the world, i.e. advance the world and react to the collision state.
      /// However for some physics engines, both is done in one step, or
      /// providing the contact information separately in UpdateCollision()
      /// would mean double work, as it can't be avoided to be done again
      /// in PhysicsEngine::UpdatePhysics() - in this case it is better that
      /// PhysicsEngine::UpdateCollision does not actually update collision
      /// and contact information, and instead leaves it to UpdatePhysics().
      /// There should be one exception however when it still does make this
      /// update: If World::PhysicsEnabled() returns false, and therefore
      /// PhysicsEngine::UpdatePhysics() will not be called in the update
      /// step, *then* PhysicsEngine::UpdateCollision will need to ensure that
      /// collision and contact information will still be updated.
      public: virtual void UpdateCollision() = 0;

      /// \brief Return the physics engine type (ode|bullet|dart|simbody).
      /// \return Type of the physics engine.
      public: virtual std::string GetType() const = 0;

      /// \brief Set the random number seed for the physics engine.
      /// \param[in] _seed The random number seed.
      public: virtual void SetSeed(uint32_t _seed) = 0;

      /// \brief Get the simulation update period.
      /// \return Simulation update period.
      public: double GetUpdatePeriod();

      /// \brief Get target real time factor
      /// \return Target real time factor
      public: double GetTargetRealTimeFactor() const;

      /// \brief Get real time update rate
      /// \return Update rate
      public: double GetRealTimeUpdateRate() const;

      /// \brief Get max step size.
      /// \return Max step size.
      public: double GetMaxStepSize() const;

      /// \brief Set target real time factor
      /// \param[in] _factor Target real time factor
      public: void SetTargetRealTimeFactor(double _factor);

      /// \brief Set real time update rate
      /// \param[in] _rate Update rate
      public: void SetRealTimeUpdateRate(double _rate);

      /// \brief Set max step size.
      /// \param[in] _stepSize Max step size.
      public: void SetMaxStepSize(double _stepSize);

      /// \brief Update the physics engine.
      /// Will only be called if the physics are enabled, which
      /// is the case when World::PhysicsEnabled() returns true.
      /// \sa PhysicsEngine::UpdateCollision()
      public: virtual void UpdatePhysics() {}

      /// \brief Create a new model.
      /// \param[in] _base Boost shared pointer to a new model.
      public: virtual ModelPtr CreateModel(BasePtr _base);

      /// \brief Create a new body.
      /// \param[in] _parent Parent model for the link.
      public: virtual LinkPtr CreateLink(ModelPtr _parent) = 0;

      /// \brief Create a collision.
      /// \param[in] _shapeType Type of collision to create.
      /// \param[in] _link Parent link.
      public: virtual CollisionPtr CreateCollision(
                  const std::string &_shapeType, LinkPtr _link) = 0;

      /// \brief Create a collision.
      /// \param[in] _shapeType Type of collision to create.
      /// \param[in] _linkName Name of the parent link.
      public: CollisionPtr CreateCollision(const std::string &_shapeType,
                                           const std::string &_linkName);

      /// \brief Create a physics::Shape object.
      /// \param[in] _shapeType Type of shape to create.
      /// \param[in] _collision Collision parent.
      public: virtual ShapePtr CreateShape(const std::string &_shapeType,
                                           CollisionPtr _collision) = 0;

      /// \brief Create a new joint.
      /// \param[in] _type Type of joint to create.
      /// \param[in] _parent Model parent.
      public: virtual JointPtr CreateJoint(const std::string &_type,
                                           ModelPtr _parent = ModelPtr()) = 0;


      /// \brief Set the gravity vector.
      /// \param[in] _gravity New gravity vector.
      public: virtual void SetGravity(
                  const ignition::math::Vector3d &_gravity) = 0;

      /// \TODO: Remove this function, and replace it with a more generic
      /// property map
      /// \brief Access functions to set ODE parameters.
      /// \param[in] _autoDisable True to enable auto disabling of bodies.
      public: virtual void SetAutoDisableFlag(bool _autoDisable);

      /// \TODO: Remove this function, and replace it with a more generic
      /// property map
      /// \brief access functions to set ODE parameters
      /// \param[in] _maxContacts Maximum number of contacts.
      public: virtual void SetMaxContacts(unsigned int _maxContacts);

      /// \TODO: Remove this function, and replace it with a more generic
      /// property map
      /// \brief access functions to set ODE parameters..
      /// \return Auto disable flag.
      public: virtual bool GetAutoDisableFlag() {return 0;}

      /// \brief Set a parameter of the physics engine.
      /// See SetParam documentation for descriptions of duplicate parameters.
      /// \param[in] _key String key
      /// Below is a list of _key parameter definitions:
      ///       -# "solver_type" (string) - returns solver used by engine, e.g.
      ///          "sequential_impulse' for Bullet, "quick" for ODE
      ///          "Featherstone and Lemkes" for DART and
      ///          "Spatial Algebra and Elastic Foundation" for Simbody.
      ///       -# "cfm" (double) - global CFM (ODE/Bullet)
      ///       -# "erp" (double) - global ERP (ODE/Bullet)
      ///       -# "precon_iters" (bool) - precondition iterations
      ///          (experimental). (ODE)
      ///       -# "iters" (int) - number of LCP PGS iterations. If
      ///          sor_lcp_tolerance is negative, full iteration count is
      ///          executed.  Otherwise, PGS may stop iteration early if
      ///          sor_lcp_tolerance is satisfied by the total RMS residual.
      ///       -# "sor" (double) - relaxation parameter for Projected
      ///          Gauss-Seidel (PGS) updates. (ODE/Bullet)
      ///       -# "contact_max_correcting_vel" (double) - truncates correction
      ///          impulses from ERP by this value. (ODE)
      ///       -# "contact_surface_layer" (double) - ERP is 0 for
      ///          interpenetration depths below this value. (ODE/Bullet)
      ///       -# "max_contacts" (int) - max number of contact constraints
      ///          between any pair of collision bodies.
      ///       -# "min_step_size" (double) - minimum internal step size.
      ///          (defined but not used in ode).
      ///       -# "max_step_size" (double) - maximum physics step size when
      ///          physics update step must return.
      ///
      /// \param[in] _value The value to set to
      /// \return true if SetParam is successful, false if operation fails.
      public: virtual bool SetParam(const std::string &_key,
                  const boost::any &_value);

      /// \brief Get an parameter of the physics engine
      /// \param[in] _attr String key
      /// \sa SetParam
      /// \return The value of the parameter
      public: virtual boost::any GetParam(const std::string &_key) const;

      /// \brief Get a parameter from the physics engine with a boolean to
      /// indicate success or failure
      /// \param[in] _key Key of the accessed param
      /// \param[out] _value Value of the accessed param
      /// \return True if the parameter was successfully retrieved
      public: virtual bool GetParam(const std::string &_key,
                  boost::any &_value) const;

      /// \brief Debug print out of the physic engine state.
      public: virtual void DebugPrint() const = 0;

      /// \brief Get a pointer to the world.
      /// \return Pointer to the world.
      public: WorldPtr World() const;

      /// \brief Get a pointer to the contact manger.
      /// \return Pointer to the contact manager.
      public: ContactManager *GetContactManager() const;

      /// \brief returns a pointer to the PhysicsEngine#physicsUpdateMutex.
      /// \return Pointer to the physics mutex.
      public: boost::recursive_mutex *GetPhysicsUpdateMutex() const
              {return this->physicsUpdateMutex;}

      /// \brief Get a pointer to the SDF element for this physics engine.
      /// \return Pointer to the physics SDF element.
      public: sdf::ElementPtr GetSDF() const;

      /// \brief virtual callback for gztopic "~/request".
      /// \param[in] _msg Request message.
      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      /// \brief virtual callback for gztopic "~/physics".
      /// \param[in] _msg Physics message.
      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      /// \brief Pointer to the world.
      protected: WorldPtr world;

      /// \brief Our SDF values.
      protected: sdf::ElementPtr sdf;

      /// \brief Node for communication.
      protected: transport::NodePtr node;

      /// \brief Response publisher.
      protected: transport::PublisherPtr responsePub;

      /// \brief Subscribe to the physics topic.
      protected: transport::SubscriberPtr physicsSub;

      /// \brief Subscribe to the request topic.
      protected: transport::SubscriberPtr requestSub;

      /// \brief Mutex to protect the update cycle.
      protected: boost::recursive_mutex *physicsUpdateMutex;

      /// \brief Class that handles all contacts generated by the physics
      /// engine.
      protected: ContactManager *contactManager;

      /// \brief Real time update rate.
      protected: double realTimeUpdateRate;

      /// \brief Target real time factor.
      protected: double targetRealTimeFactor;

      /// \brief Real time update rate.
      protected: double maxStepSize;

      // Place ignition::transport objects at the end of this file to
      // guarantee they are destructed first.

      /// \brief Ignition node for communication.
      protected: ignition::transport::Node nodeIgn;

      /// \brief Response publisher.
      protected: ignition::transport::Node::Publisher responsePubIgn;
    };
    /// \}
  }
}
#endif
