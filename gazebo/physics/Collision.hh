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
#ifndef GAZEBO_PHYSICS_COLLISION_HH_
#define GAZEBO_PHYSICS_COLLISION_HH_

#include <string>
#include <vector>

#include "gazebo/common/Event.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/CollisionState.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Base class for all collision entities
    class GZ_PHYSICS_VISIBLE Collision : public Entity
    {
      /// \brief Constructor.
      /// \param[in] _link Link that contains this collision object.
      public: explicit Collision(LinkPtr _link);

      /// \brief Destructor.
      public: virtual ~Collision();

      /// \brief Finalize the collision.
      public: virtual void Fini();

      /// \brief Load the collision.
      /// \param[in] _sdf SDF to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the collision.
      public: virtual void Init();

      /// \brief Update the parameters using new sdf values.
      /// \param[in] _sdf SDF values to update from.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Set the encapsulated collision object.
      /// Has a side effect of changing the collision bit masks
      /// \param[in] _placeable True to make the object movable.
      public: void SetCollision(bool _placeable);

      /// \brief Set if this object is moveable
      /// \param[in] _placeable True to make the object movable.
      public: void SetPlaceable(const bool _placeable);

      /// \brief Return whether this collision is movable.
      /// Example on an immovable object is a ray.
      /// \return True if the object is immovable.
      public: bool IsPlaceable() const;

      /// \brief Set the category bits, used during collision detection.
      /// \param[in] _bits The bits to set.
      public: virtual void SetCategoryBits(unsigned int _bits) = 0;

      /// \brief Set the collide bits, used during collision detection.
      /// \param[in] _bits The bits to set.
      public: virtual void SetCollideBits(unsigned int _bits) = 0;

      /// \brief Set the laser retro reflectiveness.
      /// \param[in] _retro The laser retro value.
      public: void SetLaserRetro(float _retro);

      /// \brief Get the laser retro reflectiveness
      /// \return The laser retro value.
      public: float GetLaserRetro() const;

      /// \brief Get the link this collision belongs to.
      /// \return The parent Link.
      public: LinkPtr GetLink() const;

      /// \brief Get the model this collision belongs to.
      /// \return The parent model.
      public: ModelPtr GetModel() const;

      /// \brief Get the bounding box for this collision.
      /// \return The bounding box.
      public: virtual ignition::math::Box BoundingBox() const = 0;

      /// \brief Get the shape type.
      /// \return The shape type.
      /// \sa EntityType
      public: unsigned int GetShapeType() const;

      /// \brief Set the shape for this collision.
      /// \param[in] _shape The shape for this collision object.
      public: void SetShape(ShapePtr _shape);

      /// \brief Get the collision shape.
      /// \return The collision shape.
      public: ShapePtr GetShape() const;

      /// \brief Set the scale of the collision.
      /// \param[in] _scale Scale to set the collision to.
      public: void SetScale(const ignition::math::Vector3d &_scale);

      /// \brief Get the linear velocity of the collision.
      /// \return The linear velocity relative to the parent model.
      public: virtual ignition::math::Vector3d RelativeLinearVel() const;

      /// \brief Get the linear velocity of the collision in the world
      /// frame.
      /// \return The linear velocity of the collision in the world frame.
      public: virtual ignition::math::Vector3d WorldLinearVel() const;

      /// \brief Get the angular velocity of the collision.
      /// \return The angular velocity of the collision.
      public: virtual ignition::math::Vector3d RelativeAngularVel() const;

      /// \brief Get the angular velocity of the collision in the world frame.
      /// \return The angular velocity of the collision in the world frame.
      public: virtual ignition::math::Vector3d WorldAngularVel() const;

      /// \brief Get the linear acceleration of the collision.
      /// \return The linear acceleration of the collision.
      public: virtual ignition::math::Vector3d RelativeLinearAccel() const;

      /// \brief Get the linear acceleration of the collision in the world
      /// frame.
      /// \return The linear acceleration of the collision in the world frame.
      public: virtual ignition::math::Vector3d WorldLinearAccel() const;

      /// \brief Get the angular acceleration of the collision.
      /// \return The angular acceleration of the collision.
      public: virtual ignition::math::Vector3d RelativeAngularAccel() const;

      /// \brief Get the angular acceleration of the collision in the
      /// world frame.
      /// \return The angular acceleration of the collision in the world frame.
      public: virtual ignition::math::Vector3d WorldAngularAccel() const;

      /// \brief Get the collision state.
      /// \return The collision state.
      public: CollisionState GetState();

      /// \brief Set the current collision state.
      /// \param[in] The collision state.
      public: void SetState(const CollisionState &_state);

      /// \brief Fill a collision message.
      /// \param[out] _msg The message to fill with this collision's data.
      public: void FillMsg(msgs::Collision &_msg);

      /// \brief Update parameters from a message.
      /// \param[in] _msg Message to update from.
      public: void ProcessMsg(const msgs::Collision &_msg);

      /// \brief Get the surface parameters.
      /// \return The surface parameters.
      public: inline SurfaceParamsPtr GetSurface() const
              {return this->surface;}

      /// \brief Number of contacts allowed for this collision.
      /// This overrides global value (in PhysicsEngine) if specified.
      /// \param[in] _maxContacts max num contacts allowed for this collision.
      public: virtual void SetMaxContacts(unsigned int _maxContacts);

      /// \brief returns number of contacts allowed for this collision.
      /// This overrides global value (in PhysicsEngine) if specified.
      /// \return max num contacts allowed for this collision.
      public: virtual unsigned int GetMaxContacts();

      /// \brief Indicate that the world pose should be recalculated.
      /// The recalculation will be done when Collision::GetWorldPose is
      /// called.
      public: void SetWorldPoseDirty();

      // Documentation inherited.
      public: virtual const ignition::math::Pose3d &WorldPose() const;

      /// \brief Helper function used to create a collision visual message.
      /// \return Visual message for a collision.
      private: msgs::Visual CreateCollisionVisual();

      /// \brief The link this collision belongs to
      protected: LinkPtr link;

      /// \brief Flag for placeable.
      protected: bool placeable;

      /// \brief Pointer to physics::Shape.
      protected: ShapePtr shape;

      /// \brief The surface parameters.
      protected: SurfaceParamsPtr surface;

      /// \brief The laser retro value.
      private: float laserRetro = 0.0;

      /// \brief Stores collision state information.
      private: CollisionState state;

      /// \brief Number of contact points allowed for this collision.
      private: unsigned int maxContacts;

      /// \brief Unique id for collision visual.
      private: uint32_t collisionVisualId;

      /// \brief True if the world pose should be recalculated.
      private: mutable bool worldPoseDirty;
    };
    /// \}
  }
}
#endif
