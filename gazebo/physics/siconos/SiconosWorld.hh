/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SICONOSWORLD_HH
#define _GAZEBO_SICONOSWORLD_HH

#include "siconos_inc.h"

#define NDOF 6

class SiconosWorld
{
  /// \brief Constructor
  public: SiconosWorld();

  /// \brief Destructor
  public: ~SiconosWorld();
    
  /// \brief Initialization of the simulation
  public: virtual void init();

  /// \brief Compute a step of the simulation
  public: virtual void compute();

  /// \brief Gravity
  public: void SetGravity(double x, double y, double z);

  /// \brief Gravity vector
  private: SiconosVector gravity;
  private: bool gravity_changed;

  /// \brief The Siconos OneStepIntegrator for this simulation
  private: SP::OneStepIntegrator osi;

  /// \brief The Siconos Model for this simulation
  private: SP::Model model;

  /// \brief The Siconos TimeDiscretisation for this simulation
  private: SP::TimeDiscretisation timedisc;

  /// \brief The Siconos OneStepNonSmoothProblem for this
  ///        simulation's friction constraint
  private: SP::FrictionContact osnspb;

  /// \brief The Siconos NonSmoothLaw governing contact constraints
  private: SP::NonSmoothLaw nslaw;

  /// \brief The Siconos broadphase collision manager
  private: SP::BulletSpaceFilter space_filter;

  /// \brief The Siconos TimeStepping handler for this simulation
  private: SP::BulletTimeStepping simulation;

  // temporary
  SP::btCollisionObject ground;
  SP::btCollisionShape groundShape;
};

#endif
