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

#include <gazebo/physics/PhysicsTypes.hh>

#include "siconos_inc.h"

#define NDOF 6

class SiconosWorld
{
  /// \brief Constructor
  public: SiconosWorld();

  /// \brief Destructor
  public: ~SiconosWorld();

  /// \brief Set up the simulation data structures
  public: virtual void setup();

  /// \brief Initialization of the simulation
  public: virtual void init();

  /// \brief Compute a step of the simulation
  public: virtual void compute();

  /// \brief Gravity
  public: void SetGravity(double x, double y, double z);

  /// \brief Gravity vector
  private: SiconosVector gravity;

  /// \brief The Siconos OneStepIntegrator for this simulation
  private: SP::OneStepIntegrator osi;

  public: SP::OneStepIntegrator GetOneStepIntegrator() const { return osi; }

  /// \brief The Siconos Model for this simulation
  private: SP::Model model;

  /// \brief Get the Siconos Model for this simulation
  public: SP::Model GetModel() const { return model; }

  /// \brief The Siconos TimeDiscretisation for this simulation
  private: SP::TimeDiscretisation timedisc;

  /// \brief The Siconos OneStepNonSmoothProblem for this
  ///        simulation's friction constraint
  private: SP::FrictionContact osnspb;

  /// \brief The Siconos broadphase collision manager
  private: SP::SiconosBulletCollisionManager manager;

  /// \brief Get the Siconos CollisionManager for this simulation
  public: SP::SiconosCollisionManager GetManager() const;

  /// \brief The Siconos TimeStepping handler for this simulation
  private: SP::TimeStepping simulation;

  /// \brief Get the Siconos Simulation for this simulation
  public: SP::TimeStepping GetSimulation() const { return simulation; }
};

#endif
