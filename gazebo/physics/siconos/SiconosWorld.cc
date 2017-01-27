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

#include "SiconosWorld.hh"

#include <MechanicsFwd.hpp>
#include <BodyDS.hpp>
#include <SiconosBulletCollisionManager.hpp>
#include <NewtonImpactFrictionNSL.hpp>
#include <GenericMechanical.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"

SiconosWorld::SiconosWorld()
{
    this->gravity.resize(3);
    this->gravity.zero();

    this->setup();
}

SiconosWorld::~SiconosWorld()
{
}

void SiconosWorld::setup()
{
  // User-defined main parameters (TODO: parameters from SDF)
  double h = 0.001;                // time step

  double theta = 0.5;              // theta for MoreauJeanOSI integrator

  // -----------------------------------------
  // --- Dynamical systems && interactions ---
  // -----------------------------------------

  try
  {
    // -- OneStepIntegrators --
    this->osi.reset(new MoreauJeanOSI(theta));

    // -- Model --
    this->model.reset(new Model(0, std::numeric_limits<double>::infinity()));

    // ------------------
    // --- Simulation ---
    // ------------------

    // -- Time discretisation --
    this->timedisc.reset(new TimeDiscretisation(0, h));

    // -- OneStepNsProblem --
    //this->osnspb.reset(new FrictionContact(3));
    SP::GenericMechanical _osnspb(
      std11::make_shared<GenericMechanical>(SICONOS_FRICTION_3D_ONECONTACT_NSN));
    this->osnspb = _osnspb;

    // -- Some configuration (TODO: parameters from SDF)
    _osnspb->numericsSolverOptions()->iparam[0] = 1000; // Max number of iterations
    _osnspb->numericsSolverOptions()->dparam[0] = 1e-5; // Tolerance
    _osnspb->numericsSolverOptions()->iparam[1] = SICONOS_FRICTION_3D_NSGS_ERROR_EVALUATION_LIGHT;
    _osnspb->numericsSolverOptions()->iparam[14] = SICONOS_FRICTION_3D_NSGS_FILTER_LOCAL_SOLUTION_TRUE;
    _osnspb->numericsSolverOptions()->internalSolvers->solverId = SICONOS_FRICTION_3D_ONECONTACT_NSN_GP_HYBRID;
    _osnspb->numericsSolverOptions()->internalSolvers->iparam[0] = 100;
    _osnspb->setMaxSize(16384);                         // max number of interactions
    _osnspb->setMStorageType(1);                        // Sparse storage
    _osnspb->setNumericsVerboseMode(0);                 // 0 silent, 1 verbose
    _osnspb->setKeepLambdaAndYState(true);              // inject previous solution

    // --- Simulation initialization ---

    // -- The collision manager adds or removes interactions using
    // -- Bullet-based contact detection
    this->manager.reset(new SiconosBulletCollisionManager());

    // -- insert a default non smooth law for contactors id 0
    // TODO handle friction properly
    this->manager->insertNonSmoothLaw(
      SP::NewtonImpactFrictionNSL(new NewtonImpactFrictionNSL(0.8, 0., 0.1, 3)), 0, 0);

    // -- MoreauJeanOSI Time Stepping for body mechanics
    this->simulation.reset(new TimeStepping(this->timedisc));

    this->simulation->insertIntegrator(this->osi);
    this->simulation->insertNonSmoothProblem(this->osnspb);
    this->simulation->insertInteractionManager(this->manager);

    // TODO: parameters from SDF
    this->simulation->setNewtonOptions(SICONOS_TS_NONLINEAR);
    this->simulation->setNewtonMaxIteration(2);
    this->simulation->setNewtonTolerance(1e-10);

    this->model->setSimulation(this->simulation);
  }

  catch (SiconosException e)
  {
    gzerr << e.report() << std::endl;
    exit(1);
  }
  catch (...)
  {
    gzerr << "Exception caught in SiconosWorld::init()" << std::endl;
    exit(1);
  }
}

void SiconosWorld::init()
{
    this->model->initialize();
}

void SiconosWorld::compute()
{
  try
  {
    float time = this->simulation->nextTime();

    GZ_ASSERT(this->simulation->hasNextEvent(), "Simulation has no more events.");

    this->simulation->computeOneStep();

    this->simulation->nextStep();
  }

  catch (SiconosException e)
  {
    gzerr << e.report() << std::endl;
  }
  catch (...)
  {
    gzerr << "Exception caught in SiconosWorld::compute()";
  }
}

SP::SiconosCollisionManager SiconosWorld::GetManager() const
{
  return manager;
}

void SiconosWorld::SetGravity(double _x, double _y, double _z)
{
    this->gravity.setValue(0, _x);
    this->gravity.setValue(1, _y);
    this->gravity.setValue(2, _z);
}
