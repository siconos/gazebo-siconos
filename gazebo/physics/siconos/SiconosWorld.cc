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

#include <SiconosBodies.hpp>
#include <SiconosKernel.hpp>

#include <BulletSpaceFilter.hpp>
#include <BulletTimeStepping.hpp>
//#include <BulletWeightedShape.hpp> ??
//#include <BulletDS.hpp> ??
//#include <BulletR.hpp> ??

#include <bullet/BulletCollision/btBulletCollisionCommon.h>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

SiconosWorld::SiconosWorld()
{
    this->gravity.resize(3);
    this->gravity.zero();
    this->gravity_changed = false;
}

SiconosWorld::~SiconosWorld()
{
}

void SiconosWorld::init()
{
  printf("SiconosWorld::init()\n");
  // User-defined main parameters (TODO: parameters from SDF)
  double h = 0.005;                // time step
  double position_init = 10.0;     // initial position
  double velocity_init = 0.0;      // initial velocity

  double g = 9.81;
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

#if 0
    // -- Create the first dynamical system here, but don't insert it
    //    until after the simulation starts.
    SP::BulletDS body( makeBox(g, position_init, shapes) );

    // -- Add the dynamical system in the non smooth dynamical system
    osi->insertDynamicalSystem(body);
    model->nonSmoothDynamicalSystem()->insertDynamicalSystem(body);
#endif

#if 1
    // Ground code -- TODO move to SiconosPlaneShape
    this->ground.reset(new btCollisionObject());
    ground->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
    this->groundShape.reset(new btBoxShape(btVector3(30, 30, .5)));
    btMatrix3x3 basis;
    basis.setIdentity();
    this->ground->getWorldTransform().setBasis(basis);
    this->ground->setCollisionShape(&*groundShape);
    this->ground->getWorldTransform().getOrigin().setZ(-.50);
#endif

    // ------------------
    // --- Simulation ---
    // ------------------

    // -- Time discretisation --
    this->timedisc.reset(new TimeDiscretisation(0, h));

    // -- OneStepNsProblem --
    this->osnspb.reset(new FrictionContact(3));

    // -- Some configuration (TODO: parameters from SDF)
    this->osnspb->numericsSolverOptions()->iparam[0] = 1000; // Max number of iterations
    this->osnspb->numericsSolverOptions()->dparam[0] = 1e-5; // Tolerance
    this->osnspb->setMaxSize(16384);                         // max number of interactions
    this->osnspb->setMStorageType(1);                        // Sparse storage
    this->osnspb->setNumericsVerboseMode(0);                 // 0 silent, 1 verbose
    this->osnspb->setKeepLambdaAndYState(true);              // inject previous solution

    // --- Simulation initialization ---

    this->nslaw.reset(new NewtonImpactFrictionNSL(0.8, 0., 0.0, 3));

    // -- The space filter performs broadphase collision detection
    this->space_filter.reset(new BulletSpaceFilter(this->model));

    // -- insert a non smooth law for contactors id 0
    this->space_filter->insert(this->nslaw, 0, 0);

    // -- add multipoint iterations, this is needed to gather at least
    // -- 3 contact points and avoid objects penetration, see Bullet
    // -- documentation
    this->space_filter->collisionConfiguration()->setConvexConvexMultipointIterations();
    this->space_filter->collisionConfiguration()->setPlaneConvexMultipointIterations();

    // -- The ground is a static object
    // -- we give it a group contactor id : 0
    // -- TODO move to SiconosPlaneShape
    this->space_filter->addStaticObject(this->ground, 0);

    // -- MoreauJeanOSI Time Stepping with Bullet Dynamical Systems
    this->simulation.reset(new BulletTimeStepping(this->timedisc));

    this->simulation->insertIntegrator(this->osi);
    this->simulation->insertNonSmoothProblem(this->osnspb);

    this->model->initialize(this->simulation);
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

void SiconosWorld::compute()
{
  printf("SiconosWorld::compute() (time == %f)\n", this->model->currentTime());
  try
  {
    if (this->gravity_changed)
    {
        // TODO: add gravity to all objects
        this->gravity_changed = false;
    }

    if (!this->simulation->hasNextEvent())
        return;

    this->space_filter->buildInteractions(this->model->currentTime());

    this->simulation->computeOneStep();

    this->simulation->nextStep();

    // _model->simulation()->advanceToEvent();
    // _model->simulation()->processEvents();
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

void SiconosWorld::SetGravity(double _x, double _y, double _z)
{
    this->gravity.setValue(0, _x);
    this->gravity.setValue(1, _y);
    this->gravity.setValue(2, _z);
    this->gravity_changed = true;
}
