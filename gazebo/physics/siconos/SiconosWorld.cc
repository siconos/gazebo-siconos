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
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/physics/siconos/SiconosTypes.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosSurfaceParams.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"

DEFINE_SPTR(GazeboCollisionManager)

struct SiconosWorldImpl
{
  /// \brief Gravity vector
  SiconosVector gravity;

  /// \brief The Siconos OneStepIntegrator for this simulation
  SP::OneStepIntegrator osi;

  /// \brief The Siconos Model for this simulation
  SP::Model model;

  /// \brief The Siconos TimeDiscretisation for this simulation
  SP::TimeDiscretisation timedisc;

  /// \brief The Siconos OneStepNonSmoothProblem for this
  ///        simulation's constraints
  SP::OneStepNSProblem osnspb;

  /// \brief The Siconos broadphase collision manager
  SP::GazeboCollisionManager manager;

  /// \brief The Siconos TimeStepping handler for this simulation
  SP::TimeStepping simulation;

  /// \brief Gazebo SiconosPhysics physics engine
  gazebo::physics::SiconosPhysics *physics;

  /// \brief Store timestep so we can check if it changed
  double maxStepSize;
};

class GazeboCollisionManager : public SiconosBulletCollisionManager
{
public:
  GazeboCollisionManager(const SiconosBulletOptions &options,
                         std::shared_ptr<SiconosWorldImpl> _impl)
    : SiconosBulletCollisionManager(options), impl(_impl) {}

  virtual SP::NonSmoothLaw nonSmoothLaw(long unsigned int collision_group1,
                                        long unsigned int collision_group2);

private:
  std::shared_ptr<SiconosWorldImpl> impl;
};

SP::NonSmoothLaw GazeboCollisionManager::nonSmoothLaw(
  long unsigned int collision_group1,
  long unsigned int collision_group2)
{
  // Return an existing NSL if it has already been created

  SP::NonSmoothLaw nsl =
    SiconosBulletCollisionManager::nonSmoothLaw(collision_group1, collision_group2);
  if (nsl) return nsl;

  // Create NSL on-demand based on combined group surface properties.
  // We combine them using similar rules (std::min) as the ODE physics
  // engine, see ODEPhysics::Collide().

  using namespace gazebo::physics;
  SiconosSurfaceParamsPtr surface1
    = this->impl->physics->GetCollisionGroupSurfaceParams(collision_group1);
  SiconosSurfaceParamsPtr surface2
    = this->impl->physics->GetCollisionGroupSurfaceParams(collision_group2);

  // Default friction mu = 1.0 is very high, but taken from
  // FrictionPyramid constructor.

  double mu = 1.0;
  if (surface1->FrictionPyramid() && surface2->FrictionPyramid())
    mu = std::min(surface1->FrictionPyramid()->MuPrimary(),
                  surface2->FrictionPyramid()->MuPrimary());
  else if (surface1->FrictionPyramid())
    mu = surface1->FrictionPyramid()->MuPrimary();
  else if (surface2->FrictionPyramid())
    mu = surface2->FrictionPyramid()->MuPrimary();

  double restitution = std::min(surface1->normal_restitution,
                                surface2->normal_restitution);

  // Note, in case investigating why nothing bounces: Since std::min()
  //   is used here, and the default restitution_coefficient is zero,
  //   by default nothing will bounce if it is not specified in the
  //   SDF, which is the case for the ground_plane model!

  nsl = std11::make_shared<NewtonImpactFrictionNSL>(restitution, 0, mu, 3);
  this->insertNonSmoothLaw(nsl, collision_group1, collision_group2);
  return nsl;
}

SiconosWorld::SiconosWorld(gazebo::physics::SiconosPhysics *physics)
  : impl(new SiconosWorldImpl())
{
    this->impl->gravity.resize(3);
    this->impl->gravity.zero();
    this->impl->physics = physics;
    this->impl->maxStepSize = 0;
}

SiconosWorld::~SiconosWorld()
{
}

void SiconosWorld::setup()
{
  // User-defined main parameters (TODO: parameters from SDF)
  double theta = 0.5;              // theta for MoreauJeanOSI integrator

  // -----------------------------------------
  // --- Dynamical systems && interactions ---
  // -----------------------------------------

  try
  {
    // -- OneStepIntegrators --
    this->impl->osi.reset(new MoreauJeanOSI(theta));

    // -- Model --
    this->impl->model.reset(new Model(0, std::numeric_limits<double>::infinity()));

    // ------------------
    // --- Simulation ---
    // ------------------

    // -- Time discretisation --
    this->impl->maxStepSize = impl->physics->GetMaxStepSize();
    this->impl->timedisc = std11::make_shared<TimeDiscretisation>(0, this->impl->maxStepSize);

    // -- OneStepNsProblem --
    //this->osnspb.reset(new FrictionContact(3));
    SP::GenericMechanical _osnspb(
      std11::make_shared<GenericMechanical>(SICONOS_FRICTION_3D_ONECONTACT_NSN));
    this->impl->osnspb = _osnspb;

    // -- Some configuration (TODO: parameters from SDF)
    _osnspb->numericsSolverOptions()->iparam[0] = 1000; // Max number of iterations
    _osnspb->numericsSolverOptions()->dparam[0] = 1e-5; // Tolerance
    _osnspb->numericsSolverOptions()->iparam[1] = SICONOS_FRICTION_3D_NSGS_ERROR_EVALUATION_LIGHT;
    _osnspb->numericsSolverOptions()->iparam[14] = SICONOS_FRICTION_3D_NSGS_FILTER_LOCAL_SOLUTION_TRUE;
    _osnspb->numericsSolverOptions()->internalSolvers[1].solverId = SICONOS_FRICTION_3D_ONECONTACT_NSN_GP_HYBRID;
    _osnspb->numericsSolverOptions()->internalSolvers[1].iparam[0] = 100;
    _osnspb->setMaxSize(16384);                         // max number of interactions
    _osnspb->setMStorageType(1);                        // Sparse storage
    _osnspb->setNumericsVerboseMode(0);                 // 0 silent, 1 verbose
    _osnspb->setKeepLambdaAndYState(true);              // inject previous solution

    // --- Simulation initialization ---

    // -- The collision manager adds or removes interactions using
    // -- Bullet-based contact detection.  Some options are available.
    SiconosBulletOptions options;
    this->impl->manager.reset(new GazeboCollisionManager(options, this->impl));

    // -- MoreauJeanOSI Time Stepping for body mechanics
    this->impl->simulation.reset(new TimeStepping(this->impl->timedisc));

    this->impl->simulation->insertIntegrator(this->impl->osi);
    this->impl->simulation->insertNonSmoothProblem(this->impl->osnspb);
    this->impl->simulation->insertInteractionManager(this->impl->manager);

    this->impl->simulation->setNewtonOptions(SICONOS_TS_NONLINEAR);

    this->impl->simulation->setNewtonMaxIteration(
      boost::any_cast<int>(this->impl->physics->GetParam("newton_iters")));

    // TODO: parameters from SDF
    this->impl->simulation->setNewtonTolerance(1e-10);

    this->impl->model->setSimulation(this->impl->simulation);
    this->impl->model->initialize();
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
  try
  {
    if (this->impl->maxStepSize != this->impl->physics->GetMaxStepSize())
    {
      gzerr << "Changing timestep not currently supported in Siconos." << std::endl;
    }

    float time = this->impl->simulation->nextTime();

    GZ_ASSERT(this->impl->simulation->hasNextEvent(), "Simulation has no more events.");

    AddGravityToLinks();

    this->impl->simulation->computeOneStep();

    this->impl->simulation->nextStep();
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

void SiconosWorld::AddGravityToLinks()
{
  // Add gravity to all objects
  for (const auto &m : this->impl->physics->World()->Models())
  {
    const auto& g( this->impl->physics->World()->Gravity() );
    for (auto &lk : m->GetLinks()) {
      gazebo::physics::SiconosLinkPtr link(
        boost::static_pointer_cast<gazebo::physics::SiconosLink>(lk));
      if (link->GetSiconosBodyDS())
        link->AddForce(link->GetSiconosBodyDS()->scalarMass() * g);
    }
  }
}

SP::OneStepIntegrator SiconosWorld::GetOneStepIntegrator() const
{
  return this->impl->osi;
}

SP::Model SiconosWorld::GetModel() const
{
  return this->impl->model;
}

SP::SiconosCollisionManager SiconosWorld::GetManager() const
{
  return this->impl->manager;
}

SP::TimeStepping SiconosWorld::GetSimulation() const
{
  return this->impl->simulation;
}

void SiconosWorld::SetGravity(double _x, double _y, double _z)
{
    this->impl->gravity.setValue(0, _x);
    this->impl->gravity.setValue(1, _y);
    this->impl->gravity.setValue(2, _z);
}
