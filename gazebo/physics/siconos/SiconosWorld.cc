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
#include <NewtonImpactNSL.hpp>
#include <GenericMechanical.hpp>
#include <BulletR.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/ContactManager.hh"

#include "gazebo/physics/siconos/SiconosTypes.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosSurfaceParams.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosCollision.hh"

DEFINE_SPTR(GazeboCollisionManager)

struct SiconosWorldImpl
{
  /// \brief Gravity vector
  ignition::math::Vector3<double> gravity;

  /// \brief The Siconos OneStepIntegrator for this simulation
  SP::OneStepIntegrator osi;

  /// \brief The Siconos TimeDiscretisation for this simulation
  SP::TimeDiscretisation timedisc;

  /// \brief The Siconos NonSmoothDynamicalSystem for this simulation
  SP::NonSmoothDynamicalSystem nsds;

  /// \brief The Siconos OneStepNonSmoothProblem for this
  ///        simulation's constraints (velocity level)
  SP::OneStepNSProblem osnspb;

  /// \brief The Siconos OneStepNonSmoothProblem for this
  ///        simulation's constraints (position level)
  SP::MLCPProjectOnConstraints osnspb_pos;

  /// \brief The Siconos broadphase collision manager
  SP::GazeboCollisionManager manager;

  /// \brief The Siconos TimeStepping handler for this simulation
  SP::TimeStepping simulation;

  /// \brief Gazebo SiconosPhysics physics engine
  gazebo::physics::SiconosPhysics *physics;

  /// \brief Store timestep so we can check if it changed
  double maxStepSize;

  // \brief The NSL used for joint stops.
  SP::NewtonImpactNSL jointStopNSL;
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
    this->impl->gravity.Set(0,0,0);
    this->impl->physics = physics;
    this->impl->maxStepSize = 0;
    this->impl->jointStopNSL = std11::make_shared<NewtonImpactNSL>();
}

SiconosWorld::~SiconosWorld()
{
}

void SiconosWorld::setup()
{
  // User-defined main parameters (TODO: parameters from SDF)
  double theta = 1.0;              // theta for MoreauJeanOSI integrator

  int Newton_max_iter=1;
  int Newton_tolerance=1e-10;
  bool Newton_update_interactions=false;
  bool Newton_warn_nonconverge=false;
  int itermax=100;
  double tolerance=1e-4;
  int error_evaluation=SICONOS_FRICTION_3D_NSGS_ERROR_EVALUATION_LIGHT;
  int solver_filter=SICONOS_FRICTION_3D_NSGS_FILTER_LOCAL_SOLUTION_TRUE;
  int projection_itermax=3;
  double projection_tolerance=1e-4;
  double projection_tolerance_unilateral=1e-4;
  bool numerics_verbose=false;
  int internal_solverId=SICONOS_FRICTION_3D_ONECONTACT_NSN_GP_HYBRID;
  int internal_iterations=10;
  int max_interactions=16384;
  std::string osiType="MoreauJeanOSI";
  int keep_lambda=true;

  auto stobool = [](std::string x)->long int{if (x=="true") return 1;
                                             if (x=="false") return 0;
                                             return std::stol(x);};

  std::string pair, key, value;
  std::istringstream ss(this->config);
  while (std::getline(ss, pair, ';')) {
    std::istringstream ps(pair);
    std::getline(ps, key, '=');
    ps >> value;
    if (key=="theta")
    {
      theta = std::stod(value);
    }
    else if (key=="Newton_max_iter")
    {
      Newton_max_iter = std::stol(value);
    }
    else if (key=="Newton_tolerance")
    {
      Newton_tolerance = std::stod(value);
    }
    else if (key=="Newton_update_interactions")
    {
      Newton_update_interactions = std::stol(value);
    }
    else if (key=="Newton_warn_nonconverge")
    {
      Newton_warn_nonconverge = stobool(value);
    }
    else if (key=="itermax")
    {
      itermax = std::stol(value);
    }
    else if (key=="tolerance")
    {
      tolerance = std::stod(value);
    }
    else if (key=="error_evaluation")
    {
      if (value=="full")
        error_evaluation = SICONOS_FRICTION_3D_NSGS_ERROR_EVALUATION_FULL;
      else if (value=="light-full-final")
        error_evaluation = SICONOS_FRICTION_3D_NSGS_ERROR_EVALUATION_LIGHT_WITH_FULL_FINAL;
      else if (value=="light")
        error_evaluation = SICONOS_FRICTION_3D_NSGS_ERROR_EVALUATION_LIGHT;
      else if (value=="adaptive")
        error_evaluation = SICONOS_FRICTION_3D_NSGS_ERROR_EVALUATION_ADAPTIVE;
    }
    else if (key=="solver_filter")
    {
        if (stobool(value))
          solver_filter = SICONOS_FRICTION_3D_NSGS_FILTER_LOCAL_SOLUTION_TRUE;
        else
          solver_filter = SICONOS_FRICTION_3D_NSGS_FILTER_LOCAL_SOLUTION_FALSE;
    }
    else if (key=="projection_itermax")
    {
      projection_itermax = std::stol(value);
    }
    else if (key=="projection_tolerance")
    {
      projection_tolerance = std::stod(value);
    }
    else if (key=="projection_tolerance_unilateral")
    {
      projection_tolerance_unilateral = std::stod(value);
    }
    else if (key=="numerics_verbose")
    {
      numerics_verbose = stobool(value);
    }
    else if (key=="internal_solverId")
    {
      if (value=="nsn")
        internal_solverId = SICONOS_FRICTION_3D_ONECONTACT_NSN;
      else if (value=="nsn-gp")
        internal_solverId = SICONOS_FRICTION_3D_ONECONTACT_NSN_GP;
      else if (value=="nsn-gp-hybrid")
        internal_solverId = SICONOS_FRICTION_3D_ONECONTACT_NSN_GP_HYBRID;
    }
    else if (key=="internal_iterations")
    {
      internal_iterations = std::stol(value);
    }
    else if (key=="max_interactions")
    {
      max_interactions = std::stol(value);
    }
    else if (key=="osiType")
    {
      osiType = value;
    }
    else if (key=="keep_lambda")
    {
      keep_lambda = stobool(value);
    }
  }

  // -----------------------------------------
  // --- Dynamical systems && interactions ---
  // -----------------------------------------

  try
  {
    // -- OneStepIntegrators --
    if (osiType=="MoreauJeanOSI")
      this->impl->osi.reset(new MoreauJeanOSI(theta));
    else if (osiType=="MoreauJeanDirectProjectionOSI")
      this->impl->osi.reset(new MoreauJeanDirectProjectionOSI(theta));
    else
      assert(false);

    // -- Non-Smooth Dynamical System --
    this->impl->nsds = std11::make_shared<NonSmoothDynamicalSystem>(
      0, std::numeric_limits<double>::infinity());

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
    _osnspb->numericsSolverOptions()->iparam[0] = itermax; // Max number of iterations
    _osnspb->numericsSolverOptions()->dparam[0] = tolerance; // Tolerance
    _osnspb->numericsSolverOptions()->iparam[1] = error_evaluation;
    _osnspb->numericsSolverOptions()->iparam[14] = solver_filter;
    _osnspb->numericsSolverOptions()->internalSolvers[1].solverId = internal_solverId;
    _osnspb->numericsSolverOptions()->internalSolvers[1].iparam[0] = internal_iterations;
    _osnspb->setMaxSize(max_interactions);             // max number of interactions
    _osnspb->setMStorageType(1);                       // Sparse storage
    _osnspb->setNumericsVerboseMode(numerics_verbose); // 0 silent, 1 verbose
    _osnspb->setKeepLambdaAndYState(keep_lambda);      // inject previous solution

    // --- Simulation initialization ---

    // -- The collision manager adds or removes interactions using
    // -- Bullet-based contact detection.  Some options are available.
    SiconosBulletOptions options;
    this->impl->manager.reset(new GazeboCollisionManager(options, this->impl));

    if (osiType=="MoreauJeanOSI")
    {
      // -- MoreauJeanOSI Time Stepping for body mechanics
      this->impl->simulation = std11::make_shared<TimeStepping>(this->impl->nsds,
                                                                this->impl->timedisc);

      this->impl->simulation->insertIntegrator(this->impl->osi);
      this->impl->simulation->insertNonSmoothProblem(this->impl->osnspb);
    }
    else if (osiType=="MoreauJeanDirectProjectionOSI")
    {
      SP::MLCPProjectOnConstraints _osnspb_pos = this->impl->osnspb_pos =
        std11::make_shared<MLCPProjectOnConstraints>(SICONOS_MLCP_ENUM, 1.0);

      _osnspb_pos->numericsSolverOptions()->iparam[0]=itermax;
      _osnspb_pos->numericsSolverOptions()->dparam[0]=tolerance;
      _osnspb_pos->setMaxSize(max_interactions);
      _osnspb_pos->setMStorageType(0); // "not yet implemented for sparse storage"
      _osnspb_pos->setNumericsVerboseMode(numerics_verbose);
      _osnspb_pos->setKeepLambdaAndYState(keep_lambda);

      // -- MoreauJeanDirectProjectionOSI Time Stepping for body mechanics
      SP::TimeSteppingDirectProjection simulation;
      this->impl->simulation = simulation =
        std11::make_shared<TimeSteppingDirectProjection>(
          this->impl->nsds, this->impl->timedisc, this->impl->osi,
          _osnspb, _osnspb_pos);

      simulation->setProjectionMaxIteration(projection_itermax);
      simulation->setConstraintTolUnilateral(projection_tolerance_unilateral);
      simulation->setConstraintTol(projection_tolerance);
    }

    this->impl->simulation->insertInteractionManager(this->impl->manager);

    this->impl->simulation->setNewtonOptions(SICONOS_TS_NONLINEAR);
    this->impl->simulation->setNewtonMaxIteration(Newton_max_iter);
    this->impl->simulation->setNewtonTolerance(Newton_tolerance);

    // Quiet the Newton loop
    this->impl->simulation->setWarnOnNonConvergence(Newton_warn_nonconverge);
    if (!Newton_warn_nonconverge) {
      this->impl->simulation->setCheckSolverFunction([](int, Simulation*){});
    }
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

    UpdateContactInformation();

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
    const auto& g( this->impl->gravity );
    for (auto &lk : m->GetLinks()) {
      gazebo::physics::SiconosLinkPtr link(
        boost::static_pointer_cast<gazebo::physics::SiconosLink>(lk));
      if (link->GetSiconosBodyDS() && link->GetGravityMode())
        link->AddForce(link->GetSiconosBodyDS()->scalarMass() * g);
    }
  }
}

SP::OneStepIntegrator SiconosWorld::GetOneStepIntegrator() const
{
  return this->impl->osi;
}

SP::SiconosCollisionManager SiconosWorld::GetCollisionManager() const
{
  return this->impl->manager;
}

SP::NonSmoothDynamicalSystem SiconosWorld::GetNonSmoothDynamicalSystem() const
{
  return this->impl->nsds;
}

SP::TimeStepping SiconosWorld::GetSimulation() const
{
  return this->impl->simulation;
}

void SiconosWorld::SetGravity(const ignition::math::Vector3<double> &gravity)
{
  this->impl->gravity = gravity;
}

SP::NewtonImpactNSL SiconosWorld::JointStopNSL() const
{
  return this->impl->jointStopNSL;
}

// This is needed for the unordered_map with a pair key used in
// UpdateContactInformation.
namespace std {
template <typename T, typename U>
struct hash<pair<T,U> > {
  size_t operator()(pair<T,U> x) const
    { return hash<T>()(x.first) ^ hash<U>()(x.second); }
};
}

/// Update contact information for ContactManager by traversing the
/// indexSet(1) (set of active Interactions) and finding all
/// GazeboBulletR relations.
void SiconosWorld::UpdateContactInformation()
{
  using namespace gazebo::physics;

  std::unordered_map< std::pair< SiconosCollision*, SiconosCollision* >, Contact* >
    contactMap;

  SP::InteractionsGraph indexSet = this->impl->simulation->indexSet(1);
  InteractionsGraph::VIterator vi, viend;
  for (std11::tie(vi, viend) = indexSet->vertices();
       vi != viend; ++vi)
  {
    SP::Interaction inter = indexSet->bundle(*vi);
    SP::BodyDS ds1 = std11::dynamic_pointer_cast<BodyDS>(indexSet->properties(*vi).source);
    SP::BodyDS ds2 = std11::dynamic_pointer_cast<BodyDS>(indexSet->properties(*vi).target);

    SP::BulletR rel = std11::dynamic_pointer_cast<BulletR>(inter->relation());
    if (!(rel && ds1 && ds2)) continue;

    // Some reverse lookups needed to create a contact.
    SiconosCollisionPtr collisionPtr1, collisionPtr2;
    if (rel->shape[0])
      collisionPtr1 = SiconosCollision::CollisionForShape(rel->shape[0]);
    if (rel->shape[1])
      collisionPtr2 = SiconosCollision::CollisionForShape(rel->shape[1]);

    Contact *contact = nullptr;
    auto p = std::make_pair(collisionPtr1.get(), collisionPtr2.get());
    if (contactMap.find(p) != contactMap.end())
      contact = contactMap[p];
    else
    {
      // Add a new contact to the manager. This will return nullptr if no one is
      // listening for contact information.
      contact = impl->physics->GetContactManager()->NewContact(
        collisionPtr1.get(), collisionPtr2.get(),
        collisionPtr1->GetWorld()->SimTime());

      // Exit early if NewContact returns null, since no one is listening.
      if (!contact)
        return;
    }

    if (contact->count >= MAX_CONTACT_JOINTS)
      contact = impl->physics->GetContactManager()->NewContact(
        collisionPtr1.get(), collisionPtr2.get(),
        collisionPtr1->GetWorld()->SimTime());


    if (contact && contact->count < MAX_CONTACT_JOINTS)
    {
      // Reuse this contact for same pair
      contactMap[p] = contact;

      // Fill out contact information.
      unsigned int c = contact->count++;

      // Report middle between the two contact points
      contact->positions[c] = (SiconosTypes::ConvertVector3(rel->pc1())
                               + SiconosTypes::ConvertVector3(rel->pc2()))/2;
      contact->normals[c] = SiconosTypes::ConvertVector3(rel->nc());
      contact->depths[c] = rel->distance();

      // Lambda is the output of the interaction = force of the contact.
      // It is in body1 frame, so rotate it into the world frame.
      auto lambda = SiconosTypes::ConvertVector3(inter->lambda(1));
      ignition::math::Pose3d pose( SiconosTypes::ConvertPose(*ds1->q()) );
      lambda = pose.Rot().RotateVectorReverse(lambda);
      contact->wrench[c].body1Force = lambda;
      contact->wrench[c].body2Force = -lambda;
    }
  }
}
