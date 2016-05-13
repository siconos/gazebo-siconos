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

#include <SiconosKernel.hpp>
#include <SpaceFilter.hpp>
#include <SphereNEDS.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

SiconosWorld::SiconosWorld()
    : SiconosBodies()
{
    this->gravity.resize(3);
    this->gravity.zero();
}

SiconosWorld::~SiconosWorld()
{
}

void SiconosWorld::init()
{
  try
  {
    // User-defined main parameters
    double t0 = 0;                   // initial computation time

    double T = std::numeric_limits<double>::infinity();

    double h = 0.005;                // time step
    double g = 9.81;

    double theta = 0.5;              // theta for MoreauJeanOSI integrator

    // -- OneStepIntegrators --
    SP::OneStepIntegrator osi;
    osi.reset(new MoreauJeanOSI(theta));

    // -- Model --
    _model.reset(new Model(t0, T));

    /*
     * Here the dynamic systems should be initialized and added to the
     * integrator and the model.
     */

    // ------------------
    // --- Simulation ---
    // ------------------

    SP::TimeDiscretisation timedisc_;
    SP::Simulation simulation_;
    SP::FrictionContact osnspb_;

    // User-defined main parameters
    // -- Time discretisation --
    timedisc_.reset(new TimeDiscretisation(t0, h));

    // -- OneStepNsProblem --
    osnspb_.reset(new FrictionContact(3));
    osnspb_->numericsSolverOptions()->iparam[0] = 1000;// Max number of iterations
    osnspb_->numericsSolverOptions()->iparam[1] = 20;  // compute error iterations
    osnspb_->numericsSolverOptions()->iparam[4] = 2;   // projection
    osnspb_->numericsSolverOptions()->dparam[0] = 1e-7;// Tolerance
    osnspb_->numericsSolverOptions()->dparam[2] = 1e-7;// Local tolerance

    osnspb_->setMaxSize(16384);            // max number of interactions
    osnspb_->setMStorageType(1);           // Sparse storage
    osnspb_->setNumericsVerboseMode(0);    // 0 silent, 1 verbose
    osnspb_->setKeepLambdaAndYState(true); // inject previous solution

    simulation_.reset(new TimeStepping(timedisc_));
    simulation_->insertIntegrator(osi);
    simulation_->insertNonSmoothProblem(osnspb_);

    // --- Simulation initialization ---
    SP::NonSmoothLaw nslaw(new NewtonImpactFrictionNSL(0.0, 0.0, 0.6, 3));

    _playground.reset(new SpaceFilter(3, 6, _model, _plans, _moving_plans));
    _playground->insert(nslaw, 0, 0);

    _model->initialize(simulation_);
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
    _playground->buildInteractions(_model->currentTime());
    _model->simulation()->advanceToEvent();
    _model->simulation()->processEvents();
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
}
