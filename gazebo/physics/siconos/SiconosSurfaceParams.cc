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

#include "gazebo/common/Console.hh"
#include "gazebo/physics/siconos/SiconosSurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosSurfaceParams::SiconosSurfaceParams()
  : SurfaceParams()
  , frictionPyramid(new physics::FrictionPyramid())
  , normal_restitution(1.0)
{
}

//////////////////////////////////////////////////
SiconosSurfaceParams::~SiconosSurfaceParams()
{
}

//////////////////////////////////////////////////
void SiconosSurfaceParams::Load(sdf::ElementPtr _sdf)
{
  // Load parent class
  SurfaceParams::Load(_sdf);

  if (!_sdf)
    gzerr << "Surface _sdf is null" << std::endl;
  else
  {
    {
      sdf::ElementPtr bounceElem = _sdf->GetElement("bounce");
      if (!bounceElem)
        gzerr << "Surface bounce sdf member is null" << std::endl;
      else
      {
        // Note that the default restitution_coefficient is zero if
        // not specified in the SDF.
        this->normal_restitution = bounceElem->Get<double>("restitution_coefficient");
        if (this->normal_restitution < 0)
        {
          gzwarn << "bounce restitution_coefficient ["
                 << this->normal_restitution
                 << "] < 0, so it will not be applied."
                 << std::endl;
        }
        else if (this->normal_restitution > 1)
        {
          gzwarn << "bounce restitution_coefficient ["
                 << this->normal_restitution
                 << "] > 1, which is outside the recommended range."
                 << std::endl;
        }

        // Not supported by Siconos
        // this->bounceThreshold = bounceElem->Get<double>("threshold");

        // No SDF representation of Siconos' "tangent restitution"
      }
    }

    {
      sdf::ElementPtr frictionElem = _sdf->GetElement("friction");
      if (!frictionElem)
        gzerr << "Surface friction sdf member is null" << std::endl;
      else
      {
        sdf::ElementPtr torsionalElem = frictionElem->GetElement("torsional");
        if (torsionalElem)
        {
          this->frictionPyramid->SetMuTorsion(
            torsionalElem->Get<double>("coefficient"));
          this->frictionPyramid->SetPatchRadius(
            torsionalElem->Get<double>("patch_radius"));
          this->frictionPyramid->SetSurfaceRadius(
            torsionalElem->Get<double>("surface_radius"));
          this->frictionPyramid->SetUsePatchRadius(
            torsionalElem->Get<bool>("use_patch_radius"));

          // Not supported by Siconos
          // sdf::ElementPtr torsionalOdeElem = torsionalElem->GetElement("ode");
          // if (torsionalOdeElem)
          //   this->slipTorsion = torsionalOdeElem->Get<double>("slip");
        }

        // Should not be looking in the "ode" block.
        // Update this when sdformat has siconos friction parameters.
        // See sdformat issue #31: https://bitbucket.org/osrf/sdformat/issue/31
        sdf::ElementPtr frictionOdeElem = frictionElem->GetElement("ode");
        if (!frictionOdeElem)
          gzerr << "Surface friction ode sdf member is null" << std::endl;
        else
        {
          this->frictionPyramid->SetMuPrimary(
            frictionOdeElem->Get<double>("mu"));
          this->frictionPyramid->SetMuSecondary(
            frictionOdeElem->Get<double>("mu2"));
          this->frictionPyramid->direction1 =
            frictionOdeElem->Get<ignition::math::Vector3d>("fdir1");

          // Not supported by Siconos
          // this->slip1 = frictionOdeElem->Get<double>("slip1");
          // this->slip2 = frictionOdeElem->Get<double>("slip2");
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void SiconosSurfaceParams::FillMsg(msgs::Surface &_msg)
{
  SurfaceParams::FillMsg(_msg);

  _msg.mutable_friction()->set_mu(this->frictionPyramid->MuPrimary());
  _msg.mutable_friction()->set_mu2(this->frictionPyramid->MuSecondary());
}

/////////////////////////////////////////////////
void SiconosSurfaceParams::ProcessMsg(const msgs::Surface &_msg)
{
  SurfaceParams::ProcessMsg(_msg);

  if (_msg.has_friction())
  {
    if (_msg.friction().has_mu())
      this->frictionPyramid->SetMuPrimary(_msg.friction().mu());
    if (_msg.friction().has_mu2())
      this->frictionPyramid->SetMuSecondary(_msg.friction().mu2());
  }

  // Collision group later updated by SiconosLink::UpdateSurface,
  // called by SiconosPhysics upstream caller of ProcessMsg.
}

/////////////////////////////////////////////////
FrictionPyramidPtr SiconosSurfaceParams::FrictionPyramid() const
{
  return this->frictionPyramid;
}

/////////////////////////////////////////////////
SiconosSurfaceParamsPtr SiconosSurfaceParams::Copy() const
{
  SiconosSurfaceParamsPtr copy = boost::make_shared<SiconosSurfaceParams>(*this);
  if (copy->frictionPyramid)
    copy->frictionPyramid
      = boost::make_shared<physics::FrictionPyramid>(*frictionPyramid);
  return copy;
}
