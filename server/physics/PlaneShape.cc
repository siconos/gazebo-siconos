/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "Geom.hh"
#include "PlaneShape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
PlaneShape::PlaneShape(Geom *parent) : Shape(parent)
{
  this->AddType(PLANE_SHAPE);
  this->SetName("plane_shape");

  Param::Begin(&this->parameters);
  this->normalP = new ParamT<Vector3>("normal",Vector3(0,0,1),0);
  this->normalP->Callback( &PlaneShape::SetNormal, this );
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
PlaneShape::~PlaneShape()
{
  delete this->normalP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the plane
void PlaneShape::Load(XMLConfigNode *node)
{
  this->normalP->Load(node->GetChild("plane"));
  this->CreatePlane();
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void PlaneShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->normalP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Create the plane
void PlaneShape::CreatePlane()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the altitude of the plane
void PlaneShape::SetAltitude(const Vector3 &pos) 
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the normal
void PlaneShape::SetNormal( const Vector3 &norm )
{
  this->normalP->SetValue(norm);
  this->CreatePlane();
}
