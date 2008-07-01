/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Trimesh geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#ifndef TRIMESHGEOM_HH
#define TRIMESHGEOM_HH

#include "Geom.hh"

namespace gazebo
{
  class OgreVisual;

  /// \addtogroup gazebo_physics_geom
  /// \{
  /** \defgroup gazebo_trimesh_geom Triangle Mesh geom
      \brief Trimesh geom

    \par Attributes
    The following attributes are supported.

    \htmlinclude default_geom_attr_include.html

    - scale (float tuple, meters)
      - Scale of the trimesh
      - Default: 1 1 1

    \par Example
    \verbatim
      <geom:trimesh name="pallet_geom">
        <mesh>WoodPallet.mesh</mesh>
        <scale>.2 .2 .2</scale>
        <mass>0.1</mass>

        <visual>
          <scale>.2 .2 .2</scale>
          <material>Gazebo/WoodPallet</material>
          <mesh>WoodPallet.mesh</mesh>
        </visual>
      </geom:trimesh>
    \endverbatim
  */
  /// \}
  /// \addtogroup gazebo_trimesh_geom 
  /// \{


  /// \brief Triangle mesh geom
  class TrimeshGeom : public Geom
  {
    /// \brief Constructor
    public: TrimeshGeom(Body *body);

    /// \brief Destructor
    public: virtual ~TrimeshGeom();

    /// \brief Update function 
    public: void UpdateChild();

    /// \brief Load the trimesh
    protected: virtual void LoadChild(XMLConfigNode *node);

    private: dTriMeshDataID odeData;

    private: dReal matrix_dblbuff[16*2];
    private: int last_matrix_index;

    ///  name of the mesh
    private: std::string meshName;
  };

  /// \}
}

#endif
