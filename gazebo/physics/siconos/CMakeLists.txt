include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(${SICONOS_INCLUDE_DIRS})

link_directories(${SICONOS_LIBRARY_DIRS})

add_definitions(${SICONOS_CFLAGS})

set (sources
  # SiconosBallJoint.cc
  SiconosCollision.cc
  SiconosFixedJoint.cc
  # SiconosHeightmapShape.cc
  # SiconosHinge2Joint.cc
  SiconosHingeJoint.cc
  SiconosJoint.cc
  SiconosLink.cc
  # SiconosMesh.cc
  # SiconosMeshShape.cc
  # SiconosMotionState.cc
  # SiconosMultiRayShape.cc
  SiconosPhysics.cc
  # SiconosPolylineShape.cc
  # SiconosRayShape.cc
  # SiconosScrewJoint.cc
  # SiconosSliderJoint.cc
  SiconosSurfaceParams.cc
  # SiconosUniversalJoint.cc
  # gzBtUniversalConstraint.cc
)

set (headers
  siconos_inc.h
  # siconos_math_inc.h
  # SiconosBallJoint.hh
  # SiconosBoxShape.hh
  # SiconosCollision.hh
  # SiconosCylinderShape.hh
  SiconosFixedJoint.hh
  # SiconosHeightmapShape.hh
  # SiconosHinge2Joint.hh
  SiconosHingeJoint.hh
  SiconosJoint.hh
  SiconosLink.hh
  # SiconosMesh.hh
  # SiconosMeshShape.hh
  # SiconosMotionState.hh
  # SiconosMultiRayShape.hh
  SiconosPhysics.hh
  SiconosPlaneShape.hh
  # SiconosPolylineShape.hh
  # SiconosRayShape.hh
  # SiconosScrewJoint.hh
  # SiconosSliderJoint.hh
  SiconosSphereShape.hh
  # SiconosSurfaceParams.hh
  SiconosTypes.hh
  # SiconosUniversalJoint.hh
  # gzBtUniversalConstraint.hh
)

# set (gtest_sources
#   SiconosTypes_TEST.cc
#   SiconosPhysics_TEST.cc
# )
# gz_build_tests(${gtest_sources})

gz_add_library(gazebo_physics_siconos ${sources})
target_link_libraries(gazebo_physics_siconos ${siconos_LIBRARIES})
target_include_directories(gazebo_physics_siconos PRIVATE ${siconos_INCLUDE_DIRECTORIES})

gz_install_library(gazebo_physics_siconos)
gz_install_includes("physics/siconos" ${headers})
