include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
  siconos/SiconosBallJoint.cc
  siconos/SiconosCollision.cc
  siconos/SiconosFixedJoint.cc
  # SiconosHeightmapShape.cc
  # SiconosHinge2Joint.cc
  siconos/SiconosHingeJoint.cc
  siconos/SiconosJoint.cc
  siconos/SiconosLink.cc
  # SiconosMesh.cc
  siconos/SiconosMeshShape.cc
  # SiconosMotionState.cc
  # SiconosMultiRayShape.cc
  siconos/SiconosPhysics.cc
  # SiconosPolylineShape.cc
  siconos/SiconosRayShape.cc
  # SiconosScrewJoint.cc
  siconos/SiconosSliderJoint.cc
  siconos/SiconosSurfaceParams.cc
  # SiconosUniversalJoint.cc
  # gzBtUniversalConstraint.cc
  siconos/SiconosWorld.cc
  PARENT_SCOPE
)

set (headers
  siconos_inc.h
  # siconos_math_inc.h
  SiconosBallJoint.hh
  SiconosBoxShape.hh
  SiconosCollision.hh
  SiconosCylinderShape.hh
  SiconosFixedJoint.hh
  # SiconosHeightmapShape.hh
  # SiconosHinge2Joint.hh
  SiconosHingeJoint.hh
  SiconosJoint.hh
  SiconosLink.hh
  # SiconosMesh.hh
  SiconosMeshShape.hh
  # SiconosMotionState.hh
  # SiconosMultiRayShape.hh
  SiconosPhysics.hh
  SiconosPlaneShape.hh
  # SiconosPolylineShape.hh
  SiconosRayShape.hh
  # SiconosScrewJoint.hh
  SiconosSliderJoint.hh
  SiconosSphereShape.hh
  # SiconosSurfaceParams.hh
  SiconosTypes.hh
  # SiconosUniversalJoint.hh
  # gzBtUniversalConstraint.hh
  SiconosWorld.hh
)

# set (gtest_fixture_sources
#   SiconosPhysics_TEST.cc
# )
# gz_build_tests(${gtest_fixture_sources}
#   EXTRA_LIBS gazebo_physics gazebo_test_fixture)

gz_install_includes("physics/siconos" ${headers})
