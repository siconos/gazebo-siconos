FROM nvidia/cuda

# Set up running environment for Siconos-enabled Gazebo.  Not a dev
# environment!  We remove all dev libraries and delete compiled
# sources and object files to make the image smaller.

# We are based on nvidia/cuda so that containerized OpenGL works
# through nvidia-docker.

USER root

# Optional stuff we are not including:
#
# libdart-dev
# libdart-core5-dev
# libopenal-dev
# libswscale-ffmpeg3 libavdevice-ffmpeg56 libavformat-ffmpeg56 libavcodec-ffmpeg56 libavutil-ffmpeg54
# libswscale-dev libavdevice-dev libavformat-dev libavcodec-dev libavutil-dev

# First install packages needed for running, to be a common layer
# between rebuilds
RUN \
  apt-get update -y && apt-get install -y \
    lp-solve libhdf5-10 libbulletcollision2.83.6 \
    libbulletdynamics2.83.6 libbulletfileloader2.83.6 \
    liblinearmath2.83.6 libgl1-mesa-glx libglu1-mesa libglapi-mesa \
    libzmq5 libuuid1 libprotobuf9v5 libprotobuf-lite9v5 libprotobuf-c1 \
    libprotoc9v5 libsimbody3.5v5 libgdal1i libcgraph6 libfreeimage3 \
    libcurl3-gnutls libtar libtbb2 libogre-1.9.0v5 libqwt-qt5-6 \
    libqt5opengl5 libtinyxml2-2v5 libtinyxml2.6.2v5 libopenblas-base \
    libblas3 liblapacke libgfortran3 libgmp10 libisl15 libmpc3 \
    libmpfr4 zlib1g libtmglib3 libquadmath0 mesa-utils xterm \
    libboost-filesystem1.58.0 libboost-iostreams1.58.0 \
    libboost-regex1.58.0 libboost-program-options1.58.0

# Now install packages needed for building:
# build, install, and uninstall them all in one step.
RUN \
  apt-get update -y && apt-get install -y \
    git-core wget make cmake libboost-all-dev libgmp-dev swig3.0 \
    libcppunit-dev gcc gfortran g++ libblas-dev liblapack-dev \
    libatlas-base-dev libatlas-dev lp-solve liblpsolve55-dev \
    libhdf5-dev wget libbullet-dev libglu1-mesa libgl1-mesa-dev \
    libglu1-mesa-dev libfreetype6-dev libbullet-dev pkg-config \
    libzmq3-dev uuid-dev libprotobuf-dev libprotoc-dev \
    protobuf-compiler libsimbody-dev libgdal-dev libfreeimage-dev \
    libcurl4-gnutls-dev libtar-dev libtbb-dev libogre-1.9-dev \
    libqwt-qt5-dev libqt5opengl5-dev mercurial libtinyxml2-dev \
    libtinyxml-dev ruby && \
  mkdir -p /usr/local && \
  cd ~ && \
  git clone http://github.com/siconos/siconos.git && \
  cd ~/siconos && \
  git checkout 4.1.x && \
  mkdir -p bld && cd bld && \
  cmake .. -DWITH_DOXY2SWIG=OFF -DDEV_MODE=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DWITH_MECHANISMS=OFF -DWITH_DOCUMENTATION=OFF -DWITH_BULLET=ON -DWITH_SERIALIZATION=OFF -DWITH_GENERATION=OFF -DWITH_TESTING=OFF -DWITH_PYTHON_WRAPPER=OFF -DCOMPONENTS='externals;numerics;kernel;mechanics' && \
  make -j5 && \
  make install && \
  cd ~ && rm -rf siconos && \
  gem install protobuf && \
  cd ~ && hg clone https://bitbucket.org/ignitionrobotics/ign-math && \
  cd ~/ign-math && hg up ign-math3 && mkdir bld && cd bld && \
  cmake .. -DENABLE_TESTS_COMPILATION=NO -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr && \
  make -j5 && make install && cd ~ && rm -rf ign-math && \
  cd ~ && hg clone https://bitbucket.org/ignitionrobotics/ign-tools && \
  cd ~/ign-tools && mkdir bld && cd bld && \
  cmake .. -DENABLE_TESTS_COMPILATION=NO -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr && \
  make -j5 && make install && cd ~ && rm -rf ign-tools && \
  cd ~ && hg clone https://bitbucket.org/ignitionrobotics/ign-msgs && \
  cd ~/ign-msgs && mkdir bld && cd bld && \
  cmake .. -DENABLE_TESTS_COMPILATION=NO -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr && \
  make -j5 && make install && cd ~ && rm -rf ign-msgs && \
  cd ~ && hg clone https://bitbucket.org/ignitionrobotics/ign-transport && \
  cd ~/ign-transport && mkdir bld && cd bld && \
  cmake .. -DENABLE_TESTS_COMPILATION=NO -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr && \
  make -j5 && make install && cd ~ && rm -rf ign-transport && \
  cd ~ && hg clone https://bitbucket.org/osrf/sdformat && \
  cd ~/sdformat && \
  hg up sdf5 && \
  mkdir bld && cd bld && \
  cmake .. && \
  make -j5 && \
  make install && \
  cd ~ && rm -rf sdformat && \
  cd ~ &&  \
  git clone http://github.com/siconos/gazebo-siconos gazebo && \
  cd gazebo && \
  git checkout siconos_integration && \
  cd ~/gazebo && mkdir bld && cd bld && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr && make -j5 && make install && \
  cd ~ && rm -rf gazebo && \
  apt-get remove -y *-dev && \
  apt-get remove -y git-core mercurial wget make cmake clang gcc g++ gfortran protobuf-compiler pkg-config swig3.0 ruby && \
  apt-get autoremove -y && \
  apt-get clean -y && \
  rm -rf ~/.gem
