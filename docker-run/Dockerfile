FROM nvidia/cuda

USER root

RUN \
  apt-get update -y && apt-get install -y --force-yes \
  git-core \
  wget \
  make \
  cmake \
  libboost-all-dev \
  libgmp-dev \
  swig3.0 \
  libcppunit-dev \
  gcc \
  gfortran \
  g++ \
  libblas-dev \
  liblapack-dev \
  libatlas-base-dev \
  libatlas-dev \
  lp-solve \
  liblpsolve55-dev \
  python \
  python-dev \
  python-lxml \
  python3-lxml \
  python-pytest \
  python3-pytest \
  python-scipy \
  python3-scipy \
  python-matplotlib \
  python3-matplotlib \
  libhdf5-dev \
  wget \
  bash \
  libbullet-dev \
  python-h5py \
  libgl1-mesa-glx \
  libglu1-mesa \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  libglapi-mesa \
  libfreetype6-dev \
  doxygen \
  python-sphinx \
  libbullet-dev \
  clang gcc-4.7 emacs vim python-vtk6 python-pip python3-pip \
  python3-sphinx python3-pytest python3-dev \
  emacs cgdb \
  pkg-config libzmq3-dev ruby-ronn uuid-dev libprotobuf-dev libprotoc-dev protobuf-compiler  libopenal-dev libsimbody-dev libgdal-dev xsltproc libgraphviz-dev libfreeimage-dev libcurl4-openssl-dev libtar-dev libtbb-dev libogre-1.9-dev libqwt-qt5-dev libqt5opengl5-dev libswscale-dev libavdevice-dev libavformat-dev libavcodec-dev libavutil-dev mercurial libtinyxml2-dev apt-utils libtinyxml-dev

# libdart-dev
#  libdart-core5-dev

RUN mkdir -p /usr/local

RUN pip install h5py && pip3 install h5py

RUN gem install protobuf

# Download & compile/install required libraries

RUN cd ~ && hg clone https://bitbucket.org/ignitionrobotics/ign-math && \
  cd ~/ign-math && hg up ign-math3 && mkdir bld && cd bld && \
  cmake .. -DENABLE_TESTS_COMPILATION=NO && \
  make -j5 && make install && cd ~ && rm -rf ign-math

RUN cd ~ && hg clone https://bitbucket.org/ignitionrobotics/ign-tools && \
  cd ~/ign-tools && mkdir bld && cd bld && \
  cmake .. -DENABLE_TESTS_COMPILATION=NO && \
  make -j5 && make install && cd ~ && rm -rf ign-tools

RUN cd ~ && hg clone https://bitbucket.org/ignitionrobotics/ign-msgs && \
  cd ~/ign-msgs && mkdir bld && cd bld && \
  cmake .. -DENABLE_TESTS_COMPILATION=NO && \
  make -j5 && make install && cd ~ && rm -rf ign-msgs

RUN cd ~ && hg clone https://bitbucket.org/ignitionrobotics/ign-transport && \
  cd ~/ign-transport && mkdir bld && cd bld && \
  cmake .. -DENABLE_TESTS_COMPILATION=NO && \
  make -j5 && make install && cd ~ && rm -rf ign-transport

# configure & install sdformat
RUN cd ~ && hg clone https://bitbucket.org/osrf/sdformat && \
  cd ~/sdformat && \
  hg up sdf5 && \
  mkdir bld && cd bld && \
  cmake .. && \
  make -j5 && \
  make install && \
  cd ~ && rm -rf sdformat

# Configure & install siconos
RUN \
  cd ~ && \
  git clone http://github.com/siconos/siconos.git && \
  cd ~/siconos && \
  git checkout 4.1.x && \
  mkdir -p bld && cd bld && \
  cmake .. -DWITH_DOXY2SWIG=OFF -DDEV_MODE=OFF -DCMAKE_BUILD_TYPE=Debug -DWITH_MECHANISMS=OFF -DWITH_DOCUMENTATION=OFF -DWITH_BULLET=ON -DWITH_SERIALIZATION=OFF -DWITH_GENERATION=OFF -DWITH_TESTING=OFF && \
  make -j5 && \
  make install && \
  cd ~ && rm -rf siconos

# Download, configure & install Gazebo with Siconos support
RUN \
  cd ~ &&  \
  git clone http://github.com/siconos/gazebo-siconos gazebo && \
  cd gazebo && \
  git checkout siconos_integration && \
  cd ~/gazebo && mkdir bld && cd bld && cmake .. && make -j5 && make install && \
  cd ~ && rm -rf gazebo
