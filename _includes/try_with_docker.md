
# Try with Docker

To try this experimental Gazebo without compiling it yourself, you may
run a pre-built Docker image.

## Pre-requisites

Since Gazebo is a graphical application, we piggy-back off the Nvidia
effort to make Docker work with graphics cards. Therefore using this
Docker image requires [nvidia-docker](https://github.com/NVIDIA/nvidia-docker),
and only works for Linux-based operating systems with
Nvidia graphics cards.

For Mac or Windows, you must build Siconos and Gazebo on your local
system outside of a container, which requires some more work;
instructions will be added here in the future, or you may adapt
from the [Dockerfile](https://github.com/siconos/gazebo-siconos/blob/gh-pages/docker-run/Dockerfile).

## Docker run command

This is the basic command, in all its glory, needed to get a prompt
allowing to test Gazebo:

    nvidia-docker run --rm -it --net=host --privileged
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
        -v ~/.Xauthority:/root/.Xauthority --env=DISPLAY
        siconos/gazebo /bin/bash

You may need to customize it, e.g., to add a volume with `-v` in order
to make your world file accessible.

It is recommended to test that X11 and OpenGL are correctly made
available by running the following commands within the container's
bash prompt:

    # apt-get update -y && apt-get install -y xterm mesa-utils
    # xterm    # should open a new console window, tests X11 without GLX
    # glxgears # should open a window with spinning gears using OpenGL

If these tests are successful, you can run Gazebo as detailed in the
next section.

## Testing

To run Gazebo with Siconos, you must specify the physics engine with
`-e`.  It is also recommended to start with physics paused, by
specifying `-u`:

    # gazebo -e siconos -u <filename>.world

You can also just replace `/bin/bash` in the `nvidia-docker` command
above with `gazebo -e siconos -u`.

Providing a world file is optional, otherwise you can click on
"Insert" in the top-left and insert any Gazebo models available in the
online repository.  Note that as this is a project in development, not
all models work yet.

One that does work is "Simple Arm", which we recommend testing first,
as it demonstrates a simple mechanism with two joint types.

You may also compare against other physics backends by changing
`siconos` for `ode`, `bullet`, or `simbody`.  Currently the `dart`
engine is not included in this build.

## Build the Docker image yourself

This is optional.  If you wish to build it yourself instead of relying
on the uploaded pre-built image, you may do so using a Dockerfile and
the `docker build` command.

* [View the Dockerfile](https://github.com/siconos/gazebo-siconos/blob/gh-pages/docker-run/Dockerfile)

* [Download the Dockerfile]({{site.baseurl}}/docker-run/Dockerfile)

Once downloaded, build the image using the command,

    # cd <path containing Dockerfile>
    # docker build -t gazebo-siconos .

It may take an hour or two, as it installs several Ubuntu "dev"
packages, downloads and builds Gazebo and Siconos pre-requisites, then
builds Siconos and Gazebo.  You should have around 10 GB of hard drive
space available.  This image erases the source files before cleaning
up so as to save space, so the final image should be around 2 GB.

Removing the `rm` and other clean-up commands allows to use this
environment for development, should you wish to contribute.

