
# Try with Docker

To try this new without compiling it yourself, using a pre-built
Docker image.

## Pre-requisites

Since Gazebo is a graphical application, using the Docker image
requires [nvidia-docker](https://github.com/NVIDIA/nvidia-docker).

This therefore only works for Linux-based operating systems.  For Mac
or Windows, you must build Siconos and Gazebo on your local system
outside of a container, which requires some more work; instructions
will be added here in the future.

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
up so as to save space, so the final image should be around 5 GB.

Removing the `rm` commands allows to use this environment for
development.

## Docker run command

TODO, update the command to indicate the name on Docker Hub once the
image has been uploaded there.

This is the basic command, in all its glory, needed to get a prompt
allowing to test Gazebo:

    nvidia-docker run --rm -it --net=host --privileged
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
        -v ~/.Xauthority:/root/.Xauthority --env=DISPLAY
        -v /dev/log:/dev/log --device /dev/dri
        siconos-gazebo /bin/bash

This assumes the image is labeled `siconos-gazebo`.
You may need to add a volume with `-v` in order to make your world
file accessible.

It is recommended to test that X11 and OpenGL are correctly made
available by running the following commands and verifying that windows
open and you see some 3D gears.

    # apt-get update -y && apt-get install -y xterm mesa-utils
    # xterm    # should open a new console window, tests X11 without GLX
    # glxgears # should open a window with spinning gears using OpenGL

## Testing

    # gazebo -e siconos -u <filename>.world

TODO, add some basic instructions for testing the Docker image.
