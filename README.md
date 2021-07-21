# Modified libfranka for Research

[![Build Status][travis-status]][travis]
[![codecov][codecov-status]][codecov]

With this library, you can control research versions of Franka Emika robots. See the [Franka Control Interface (FCI) documentation][fci-docs] for more information about what `libfranka` can do and how to set it up. The [generated API documentation][api-docs] also gives an overview of its capabilities.

## License

`libfranka` is licensed under the [Apache 2.0 license][apache-2.0].

This modified version of libfranka contains no change to most of the files in `libfranka` provided by the Franka Emika GmbH repo, see [libfranka]. This repo should only be used for research purposes. Modifications mostly seen in the [examples] folder.

## Build

1. Uninstall existing installations.

`sudo apt remove "*libfranka*"`

2. Install the following dependencies.

`sudo apt install build-essential cmake git libpoco-dev libeigen3-dev`

3. Clone the source code.

`git clone https://github.com/VinceGuan/Franka_Teleop.git`

`cd Franka_Teleop`

4. Use CMake.

`mkdir build`

`cd build`

`cmake -DCMAKE_BUILD_TYPE=Release ..`

`cmake --build .`

## Run

In the examples directory in the build folder, you can execute the example applications. `<fci-ip>` is the local IP of the Franka robot, it is 172.16.0.2 is most cases. For example you can run:

`./HD_Franka_Simple_Motion_Mirror <fci-ip>`

## Dependancy 

Some of the examples will have to install other dependant packages.

- OpenHaptics

- SDL2

- OpenGL




[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[api-docs]: https://frankaemika.github.io/libfranka
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/libfranka.svg?branch=master
[travis]: https://travis-ci.org/frankaemika/libfranka
[codecov-status]: https://codecov.io/gh/frankaemika/libfranka/branch/master/graph/badge.svg
[codecov]: https://codecov.io/gh/frankaemika/libfranka
[libfranka]: https://github.com/frankaemika/libfranka.git
[examples]: https://github.com/VinceGuan/Franka_Teleop/tree/main/examples
