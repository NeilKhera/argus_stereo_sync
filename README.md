# Argus Stereo Sync
> Disclaimer: The following piece of software is under development and has only undergone a couple of "tabletop" tests so far. It is not guaranteed to work as intended, or to expectations.

### Table of Contents
- [Introduction] (#introduction)
- [Development Configuration](#development-configuration)
- [Installation](#installation)
- [ROS Output](#ros-output)
- [Technical Documentation](#technical-documentation)
- [Known Issues](#known-issues)
- [Todos](#todos)
- [License](#license)

### Introduction
Synchronization describes the process of triggering two or more cameras in such a way that allows them to produce image frames at the same instance of time. Synchronization of cameras is essential to achieving good quality real-time stereo matching for depth estimation, especially on fast-moving systems. 

Typical methods to accomplish synchronization involve using serial connections between cameras where the 'master' camera triggers its 'slaves' to grab an image frame along with it (hardware synchronization), or by starting the capture session at identical framerates, as close as possible, and discarding image pairs that lie beyond a certain threshold (software synchronization). In the absence of hardware trigger GPIO (such as on the LI-IMX274-CS cameras we utilize), software synchronization remains the only option - however, dropping image pairs effectively lowers the output framerate of the cameras.

Effective stereo matching requires a low (In an ideal world, zero) time latency between frames of a stereo image pair. As such software synchronization as described above is ineffective in most real-time systems.

argus_stereo_sync is a ROS package that utilizes Nvidia's Argus API ([Libargus](https://docs.nvidia.com/jetson/l4t-multimedia/group__LibargusAPI.html)), a part of the Jetson Multimedia API package, for Nvidia's COTS Jetson computing systems to register multiple cameras on a single session, allowing for greater synchrony between frame captures than conventional software-based methods.

### Development Configuration
The above software has been developed on the following configuration:
- Nvidia Jetson TX2 Module L4T v32.3.1 (Leopard Imaging drivers)
- ROS Melodic Morenia
- Nvidia TX2 development kit board
- Leopard Imaging camera adapter LI-JTX1-MIPI-ADPT
- 2 x Leopard Imaging MIPI-CSI2 camera modules LI-IMX274-MIPI-CS
- 2 x Leopard Imaging MIPI cables FAW-1233-05

Other Jetson configurations are likely to work as well but have not been tested.

### Installation
> Argus Stereo Sync requires ROS, CUDA toolkit, and GTK3 as dependencies

Follow these steps to create a ROS workspace:
```sh
$ mkdir catkin_ws && cd catkin_ws
$ mkdir src && cd src
$ catkin_init_workspace
```
Install Argus Stereo Sync:
```sh
$ git clone https://github.com/Nekhera/argus_stereo_sync.git
$ cd argus_stereo_sync/libs
```
- Untar the contents of the Jetson Multimedia API (A folder called jetson_multimedia_api, or formerly tegra_multimedia_api) here. Make to relabel the folder to jetson_multimedia_api if needed.
> Currently, the installation of the Jetson Multimedia API to the libs folder must be done manually. Automated installation is planned.

> We have utilized the API provided by Leopard Imaging with their adapter drivers (Please contact Leopard Imaging at sales@leopardimaging.com to get the most up-to-date drivers). We are unaware of differences (if any) existing between this version and the downloads provided on the [Jetson Download Center](https://developer.nvidia.com/embedded/downloads).

```sh
$ cd /path/to/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```
This will build the package. To run the stereo sync node, type:
```sh
$ rosrun argus_stereo_sync argus_stereo_sync
```

### ROS Output
The _argus_stereo_sync_ node outputs frames from the left and right cameras as sensor_msgs::Image messages on the ```/camera/left/image``` and the ```/camera/right/image``` rostopics respectively. Images are in the BGR8 encoding format.

### Technical Documentation
Kindly refer to the _Technical Documentation_ page on the [wiki](https://github.com/Nekhera/argus_stereo_sync/wiki/Technical-Documentation).

### Known Issues
- In our tests, a resolution of 960x540 delivered a clean 60FPS however anything above that led to significant drops in framerate. The bottleneck is likely in copying the image buffer from the CUDA device to host memory.

_Please report any bugs and issues encountered to the repo issue tracker. Pull requests are welcome!_
### Todos
- Published messages should use sensor timestamp, not _ros::now::time()_.
- Add the ability to dynamically set camera exposure range, allowing with other parameters such as framerate and resolution (preferably without restarting node).
- Automate API installation.
- Optimize CUDA kernel for improved performance at higher resolutions. (YUV 422 supported by ROS requires 16-bit per pixel as opposed to the 24-bit per pixel of BGR8 - Thereby reducing buffer size by 33%) 
### License
MIT License. Copyright (c) 2020 Neil Khera

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
___
_fin._
