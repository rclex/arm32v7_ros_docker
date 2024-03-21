# arm32v7_ros_docker

Dockerfile for ROS 2 on arm32v7 arch by building from source

## Description

This Dockerfile is based on the procedure in the following link.

- https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

The prebuilt image has been published on our Docker Hub repository

- https://hub.docker.com/repository/docker/rclex/arm32v7_ros_docker
- https://hub.docker.com/repository/docker/rclex/arm32v7_ros_docker_with_vendor_resources

Note that our motivation to prepare this repository and Docker image is just for [Rclex](https://github.com/rclex/rclex).
When operating Rclex on Nerves, we need to copy ROS2 resources from the Docker container to the Nerves project.
In other words, the operation of this Dockerfile and Docker container on the arm32v7 device is not guaranteed.

## How to build

```
# cross compile ROS 2 resources
docker buildx build --platform linux/arm/v7 -t rclex/arm32v7_ros_docker:humble -f Dockerfile.humble .
# install vendor resources ex) libspdlog, libtinyxml2 libfmt
docker buildx build --platform linux/arm/v7 -t rclex/arm32v7_ros_docker_with_vendor_resources:humble -f Dockerfile.with_vendor_resources.humble .
```
