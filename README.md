# arm32v7_ros_docker

This Dockerfile is based on the procedure in the following link.

https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html

## How to build

```
# cross compile ROS 2 resources
docker buildx build --build-arg UBUNTU_CODENAME=focal ROS_DISTRO=foxy -t rclex/arm32v7_ros_docker:foxy .
# install vendor resources ex) libspdlog, libtinyxml2
docker buildx build --build-arg ROS_DISTRO=foxy -t rclex/arm32v7_ros_docker_with_vendor_resources:foxy -f Dockerfile.with_vendor_resources .
```
