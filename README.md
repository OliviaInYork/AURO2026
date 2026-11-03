# AURO2026
This is the repository for the AURO course's practicals. It is based on Ubuntu 26.04 LTS and contain.

# Podman configuration
You can use the included script [setup-podman.sh](setup-podman.sh) to configure Podman and VS Code under Linux in the CS labs (CSE/069 and CSE/270) for running the Dev Containers used in this repository. That automated setup script configures Podman to store images under `/scratch/$USER/containers` and to map your UID to the `ubuntu` user used in the container.

# Dev containers
If you have Podman (or Docker) and VS Code installed, you can use the following instructions to spin up
the Dev Container configuration contained in this repository to get started with ROS2.

1. Open VS Code
2. Ensure that the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) has been installed.
3. In VS Code, open the command palette (eg. using the key combination `Shift+P` on Windows, or `Command+Shift+P`) and type `Dev Containers: Clone Repository in Container Volume`.
3. Then select `GitHub` and select this repository's name/URL.
4. Wait for the container to finish building.

## Dev Container Configurations available

* `.devcontainer/devcontainer.json`: Uses VNC to expose a desktop and allow interaction with GUI applications via a VNC client or web browser at http://localhost:6080/
* `.devcontainer/linux-auto/devcontainer.json`: Allows use of X11 for natively running the container on Linux with Podman, using either Intel/AMD or NVidia GPUs. GUI applications are shown on the host operating system.
* `.devcontainer/windows-wsl/devcontainer.json`: Allows use of WSL2 to interact with GUI applications showing them natively on the Windows desktop. This option also takes advantage of GPU available on the host for rendering by leveraging support for translating OpenGL calls to Direct3D.

## Container configuration
The default user in a container is `ubuntu` and it comes with `sudo` privileges allowing you to install additional software if needed.

## Included software
The [base image](https://github.com/UoY-RoboStar/ros2-tb3/tree/lyrical) provides a custom installation of ROS2 Lyrical Luth, based on Ubuntu 26.04 LTS, targetting both x86_64 and arm64 architectures. 

It includes ROS2 packages for:
* [TurtleBot3](https://docs.robotis.com/docs/systems/turtlebot3/overview/)
* [Nav2](https://docs.nav2.org/lyrical/)
* [slam-toolbox](https://github.com/SteveMacenski/slam_toolbox)
* [MoveIt2](https://moveit.picknik.ai/main/index.html) (including its [Python API](https://github.com/moveit/moveit2/blob/main/moveit_py/README.md), [tutorials](https://github.com/moveit/moveit2_tutorials) and support for the [Kinova Gen 3 arm](https://github.com/Kinovarobotics/ros2_kortex))
* [rtabmap](https://github.com/introlab/rtabmap)
* [stella-vslam](https://github.com/stella-cv/stella_vslam)
* as well as other useful ROS2 packages and utilities.

It also includes [Gazebo Jetty (LTS)](https://gazebosim.org/docs/latest/install/#gazebo-jetty) and the [ArGoS3](https://www.argos-sim.info/) multi-robot simulator, including a ROS2 bridge.
