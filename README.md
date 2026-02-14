# Turtlebot3 Docker based Navigation Stack

This main branch contains instructions for the ROS2 Humble system.

## Usage with Docker

Clone this reposiotory

```bash
git clone https://github.com/CollaborativeRoboticsLab/turtlebot3-docker.git
```

### Prebuilt Usage

Pull the Docker image and start compose (No need to run `docker compose build`)

```bash
cd turtlebot3-docker/docker
docker compose pull
docker compose up
```

To clean the system,
```bash
docker compose down
```

### Local build

Run the following commands to build locally
```bash
cd turtlebot3-docker/docker
docker compose -f compose-build.yaml build
docker compose -f compose-build.yaml up
```

To clean the system,
```bash
docker compose -f compose-build.yaml down
```


## Usage with without Docker

Though this is geared toward docker, the package itself can be directly used with ROS2 without docker.

### Install dependencies

```bash
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-nav2-minimal-tb*
```

### Clone the package

```bash
mkdir -p colcon_ws/src
cd colcon_ws/src
```

```bash
git clone https://github.com/CollaborativeRoboticsLab/turtlebot3-docker.git
cd ..
rosdep install --from-paths src -y --ignore-src
```


### Build the package
```bash
colcon build
```

## Launch Gazebo Simulation

```bash
source install/setup.bash
ros2 launch turtlebot3-docker turtlebot3_world.launch.launch.py
```

## Launch Nav2 with SLAM Toolbox

```bash
source install/setup.bash
ros2 launch turtlebot3-docker system.launch.py
```
