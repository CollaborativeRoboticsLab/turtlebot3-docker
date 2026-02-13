# Navigation Stack

## Usage with Docker

Clone this reposiotory

```bash
git clone https://github.com/CollaborativeRoboticsLab/nav_stack.git && cd nav_stack/docker
```

Pull the Docker image and start compose (No need to run `docker compose build`)
```bash
docker compose pull
docker compose up
```

To clean the system,
```bash
docker compose down
docker volume rm docker_navigation
```

## Usage with without Docker

### Install dependencies

```bash
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox
```

### Clone the package

```bash
mkdir -p navstack_ws/src
cd navstack_ws/src
```

```bash
git clone https://github.com/CollaborativeRoboticsLab/nav_stack.git
cd ..
rosdep install --from-paths src -y --ignore-src
```


### Build the package
```bash
colcon build
```

## Launch Nav2 with SLAM Toolbox

```bash
source install/setup.bash
ros2 launch nav_stack system.launch.py
```
