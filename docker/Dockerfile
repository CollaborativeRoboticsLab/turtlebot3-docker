#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start base image
#----
#---------------------------------------------------------------------------------------------------------------------------

FROM ros:humble-ros-base-jammy as base

## Parameters
ENV WORKSPACE_ROOT=/colcon_ws
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV TURTLEBOT3_MODEL=waffle

#############################################################################################################################
#####
#####   Install Dependencies
#####
#############################################################################################################################

WORKDIR /

RUN apt-get update -y
RUN apt-get install -y --no-install-recommends python3-colcon-common-extensions \
                                               ros-dev-tools \
                                               ros-$ROS_DISTRO-tf2-eigen \
                                               ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
                                               ros-$ROS_DISTRO-navigation2 \
                                               ros-$ROS_DISTRO-nav2-bringup \
                                               ros-$ROS_DISTRO-slam-toolbox \
                                               ros-$ROS_DISTRO-cartographer \
                                               ros-$ROS_DISTRO-cartographer-ros \
                                               ros-$ROS_DISTRO-rviz2 \
                                               ros-$ROS_DISTRO-turtlesim \
                                               ros-$ROS_DISTRO-turtlebot3 \
                                               ros-$ROS_DISTRO-turtlebot3-cartographer \
                                               ros-$ROS_DISTRO-turtlebot3-teleop \
                                               ros-$ROS_DISTRO-turtlebot3-gazebo && \
                                               apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get clean

#############################################################################################################################
#####
#####   Build Workspace
#####
#############################################################################################################################

WORKDIR ${WORKSPACE_ROOT}/src

RUN git clone https://github.com/CollaborativeRoboticsLab/turtlebot3-docker.git

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list

RUN rosdep init && rosdep update && rosdep install --from-paths ${WORKSPACE_ROOT}/src -y --ignore-src

WORKDIR ${WORKSPACE_ROOT}

RUN . /opt/ros/humble/setup.sh && colcon build

WORKDIR /

COPY docker/workspace_entrypoint.sh /workspace_entrypoint.sh

RUN chmod +x /workspace_entrypoint.sh

ENTRYPOINT [ "/workspace_entrypoint.sh" ]
