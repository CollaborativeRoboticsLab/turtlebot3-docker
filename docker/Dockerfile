#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start base image
#----
#---------------------------------------------------------------------------------------------------------------------------

FROM ros:humble-ros-base-jammy as base

## Parameters
ENV WORKSPACE_ROOT=/colcon_ws

#############################################################################################################################
#####
#####   Install Dependencies
#####
#############################################################################################################################

WORKDIR /

RUN apt-get update -y
RUN apt-get install -y --no-install-recommends python3-colcon-common-extensions \
                                               x11-apps \
                                               mesa-utils \
                                               libgl1-mesa-glx \
                                               libxrender1 \
                                               libsm6 \
                                               libxext6 \
                                               libx11-xcb1 \
                                               libxcb-icccm4 \
                                               libxcb-image0 \
                                               libxcb-keysyms1 \
                                               libxcb-render-util0 \
                                               libxcb-xinerama0 \
                                               libxcb-xkb1 \
                                               libxcb-xinput0 \
                                               libxcb-xfixes0 \
                                               libxcb-shape0 \
                                               libxcb-randr0 \
                                               libxcb-glx0 \
                                               libnss3 \
                                               libgtk-3-0 \
                                               ros-dev-tools \
                                               ros-$ROS_DISTRO-tf2-eigen \
                                               ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
                                               ros-$ROS_DISTRO-navigation2 \
                                               ros-$ROS_DISTRO-nav2-bringup \
                                               ros-$ROS_DISTRO-slam-toolbox \
                                               ros-$ROS_DISTRO-gazebo-ros-pkgs \
                                               ros-$ROS_DISTRO-cartographer \
                                               ros-$ROS_DISTRO-cartographer-ros

RUN apt-get clean

#############################################################################################################################
#####
#####   Install Astra legacy packages
#####
#############################################################################################################################

WORKDIR ${WORKSPACE_ROOT}/src

RUN git clone https://github.com/CollaborativeRoboticsLab/turtlebot3-docker.git
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list

RUN rosdep init && rosdep update && rosdep install --from-paths ${WORKSPACE_ROOT}/src -y --ignore-src

#############################################################################################################################
#####
#####   Build Kobuki packages
#####
#############################################################################################################################

WORKDIR ${WORKSPACE_ROOT}

RUN . /opt/ros/humble/setup.sh && colcon build

WORKDIR /

#############################################################################################################################
#####
#####   Remove workspace source and build files that are not relevent to running the system
#####
#############################################################################################################################

RUN rm -rf ${WORKSPACE_ROOT}/src
RUN rm -rf ${WORKSPACE_ROOT}/log
RUN rm -rf ${WORKSPACE_ROOT}/build

RUN rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/*
RUN apt-get clean


#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start final release image
#----
#---------------------------------------------------------------------------------------------------------------------------


FROM ros:humble-ros-base-jammy as final

## Parameters
ENV WORKSPACE_ROOT=/colcon_ws
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV TURTLEBOT3_MODEL=waffle

WORKDIR /

COPY --from=base / /

COPY docker/workspace_entrypoint.sh /workspace_entrypoint.sh

RUN chmod +x /workspace_entrypoint.sh

ENTRYPOINT [ "/workspace_entrypoint.sh" ]
