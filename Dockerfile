FROM osrf/ros:melodic-desktop-full


# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*





# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


USER $USERNAME
WORKDIR /home/$USERNAME


# Dependencies
RUN sudo apt update -y\
    && sudo apt install wget -y\
    && sudo apt install ros-melodic-four-wheel-steering-msgs -y \
    && sudo apt install ros-melodic-urdf-geometry-parser -y \
    && sudo apt install ros-melodic-jsk-rviz-plugins -y \
    && sudo apt install ros-melodic-moveit-ros-planning-interface -y

# Update CMake version
RUN sudo apt-get update -y\
    && sudo apt-get install -y wget gnupg software-properties-common lsb-release \
    && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg > /dev/null \
    && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ bionic main" | sudo tee /etc/apt/sources.list.d/kitware.list \
    && sudo apt-get update -y \
    && sudo apt-get install -y cmake

# ROS packages
RUN sudo apt install ros-melodic-four-wheel-steering-msgs -y \
    && sudo apt install ros-melodic-urdf-geometry-parser -y \
    && sudo apt install ros-melodic-jsk-rviz-plugins -y \
    && sudo apt install ros-melodic-moveit-ros-planning-interface -y


# Openwalker
RUN cd ~/ \
    && git clone --recursive https://github.com/TUM-ICS/openwalker.git \
    && cd openwalker \
    && /bin/bash -c "source /opt/ros/melodic/setup.bash" \
    && yes | ./doc/install/install.sh \
    && rosinstall src/reemc /opt/ros/melodic doc/install/melodic.rosinstall
    # && rosdep update \
    # && rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys="opencv2 pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev gmock walking_utils rqt_current_limit_controller simple_grasping_action reemc_init_offset_controller walking_controller" \
    # && catkin build -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release


RUN bash -c "source /opt/ros/melodic/setup.bash \
    && cd /home/$USERNAME/openwalker \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys='opencv2 pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev gmock walking_utils rqt_current_limit_controller simple_grasping_action reemc_init_offset_controller walking_controller'"
RUN bash -c "source /opt/ros/melodic/setup.bash \
    && cd /home/$USERNAME/openwalker \
    && catkin build -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release"









# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# COPY my_octomap.ot /home/$USERNAME/my_octomap.ot

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]