FROM osrf/ros:humble-desktop-jammy

# Define build arguments
ARG USERNAME
ARG UID=1000
ARG GID=$UID

# Install dependencies
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    ros-humble-desktop-full \
    python3 \
    python3-pip \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt update -q \
    && apt install -y --no-install-recommends \
    ros-humble-rqt* \
    ignition-fortress \
    ros-humble-tf2 \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create non-root user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to non-root user and set up workspace
USER $USERNAME
RUN mkdir -p /home/$USERNAME/ros2_ws \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /home/$USERNAME/ros2_ws/victor_ws/install/local_setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export DISPLAY=:1" >> /home/$USERNAME/.bashrc

# Copy local project files to the container
COPY --chown=$USERNAME:$USERNAME . /home/$USERNAME/ros2_ws/

# Set working directory
WORKDIR /home/$USERNAME/ros2_ws

# Copy entrypoint script
COPY ./ros_entrypoint.sh /usr/local/bin/

# Setup entrypoint
ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["bash"]
