FROM ubuntu:20.04

# Set noninteractive mode to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update and install required packages
RUN apt update && apt install -y \ 
    locales \ 
    curl \ 
    gnupg2 \ 
    lsb-release && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Setup ROS 2 repository
RUN curl -sSL http://packages.ros.org/ros2/keys/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS 2 Foxy base and development tools
RUN apt update && apt install -y \ 
    ros-foxy-desktop \ 
    python3-colcon-common-extensions \ 
    python3-rosdep \ 
    python3-argcomplete \ 
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up environment variables
ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV PATH=$ROS_ROOT/bin:$PATH
ENV LD_LIBRARY_PATH=$ROS_ROOT/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=$ROS_ROOT/lib/python3.8/site-packages:$PYTHONPATH
ENV CMAKE_PREFIX_PATH=$ROS_ROOT

# Source ROS 2 setup script automatically
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Set up an entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
