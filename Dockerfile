FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get install -y git python3-pip

RUN apt install -y python3-colcon-common-extensions

# Install pygame and its dependencies
RUN apt-get install -y python3-pygame

SHELL [ "/bin/bash" , "-c" ]
RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN source /opt/ros/foxy/setup.bash

RUN  apt install -y python3-colcon-common-extensions \
    && apt-get install -y ros-foxy-joint-state-publisher-gui \
    && apt-get install -y ros-foxy-gazebo-plugins

RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

RUN curl -sSL http://get.gazebosim.org | sh

RUN source /opt/ros/foxy/setup.bash



#RUN git clone https://github.com/ai-winter/ros_motion_planning.git

