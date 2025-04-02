FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get install -y git python3-pip

RUN apt install -y python3-colcon-common-extensions

RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc


#RUN git clone https://github.com/ai-winter/ros_motion_planning.git

# Set the DISPLAY environment variable (if you need GUI support)
ENV DISPLAY=:0
