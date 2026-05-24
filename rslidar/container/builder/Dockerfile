FROM ros:humble

ARG home="/home/ubuntu"
ARG workspace="$home/ros_ws"
ARG workspace_src="$workspace/src"
ARG ros_distro="humble"

USER root

RUN \
--mount=type=bind,\
source=./container/builder/requirements/robot.txt,\
target=/tmp/requirements.txt true \
&& apt-get update \
&& apt-get install -q -y \
  --no-install-recommends \
    $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
&& apt-get clean -q -y \
&& apt-get autoremove -q -y \
&& rm -rf /var/lib/apt/lists/*

RUN pip install paho-mqtt

WORKDIR $home

COPY rslidar/ $workspace_src
RUN /bin/bash -c 'source /opt/ros/$ros_distro/setup.bash; \
    cd $workspace; colcon build;'

RUN bash -c "echo 'source /opt/ros/$ros_distro/setup.bash'   >> ~/.bashrc"
RUN bash -c "echo 'source /home/ubuntu/ros_ws/install/setup.bash' >> ~/.bashrc"




