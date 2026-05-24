FROM ros:humble

ARG home="/home/ubuntu"
ARG workspace="$home/ros_ws"
ARG workspace_src="$workspace/src"
ARG ros_distro="humble"

USER root

RUN \
--mount=type=bind,\
source=./container/builder/requirements/camera.txt,\
target=/tmp/requirements.txt true \
&& apt-get update \
&& apt-get install -q -y \
  --no-install-recommends \
    $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
&& apt-get clean -q -y \
&& apt-get autoremove -q -y \
&& rm -rf /var/lib/apt/lists/*

# RUN pip install paho-mqtt

# WORKDIR $home

# COPY luxonis/ $workspace_src
# RUN git clone https://github.com/luxonis/depthai-ros.git $workspace_src/depthai-ros

# WORKDIR $workspace
# RUN apt-get update && rosdep update && \
#     rosdep install --from-paths src --ignore-src -y --rosdistro humble


#RUN /bin/bash -c 'source /opt/ros/$ros_distro/setup.bash; \
#    colcon build --symlink-install --cmake-args "-DDEPTHAI_ROS_BUILD_EXAMPLES=OFF"'

#RUN bash -c "echo 'source /opt/ros/$ros_distro/setup.bash'   >> ~/.bashrc"
#RUN bash -c "echo 'source /home/ubuntu/ros_ws/install/setup.bash' >> ~/.bashrc"

# RUN /bin/bash -c 'source /opt/ros/$ros_distro/setup.bash; \
#     colcon build --symlink-install --cmake-args "-DDEPTHAI_ROS_BUILD_EXAMPLES=OFF"'

# Configuramos el bashrc para cuando entres manualmente
# RUN echo "source /opt/ros/$ros_distro/setup.bash" >> ~/.bashrc && \
#     echo "source $workspace/install/setup.bash" >> ~/.bashrc

# Instalamos depthai en el python del sistema para que los nodos funcionen
# RUN pip install depthai


