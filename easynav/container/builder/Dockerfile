# Forzamos la arquitectura de la Jetson
FROM --platform=linux/arm64 ros:jazzy

# Definimos las variables como hace tu tutor
ARG home="/home/ubuntu"
ARG workspace="$home/ros_ws"
ARG workspace_src="$workspace/src"
ARG ros_distro="jazzy"

USER root

# Instalación usando el método de "bind mount" de tu tutor
RUN --mount=type=bind,source=./container/builder/requirements/robot.txt,target=/tmp/requirements.txt \
    apt-get update \
    && apt-get install -q -y --no-install-recommends \
       $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/*

WORKDIR $home
COPY src/easynav_plugins $workspace_src/easynav_plugins
COPY src/easynavigation $workspace_src/easynavigation
COPY src/EasyNavigation $workspace_src/EasyNavigation
COPY src/NavMap $workspace_src/NavMap
COPY src/ThirdParty $workspace_src/ThirdParty
COPY src/yaets $workspace_src/yaets
COPY yhs_can_interfaces $workspace_src/yhs_can_interfaces


WORKDIR $workspace
RUN rm -r /etc/ros/rosdep/sources.list.d/20-default.list \
    && apt-get update \
    && sudo rosdep init \
    && rosdep update \
    && rosdep install --from-paths src -y --ignore-src

# Build ROS 2 package
RUN /bin/bash -c 'source /opt/ros/$ros_distro/setup.bash; \
    cd $workspace; colcon build --symlink-install;'

COPY src/fwmini $workspace_src/fwmini
RUN /bin/bash -c 'source /opt/ros/$ros_distro/setup.bash; \
    cd $workspace; colcon build --packages-select fwmini  --symlink-install;'

COPY src/easynav_indoor_testcase $workspace_src/easynav_indoor_testcase

RUN /bin/bash -c 'source /opt/ros/$ros_distro/setup.bash; \
    cd $workspace; colcon build --packages-select easynav_indoor_testcase  --symlink-install;'

COPY src/easynav_behaviors $workspace_src/easynav_behaviors
RUN /bin/bash -c 'source /opt/ros/$ros_distro/setup.bash; \
    cd $workspace; colcon build --packages-select easynav_patrolling_behavior easynav_patrolling_behavior_py --symlink-install;'

# Configuramos el bashrc para que cargue ROS al entrar
RUN bash -c "echo 'source /opt/ros/$ros_distro/setup.bash'   >> ~/.bashrc"
RUN bash -c "echo 'source ${workspace}/install/setup.bash' >> ~/.bashrc"

# Nota: He quitado la parte de 'fixposition_driver' porque imagino que 
# aún no tienes esa carpeta en tu nueva estructura de Jazzy.

