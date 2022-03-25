ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO
LABEL description="HWT905 inclinometer ros node"
LABEL maintainer="ponomarevda96@gmail.com"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/inclinometer
RUN apt-get update                          &&  \
    apt-get upgrade -y                      &&  \
    apt-get install -y  git                     \
                        iproute2                \
                        ros-$ROS_DISTRO-catkin  \
                        python3-catkin-tools    \
                        net-tools               \
                        tcpdump
RUN if [[ "$ROS_DISTRO" = "melodic" ]] ; then apt-get install -y python-pip python-catkin-tools ; fi

# Install inclinometer package
COPY inclinometer/install_requirements.sh   inclinometer/install_requirements.sh
COPY inclinometer/python3_requirements.txt  inclinometer/python3_requirements.txt
RUN ./inclinometer/install_requirements.sh
COPY inclinometer/     inclinometer/
RUN source /opt/ros/$ROS_DISTRO/setup.bash  && cd ../../ && catkin build
COPY scripts/               scripts/

CMD echo "main process has been started"            &&  \
    echo "container has been finished"
