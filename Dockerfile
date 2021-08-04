ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-controller-base:${VERSION}

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libboost-system-dev \
        git \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

## Install Raspberry Pi GPIO drivers
RUN pip3 install RPi.GPIO

ARG START_HERE=blah

## My own files for clover core libraries
COPY led_msgs /ros_ws/src/led_msgs
COPY ws281x /ros_ws/src/ws281x
COPY clover_ros2 /ros_ws/src/clover_ros2

WORKDIR /ros_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select \
        # mavros_msgs \
        led_msgs \
        ws281x \
        clover_ros2 \
    && rm -r build

COPY docker/starling-clover.launch.xml .
COPY docker/run.sh .

CMD ["./run.sh"]