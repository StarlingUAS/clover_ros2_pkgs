ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-controller-base:${VERSION}

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libboost-system-dev \
        git \
        wget \
        python3-pip \
        libgeographic-dev \
        geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

## Install Raspberry Pi GPIO drivers
RUN pip3 install RPi.GPIO

## Ensure geographiclib is properly installed with FindGeographicLib available
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh \
    && ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.16/Modules/

ARG START_HERE=blah

## My own files for clover core libraries
COPY led_msgs /ros_ws/src/led_msgs
COPY ws281x /ros_ws/src/ws281x
COPY vl53l1x_ros2 /ros_ws/src/vl53l1x_ros2
COPY clover_ros2_msgs /ros_ws/src/clover_ros2_msgs
COPY clover_ros2 /ros_ws/src/clover_ros2
COPY .git /ros_ws/src/.git
COPY .gitmodules /ros_ws/src/.gitmodules

WORKDIR /ros_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select \
        # mavros_msgs \
        led_msgs \
        simple_offboard_msgs \
        ws281x \
        vl53l1x \
        clover_ros2_msgs \
        clover_ros2 \
    && rm -r build

COPY docker/starling-clover.launch.xml .
COPY docker/run.sh .

CMD ["./run.sh"]