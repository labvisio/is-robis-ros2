FROM ros:humble

RUN apt update
RUN apt install -y usbutils net-tools software-properties-common wget
RUN apt-get install -y libjpeg-dev libjpeg8-dev libfreetype6-dev vim

RUN wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py
RUN python3 -m pip install --upgrade odrive

RUN apt-get install -y ros-humble-diagnostic-updater
RUN apt-get install -y ros-humble-tf-transformations

WORKDIR /workspace/ros2_ws
RUN mkdir src/
RUN git clone https://github.com/labvisio/is-robis-ros2.git \
    && mv is-robis-ros2/odrive_ros2_pkg src/

WORKDIR /workspace/ros2_ws
RUN colcon build --packages-select odrive_ros2_pkg

SHELL [ "/bin/bash" , "-c" ]
RUN source install/setup.bash
WORKDIR /workspace/ros2_ws/src/odrive_ros2_pkg
RUN python3 -m pip install .

# Lidar
RUN apt install cmake pkg-config

WORKDIR /workspace

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git
WORKDIR /workspace/YDLidar-SDK/build
RUN cmake ..
RUN make
RUN sudo make install
RUN cpack

WORKDIR /workspace/ros2_ws

RUN cd src/ \
    && git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git \
    && cd .. \
    && source /opt/ros/humble/setup.bash \
    && colcon build --packages-select ydlidar_ros2_driver \
    && source install/setup.bash \