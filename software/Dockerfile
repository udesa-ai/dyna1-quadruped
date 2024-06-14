FROM ros:humble-ros-base-jammy

RUN apt update && apt -y install libboost-all-dev

RUN apt-get -y install libjsoncpp-dev

RUN apt-get -y install nano

RUN apt -y install libsdl1.2-dev

RUN apt-get install -y python3-pip

RUN pip3 install scipy

ADD /robot-utils /robot-utils

WORKDIR /robot-utils

RUN pip3 install .

RUN apt update

RUN apt install -y "ros-humble-joy*"

RUN apt-get install -y jstest-gtk

RUN apt-get install -y iputils-ping

RUN apt-get install -y libi2c-dev

RUN echo 'alias walk="ros2 launch launcher walking_teleop_launch.py"' >> ~/.bashrc

RUN apt update

RUN apt install -y ros-humble-rmw-cyclonedds-cpp

RUN apt-get update

RUN apt-get install -y lttng-tools liblttng-ust-dev

RUN apt-get install -y python3-babeltrace python3-lttng

RUN apt-get install -y lttng-modules-dkms

RUN pip3 install smbus

RUN pip3 install pyserial

RUN echo 'walking(){ ros2 launch launcher walking_teleop_launch.py MAX_CURRENT:=$1 ; }' >> ~/.bashrc
