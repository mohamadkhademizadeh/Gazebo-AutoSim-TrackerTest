FROM osrf/ros:humble-desktop
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip python3-colcon-common-extensions gazebo && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY . /ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
SHELL ["/bin/bash","-c"]
CMD source /opt/ros/humble/setup.bash && source /ws/install/setup.bash &&     ros2 launch sim_autotest bringup.launch.py targets:=5 duration:=30.0 noise:=0.25
