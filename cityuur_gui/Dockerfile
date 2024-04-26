# Use the official ROS2 Humble image as the base
FROM osrf/ros:humble-desktop

# Set the working directory
WORKDIR /mnt/d/Users/Eric/Desktop/UR_workshop/cityuur_gui_main_ros2_test/cityuur_gui

# Copy your ROS2 package into the Docker container
COPY ./entry_point.sh /entry_point.sh
ENTRYPOINT ["/entry_point.sh"]

# Install any necessary dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-colcon-common-extensions

# Build your ROS2 package
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; cd cityuur_gui; colcon build'

# Source the ROS2 setup script
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /root/cityuur_gui/install/setup.bash' >> ~/.bashrc
