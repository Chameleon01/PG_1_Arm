FROM ros:humble

# install ros package
# sudo docker exec -it humble_container bash
RUN apt-get update && apt-get install -y
RUN apt install ros-dev-tools -y
RUN apt install ros-humble-desktop python3-argcomplete -y
RUN apt install ros-humble-moveit -y
RUN apt -y install python3-colcon-common-extensions
RUN apt -y install ros-humble-rviz2
RUN apt -y install ros-humble-controller-manager
RUN apt -y install ros-humble-joint-state-broadcaster
RUN apt -y install ros-humble-joint-trajectory-controller
RUN apt -y install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-moveit-simple-controller-manager
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc 
RUN echo 'source /home/PG_1_Arm/install/setup.bash' >> ~/.bashrc 
RUN echo 'cd /home/PG_1_Arm/' >> ~/.bashrc 
