FROM bitbots/bitbots-domestic:melodic-base

WORKDIR /travis
COPY mas-domestic.rosinstall /travis

RUN wstool init --shallow src && \
    wstool merge -t src mas-domestic.rosinstall
WORKDIR /travis/src
RUN wstool remove mas_domestic_robotics mas_common_robotics orocos_kinematics_dynamics kdl_parser zbar_ros dmp ROSPlan mongodb_store occupancy_grid_utils kdl_parser ftsm && \
    wstool update

WORKDIR /travis
COPY ./ /travis/src/mas_domestic_robotics

RUN . /opt/ros/mas_stable/setup.sh && \
    catkin config --init && \
    catkin config --extend /opt/ros/mas_stable && \
    apt-get update -qq && \
    rosdep update -qq && \
    rosdep install -q --from-paths src --ignore-src --rosdistro=melodic -y && \
    rm -rf /var/lib/apt/lists/* && \
    catkin build --no-status -c
