FROM bitbots/bitbots-common:kinetic

WORKDIR /kinetic
COPY mas-domestic.rosinstall /kinetic

RUN wstool init --shallow src && \
    wstool merge -t src mas-domestic.rosinstall && \
    cd src && \
    wstool remove mas_domestic_robotics mas_common_robotics orocos_kinematics_dynamics kdl_parser && \
    cd - && \
    wstool update -t src

ADD . /kinetic/src/mas_domestic_robotics


RUN . /opt/ros/mas_stable/setup.sh && \
    apt-get update -qq && \
    rosdep update -q && \
    rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y && \
    rm -rf /var/lib/apt/lists/* && \
    catkin config --init && \
    catkin config --extend /opt/ros/mas_stable && \
    catkin config --install --install-space /opt/ros/mas_stable && \
    catkin build && \
    rm -rf /kinetic/

WORKDIR /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
