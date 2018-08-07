FROM ros:kinetic
RUN apt-get update && apt-get install -y python-catkin-tools \
    python-wstools
RUN rosdep update

ADD catkin_ws /catkin_ws

RUN cd /catkin_ws && rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
