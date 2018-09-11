FROM bitbots/bitbots-common:kinetic

WORKDIR /kinetic

RUN rosdep update

RUN wstool init src && wstool merge -t src https://raw.githubusercontent.com/b-it-bots/mas_domestic_robotics/kinetic/mas-domestic.rosinstall
RUN wstool update -t src
RUN rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
