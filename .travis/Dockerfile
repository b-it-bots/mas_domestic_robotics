FROM argenos/bitbots-base

RUN rosdep update

ADD ./ /kinetic/src/mas_domestic_robotics
WORKDIR /kinetic

RUN wstool init src && wstool merge -t src src/mas_domestic_robotics/.travis/travis.rosinstall
RUN wstool update -t src
RUN apt-get update && \
    rosdep update && \
    rosdep install -q --from-paths src --ignore-src --rosdistro=kinetic -y && \
    rm -rf /var/lib/apt/lists/*
