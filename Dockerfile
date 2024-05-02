FROM registry.hub.docker.com/library/ros:noetic

# install apt deps
RUN apt-get update && \
    apt-get install -y git libgflags-dev libpopt-dev \
                       libgoogle-glog-dev liblua5.1-0-dev \
                       libboost-all-dev libqt5websockets5-dev \
                       cmake \
                       python-is-python3 libeigen3-dev sudo tmux

# install ros apt deps
RUN apt-get install -y ros-noetic-tf ros-noetic-angles

ARG HOST_UID
RUN useradd dev -m -s /bin/bash -u $HOST_UID -G sudo
USER dev
WORKDIR /home/dev
RUN rosdep update

# clone deps
RUN git clone https://github.com/ut-amrl/amrl_maps.git && \
    git clone https://github.com/ut-amrl/amrl_msgs.git && \
    git clone https://github.com/ut-amrl/ut_automata.git --recurse-submodules

# set up .bashrc
RUN echo "source /opt/ros/noetic/setup.sh\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/ut_automata\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/cs393r_starter\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_maps\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.profile
RUN echo "source /opt/ros/noetic/setup.bash\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/ut_automata\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/cs393r_starter\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_maps\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.bashrc

# clone and build GTSAM
RUN git clone https://github.com/borglab/gtsam.git 
WORKDIR /home/dev/gtsam
RUN mkdir build && mkdir local && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=/home/dev/gtsam/local -DGTSAM_USE_SYSTEM_EIGEN=ON && make install 
ENV CMAKE_PREFIX_PATH=/home/dev/gtsam/local:${CMAKE_PREFIX_PATH}
ENV GTSAM_DIR=/home/dev/gtsam/local
ENV LD_LIBRARY_PATH=/home/dev/gtsam/local/lib:${LD_LIBRARY_PATH}
WORKDIR /home/dev

# build deps
RUN /bin/bash -lc "cd amrl_msgs && make"
RUN /bin/bash -lc "cd ut_automata && make"

# add launcher
ENV CS393R_DOCKER_CONTEXT 1
COPY --chown=dev:dev ./tmux_session.sh /home/dev/tmux_session.sh
RUN chmod u+x /home/dev/tmux_session.sh
CMD [ "/home/dev/tmux_session.sh" ]
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
