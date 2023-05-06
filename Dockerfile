ARG UBUNTU_VERSION=jammy
ARG ROS_DISTRO=humble
ARG FROM_IMAGE=ros:$ROS_DISTRO
ARG ROS_SETUP=/opt/ros/$ROS_DISTRO/setup.sh
ARG OVERLAY_WS=/app

# MAKE SOME BASIC MODIFICATION TO THE BASE IMAGE
FROM $FROM_IMAGE AS dependencies_setter

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
  && apt-get update \
  && apt-get install -y -q --no-install-recommends\
  openssh-server \
  cmake \
  curl \
  wget \
  nano \ 
  gnupg2 \
  gdb \
  sudo \
  clang \
  clang-tidy \
  build-essential \
  dirmngr \
  python3-rosdep \
  python3-pip \
  nano \ 
  git \
  ros-$ROS_DISTRO-ros-ign \
  ros-$ROS_DISTRO-launch-testing \
  ros-$ROS_DISTRO-tf-transformations \
  ros-$ROS_DISTRO-rmw-fastrtps-cpp \
  python3-rosgraph \
  && pip install transforms3d \
  && pip install -U autopep8 xacro rosgraph \
  && rm -rf /var/lib/apt/lists/*


ENV CMAKE_MAKE_PROGRAM=/usr/bin/make
ENV CMAKE_CXX_COMPILER=/usr/bin/g++
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ARG OVERLAY_WS
ARG ROS_DISTRO
WORKDIR $OVERLAY_WS



# MAKE THE DEVELOPER VERSION OF THE IMAGE
FROM dependencies_setter as developer

# Set timezone
ENV TZ=Europe/Copenhagen 


ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ARG USERNAME=rnz
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
  && mkdir -p /home/$USERNAME/.vscode-server /home/$USERNAME/.vscode-server-insiders \
  && chown ${USER_UID}:${USER_GID} /home/$USERNAME/.vscode-server* \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME


WORKDIR /home/$USERNAME

USER $USERNAME

ARG ROS_SETUP
ARG WORKSPACE
ARG OVERLAY_WS

RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.sh; fi" >> /home/${USERNAME}/.bashrc \
  && echo ". $ROS_SETUP" >> /home/${USERNAME}/.bashrc \
  && echo ". $OVERLAY_WS/install/setup.sh" >> /home/${USERNAME}/.bashrc \
  && echo ${WORKSPACE}



# MULTI-STAGE FOR CACHING
FROM dependencies_setter AS cacher

# copy overlay source   
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS

COPY /ignition-bridge/skuid_description /app/skuid_description


# MULTI-STAGE FOR BUILDING DEPENDENCIES FROM SOURCE
FROM dependencies_setter AS prebuilder
ARG DEBIAN_FRONTEND=noninteractive

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS

# install CI dependencies
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  ccache \
  sudo \
  lcov \
  && rm -rf /var/lib/apt/lists/*


# build overlay source
COPY --from=cacher $OVERLAY_WS ./
ARG OVERLAY_MIXINS="release ccache"
ARG ROS_SETUP

RUN . ${ROS_SETUP} && colcon build 



# BUILD CURRENT PROJECT WITH COLCON (CMAKE)
FROM dependencies_setter AS builder
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS

RUN apt update \
  && apt install -y -q --no-install-recommends \
  clang-tidy \
  && rm -rf /var/lib/apt/lists/*


COPY --from=prebuilder $OVERLAY_WS/ ./
RUN rm -rf ./build ./install ./log

ARG ROS_SETUP

RUN . ${ROS_SETUP} && colcon build

# CMD ["tail", "-f", "/dev/null"]

# RUN ALL TESTS IN GTEST
FROM builder AS tester

ARG OVERLAY_WS
ENV OVERLAY_WS ${OVERLAY_WS}
# RUN cd $OVERLAY_WS/src/project_folder/build && ctest

# WORKDIR $OVERLAY_WS/src/project_folder
# COPY ./run_tests.sh $OVERLAY_WS
# CMD ["./run_tests.sh"]
# CMD ["tail", "-f", "/dev/null"]

# MULTI-STAGE FOR RUNNING   
FROM dependencies_setter AS runner

ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
# THIS SHOULD BE CHANGED FOR A INSTALLED FOLDER!!!!!!!!
COPY --from=builder $OVERLAY_WS/install $OVERLAY_WS/install
# RUN rm -rf $(find . -type d -name include)

ENV OVERLAY_WS $OVERLAY_WS

# RUN A APP FILE
COPY entrypoint.sh entrypoint.sh 
RUN chmod +x entrypoint.sh

ENTRYPOINT ["bash"]
CMD [ "entrypoint.sh" ]


