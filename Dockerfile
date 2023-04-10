ARG UBUNTU_VERSION=jammy
ARG FROM_IMAGE=ubuntu:$UBUNTU_VERSION
ARG OVERLAY_WS=/opt/rnz/overlay_ws
# ARG TEST_PATH=capra-ros-local-planner/capra-ros-local-planner/test


# MAKE SOME BASIC MODIFICATION TO THE BASE IMAGE
FROM $FROM_IMAGE AS dependencies_setter


RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
  apt-get update \
  && apt-get install -y -q --no-install-recommends\
  cmake \
  curl \
  python3-pip \
  nano \ 
  gnupg \
  gdb \
  sudo \
  clang-tidy \
  build-essential \
  && pip install -U autopep8 \
  && rm -rf /var/lib/apt/lists/*

ENV CMAKE_MAKE_PROGRAM=/usr/bin/make
ENV CMAKE_CXX_COMPILER=/usr/bin/g++
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

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


# MULTI-STAGE FOR CACHING
FROM dependencies_setter AS cacher

# copy overlay source   
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src

# MULTI-STAGE FOR BUILDING DEPENDENCIES FROM SOURCE
FROM dependencies_setter AS prebuilder
ARG DEBIAN_FRONTEND=noninteractive

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS

# install CI dependencies
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  ccache \
  lcov \
  && mkdir src \
  && rm -rf /var/lib/apt/lists/*


# build overlay source
COPY --from=cacher $OVERLAY_WS ./
ARG OVERLAY_MIXINS="release ccache"

# BUILD CURRENT PROJECT WITH COLCON (CMAKE)
FROM dependencies_setter AS builder
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS

RUN apt update \
  && apt install -y -q --no-install-recommends \
  clang-tidy \
  && rm -rf /var/lib/apt/lists/*


COPY --from=prebuilder $OVERLAY_WS/ ./
COPY ./project_folder ./src/project_folder
RUN rm -rf ./src/project_folder/build


RUN mkdir $OVERLAY_WS/src/project_folder/build 
RUN cd $OVERLAY_WS/src/project_folder/build && cmake .. && make

CMD ["tail", "-f", "/dev/null"]

# RUN ALL TESTS IN GTEST
FROM builder AS tester

ARG OVERLAY_WS
RUN cd $OVERLAY_WS/src/project_folder/build && ctest

WORKDIR $OVERLAY_WS/src/project_folder
# COPY ./run_tests.sh $OVERLAY_WS
# CMD ["./run_tests.sh"]
CMD ["tail", "-f", "/dev/null"]

# MULTI-STAGE FOR RUNNING   
FROM dependencies_setter AS runner

ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
# THIS SHOULD BE CHANGED FOR A INSTALLED FOLDER!!!!!!!!
COPY --from=builder $OVERLAY_WS/src/project_folder $OVERLAY_WS/src/project_folder
# RUN rm -rf $(find . -type d -name include)

ENV OVERLAY_WS $OVERLAY_WS

# RUN A APP FILE
COPY entrypoint.sh /entrypoint.sh 
RUN chmod -x /entrypoint.sh
ENTRYPOINT ["tail", "-f", "/dev/null"]
