# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
ARG ARCH
FROM franklinselva/isaac-ros:${ARCH}-humble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-distutils \
    libboost-all-dev \
    libboost-dev \
    libpcl-dev \
    liboctomap-dev \
    libode-dev \
    lcov \
    python3-zmq \
    libxaw7-dev \
    libgraphicsmagick++1-dev \
    graphicsmagick-libmagick-dev-compat \
    libceres-dev \
    libsuitesparse-dev \
    libncurses5-dev \
    libassimp-dev \
    libyaml-cpp-dev \
    libpcap-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Install depthai dependencies
RUN apt-get update \
    && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
RUN wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | bash
RUN git clone https://github.com/luxonis/depthai-ros.git /workspaces/deps_ws/src/depthai-ros --branch humble --depth 1
RUN cd /workspaces/deps_ws/ && source /opt/ros/humble/setup.bash && ./src/depthai-ros/build.sh -s 1 -r 1 -m 1

# Setup dependencies
RUN apt-get install -y --no-install-recommends python3-vcstools libgeographic-dev
COPY docker/deps/ros2.repos /workspaces/deps_ws/ros2.repos
RUN vcs import --shallow /workspaces/deps_ws/src < /workspaces/deps_ws/ros2.repos

# Install realsense dependencies
COPY docker/scripts/install-realsense-dependencies.sh /tmp/install-realsense-dependencies.sh
COPY docker/scripts/build-librealsense.sh /tmp/build-librealsense.sh
# Copy hotplug script which will get invoked whenever a devices plugged or un-plugged
RUN mkdir -p /opt/realsense/
COPY docker/scripts/hotplug-realsense.sh /opt/realsense/hotplug-realsense.sh

RUN : \
    # TODO: Remove this once the issue with realsense is fixed
    # Run only if x86_64 architecture
    && if [ "$(uname -m)" = "x86_64" ]; then \
    bash /tmp/install-realsense-dependencies.sh; \
    bash /tmp/build-librealsense.sh; \
    else \
    echo "Skipping realsense dependencies installation as architecture is not x86_64"; \
    fi

# Setup project
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel
COPY docker/deps/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

RUN cd /workspaces/deps_ws && rm -rf build/ install /log/ && \
    source /opt/ros/humble/setup.bash && \
    colcon build --merge-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/opt/ros/humble

RUN cd /workspaces/deps_ws && rm -rf build/ log/ src/ && \
    echo "source /workspaces/isaac_ros-dev/ros_ws/install/setup.bash" >>~/.bashrc && \
    echo "source /workspaces/deps_ws/install/setup.bash" >>~/.bashrc

# Project related setup
# COPY docker/middleware_profiles/rtps_udp_profile.xml /usr/local/share/middleware_profiles/rtps_udp_profile.xml
