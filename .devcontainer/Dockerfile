# Declare the global build argument
ARG ROS_DISTRO=jazzy

# ==========================================================================
# TARGET 1: Base Configuration for ROS 2 Jazzy (Ubuntu 24.04)
# ==========================================================================
FROM osrf/ros:jazzy-desktop-full AS ros-jazzy
ENV ROS_DISTRO=jazzy
ENV NODE_VERSION=24.17.0
# Ubuntu 24.04 marks its Python as externally managed (PEP 668); allow system-wide pip installs.
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# Auto-source the bench workspace overlay in every shell once it's been built (puts lidar_zones
# etc. on PYTHONPATH). The `[ -f ]` guard makes this safe before the first colcon build, when
# install/ doesn't exist yet. Baked into the image so it doesn't depend on the devcontainer's
# postCreateCommand running.
RUN echo '[ -f /lidar_test_bench/install/setup.bash ] && source /lidar_test_bench/install/setup.bash' >> ~/.bashrc

# Consolidated installation: Combined system tools, Git LFS, and libprometheus-cpp-dev.
# foxglove_bridge serves the ROS graph over a websocket (default port 8765) so Foxglove can
# visualize point clouds without RViz/X11.
RUN apt-get update && apt-get install -y \
    curl \
    git \
    git-lfs \
    build-essential \
    python3-pip \
    python3-dev \
    ros-${ROS_DISTRO}-foxglove-bridge \
    && git lfs install --system \
    && rm -rf /var/lib/apt/lists/*

# `just` command runner. Ubuntu 24.04's universe package is well behind upstream, so pull the
# pinned prebuilt binary from the official installer into /usr/local/bin.
ENV JUST_VERSION=1.42.4
RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh \
    | bash -s -- --tag ${JUST_VERSION} --to /usr/local/bin \
    && just --version

# Fix NVM path logic by declaring NVM_DIR explicitly
ENV NVM_DIR=/usr/local/share/nvm
RUN mkdir -p $NVM_DIR \
    && curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash \
    && . $NVM_DIR/nvm.sh \
    && nvm install ${NODE_VERSION}

# Dynamically link node to your system path
ENV PATH=$NVM_DIR/versions/node/v${NODE_VERSION}/bin:$PATH

# Scratch dir for the dependency install only; the final WORKDIR is set below.
WORKDIR /tmp/deps
COPY requirements.txt* ./

# Ubuntu 24.04 ships a Debian-managed pip that can't be self-upgraded (no RECORD file),
# and marks the environment externally managed (PEP 668) -> use the stock pip with --break-system-packages.
RUN if [ -f requirements.txt ]; then pip3 install --no-cache-dir -r requirements.txt --break-system-packages; fi \
    && pip3 install --no-cache-dir --break-system-packages \
        numpy \
        streamlit \
        watchdog \
        plotly \
        urdf-parser-py \
        google-api-python-client \
        google-auth \
        google-auth-oauthlib \
        google-auth-httplib2

RUN rm -rf /tmp/deps

# Land every shell in the bench workspace (where the Justfile lives), not a scratch /workspace.
WORKDIR /lidar_test_bench

# 3001 (node), 8501 (streamlit/polyview), 8765 (foxglove_bridge websocket)
EXPOSE 3001 8501 8765
CMD ["/bin/bash"]


# ==========================================================================
# TARGET 2: Base Configuration for ROS 2 Jazzy (Ubuntu 24.04) - Evaluation
# ==========================================================================
# Note: Keeping this as a separate stage if you need a different workspace/setup
FROM ros-jazzy AS ros-jazzy-eval
WORKDIR /lidar_evaluation_workspace
# Inherits all previous layers (node, dependencies, etc) from ros-jazzy
# You can add specific eval-only packages here if needed
CMD ["/bin/bash"]