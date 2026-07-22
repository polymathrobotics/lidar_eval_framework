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

# Consolidated installation: Combined system tools, Git LFS, and libprometheus-cpp-dev
RUN apt-get update && apt-get install -y \
    curl \
    git \
    git-lfs \
    build-essential \
    python3-pip \
    python3-dev \
    && git lfs install --system \
    && rm -rf /var/lib/apt/lists/*

# Fix NVM path logic by declaring NVM_DIR explicitly
ENV NVM_DIR=/usr/local/share/nvm
RUN mkdir -p $NVM_DIR \
    && curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash \
    && . $NVM_DIR/nvm.sh \
    && nvm install ${NODE_VERSION}

# Dynamically link node to your system path
ENV PATH=$NVM_DIR/versions/node/v${NODE_VERSION}/bin:$PATH

WORKDIR /workspace
COPY requirements.txt* ./

# Ubuntu 24.04 ships a Debian-managed pip that can't be self-upgraded (no RECORD file),
# and marks the environment externally managed (PEP 668) -> use the stock pip with --break-system-packages.
RUN if [ -f requirements.txt ]; then pip3 install --no-cache-dir -r requirements.txt --break-system-packages; fi \
    && pip3 install --no-cache-dir --break-system-packages \
        streamlit \
        watchdog \
        plotly \
        urdf-parser-py \
        google-api-python-client \
        google-auth \
        google-auth-oauthlib \
        google-auth-httplib2

EXPOSE 3001 8501
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