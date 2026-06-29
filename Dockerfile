# Declare the global build argument at the very top so FROM can read it
ARG ROS_DISTRO=humble

# ==========================================================================
# TARGET 1: Base Configuration for ROS 2 Humble (Ubuntu 22.04)
# ==========================================================================
FROM osrf/ros:humble-desktop-full AS ros-humble
ENV ROS_DISTRO=humble
ENV NODE_VERSION=24.17.0
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Jump down to the shared setup steps using a helper macro block
ONBUILD RUN apt-get update && apt-get install -y curl git build-essential python3-pip python3-dev && rm -rf /var/lib/apt/lists/*

# Copy the core dependencies install script directly into this stage
RUN apt-get update && apt-get install -y curl git build-essential python3-pip python3-dev && rm -rf /var/lib/apt/lists/*

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
RUN pip3 install --no-cache-dir --upgrade pip \
    && if [ -f requirements.txt ]; then pip3 install --no-cache-dir -r requirements.txt; fi \
    && pip3 install --no-cache-dir streamlit watchdog google-api-python-client google-auth

EXPOSE 3001 8501
CMD ["/bin/bash"]


# ==========================================================================
# TARGET 2: Base Configuration for ROS 2 Jazzy (Ubuntu 24.04)
# ==========================================================================
FROM osrf/ros:jazzy-desktop-full AS ros-jazzy
ENV ROS_DISTRO=jazzy
ENV NODE_VERSION=24.17.0
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y curl git build-essential python3-pip python3-dev && rm -rf /var/lib/apt/lists/*

# Fix NVM path logic by declaring NVM_DIR explicitly
ENV NVM_DIR=/usr/local/share/nvm
RUN mkdir -p $NVM_DIR \
    && curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash \
    && . $NVM_DIR/nvm.sh \
    && nvm install ${NODE_VERSION}

# Dynamically link node to your system path
ENV PATH=$NVM_DIR/versions/node/v${NODE_VERSION}/bin:$PATH
WORKDIR /lidar_evaluation_workspace
COPY requirements.txt* ./
RUN pip3 install --no-cache-dir --upgrade pip \
    && if [ -f requirements.txt ]; then pip3 install --no-cache-dir -r requirements.txt; fi \
    && pip3 install --no-cache-dir streamlit watchdog google-api-python-client google-auth

EXPOSE 3001 8501
CMD ["/bin/bash"]
