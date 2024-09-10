FROM ubuntu:22.04
#adopted from: https://github.com/IntelRealSense/librealsense/blob/v2.51.1/scripts/Docker/Dockerfile

ARG DEBIAN_FRONTEND=noninteractive

# Builder dependencies installation
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    curl \
    python3 \
    python3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

FROM osrf/ros:humble-desktop-full

ARG USERNAME=ambi
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

#dependencies 
RUN apt update; apt install build-essential \
                sudo \
                autoconf \
                git \
                libargon2-dev \
                libssl-dev \
                libx11-dev \
                cmake \
                libgstreamer1.0-dev \
                libgstreamer-plugins-base1.0-dev \
                libgstreamer-plugins-bad1.0-dev \
                libxext-dev \
                flex \
                bison \
                gstreamer1.0-pulseaudio \
                python3-pip \
                ros-humble-gazebo-* -y

#add user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

#add user to plugdev group for librealsense
RUN usermod -a -G plugdev $USERNAME

USER $USERNAME

RUN pip install --user --upgrade numpy==1.26.2

#add librealsense-ros
RUN sudo apt install python3-rosdep wget -y
RUN rosdep update

RUN sudo apt update && sudo apt install ros-humble-navigation2 \
                                        ros-humble-nav2-bringup -y

RUN pip install python-dotenv langgraph langchain ipython langchain_groq pydantic

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/agentic_workflow/mnt/aws-robomaker-small-house-world/models" >> ~/.bashrc
