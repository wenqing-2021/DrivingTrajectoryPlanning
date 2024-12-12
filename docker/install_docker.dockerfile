# user: shijie.yuan
# email: yuansj@hnu.edu.cn
# time: 2024-11-22

FROM ubuntu:22.04

# not interactive mode
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8 

WORKDIR /root/workspace

# 更换源
RUN sed -i s@/archive.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list
RUN sed -i s@/security.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    gdb \
    vim \
    make \
    wget \
    unzip \
    curl \
    ffmpeg \
    openssl \
    tmux \
    gfortran \
    libssl-dev \
    libboost-all-dev \
    libeigen3-dev \
    libomp-dev \
    libyaml-cpp-dev \
    python3-pip

# install cmake
RUN mkdir -p /root/workspace/cmake && \
    cd /root/workspace/cmake && \
    wget https://cmake.org/files/v3.23/cmake-3.23.0.tar.gz && \
    tar -zxvf cmake-3.23.0.tar.gz && \
    cd cmake-3.23.0 && \
    ./configure && \
    make -j8 && make install

# # 更换pip源
RUN pip3 config set global.index-url https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple

RUN pip3 install --upgrade pip

LABEL author="shijie.yuan" \
      time="2024-11-22" \
      description="docker for motion planning" \
      version="1.0" 

ENTRYPOINT ["/bin/bash"]