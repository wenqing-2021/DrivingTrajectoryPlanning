#!/bin/bash  
  
# 设置Docker镜像名称和版本号  
IMAGE_NAME=${1}
IMAGE_VERSION=${2}  

# 进入Dockerfile所在目录  
current_directory=$(pwd)
echo "root path is: $current_directory"
# 使用basename命令提取文件名部分
folder_name=$(basename "$current_directory")

echo "当前终端所在的文件夹名: $folder_name"
cd ..

# 构建Docker镜像  
docker build -t $IMAGE_NAME:$IMAGE_VERSION -f ./$folder_name/docker/install_docker.dockerfile .