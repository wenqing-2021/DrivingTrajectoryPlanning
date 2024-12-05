## Install Docker && Build images
### 1. Install docker
Install the [docker](https://www.docker.com/).
### 2. Optinal: Install nvidia-container-toolkit
**Note**: if you want to use gpu in your docker, you may need to install the [nvidia-container-toolkit](https://github.com/NVIDIA/nvidia-container-toolkit).Here is the common way to install it:

before you start, you need to check your NVIDIA DRIVER by running the following command:
```bash
nvidia-smi
```
run the following commond to configure it:
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && \
    sudo apt-get update
```
and then, install the nvidia-container-toolkit:
```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

you can check the installation by running the following command:
```bash
sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

### 3. Build Images && Container
In the root folder, run the following command in the terminal to build the images:
```bash
python3 docker/run_docker.py --build
```

after you have builded your image, the program will automaticly run the container. You can new a container based on the image. BUT, you need to type the container name. IF you want to use GPU in your container, you MUST add the string '_gpu' into your container name. Example: "my_container_gpu".

### 4. Optinal: Enter the Container
enter the containers, you can run the following command:
```bash
python3 docker/run_docker.py --enter
```

### 5. Optinal: Install Extensions
after enter the container, you can run the following command to install the extension. You can read the `install_extensions.sh` to see the details and remove some extensions you don't need.
```bash
python3 docker/install_extensions.py -i
```