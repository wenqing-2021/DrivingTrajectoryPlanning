# user: shijie.yuan
# email: yuansj@hnu.edu.cn
# time: 2023-10-24

import importlib
import os
import sys


def _get_params(image_name, image_tag, container_name):
    import docker

    work_file_path = os.getcwd()
    print(f"current root path is: {work_file_path}")
    work_space_name = os.path.basename(work_file_path)
    print(f"current work space is: {work_space_name}")
    params = {
        "image": image_name + ":" + image_tag,
        "name": container_name,
        "working_dir": "/root/workspace",
        "tty": True,
        "detach": True,
        "volumes": [work_file_path + ":/root/workspace/" + work_space_name],
        "entrypoint": "/bin/bash",
    }
    if "gpu" in container_name:
        params["runtime"] = "nvidia"
        params["device_requests"] = [docker.types.DeviceRequest(count=-1, capabilities=[["gpu"]])]
    return params


def _create_container_name(client):
    enter_container = True
    while enter_container:
        print("if you want use GPU, you MUST add 'gpu' in the container name")
        container_name = input("please enter the container name (q:quit): ")
        if container_name == "q":
            print("exit. NOT run container.")
            return None
        enter_container = False
        containers = client.containers.list()
        for container in containers:
            if container.name == container_name:
                print(f"container '{container_name}' exist, please enter another name")
                enter_container = True
                break
    return container_name


def _list_containers(client):
    # show the all containers
    container_names = []
    print("-------------------------\nCurrent Containers:\n-------------------------")
    try:
        containers = client.containers.list()
        for container in containers:
            container_names.append(container.name)
    except:
        print("Failed to get container names")
        return None

    print(f"id: \t container name:")
    for idx, container_name in enumerate(container_names):
        print(f"{idx} \t {container_name}")

    return container_names


def _list_images(client):
    # show the all images
    image_names = []
    print("-------------------------\nCurrent Images:\n-------------------------")
    try:
        images = client.images.list()
        for image in images:
            image_names.append(image.tags[0])
    except:
        print("Failed to get image names")
        return None

    print(f"id: \t image name:")
    for idx, image_name in enumerate(image_names):
        print(f"{idx} \t {image_name}")

    return image_names


def _check_module(module_name) -> bool:
    try:
        importlib.import_module(module_name)
        return True
    except:
        return False


def _install_module(module_name):
    if not _check_module(module_name):
        print(f"installing '{module_name}' ")
        os.system(f"pip3 install {module_name}")
    else:
        print(f"has installed '{module_name}' ")


def enter_docker():
    # run container
    _install_module("docker")
    import docker

    client = docker.from_env()
    containers = client.containers.list(all=True)
    print("-------------------------\nCurrent Containers:\n-------------------------")
    print("id: \t container name: \t container status: ")
    for idx, container in enumerate(containers):
        print(f"{idx}\t{container.name}\t{container.status}")

    # get the container name from input
    input_container_idx = input("Please input the container idx for ENTER (q: quit, n: new a container): ")
    if input_container_idx == "q":
        print("exit. NOT enter container.")
        return

    elif input_container_idx == "n":
        print("NEW a container...")
        print("choose the image idx to run...")
        image_names = _list_images(client)
        if image_names is None:
            return
        input_image_idx = input("Please input the image idx for RUN (q:quit):")
        if input_image_idx == "q":
            print("exit. NOT run container.")
            return
        if not input_image_idx.isdigit():
            print("input error. MUST input a int number")
            return
        if int(input_image_idx) >= len(image_names):
            print("input error. MUST less than the number of images")
            return
        image_name = image_names[int(input_image_idx)].split(":")[0]
        image_tag = image_names[int(input_image_idx)].split(":")[1]
        container_name = _create_container_name(client)
        if container_name is None:
            return
        params = _get_params(image_name, image_tag, container_name)
        mp_container = client.containers.run(**params)
        print(f"successfully run container '{container_name}'")
    else:
        if not input_container_idx.isdigit():
            print("input error. MUST input a int number")
            return
        if int(input_container_idx) >= len(containers):
            print("input error. MUST less than the number of containers")
            return

        container_name = containers[int(input_container_idx)].name

    print(f"start to enter container: {container_name}...")
    enter_docker_file = "docker/scripts/enter_docker.sh"
    os.system(f"sudo chmod 777 {enter_docker_file}")
    params = " " + container_name
    os.system(enter_docker_file + params)


def build_docker():
    _install_module("docker")
    import docker

    client = docker.from_env()
    build_docker_file = "docker/scripts/build_docker.sh"
    os.system(f"sudo chmod 777 {build_docker_file}")
    # enter the image name and tag
    enter_name = True
    while enter_name:
        image_name = input("please enter the image name (q:quit): ")
        if image_name == "q":
            print("exit. NOT build image.")
            return
        image_tag = input("please enter the image tag: ")
        enter_name = False
        images = client.images.list()
        for image in images:
            if image.tags[0] == image_name + ":" + image_tag:
                print(f"image '{image_name}:{image_tag}' exist, please enter another name and tag")
                enter_name = True
                break

    params = " " + image_name + " " + image_tag
    os.system(build_docker_file + params)
    print(f"successfully build image '{image_name}:{image_tag}'")
    print("entering the docker...")
    enter_docker()


def remove_images():
    _install_module("docker")
    import docker

    client = docker.from_env()

    container_names = _list_containers(client)
    if container_names is None:
        return
    # get the container name from input
    input_container_idx = input("Please input the container idx for DELETE (q: quit, c: continue): ")
    if input_container_idx == "q":
        print("exit. NOT delete container and images.")
        return
    elif input_container_idx == "c":
        print("not delete container, continue to delete image...")
    else:
        if not input_container_idx.isdigit():
            print("input error. MUST input a int number")
            return
        if int(input_container_idx) >= len(container_names):
            print("input error. MUST less than the number of containers")
            return
        input_container_name = container_names[int(input_container_idx)]
        mp_container = client.containers.get(input_container_name)
        sure_delete = input(f"Are you sure to delete container '{input_container_name}'? (y/n):")
        if sure_delete == "y":
            mp_container.stop()
            mp_container.remove()
            print(f"container '{input_container_name}' removed")
        else:
            print(f"not delete container {input_container_name}, continue to delete image...")

    image_names = _list_images(client)
    if image_names is None:
        return

    # get the image name from input
    input_image_idx = input("Please input the image idx for DELETE (q:quit):")
    if input_image_idx == "q":
        print("exit. NOT delete image.")
        return
    if not input_image_idx.isdigit():
        print("input error. MUST input a int number")
        return
    if int(input_image_idx) >= len(image_names):
        print("input error. MUST less than the number of images")
        return

    input_image_name = image_names[int(input_image_idx)]

    # mp_image = client.images.get(input_image_name)
    sure_delete = input(f"Are you sure to delete image '{input_image_name}'? (y/n):")
    if sure_delete == "y":
        client.images.remove(image=input_image_name, force=True)
        print(f"image '{input_image_name}' removed")
    else:
        print("exit. NOT delete image.")
        return


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="docker run")
    parser.add_argument("--build", "-b", action="store_true", help="build docker")
    parser.add_argument("--enter", "-e", action="store_true", help="enter docker")
    parser.add_argument("--remove", "-r", action="store_true", help="remove container and images")
    args = parser.parse_args()
    if args.build:
        build_docker()
    elif args.enter:
        enter_docker()
    elif args.remove:
        remove_images()
