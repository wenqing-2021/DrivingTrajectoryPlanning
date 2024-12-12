#! /bin/bash

echo "installing extensions && thirdparty ... "

ROOT_DIR=`pwd`

echo ROOT_DIR: $ROOT_DIR

function has_pkg(){
    local find_pkg=0
    for pkg_name in $1
    do
        result=$(echo $pkg_name | grep "$2")
        if [[ $result != "" ]]; then
            find_pkg=1
        fi
    done
    return $find_pkg
}

# install vscode extensions
for i in $@
do
    echo "installing $i"
    code --install-extension $i
done

# install zsh
all_shells=$(cat /etc/shells)
has_pkg "${all_shells[@]}" '/bin/zsh'
find_zsh=$?

if [[ $find_zsh == 0 ]]; then
    echo "installing zsh"
    apt-get install -y zsh
    git clone https://gitee.com/mirrors/oh-my-zsh.git ~/.oh-my-zsh
    git clone https://github.com/zsh-users/zsh-syntax-highlighting ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
    cp ~/.oh-my-zsh/templates/zshrc.zsh-template ~/.zshrc
    chsh -s /bin/zsh
    echo "successfully installed zsh"
else
    echo "has installed zsh"
fi

# install protobuf
protoc_path=$(which protoc)
if [[ $protoc_path == "" ]]; then
    echo "start installing protobuf"
    cd /root/workspace
    apt-get install autoconf automake libtool -y
    git clone https://gitee.com/mirrors/protobufsource.git protobuf
    cd protobuf
    git checkout 3.19.0
    git submodule update --init --recursive
    ./autogen.sh
    ./configure
    make -j8 && make install
    ldconfig
    pip3 install protobuf==3.19.0
    pip3 install mypy-protobuf==3.0.0
else
    echo "has installed protobuf"
fi

# # install MPI
mpicc_path=$(which mpicc)
if [[ $mpicc_path == "" ]]; then
    echo "start installing mpi4py"
    apt-get install -y libopenmpi-dev
    pip3 install mpi4py
    echo "has installed mpi4py"
else
    echo "has installed mpi4py"
fi

# install clangd-12
if command -v clangd-12 &> /dev/null
then
    echo "clangd-12 has installed"
else
    echo "clangd-12 has not installed, start to install it."
    apt-get install -y clang-12
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-12 100
fi

# install sklearn
pip_list=$(pip3 list)
has_pkg "$pip_list" 'scikit-learn'
find_sklearn=$?
if [[ $find_sklearn == 0 ]]; then
    echo "start installing sklearn"
    pip3 install scikit-learn
    echo "has installed sklearn"
else
    echo "has installed sklearn"
fi

# #install glog
cd $ROOT_DIR
folder_path="third-party"
if [ -d "$folder_path" ]; then
    echo "$folder_path exists."
    if [ -d "$folder_path/glog" ]; then
        echo "$folder_path/glog exists."
    else
        cd $folder_path &&
        git clone https://github.com/google/glog.git && cd glog &&
        git checkout tags/v0.6.0 -b v0.6.0
    fi
else
    mkdir $folder_path 
    echo "$folder_path has created."
    cd $folder_path &&
    git clone https://github.com/google/glog.git && cd glog &&
        git checkout tags/v0.6.0 -b v0.6.0
fi

# # install pybind11
cd $ROOT_DIR
if [ -d "$folder_path" ]; then
    echo "$folder_path exists."
    if [ -d "$folder_path/pybind11" ]; then
        echo "$folder_path/pybind11 exists."
    else
        cd $folder_path &&
        git clone https://gitee.com/mirrors/pybind11.git && cd pybind11 &&
        git checkout tags/v2.10.0 -b v2.10.0
    fi
else
    mkdir $folder_path
    echo "$folder_path has created."
    cd $folder_path &&
    git clone https://gitee.com/mirrors/pybind11.git && cd pybind11 &&
    git checkout tags/v2.10.0 -b v2.10.0
fi

# install jupyternotebook
pip_list=$(pip3 list)
has_pkg "$pip_list" 'notebook'
find_notebook=$?
if [[ $find_notebook == 0 ]]; then
    echo "start installing jupyter notebook"
    pip3 install notebook
    echo "has installed jupyter notebook"
else
    echo "has installed jupyter notebook"
fi