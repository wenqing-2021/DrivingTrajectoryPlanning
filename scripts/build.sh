folder_path="build"
if [ -d "$folder_path" ]; then
    echo "$folder_path exists."
else
    mkdir build
    echo "$folder_path has created."
fi
cd build &&
cmake .. &&
make -j4 install

