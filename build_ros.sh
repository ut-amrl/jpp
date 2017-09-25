echo "Building ROS nodes"

cd ROS/jpp
mkdir build || true
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j