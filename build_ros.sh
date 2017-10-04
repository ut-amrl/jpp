echo "Building ROS nodes"

cd ROS/jpp
if [ ! -d "build" ]; then
  mkdir build
fi
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j