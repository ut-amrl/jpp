echo "Configuring and building JPP..."

mkdir build || true
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j