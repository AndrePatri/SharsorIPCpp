mkdir -p ./build

cd ./build

cmake ../SharsorIPCpp/ -DWITH_TESTS=OFF -DWITH_PYTHON=ON

make install