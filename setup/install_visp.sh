git clone https://github.com/lagadic/visp.git
mkdir -p visp/build && cd visp/build
cmake ..
make
sudo make install
cd ../../
rm -rf visp