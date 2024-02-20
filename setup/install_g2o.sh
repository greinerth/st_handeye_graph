git clone https://github.com/RainerKuemmerle/g2o.git
mkdir -p g2o/build && cd g2o/build
cmake ..
make
sudo make install
cd ../../
rm -rf g2o