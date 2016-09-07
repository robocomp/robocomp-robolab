#Script to install BifTK version of apriltags

sudo apt-get install libv4l-dev libeigen3-dev
git clone https://github.com/NifTK/apriltags.git
cd apriltags
cmake .
make
sudo make install
