Falcon Library installation in Linux

cd to your usual software directory
git clone https://github.com/qdot/libnifalcon.git
mkdir build
cd build
cmake ..
make
sudo make install

//Old stuff

Download https://sourceforge.net/projects/libnifalcon/files/libnifalcon/1.0.1/libnifalcon-1.0.1.tar.gz/download
untar, compile 
Ther will be some errors but copy libnifalcon.a to /usr/local/lib
copy recursively from the top directory the contents of "include" to /usr/local/include
go!
