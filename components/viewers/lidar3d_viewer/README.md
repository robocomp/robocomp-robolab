# lidar3dviewer
Intro to component here

# Installation
## Install QGLViewer
    For Qt5 (deprecated in RoboComp) 
        sudo apt install libqglviewer-dev-qt5
    For Qt6 download and compile from https://libqglviewer.com/ the library from sources
        remore Makefiles in root and QGlViewer folders and regenerate them with qmake6 *.pro

## Install cppitertools
    sudo git clone https://github.com/ryanhaining/cppitertools /usr/local/include/cppitertools
    cd /usr/local/include/cppitertools
    sudo mkdir build
    cd build
    sudo cmake ..
    sudo make install

## Configuration parameters
As any other component, *lidar3dviewer* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <lidar3dviewer's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/lidar3dviewer config
```
