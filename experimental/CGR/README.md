# CGR (Laser Scanner, Kinect - FSPF) Localization Component


## 1. Summary

This package provides the code for CGR localization to localize a robot in 2D using either laser rangefinder readings or depth images obtained from Kinect-style sensors.

    Authors: Joydeep Biswas (joydeepb AT ri DOT cmu DOT edu), Brian Coltin (bcoltin AT ri DOT cmu DOT edu)
    License: LGPL
    Source: hg clone http://hg.cobotrobots.com/cgr_localization


## 2. Dependencies

To install all dependencies on Ubuntu, run "./InstallPackages", or copy & run the following command:
```
sudo apt-get install g++ libqt4-dev cmake libpopt-dev libusb-1.0-0-dev liblua5.1-dev libglew1.5-dev libeigen3-dev lua5.1 liblua5.1-0-dev libopencv-dev
```

On other platforms, you will have to manually install the following packages:

    A C++ compiler like GCC
    cmake
    QT
    popt
    libusb
    LUA
    GLEW
    eigen3

    
## 3. Compiling
```
    mkdir build
    cd build
    cmake ..
    make
```

## 4. Testing with demo data

    Pending


## 5. Running on your own robot

    You need binary file to prerender. You can download a tested version in https://github.com/mhaut/pre_render
    Please, read the instructions to compile.

    Creating A Vector Map: To run cgr localization on your own robot, you need to generate a vector map of your environment. For an example vector map, see maps/GHC7/GHC7_vector.txt . Each entry in the vector map represents a line (wall) in the world, and is of the form:

    x1, y1, x2, y2 

    where x1, y1 is the start location and x2, y2 the end location of the line. The vector maps reside in the maps folder. For example, the map for "GHC7" resides in the file maps/GHC7/GHC7_vector.txt . Once the map is created, it needs to be added to maps/atlas.txt so that cgr_localization loads it on startup.
    Generate Map Visibility Lists: CGR localization uses analytic raycasts on vector maps for the observation functions. To speed up the analytic renders at runtime, visibility lists are pre-computed for the vector maps. To generate the visibility list for your map, run the following command:

    ./bin/pre_render -m"YourMapName" [ -n NumberOfThreads ]

    For example:

    ./bin/pre_render -m"GHC7" -n8


    Publications

    "Corrective Gradient Reﬁnement for Mobile Robot Localization", Joydeep Biswas, Brian Coltin, and Manuela Veloso, Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems, September, 2011, pp. 73 - 78. PDF
    "Depth Camera Based Indoor Mobile Robot Localization and Navigation", Joydeep Biswas and Manuela Veloso, Proceedings of IEEE International Conference on Robotics and Automation, May, 2012. PDF






## Requisites?
```
 sudo ln -s /usr/lib/x86_64-linux-gnu/lublua5.1.so /usr/lib/liblua5.1.so 
 echo "find_package(Lua51 REQUIRED)" >> src/CMakeListsSpecific.txt
 echo "set (SPECIFIC_LIBS ${LUA_LIBRARIES})" >> src/CMakeListsSpecific.txt
```



## Configuration parameters
As any other component,
``` *CGR* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <CGR 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```CGR ```

    --Ice.Config=config
