/** @mainpage CGR (Laser Scanner, Kinect - FSPF) Localization
*
* @section intro 1. Summary
* This package provides the code for CGR localization to localize a robot in 2D using either 
* laser rangefinder readings or depth images obtained from Kinect-style sensors.
* - Authors: Joydeep Biswas (joydeepb AT ri DOT cmu DOT edu), Brian Coltin (bcoltin AT ri DOT cmu DOT edu)
* - License: LGPL
* - Source: hg http://hg.cobotrobots.com/cgr_localization
*
* @section depend 2. Dependencies
* To install all dependencies on Ubuntu, run "./InstallPackages", or copy & run the following command:
* @verbatim sudo apt-get install g++ libqt4-dev cmake libpopt-dev libusb-1.0-0-dev liblua5.1-dev libglew1.5-dev libeigen3-dev @endverbatim
* On other platforms, you will have to manually install the following packages:
* - A C++ compiler like <a href="http://gcc.gnu.org/">GCC</a>
* - <a href="http://www.cmake.org/cmake/resources/software.html">cmake</a>
* - <a href="http://qt.nokia.com/downloads">QT</a>
* - <a href="http://freecode.com/projects/popt">popt</a>
* - <a href="http://libusb.org/">libusb</a>
* - <a href="http://www.lua.org/">LUA</a>
* - <a href="http://glew.sourceforge.net/">GLEW</a>
* - <a href="http://glew.sourceforge.net/">eigen3</a>
*
* @section compile 3. Compiling
* Run "make" in the cgr_localization folder OR 
* run "rosmake cgr_localization" after adding the full path of the cgr_localization package to the ROS_PACKAGES_PATH environemt variable.
*
* @section test 4. Testing with demo data
* The following demo data sets are available:
* -# CoBot2 in GHC7 (LIDAR)
*   - Download the ROS bag file from http://cobotrobots.com/data/cgr_localization/cobot2_ghc7_lidar.bag
*   - Modify the file config/localization_parameters.cfg, line 3 to @verbatim mapName = "GHC7"; @endverbatim
* -# CoBot2 in GHC7 (Kinect)
*   - Download the ROS bag file from http://cobotrobots.com/data/cgr_localization/cobot2_ghc7_kinect.bag
*   - Modify the file config/localization_parameters.cfg, line 3 to @verbatim mapName = "GHC7"; @endverbatim
* -# CoBot2 in NSH4 (LIDAR)
*   - Download the ROS bag file from http://cobotrobots.com/data/cgr_localization/cobot2_nsh4_lidar.bag
*   - Modify the file config/localization_parameters.cfg, line 3 to @verbatim mapName = "NSH4"; @endverbatim
*
* To run the code with the demo data:
* -# Use @verbatim roslaunch cgr_localization cgr_demo_laser.launch @endverbatim to run CGR localization using LIDAR observations and display that on the bundled GUI, 
*   OR @verbatim roslaunch cgr_localization cgr_demo_kinect.launch @endverbatim to run CGR localization using Kinect depth image observations.
* -# Play back the data file with @verbatim rosbag play <bagfile.bag> --clock @endverbatim 
*
* @section robot 5. Running on your own robot
* -# <b>Creating A Vector Map</b>: 
*   To run cgr localization on your own robot, you need to generate a vector map of your environment. For an example vector map, see 
*   maps/GHC7/GHC7_vector.txt . Each entry in the vector map represents a line (wall) in the world, and is of the form:
*   @verbatim x1, y1, x2, y2 @endverbatim where x1, y1 is the start location and x2, y2 the end location of the line. The vector maps reside in 
*   the maps folder. For example, the map for "GHC7" resides in the file maps/GHC7/GHC7_vector.txt . Once the map is created, 
*   it needs to be added to maps/atlas.txt so that cgr_localization loads it on startup.
* -# <b>Generate Map Visibility Lists</b>:
*   CGR localization uses analytic raycasts on vector maps for the observation functions. To speed up the analytic renders at runtime,
*   visibility lists are pre-computed for the vector maps. To generate the visibility list for your map,
*   run the following command:
 @verbatim 
./bin/pre_render -m"YourMapName" [ -n NumberOfThreads ]
@endverbatim
*   For example:
@verbatim 
./bin/pre_render -m"GHC7" -n8
@endverbatim
* -# <b>ROS topics</b>:
*   The required ROS topics are "odom" of type <a href="http://www.ros.org/doc/api/nav_msgs/html/msg/Odometry.html">nav_msgs/Odometry</a> ,
*   and "scan" of type <a href="http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html">sensor_msgs/LaserScan</a> and/or 
*   "kinect_depth" of type <a href="http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html">sensor_msgs/Image</a> 
*   (Raw 16-bit depth data gathered via libfreenect from the kinect
*
*
* @section cite Publications
* -# "Corrective Gradient Reﬁnement for Mobile Robot Localization", Joydeep Biswas, Brian Coltin, and Manuela Veloso, 
*     Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems, September, 2011, pp. 73 - 78. 
*     <a HREF="http://joydeepb.com/Publications/iros2011_cgr.pdf">PDF</a>
* -# "Depth Camera Based Indoor Mobile Robot Localization and Navigation", Joydeep Biswas and Manuela Veloso, 
*     Proceedings of IEEE International Conference on Robotics and Automation, May, 2012. 
*     <a HREF="http://joydeepb.com/Publications/icra2012_kinectLocalization.pdf">PDF</a>
*/
