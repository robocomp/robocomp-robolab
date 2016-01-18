# pre_render

This package provides the code for CGR localization to localize a robot in 2D using either laser rangefinder readings or depth images obtained from Kinect-style sensors.

    Authors: Joydeep Biswas (joydeepb AT ri DOT cmu DOT edu), Brian Coltin (bcoltin AT ri DOT cmu DOT edu)
    License: LGPL


Creating A Vector Map: To run cgr localization on your own robot, you need to generate a vector map of your environment. For an example vector map, see maps/GHC7/GHC7_vector.txt . Each entry in the vector map represents a line (wall) in the world, and is of the form:

x1, y1, x2, y2 

where x1, y1 is the start location and x2, y2 the end location of the line. The vector maps reside in the maps folder. For example, the map for "GHC7" resides in the file maps/GHC7/GHC7_vector.txt . Once the map is created, it needs to be added to maps/atlas.txt so that cgr_localization loads it on startup.
Generate Map Visibility Lists: CGR localization uses analytic raycasts on vector maps for the observation functions. To speed up the analytic renders at runtime, visibility lists are pre-computed for the vector maps. To generate the visibility list for your map, run the following command:

./bin/pre_render -m"YourMapName" [ -n NumberOfThreads ]

For example:

./bin/pre_render -m"GHC7" -n8

