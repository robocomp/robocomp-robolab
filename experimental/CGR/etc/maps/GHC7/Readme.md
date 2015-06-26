# Creating A Vector Map:

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
