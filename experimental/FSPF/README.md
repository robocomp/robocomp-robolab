```
```
 Fast Sampling Plane Filtering (FSPF) is a RANSAC based algorithm for extracting 3D points corresponding to planar features, given a depth image.
 The plane filtered points maybe used for localization, or to build polygon maps of environments. See:
 Depth Camera Based Indoor Mobile Robot Localization and Navigation, Joydeep Biswas, and Manuela Veloso.
 In Proceedings of ICRA'12, the IEEE International Conference on Robotics and Automation, St. Paul, MN, May 2012.
 Source: hhtp://www.cs.cmu.edu/~coral/projects/localization/source.html
``` peopleCounterRGBD
```
Intro to component here


## Configuration parameters
As any other component,
``` *peopleCounterRGBD* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <peopleCounterRGBD 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```peopleCounterRGBD ```

    --Ice.Config=config
