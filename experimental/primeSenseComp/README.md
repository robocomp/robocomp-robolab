To install Xtion drivers go to structure.io and download the .tar file
Execute install.sh


To check the driver is working, cd to ~/Openni2XXX/Tools/NiViewer and run ./NiViewer
If you access that file from a different directory, you will probably get a "Could not open device" error

If you want to use SimpleViewer you need to execute it from the parent directory.

To run the component,

    bin/primerSenseComp --Ice.Config=etc/config 
    
You should see in the console a changing line indicating the current frames per second

Any component subscribing to RGBD.ice can get the RGBD data and the transformed <xyx> point cloud from the camera reference system.
