# RicohOmni
## Warning
Important, you must have nvidia graphics output if you want it to work, and not come out with a green image.


## Installation of Ricoh drivers
```bash
sudo aptitude install libgstreamer1.0-dev libusb-1.0-0-dev autopoint gtk-doc-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 
```
In case you have installed libuvc uninstall it

# Install decode Nvidia 
Video_Codec_SDK_12.0.16 works
## If is Jetson
You do nothing
## If is normal PC 
You need install plugin gstreamer1.0-plugins-bad, with: https://gist.github.com/corenel/a615b6f7eb5b5425aa49343a7b409200

# Install driver gstthetauvc
Source: https://codetricity.github.io/theta-linux/software/  and choose the option

```Using gstreamer and OpenCV without v4l2loopback```
## Dependencies for gstthetauvc
- Install libuvc-theta from this repo https://github.com/ricohapi/libuvc-theta
- Install https://github.com/nickel110/gstthetauvc
```bash
/sbin/ldconfig -v 
`````
Add to .bashrc and source:

The paths where you compiled the thetauvc:
- If Jetson
    ```echo "export GST_PLUGIN_PATH=/home/nvidia/software/gstthetauvc/thetauvc:/usr/lib/aarch64-linux-gnu/gstreamer-1.0" >> ~/.bashrc```
- If normal PC
    ```echo "export GST_PLUGIN_PATH=/home/robocomp/software/gstthetauvc/thetauvc" >> ~/.bashrc```

check with:
```bash
gst-inspect-1.0 thetauvcsrc
```

## Comands video with terminal:
- If Jetson
    - With CPU 
    - With GPU ``gst-launch-1.0 mode=2K ! queue ! h264parse ! nvv4l2decoder ! nvvidconv ! queue ! glimagesink sync=false``
- If normal PC
    - With CPU ``gst-launch-1.0 thetauvcsrc mode=2K ! queue ! h264parse ! decodebin ! queue ! autovideosink sync=false``
    - With GPU ``gst-launch-1.0 thetauvcsrc mode=2K ! queue ! h264parse ! nvdec ! queue ! glimagesink sync=false ``


## Configuration parameters
As any other component, *RicohOmni* needs a configuration file to start. In
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
cd <RicohOmni's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/RicohOmni config
```
