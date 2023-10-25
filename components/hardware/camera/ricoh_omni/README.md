# RicohOmni

Installation of Ricoh drivers

sudo aptitude install libgstreamer1.0-dev libusb-1.0-0-dev autopoint gtk-doc-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 
# Install plugin gstreamer1.0-plugins-bad

Install de NVidia GStreamer plugin with: https://gist.github.com/corenel/a615b6f7eb5b5425aa49343a7b409200
or
unzip  Video_Codec_SDK_12.0.16.zip in ~/software/ and follow instructions in corenel's git hub

# Install driver gstthetauvc
Goto: https://codetricity.github.io/theta-linux/software/  and choose the option

```Using gstreamer and OpenCV without v4l2loopback```


## Dependencies for gstthetauvc
Install libuvc-theta from this repo https://github.com/ricohapi/libuvc-theta

Remove system version
```bash
/sbin/ldconfig -v 
```

Install https://github.com/nickel110/gstthetauvc

Add to .bashrc and source:
    ```export GST_PLUGIN_PATH=/home/robocomp/software/gstthetauvc/thetauvc```



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
