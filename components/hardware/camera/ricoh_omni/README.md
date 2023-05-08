# RicohOmni

Installation of Ricoh drivers

sudo aptitude install libgstreamer1.0-dev
sudo aptitude install libusb-1.0-0-dev
sudo aptitude install autopoint
sudo aptitude install gtk-doc-tools
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

Install de NVidia GStreamer plugin with: https://gist.github.com/corenel/a615b6f7eb5b5425aa49343a7b409200
or
unzip  Video_Codec_SDK_12.0.16.zip in ~/software/ and follow instructions in corenel's git hub

Goto: https://codetricity.github.io/theta-linux/software/  and choose the

    Using gstreamer and OpenCV without v4l2loopback

option, and install https://github.com/nickel110/gstthetauvc

Add to .bashrc and source:
    export GST_PLUGIN_PATH=/home/robocomp/software/gstthetauvc/thetauvc


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
