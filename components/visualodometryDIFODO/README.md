# Instructions:

For use this component, you can need install to MRPT and the openni2RGBD component locate in our Github repository robocomp-robolab/components/openni2RGBD/
```
mkdir ~/software
cd ~/software
git clone https://github.com/jlblancoc/mrpt
cd mrpt
mkdir build
cd build
ccmake ..
make
sudo make install
echo <path to MRPT> >> ~/.bashrc'
source ~/.bashrc

In openni2RGBD component:
mkdir build
cd build
cmake ..
make
bin/openni2RGBD --Ice.Config=../config


In this component:
mkdir build
cd build
cmake ..
make
bin/visualodometryDIFODO --Ice.Config=../config
```



```
```
#
``` visualodometryDIFODO
```
Intro to component here


## Configuration parameters
As any other component,
``` *visualodometryDIFODO* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <visualodometryDIFODO 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```visualodometryDIFODO ```

    --Ice.Config=config
