```
```
#
``` sickLaser
```
sickLaser LD OEM 1000


Examples and library in this [repository](https://github.com/mhaut/laserSickLD1000)

## Configuration parameters
As any other component,
``` *sickLaser* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
To get laserdata, in the client, only write
	RoboCompLaser::TLaserData laserData;
	laserData = laser_proxy->getLaserData();
	for (unsigned int i = 0; i < laserData.size(); i++)
	{
		cout << laserData[i].dist << " --- " << laserData[i].angle <<endl;
	}
    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <sickLaser 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```sickLaser ```

    --Ice.Config=config
