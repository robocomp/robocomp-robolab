```
```
#
``` faceidentification
```
Intro to component here


## Configuration parameters
As any other component,
``` *faceidentification* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <faceidentification 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```faceidentification ```

    --Ice.Config=config

## Python Libraries Required
- Numpy 1.16.4
- OpenCV 4.0.0
- Scipy 1.2.2
- Scikit Learn 0.20.3
- Tensorflow 1.13.1

