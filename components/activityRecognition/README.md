```
```
#
``` activityRecognition
```
This component predict human activity based on the sequence of human skeletons (set of a human joints, can also be referred to as the pose).  
The component implements interface ActivityRecognition. It provides two functions: addSkeleton() which expects a numpy array of shape (3, 15) for xyz positions of 
15 joints, and getCurrentActivity(), which returns top 1 prediction of the activity.  
After the component receives its first 4 skeletons, it runs inference and prints its top 5 predictions. After that it will produce inference after each 1 skeleton added.

## Requirements

python2  
numpy  
matplotlib  
scikit-learn

## Configuration parameters
As any other component,
``` *activityRecognition* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <activityRecognition 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```activityRecognition ```

    --Ice.Config=config
