```
```
#
``` activityRecognition
```
This component predicts human activity based on the sequence of human skeletons (set of a human joints, can also be referred to as the pose).  
The component implements interface ActivityRecognition. It provides two functions: addSkeleton() which expects an array of shape (3, 15) for xyz positions of 
15 joints, and getCurrentActivity(), which returns top 1 prediction of the activity.  
After the component receives its first 4 skeletons, it runs inference and prints its top 5 predictions. After that it will produce inference after each 1 skeleton added.  
It is possible to run the component in the debug mode without other components by setting the flag ```_DEBUG``` in the specificworker.py to True. It will read one test file and produce inference for it.  
For normal functioning relies on the input from activityRecognitionClient.

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

Make sure that number of the port of PoseEstimation.Endpoints is the same as the corresponding number of the client component using the poseEstimation component.


    
## Starting the component
To run the code from activityRecognition directory:

```shell
python src/activityRecognition.py --Ice.Config=etc/config
```
