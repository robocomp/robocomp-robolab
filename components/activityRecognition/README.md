```
```
#
``` activityRecognition
```
This component predicts human activity based on the sequence of human skeletons (set of a human joints, can also be referred to as the pose).  
The component implements interface ActivityRecognition. It provides two functions: addSkeleton() which expects an array of shape (3, 15) for xyz positions of 
15 joints, and getCurrentActivity(), which returns a json string of top activities with their probabilities.  
The component requires a minimum of 32 skeletons to produce a prediction.

## Requirements

python2  

then install with pip (pip install name):  

numpy  
matplotlib  
PyTorch (>=0.4.0)
torchnet
openpyxl  


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

## Additional Information

The component also contains specificworkerSVM.py which can be used instead of the default specificworker.py. It uses SVM instead of CNN. In this case Scikit-Learn library is required. However the SVM classifier does not generalize well.