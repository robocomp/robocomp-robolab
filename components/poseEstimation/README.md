```
```
#
``` poseEstimation
```
This component produces 3D and 2D pose estimation based on a RGB image. getSkeleton() method returns (16, 3) and (16, 2) arrays respectively. The 16 joints are 
listed at the bottom of the file.

## Requirements

1. for Pose estimation, clone the following repository to ```*poseEstimation/src*``` directory

```shell
git clone https://github.com/xingyizhou/pytorch-pose-hg-3d.git
```
or in case of significant changes happen to the above original repository, you can clone the following fork: https://github.com/mfedoseeva/pytorch-pose-hg-3d

2. The trained model which will be used to predict the poses has to be downloaded from https://drive.google.com/file/d/1_2CCb_qsA1egT5c2s0ABuW3rQCDOLvPq/view and placed into the cloned directory ```*/pytorch-pose-hg-3d/models*```

3. The code requires numpy, pytorch, opencv and progress. Everything can be installed with pip (e.g. pip install progress)


## Configuration parameters
As any other component,
``` *poseEstimation* ```
needs a configuration file to start. In

    etc/config

Make sure that number of the port of PoseEstimation.Endpoints is the same as the corresponding number of the client component using the poseEstimation component.

    
## Starting the component

To run the code from poseEstimation directory:

```shell
python src/poseEstimation.py --Ice.Config=etc/config
```

## Joints

<!-- # 0 right_foot
# 1 right_knee
# 2 right_hip
# 3 left_hip
# 4 left_knee
# 5 left_foot
# 6 between_hips
# 7 neck
# 8 neck - this is not a typo, neck is returned twice
# 9 head
# 10 right_hand
# 11 right_elbow
# 12 right_shoulder
# 13 left_shoulder
# 14 left_elbow
# 15 left_hand -->