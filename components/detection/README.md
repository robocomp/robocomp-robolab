# Detection Components

**Detection components** handle cognitive tasks such as detection, recognition and tracking the various targets of interest, e.g faces and emotions, human joint states, objects or specialized tags. This category consists of these components:
- `activityRecognition`: component predicts human activity based on the sequence of human skeleton gesture movements(e.g set of human joints)
- `apriltagsComp`: component recognizes augmented reality (AR) April tags. April tags are AR tags designed to be easily detected by (robot) cameras.
- `colorTracking`: component is an interactive application that allows the user to track an area of color through an image stream from an RGBD sensor.
- `emotionRecognition2`: component can detect faces and recognize emotions on an image stream from any type of camera.
- `faceidentification`: component recognizes human faces and tags them according to training dataset.
- `handDetection`: component can detect and track human hands through an image stream from an RGBD sensor.
- `test`: folder consists of various testing components of the detection components.

The detail of the description, installation, and configuration of each component can be found on the respective component folder.

A directory tree to help you navigate can be found below.

```
├───activityRecognition
│   ├───dl_training
│   │   ├───experiments
│   │   │   ├───CAD-60
│   │   │   │   ├───1
│   │   │   │   │   └───HCN18
│   │   │   │   │       └───checkpoint
│   │   │   │   ├───2
│   │   │   │   │   └───HCN19
│   │   │   │   │       └───checkpoint
│   │   │   │   ├───3
│   │   │   │   │   └───HCN20
│   │   │   │   │       └───checkpoint
│   │   │   │   └───4
│   │   │   │       └───HCN19
│   │   │   │           └───checkpoint
│   │   │   └───NTU-RGB-D-CS
│   │   │       └───HCN06
│   │   │           └───checkpoint
│   │   ├───feeder
│   │   ├───model
│   │   ├───resource
│   │   │   └───NTU-RGB-D
│   │   └───utils
│   ├───etc
│   ├───src
│   │   ├───data
│   │   ├───model
│   │   └───utils
│   └───SVM_hand_crafted
│       ├───feature_extraction
│       ├───feeder
│       ├───models
│       └───support_operations
├───apriltagsComp
│   ├───etc
│   └───src
│       └───AprilTags
├───colorTraking
│   ├───etc
│   └───src
│       └───ColorDetection
├───emotionrecognition2
│   ├───assets
│   ├───CNN
│   │   ├───checkpoints
│   │   └───data
│   │       ├───angry
│   │       ├───happy
│   │       ├───neutral
│   │       ├───sad
│   │       └───surprised
│   ├───etc
│   └───src
├───extrinsic_camera_calibration
│   ├───etc
│   └───src
├───faceidentification
│   ├───assets
│   │   └───src
│   ├───etc
│   └───src
├───handDetection
│   ├───etc
│   └───src
│       ├───images
│       │   ├───depth_images
│       │   ├───masks
│       │   └───output
│       └───libs
│           └───HandDetection
│               └───resources
├───handGesture
│   ├───assets
│   ├───etc
│   └───src
├───handKeypoint
│   ├───etc
│   └───src
├───objDetection
│   ├───etc
│   └───src
├───realsensehuman
│   ├───etc
│   └───src
├───realsense_camera
│   ├───etc
│   └───src
├───simplecamerargbd_realsense
│   ├───etc
│   └───src
└───test
    ├───activityRecognitionClient
    │   ├───etc
    │   └───src
    ├───emotionrecognitionclient
    │   ├───etc
    │   └───src
    ├───faceidentificationclient
    │   ├───assets
    │   │   ├───save_model
    │   │   └───src
    │   ├───etc
    │   └───src
    └───handGestureClient
        ├───assets
        │   └───protos
        ├───etc
        └───src
```
