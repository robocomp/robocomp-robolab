<p align="center">
  <img src="./assets/Robocomp.png" width="550" title="hover text">
</p>

# GSoC'21 RoboComp project: Computer vision for detecting elements of a vehicle’s environment with RoboComp

## About the Project

The detection of risk situations during the navigation of mobile robots is an essential task for future applications. The goal is to create a software agent in robocomp with the aim of improving vehicle driving, using deep learning and computer vision techniques.

The main idea is to use one of several RGB cameras placed in a vehicle for lane detection, pedestrian detection, vehicle detection, sign detection and more elements that affect driving. To perform this task, it is possible to work either with real datasets of cameras placed in vehicles or to use the carla vehicle driving simulator.

# **Finding Lane Lines on the Road**

When we drive, we use our eyes to decide where to go. The Lane lines on the road that shows us where the lanes are act as our constant reference for where to steer the vehicle. Naturally, one of the first things I would like to do in this project is to automatically detect lane lines using an algorithm.
In this project I detect lane lines in images using Python and OpenCV and later on video. OpenCV means “Open-Source Computer Vision”, which is a package that has many useful tools for analyzing images.
Some steps include :-

1. Convert BGR image to Gray.
2. Convert into Blurred image using Gaussian Filter(to avoid the noise)
3. Apply Canny Edge Detection into the image.
4. we define Region of interest(i.e. bottom half of the image)
5. Apply Hough Transform and smoothing the final detected lane lines.

https://user-images.githubusercontent.com/42083679/123698027-411e1680-d87b-11eb-959a-8989861c0b0f.mp4

For more details please [click here](https://github.com/robocomp/web/blob/master/gsoc/2021/posts/garv_tambi/post02.md) 



# **Advanced Lane Lines**

**Advanced Lane Finding on the Road**

Now I move towards the next very interesting steps of this project i.e Advanced Lane Line Detection using computer vision.
The video below talks about how I process the input stream and the steps involved to build an image processing pipeline that gives filling area between left lines and right lines as well as Curvature as outputs.




**Advanced Lane Finding Finding**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients of given images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

https://user-images.githubusercontent.com/42083679/125023912-6c83db00-e09d-11eb-8f9b-025388401428.mp4

For more details please [click here 
](https://github.com/robocomp/web/edit/master/gsoc/2021/posts/garv_tambi/post03.md
)

## Pedestrian detection using Yolo

Pedestrian detection in image or video data is a very important and challenging task in security surveillance. The difficulty of this task is to locate and detect pedestrians of different scales in complex scenes accurately.

To solve these problems, a deep neural network (RT-YOLOv3) is proposed to realize real-time pedestrian detection at different scales in security monitoring. RT-YOLOv3 improves the traditional YOLOv3 algorithm. Firstly, the deep residual network is added to extract vehicle features. 

Then six convolutional neural networks with different scales are designed and fused with the corresponding scale feature maps in the residual network to form the final feature pyramid to perform pedestrian detection tasks. This method can better characterize pedestrians. 

In order to further improve the accuracy and generalization ability of the model, a hybrid pedestrian data set training method is used to extract pedestrian data.


https://user-images.githubusercontent.com/42083679/128036811-ad93ce8e-d14e-412a-bf2c-7fb1751d860f.mp4


For more details please [click here](https://github.com/robocomp/web/edit/master/gsoc/2021/posts/garv_tambi/post04.md
) 

# **Finding Lane Lines on the Carla Simulator**

**Finding Lane Lines on the Road**

I run this algorithm on the Carla Simulator that I already ran on real dataset. 

When we drive, we use our eyes to decide where to go. The Lane lines on the road that shows us where the lanes are act as our constant reference for where to steer the vehicle. Naturally, one of the first things I would like to do in this project is to automatically detect lane lines using an algorithm.

I run this algorithm on the Carla Simulator that I already run on real dataset. For more details please check out my Post02.

**About the Simulator**

This simulation is carried out in Carla Simulator. The CARLA python client runs on Python 3.5.x or Python 3.6.x (x is any number). Python 3.7 is not compatible with CARLA.

https://user-images.githubusercontent.com/42083679/128772782-40a984da-7754-4d00-9a1b-30c43271d9c9.mp4


For more details please [click here](https://github.com/robocomp/web/edit/master/gsoc/2021/posts/garv_tambi/post05.md
) 

# **Advanced Lane Lines on Carla Simulator**

**Advanced Lane Finding on the Road**

The video below talks about how I process the input stream and the steps involved to build an image processing pipeline that gives filling area between left lines and right lines.


https://user-images.githubusercontent.com/42083679/128978981-fbe7a950-ed50-4118-94bb-a54130e70728.mp4

For more details please [click here ](https://github.com/robocomp/web/edit/master/gsoc/2021/posts/garv_tambi/post06.md
)

# **Pedestrian and Vehicle Detection on the Carla Simulator**

## Pedestrian and Vehicle detection using Yolov3



https://user-images.githubusercontent.com/42083679/129405414-38308de9-e001-4b26-be7e-e77b304d8007.mp4



Pedestrian detection in image or video data is a very important and challenging task in security surveillance. The difficulty of this task is to locate and detect pedestrians of different scales in complex scenes accurately.

To solve these problems, a deep neural network (RT-YOLOv3) is proposed to realize real-time pedestrian detection at different scales in security monitoring. RT-YOLOv3 improves the traditional YOLOv3 algorithm. Firstly, the deep residual network is added to extract vehicle features. 

Then six convolutional neural networks with different scales are designed and fused with the corresponding scale feature maps in the residual network to form the final feature pyramid to perform pedestrian detection tasks. This method can better characterize pedestrians. 

In order to further improve the accuracy and generalization ability of the model, a hybrid pedestrian data set training method is used to extract pedestrian data.


For more details please [click here](https://github.com/robocomp/web/edit/master/gsoc/2021/posts/garv_tambi/post07.md
) 

# **System Integration on Carla Simulator**

The video below is the integration of Lane line detection, Pedestrian detection, Vehicle detection and classifying traffic signs.

For more details please refer
Post03, 
Post04, Post07 


https://user-images.githubusercontent.com/42083679/129402245-38573385-a33d-4731-8f4b-42b338390c18.mp4


**Thank You**

