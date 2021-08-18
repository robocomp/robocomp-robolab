#!/bin/bash

# echo $1
echo Preparing folders....
rm ./boxes/*

echo Performing Detection...
python3 ./yolov3/detect.py --weights ./yolov3/yolov3.pt --conf 0.25 --source $1 --save-txt

echo Generating Depth Map...
python3 ./LapDepth-release-master/demo.py --model_dir ./LapDepth-release-master/pretrained/LDRN_KITTI_ResNext101_pretrained_data_grad.pkl --img_folder_dir $1 --pretrained KITTI --cuda --gpu_num 0

echo Parsing Bounding Boxes...
python3 2d_bboxes.py --folder $1

echo Predicting 3D Locations... 
python3 3d_predict.py --folder $1 --width $2 --height $3 