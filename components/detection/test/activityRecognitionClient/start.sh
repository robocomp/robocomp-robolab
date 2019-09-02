#!/bin/sh

if [ -z "$ROBOCOMP" ]; then
  echo "Env variable ROBOCOMP is not set. Exiting...";
  exit;
fi

COMPONENTS=$ROBOCOMP/components/robocomp-robolab/components

runPythonComponent() {
  comp=$COMPONENTS/$1
  config_path=$comp/etc/config
  python2 $comp/src/$1.py --Ice.Config=$config_path
}

runPythonComponent camerasimple & \
sleep 3 && \
runPythonComponent poseEstimation & \
sleep 10 && echo "===============activityRecognition=================" && \
runPythonComponent activityRecognition & \
sleep 20 && echo "===============activityRecognitionClient=================" && \
runPythonComponent activityRecognitionClient
