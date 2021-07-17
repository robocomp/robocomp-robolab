#!/bin/sh

if [ -z "$ROBOCOMP" ]; then
  echo "Env variable ROBOCOMP is not set. Exiting...";
  exit;
fi

COMPONENTS=/home/trung/work/robocomp_example/robocomp-robolab/components

runPythonComponent() {
  comp=$COMPONENTS/$1/$2
  config_path=$comp/etc/config
  python $comp/src/$2.py $config_path
}

runPythonComponent hardware/camera camerasimple & \
sleep 3 && \
runPythonComponent detection BodyHandJointsDetector & \
sleep 10 && echo "===============activityRecognition================="
