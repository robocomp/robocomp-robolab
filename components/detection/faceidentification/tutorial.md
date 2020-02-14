# Tutorial

In order to run the Face Identification Component follow the following steps:

```
cd robocomp/components/robocomp-robolab/components
```
Open 2 new terminals.

Terminal 1:
```
cd camerasimple
python src/camerasimple.py --Ice.Config=etc/config
```

Terminal 2:
```
cd faceidentification
python src/faceidentification.py --Ice.Config=etc/config
```

Terminal 3:
```
cd faceidentificationclient
python src/faceidentificationclient.py --Ice.Config=etc/config
```

Now one can see the captured frames with a bounding box and person's name on top of the bounding box. Another popup window will open which allows the user to Add/Delete labels. One can also add an image directly from the camera feed and recognize faces in the images. Images are stored in the folder `robocomp/components/robocomp-robolab/components/detection/test/faceidentificationclient`.
