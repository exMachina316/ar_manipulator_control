# Gestures tracker

Gestures dictionary needs to be adjusted in `mr_manipulator/mr_manipulator/hand_drawing.py` (`self.labels_dict`) according to how many gestures you've used for classification.

Data collection is done using the DepthAI file right now, because the `/dev/video0` file was not being created for any attached camera due to unavailability of the driver. The issue can be fixed by installing the `depthai-v4l2` bridge from Luxonis' GitHub repository, but it was easier to just use the DepthAI-SDK for data collection as that is the recommended way by OAK developers.

## 3 Gestures (Current Usage)
0 -> Pointer
1 -> Peace
2 -> Hold

## 5 Gestures (Not being used in the main launch files)
0 -> Pointer
1 -> Peace
2 -> Thumbs Up
3 -> Thumbs Down
4 -> Hold