# Introduction
This project is another open source project on github SensorCalibration in lidar2camera interpretation and expansion, and this project is based on the dnet.
# Prerequisites
 * CMake  
 * PCL 1.9  
 * eigen3  
 * Pangolin
 * dnet
 * dqt  
## 1. Four input files:  
You need to play back the bag file in that link with the rosbag command to get the point cloud information and image information needed for that node. Intrinsic parameter and extrinsic parameter for initialization are in the data folder. You should download them to the appropriate path, and modify the relevant path in the cpp source code in the src folder.  
 * extrinsic_json: JSON file of initial values of extrinsic parameters between sensors
## 2. Calibration panel:
The calibration window consists of the left control panel for manual calibration and the right point projection image. Users can check whether the points cloud and the points of radar are aligned by clicking the corresponding button in the panel or using Keyboard as input to adjust the extrinsic parameter. When the points cloud and the image are aligned, the calibration ends, click the save button to save the result.

Extrinsic Params  | Keyboard_input	  | Extrinsic Params	  | Keyboard_input  
 ---- | ----- | ------ | ------  
 +x degree  | q | -x degree | a |  
 +y degree  | w | -y degree | s |  
 +z degree  | e | -z degree | d |  
 +x trans  | r | -x trans | f |  
 +x trans  | t | -y trans | g |  
 +x trans  | y | -z trans | h |  

 Intrinsic Params  | Keyboard_input	  | Intrinsic Params	  | Keyboard_input  
 ---- | ----- | ------ | ------  
 +fy  | i | -fy | k |  
 +fx  | u | -fx | j |  

`Intensity Color`: LiDAR intensity is recorded as the return strength of a laser beam, partly based on the reflectivity of the object struck by the laser pulse. This button can change the display mode to intensity map display mode. This can help to check if the ground lane lines are aligned.

`deg step t step`: fxfy scale : These three buttons change the adjustment step for every click or keyboard input.

`point size`: Adjust the size of Lidar points in the projection image.

`Reset`: Press button to reset all manual adjustment.

`Save Image`: If the this button was pressed, the results (calibrated image, extrinsic and intrinsic matrix) are stored by default at running directory ~./manual_calib/:
