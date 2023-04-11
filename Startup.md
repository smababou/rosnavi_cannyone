# CannyOne Full System Startup

## 1st Terminal (CannyOne ROS Master)
### (Launch firstly all commands in this terminal)

Login to CannyOne-Raspi, using the following command:
> ssh ubuntu@pi-nhs-0005

Launch the 1st Docker image, using the following command:
> source cannyone_docker_bringup.sh

Once the Docker image is launched, type the following command:
> source cannyone_bringup.sh


## 2nd Terminal (ARUCO Localization)
Login to CannyOne-Raspi, using the following command:
> ssh ubuntu@pi-nhs-0005

Launch the 2nd Docker image, using the following command:
> source cannyone_docker_localization.sh

Once Docker image is launched, type the following command:
> source cannyone_localization.sh

wait until see the following msgs:
```
[ INFO] [1629545740.161342813]: First marker with ID: 0 detected
[ WARN] [1629545740.210164807]: TF to MSG: Quaternion Not Properly Normalized
```
then wait until all ROS nodes are launched in the `1st Terminal`(## 1st Terminal (CannyOne ROS Master))


## 3rd Terminal (RVIZ)
Export CannyOne AGV ROS_MASTER_URI, using the following command:
> export ROS_MASTER_URI=http://pi-nhs-0005:11311

Then launch the rviz configuration file, using the following command:
> roslaunch cannyone_maps cannyone_rviz.launch


## 4th Terminal (extra)
Login to CannyOne-Raspi, using the following command:
> ssh ubuntu@pi-nhs-0005

To Publish NAVI-TYPE, by copying the following command:
```
rostopic pub -1 /eCMD_t cannyone_navigation/CannyOnexNaviType 
"header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'base_footprint'
xNaviType: 'NAVI_FLYER'"
```
planner is ready once see in `1st Terminal`, the following msg:
```
[INFO] [1629545856.000475]: /cannyone_localization_node
 CannyOne New NaviType [NAVI_FLYER]
[DEBUG] [1629545856.753429]: /cannyone_localization_node
 Communication data ready Topic[xNaviLog] DataTyp[[8]] RegVal[bytearray(b'\x07')]
[DEBUG] [1629545856.766678]: /cannyone_localization_node
 Communication MSG[bytearray(b'\xa1')]
```

To Request a path for a specific point, by copying the following command:
```
rosservice call /cannyone_planner "goal:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose:
    position:
      x: 5.0
      y: 3.51
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0"
```