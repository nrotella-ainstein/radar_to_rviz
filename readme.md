## radar_to_rviz:
ROS nodes for AInstein's Kanza(K77), and Tipi(T79) radars .  These nodes publish radar data to a [visualization_msgs::markerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html) topic named either 'kanza_marker_array', or 'tipi_marker_array'. These nodes receive CAN frames from the"received_messages" topic. commands to the radar are published to the "sent_messages" topic.  The data structure used for the CAN Frame is the [can_msgs::Frame](http://docs.ros.org/api/can_msgs/html/msg/Frame.html) msg. These topics are provided and managed by the deafult settings of the [socketcan_bridge_node](http://wiki.ros.org/socketcan_bridge?distro=lunar). This node is included in the [ros_canopen package](http://wiki.ros.org/ros_canopen?distro=lunar).  Default firmware will only work with one radar per a CANBUS.

## Quick Set Up:
Open a terminal and navigate to your catkin workspace, the following commands will download and install the ROS nodes for radar communication and CANBUS transport.


- cd src/
- git clone https://github.com/AinsteinAI/radar_to_rviz
- git clone https://github.com/ros-industrial/ros_canopen
- cd ..
- source devel/setup.bash
- rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
- catkin_make
- catkin_make install
- source install/setup.bash


For CANBUS mangement execute:

- rosrun socketcan_bridge socketcan_bridge_node
##### NOTE: by default socketcan_bridge_node binds to CAN0


For publishing radar data to topic execute:

For Kanza(K77):
- rosrun radar_to_rviz kanza_to_rviz_node

For Tipi (T79):
- rosrun radar_to_rviz tipi_to_rviz_node

## Can IDs:

For more detailed information concerning the CAN Frames, and CAN IDs, please reference the 'Getting Started' documentation you received with your radar.

## Configuring output of 'Tracked Detections' and/or 'Raw Detections':
*This functionality is currently only available on T-79* 
The code currently defaults to 'Raw Detections Only' output.

The T-79 Short range radar has the following output options.

- Raw Detections Only:  In the file 'src/tipi_to_rviz.cpp' adjust lines #53 - 55 as so:
```
    //you can adjust the returns that the radar puts to the CANBUS
    //start_radar_dual();
    //start_radar_tracked();
    start_radar_raw();
```

- Tracked Detections Only: In the file 'src/tipi_to_rviz.cpp' adjust lines #53 - 55 as so:
```
    //you can adjust the returns that the radar puts to the CANBUS
    //start_radar_dual();
    start_radar_tracked();
    //start_radar_raw();
```

- Raw and Tracked Detections: In file 'src/tipi_to_rviz.cpp' adjust lines #53 - 55 as so:
```
    //you can adjust the returns that the radar puts to the CANBUS
    start_radar_dual();
    //start_radar_tracked();
    //start_radar_raw();
```

#### After making any adjustments you will have to recompile and source your nodes using 
- catkin_make
- catkin_make install
- source install/setup.bash


