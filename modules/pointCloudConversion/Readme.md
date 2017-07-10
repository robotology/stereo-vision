RFModule to convert from RGB + Depth map into pointcloud.
------

Required parameters:

remoteImagePort: input source port for rgb color image
remoteDepthPort: input source port for depth image

Note: the 2 source images needs to be of the same size!!

optional parameters:

scale: scale factor for depth values -> 1 for data coming from gazebo depth plugins, 0.001 for data coming from ASUS xtion device  through OpenNI2DeviceServer device driver. (if missing is set to 1)

--roll   --> adds a custom rotation on x axis between base_link and camera_link
--pitch  --> adds a custom rotation on y axis between base_link and camera_link
--yaw    --> adds a custom rotation on z axis between base_link and camera_link

--frame_id --> name of the frame to use in ROS pointCloud2 message


How to run:

FROM XTION DEVICE:
```
roscore
yarpserver --ros
yarpdev --device OpenNI2DeviceServer
./RGBD2pointCloud --remoteImagePort /camera/rgb/image_rect_color  --remoteDepthPort /camera/depth/image --pitch 1.5 --scale 0.001
rviz  --> add pointCloud2; topic /RGBS2PointCloud/pcloud_out, fixed frame camera_link
```

FROM GAZEBO:
```
roscore
yarpserver --ros
gazebo  --> insert model with depthCamera sensor
./RGBD2pointCloud  --remoteImagePort /iCub/colorCamera  --remoteDepthPort /iCub/depthCamera --pitch 1.5 --scale 1
rviz  --> add pointCloud2; topic /RGBS2PointCloud/pcloud_out, fixed frame camera_link
```

