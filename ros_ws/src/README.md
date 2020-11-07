# lidar-camera-fusion

## 运行
 - 启动原始图像节点
   ```Shell
   roslaunch usb_cam usb_cam-test.launch
   ```
 - 启动矫正图像节点
   ```Shell
   ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
   ```
 - 启动激光雷达节点（根据所使用的激光雷达）
   ```Shell
   roslaunch velodyne_pointcloud VLP16_points.launch
   ```
   ```Shell
   roslaunch hesai_lidar p40.launch
   ```
 - 启动lidar_camera_fusion_test
   ```Shell
   roslaunch lidar_camera_fusion_test lidar_camera_fusion_test.launch
   ```
 - 融合图像话题
   `/image_fusion`



