# lidar-camera-fusion

# 安装
 - 拷贝这个库并编译
   ```Shell
   catkin_make -DCATKIN_WHITELIST_PACKAGES="aruco;aruco_ros;aruco_msgs"
   catkin_make -DCATKIN_WHITELIST_PACKAGES="aruco_mapping;lidar_camera_calibration"
   catkin_make -DCATKIN_WHITELIST_PACKAGES=""
   ```

# 使用说明
## 1.相机内参矩阵标定
## 1.1.准备
### usb_cam
 - 查看相机编号
   ```Shell
   ls /dev/video*
   ```
 - 查看相机参数
   ```Shell
   v4l2-ctl -d /dev/video1 --all
   ```
 - 查看相机支持的分辨率及帧率
   ```Shell
   v4l2-ctl -d /dev/video1 --list-formats-ext
   ```
 - 修改`usb_cam/launch/usb_cam-test.launch`
   ```Shell
   <param name="video_device" value="/dev/video1" />
   <param name="image_width" value="640" />
   <param name="image_height" value="480" />
   <param name="pixel_format" value="mjpeg" />
   ```
    - image_width和image_height可以自主设置，不同分辨率对应不同标定结果。
### camera_calibration
   ```Shell
   rosdep install camera_calibration
   ```
### image_proc
   ```Shell
   rosdep install image_proc
   ```
### 棋盘格标定板
 - http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf
## 1.2.标定过程
 - 启动原始图像节点
   ```Shell
   roslaunch usb_cam usb_cam-test.launch
   ```
 - 原始图像话题
   `/usb_cam/image_raw`
 - 启动标定程序
   ```Shell
   rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.06 image:=/usb_cam/image_raw camera:=/usb_cam
   ```
    - size：有效方格顶点数量，有效方格顶点指的是四个方格公共交点
    - square：方格边长，单位m
 - 采集样本数据时，应使标定板以尽可能多的位置和角度出现在相机视野中，每一个位置下都要保持标定板不动直到标定窗口中的图像高亮。
 - 标定窗口中的各指标
    - X：表示标定板在视野中的左右位置
    - Y：表示标定板在视野中的上下位置
    - Size：标定板在视野中所占尺寸大小
    - Skew：标定板在视野中倾斜程度
 - 移动标定板时，标定窗口中的各指标增加。
 - 当CALIBRATE按钮亮起时，单击计算相机内参矩阵及矫正参数，并输出至终端，此过程可能需要1-2分钟。
 - 当SAVE按钮亮起时，单击将采集的样本图片和计算结果保存至/tmp/calibrationData.tar.gz。
 - 当COMMIT按钮亮起时，单击将标定结果写入~/.ros/camera_info/head_camera.yaml。
 - 标定文件如下
   ```Shell
   image_width: 640
   image_height: 480
   camera_name: head_camera
   camera_matrix:
     rows: 3
     cols: 3
     data: [840.7753563595409, 0, 335.526788814159, 0, 840.5589177686173, 202.4963842754806, 0, 0, 1]
   distortion_model: plumb_bob
   distortion_coefficients:
     rows: 1
     cols: 5
     data: [-0.3129866465230049, 0.06509316296498795, -0.0008767908010405245, 0.001404493264689929, 0]
   rectification_matrix:
     rows: 3
     cols: 3
     data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
   projection_matrix:
     rows: 3
     cols: 4
     data: [799.3496704101562, 0, 337.8454315774161, 0, 0, 816.1021728515625, 200.2329448380315, 0, 0, 0, 1, 0]
   ```
 - 参数含义如下
   ```Shell
   camera_matrix:
         [fx  0 cx]
     K = [ 0 fy cy] 为矫正畸变前的相机内参矩阵。
         [ 0  0  1]
   distortion_model:
     plumb_bob 为径向和切向畸变模型。
   distortion_coefficients:
     [k1, k2, t1, t2, k3] 为畸变系数。
   projection_matrix:
         [fx'  0  cx' Tx]
     P = [ 0  fy' cy' Ty]
         [ 0   0   1   0]
   其中：
         [fx'  0  cx']
     K'= [ 0  fy' cy'] 为矫正畸变后的相机内参矩阵。
         [ 0   0   1 ] 
   单目相机中Tx、Ty均为0，双目相机中[Tx Ty 0]'与两相机坐标转换相关。
   ```
 - 启动矫正图像节点
   ```Shell
   ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
   ```
 - 矫正图像话题
   `/usb_cam/image_rect_color`
## 1.3.参考
 - https://blog.csdn.net/qq_36804363/article/details/89269776
 - https://blog.csdn.net/qq_36804363/article/details/90134707
 - https://blog.csdn.net/qq_30460905/article/details/80390576
 - http://wiki.ros.org/usb_cam
 - http://wiki.ros.org/camera_calibration
 - http://wiki.ros.org/image_proc
 - https://github.com/dimatura/ros_vimdoc/blob/master/doc/ros-camera-info.txt

## 2.相机与激光雷达转换矩阵标定
## 2.1.准备
### 相机和激光雷达相关功能包
 - 支持velodyne激光雷达和hesai激光雷达
### aruco_ros功能包、aruco_mapping功能包、lidar_camera_calibration功能包
 - 修改`lidar_camera_calibration/conf/config_file.txt`
   ```Shell
   640 480 #相机分辨率
   -2.0 2.0 #点云范围限制，根据相机坐标系X方向筛选
   -2.0 2.0 #点云范围限制，根据相机坐标系y方向筛选
   0 3.0    #点云范围限制，根据相机坐标系z方向筛选
   0.0004
   2
   0
   799.349670 0.000000 337.845432 0.000000
   0.000000 816.102173 200.232945 0.000000
   0.000000 0.000000 1.000000 0.000000
   100
   1.57 -1.57 0
   0 0 0
   0
   ```
 - 修改`lidar_camera_calibration/conf/lidar_camera_calibration.yaml`
   ```Shell
   ﻿lidar_camera_calibration:
     camera_frame_topic: /usb_cam/image_rect_color
     camera_info_topic: /usb_cam/camera_info
     velodyne_topic: /velodyne_points
   ```
 - 修改`lidar_camera_calibration/conf/marker_coordinates.txt`
   ```Shell
   2
   50
   50
   4.9
   4.9
   25.2
   50
   50
   4.9
   4.9
   25.2
   ```
 - 修改`aruco_mapping/launch/aruco_mapping.launch`
   ```Shell
   <remap from="/image_raw" to="/usb_cam/image_rect_color"/>	
   <param name="calibration_file" type="string" value="$(find aruco_mapping)/data/head_camera.ini" />
   <param name="num_of_markers" type="int" value="2" />
   <param name="marker_size" type="double" value="0.252"/>
   ```
 - 编写`aruco_mapping/data/head_camera.ini`
   ```Shell
   [image]
   width
   640
   height
   480
   [camera]
   camera matrix
   840.775356 0.000000 335.526789
   0.000000 840.558918 202.496384
   0.000000 0.000000 1.000000
   distortion
   -0.312987 0.065093 -0.000877 0.001404 0.000000
   rectification
   1 0 0
   0 1 0
   0 0 1
   projection
   799.349670 0.000000 337.845432 0.000000
   0.000000 816.102173 200.232945 0.000000
   0.000000 0.000000 1.000000 0.000000
   ```
### velodyne激光雷达和hesai激光雷达兼容问题
 - 标定程序默认采用velodyne激光雷达，如果采用hesai激光雷达，则需做出特定修改。
 - 修改`lidar_camera_calibration/conf/config_file.txt`
   ```Shell
   ...
   
   1.57 -1.57 0
   0 0 0
   1
   ```
 - 修改`lidar_camera_calibration/conf/lidar_camera_calibration.yaml`
   ```Shell
   ﻿lidar_camera_calibration:
     camera_frame_topic: /usb_cam/image_rect_color
     camera_info_topic: /usb_cam/camera_info
     velodyne_topic: /pandar_points
   ```
 - 修改`lidar_camera_calibration/include/lidar_camera_calibration/PreprocessUtils.h`
   ```C++
   pcl::PointCloud<myPointXYZRID> intensityByRangeDiff(pcl::PointCloud<myPointXYZRID> point_cloud, config_settings config){
      std::vector<std::vector<myPointXYZRID*>> rings(40); // 激光雷达线程数
   ```
 - 修改后需重新编译。
### 二维码标定板
 - https://chev.me/arucogen/
 - 放置标定板时，两个标定板间距约0.5m，标定板与传感器高度相近且间距约2.5m。
## 2.2.标定过程
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
 - 启动aruco_mapping
   ```Shell
   roslaunch aruco_mapping aruco_mapping.launch
   ```
    - 自动弹出mono8窗口，显示对标定板中二维码的检测情况，并将信息输出至终端。
 - 启动lidar_camera_calibration
   ```Shell
   roslaunch lidar_camera_calibration find_transform.launch
   ```
    - 自动弹出cloud窗口和polygon窗口，此时需要在cloud窗口中点击绘制标定板各边的包络四边形。
    - 每次点击需要按键确认，包络四边形会在polygon窗口中显示。
    - 当绘制好所有包络四边形后，程序将自动计算Average translation、Average rotation、Final rotation。
    - 计算结果将输出至终端，此时需要记录Average translation和Final rotation。
## 2.3.参考
 - https://github.com/ankitdhall/lidar_camera_calibration
 - http://wiki.ros.org/aruco_mapping

## 3.测试联合标定效果
## 3.1.准备
### lidar_camera_fusion功能包
 - 修改`lidar_camera_fusion/launch/lidar_camera_fusion.launch`
   ```Shell
   <param name ="image_topic" value="/usb_cam/image_rect_color"/>
   <param name ="lidar_topic" value="/velodyne_points"/>
   <param name ="calibration_file_path" value="$(find lidar_camera_fusion)/conf/head_camera.yaml"/>
   <param name ="the_sensor_height" value="1.0"/>
   <param name ="the_view_higher_limit" value="2.0"/>
   <param name ="the_view_lower_limit" value="-2.0"/>
   <param name ="the_min_distance" value="2.0"/>
   <param name ="the_max_distance" value="50.0"/>
   <param name ="jet_color" value="22.5"/>
   ```
    - image_topic指明订阅的相机话题。
    - lidar_topic指明订阅的激光雷达话题。
    - the_sensor_height指明传感器距地面高度，单位为米。
    - the_view_higher_limit/lower_limit指明期望的点云相对地面的限制高度，单位为米。
    - the_min_distance/max_distance指明期望的点云相对传感器的限制距离，单位为米。
    - jet_color与点云成像颜色有关。
 - 编写`lidar_camera_fusion/conf/head_camera.yaml`
   ```Shell
   %YAML:1.0
   ProjectionMat: !!opencv-matrix
     rows: 3
     cols: 4
     dt: d
     data: [799.349670, 0.000000, 337.845432, 0.000000, 0.000000, 816.102173, 200.232945, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
   LidarToCameraMat: !!opencv-matrix
     rows: 4
     cols: 4
     dt: d
     data: [-0.0382366, -0.999206, -0.0111684, 0.120955, -0.0542659, 0.0132364, -0.998439, -0.00949704, 0.997794, -0.0375708, -0.054729, 0.00527886, 0, 0, 0, 1]
   RotationAngleX: 0
   RotationAngleY: 0
   RotationAngleZ: 0
   ```
 - 参数含义如下
   ```Shell
   ProjectionMat:
     该3x4矩阵为通过相机内参矩阵标定得到的projection_matrix。
   LidarToCameraMat:
     该4x4矩阵为通过相机与激光雷达转换矩阵标定得到的Average translation和Final rotation。
     例如：
     Average translation = [0.120955, -0.00949704, 0.00527886]T
                      [-0.0382366, -0.999206, -0.0111684]
     Final rotation = [-0.0542659, 0.0132364, -0.998439]
                      [0.997794, -0.0375708, -0.054729]
     则：
     LidarToCameraMat = [-0.0382366, -0.999206, -0.0111684, 0.120955]
                        [-0.0542659, 0.0132364, -0.998439, -0.00949704]
                        [0.997794, -0.0375708, -0.054729, 0.00527886]
                        [0, 0, 0, 1]
   RotationAngleX/Y/Z:
     该值是对LidarToCameraMat矩阵进行修正的旋转角度，初始应设置为0，之后根据标定效果进行细微调整，单位为度。
   ```
## 3.2.测试
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
 - 启动lidar_camera_fusion
   ```Shell
   roslaunch lidar_camera_fusion lidar_camera_fusion.launch
   ```
 - 融合图像话题
   `/image_fusion`

## 附图
   ```Shell
                               \     /    Initial rotation:
                                \ |z/     [0 -1  0]
                                 \|/      [0  0 -1]
                                  █————x  [1  0  0]
                               forward    => (pi/2, -pi/2, 0) Euler angles
                                 cam_1    Final rotation = Average rotation * Initial rotation
  
                                █████
                   |x         ██  |x ██
   [1  0  0]       |         █    |    █                 [-1 0  0]
   [0  0 -1]  z————█ cam_4  █ y———.z    █  cam_2 █————z  [0  0 -1]
   [0  1  0]                 █         █         |       [0 -1  0]
   => (pi/2, 0, 0)            ██     ██          |x      => (-pi/2, 0, pi)
                                █████
                                lidar

                             x————█       [0  1  0]
                                  |       [0  0 -1]
                                  |z      [-1 0  0]
                                 cam_3    => (pi/2, pi/2, 0)
   ```

## 补充说明
 - 建议使用小畸变定焦相机。
 - 相机与激光雷达转换矩阵标定时，若启动lidar_camera_calibration而cloud窗口中没有点云出现，应尝试改变标定板的距离、角度、高度。
 - 测试标定效果之前停止其他正在运行的节点。



