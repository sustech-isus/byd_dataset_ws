
# 环境配置

* 系统镜像: Ubuntu 18.04
* ROS Melodic


## 安装ROS包依赖

```bash
sudo apt update
sudo apt install libpcap0.8-dev libyaml-cpp-dev libpcl-dev libcurl4 openssl-dev curl libboost-all-dev dpkg libusb-1.0-0-dev
sudo apt install ros-melodic-geographic-msgs
```

## 安装 Flir Spinnaker SDK

下载地址: 
* [官网](https://www.flir.asia/support-center/iis/machine-vision/downloads/spinnaker-sdk-and-firmware-download/)   
* [内网](http://fs.isus.tech/library/9600aab3-9b02-4fab-b3a0-6081a934964c/%E5%B8%B8%E7%94%A8%E4%B8%8B%E8%BD%BD/byd/blackflys)

得到 `spinnaker-*-amd64-pkg.tar.gz` , 解压后执行 `install_spinnaker.sh` .

## 拉取代码并编译:

```bash
#拉取工作区代码
git clone --recursive https://github.com/sustech-isus/byd_dataset_ws
cd byd_dataset_ws
catkin_make
source devel/setup.bash

```
> 本机测试请在huaxun_radar包下添加CATKIN_IGNORE以屏蔽编译.



# 数据采集操作流程

## 硬件端操作

1. 依次开启汽车电源, 后备箱逆变器电源, 后备箱PDU电源, (等待10s左右)工控机电源, 等待所有设备初始化完成.
2. 检查交换机指示灯,相机上电情况,同步模块指示灯是否正常闪烁(10ms闪烁两次)
3. 以上步骤没有问题可以进入车内调试工控机

## 工控机端操作

```bash
# 开启PTP Master
sudo ptp4l -i enp1s0f0 -m

# 启动激光雷达节点
roslaunch hesai_lidar hesai_lidar.launch

# 启动相机节点, 相机通用参数存放在 6cam_sync_param.yaml 中
roslaunch spinnaker_camera_driver 6cam_sync.launch

# 启动毫米波雷达节点
roslaunch huaxun_radar single.launch

# 录制无压缩图片ROS Bag, 分包1分钟 (数据过大,不推荐)
rosbag record -b 4096 --chunksize=1024 --duration=1m /pandar /radar_points /radar_tracks /cameras/180_front/image_color /cameras/230_front_left/image_color /cameras/130_front_right/image_color /cameras/290_rear_left/image_color /cameras/070_rear_right/image_color /cameras/000_rear/image_color

# 录制压缩图片ROS Bag, 分包1分钟
rosbag record -b 4096 --chunksize=1024 --duration=1m /pandar /radar_points /radar_tracks /cameras/180_front/image_color/compressed /cameras/230_front_left/image_color/compressed /cameras/130_front_right/image_color/compressed /cameras/290_rear_left/image_color/compressed /cameras/070_rear_right/image_color/compressed /cameras/000_rear/image_color/compressed

# 建议开启df查看硬盘写入情况, 避免写满磁盘
watch df -h
```



