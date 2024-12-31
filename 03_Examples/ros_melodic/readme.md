# ROS1串列埠例程

	本文件介紹如何在ROS下來讀取超核慣導產品的數據，並提供了c++語言例程程式碼，通過執行ROS命令，執行相應的節點，就可以看到列印到終端上的資訊。

* 測試環境：Ubuntu18.04   

* ROS版本：ROS Melodic Morenia

* 測試裝置：CH0x0系列 HI14系列 CH10x系列

## 安裝USB-UART驅動

	Ubuntu 系統自帶CP210x的驅動，預設不需要安裝串列埠驅動。將除錯版連線到電腦上時，會自動識別裝置。識別成功后，會在dev目錄下出現一個對應的裝置:ttyUSBx

檢查USB-UART裝置是否被Ubuntu識別：

1. 打開終端，輸入`ls /dev`,先檢視已經存在的串列埠裝置。
2. 檢視是否已經存在  ttyUSBx 這個裝置檔案，便於確認對應的埠號。x表示USB裝置號，由於Ubuntu USB裝置號為從零開始依次累加，所以多個裝置每次開機後設備號是不固定的，需要確定裝置的裝置號。
3. 接下來插入USB線，連線除錯板，然後再次執行`ls /dev`。 dev目錄下多了一個裝置`ttyUSB0`：

```shell
linux@ubuntu:~$ ls /dev
.....
hpet             net           tty11     tty4   ttyS0      ttyUSB0    vhost-vsock
hugepages        null          tty12     tty40  ttyS1      udmabuf  vmci
......
```

	4.打開USB裝置的可執行許可權：

```shell
   $ sudo chmod 777 /dev/ttyUSB0
```

##  編譯serial_imu_ws工作空間

1. 打開終端進入serial_imu_ws 目錄
2. 執行`catkin_make`命令，編譯成功后出現完成度100%的資訊。
3. 如果是其他的ROS1系統，只需要把`serial_imu_ws/src/`下的`hipnuc_imu`資料夾移動到其他ROS1的工作空間下，直接編譯就可以了。

##  修改串列埠波特率和裝置號

1. 在Ubuntu環境中，支援的波特率為115200, 460800, 921600。本例程使用的預設波特率是115200，預設打開的串列埠名稱是/dev/ttyUSB0。	

2. 如果您需要更高的輸出頻率，請編輯`hipnuc_imu/config/hipnuc_config.yaml`檔案，修改如下兩個參數：

   imu_serial:IMU對應的裝置檔名稱

   baud_rate:IMU的波特率

```c
# config
imu_serial: "/dev/ttyUSB0"
baud_rate: 115200
frame_id: "base_link"
imu_topic: "/IMU_data"

#hipnuc data package ---> 0x91 
frame_id_costom: "base_0x91_link"
imu_topic_costom: "/imu_0x91_package"
```

修改完之後，儲存，使新配置生效。

## 顯示數據

本例程提供了兩種檢視數據方式：

1. 列印ROS標準imu.msg 數據
2. rviz工具實現視覺化

### 	輸出ROS標準 Imu.msg

	1.打開一個終端，執行：

```shell
linux@ubuntu:~$ roslaunch hipnuc_imu imu_msg.launch
```

	2.如果執行失敗，提示找不到相應的launch檔案，則需要配置環境，在目前終端執行：

```shell
linux@ubuntu:~$source <serial_imu_ws_dir>/devel/setup.bash
```

	3.執行成功后，就可以看到所有的資訊：

```txt
header: 
  seq: 595
  stamp: 
    secs: 1595829903
    nsecs: 680423746
  frame_id: "base_link"
orientation: 
  x: 0.0663746222854
  y: -0.611194491386
  z: -0.17232863605
  w: 0.769635260105
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: 
  x: 0.0851199477911
  y: 0.0470183677971
  z: 0.00235567195341
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: 
  x: 0.93323135376
  y: 0.317857563496
  z: 0.247811317444
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

```

### rviz視覺化

	1、打開終端，執行:

```shell
linux@ubuntu:~$roslaunch hipnuc_imu imu_rviz.launch
```

	2、先點選左下角的`Add`標籤，然後在彈出視窗中，選擇 `By display type`標籤，查詢`rviz_imu_plugin`；找到之後，選擇它下面的`imu`標籤，點選OK, 這時，我們可以看到rviz的左側的展示視窗中已經成功新增上了Imu的標籤。在`FixedFrame`中填入**base_link** 。`topic`中新增 **/IMU_data**。這時，可以看到座標系隨感測器改變而改變。

<img src="img/4.png">
