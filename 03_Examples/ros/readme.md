## 單元 : ROS 序列埠範例

原始碼(瀏覽器下載並解壓 ): [CH_demo_ros.zip](https://github.com/avmm9898/hipnuctw_doc/raw/master/03_Examples/ROS/CH_demo_ros.zip)

介紹如何在ROS下來讀取HI226/HI229的數據，並提供了c++代碼，通過ROS命令，運行相應的節點，就可以看到打印到終端機上的資訊。

- 測試環境：Ubuntu16.04   

- ROS版本：ROS Kinetic Kame

- 測試設備：HI226 HI229

### Step 1. 安裝驅動與 ROS serial

Ubuntu 系統自帶 CP210x 的驅動，默認不需要安裝序列埠驅動。將 USB 模組連接到電腦上時，會自動識別設備。識別成功後，會在 `/dev`目錄下出現一個對應的設備 : `ttyUSBx`

檢查 USB-UART 設備是否被識別：

1. 打開終端，輸入

    ```
    ls /dev
    ```

    查看已經存在的序列埠設備。

2. 查看是否已經存在 `ttyUSBx` 這個設備文件，便於確認對應的端口號。x表示USB設備號，由於 Ubuntu USB 設備號為從零開始依次累加，所以多個設備每次開機後設備號是不固定的，需要確定設備的設備號。

4. 接下來插入USB線，連接調試板，然後再次執行`ls /dev`。 `dev`目錄下多了一個設備, 如圖：
![](ROSFigures/2.png)
`ttyUSB0`文件就是調試版在ubuntu系統中生成的設備(後面的數字是不固定的，有可能為 `ttyUSB`  或 `ttyUSB`)

5. 打開USB設備的可執行權限：
    ```shell
    $ sudo chmod 777 /dev/ttyUSB0
    ```

接下來需要安裝 ROS serial 軟體包 :

1. 執行命令，下載安裝 serial 套件：

    ```shell
    $ sudo apt-get install ros-kinetic-serial
    ```

2. 輸入`roscd serial`命令，進入 serial 下載位置，如果安裝成功，就會出現如下資訊：

    ```shell
    $:/opt/ros/kinetic/share/serial
    ```
3. 如果出現裝不上，可以到 Step 5 的查看建議的解決方式。

### Step 2. 編譯 serial_imu_ws 工作空間

1. 打開終端進入`/examples/ROS/serial_imu_ws` 目錄

2. 執行`catkin_make`命令，編譯成功後出現完成度 100% 的資訊。

### Step 3. 修改序列埠波特率和設備號

1. 在Ubuntu環境中，支援的波特率為115200, 460800, 921600。本範例使用的默認波特率是115200，默認打開的序列埠名稱是`/dev/ttyUSB0`。	

2. 如果您需要更高的輸出頻率，請編輯 serial_imu.cpp 文件，修改 serial_imu.cpp 文件中的宏定義，改為其他波特率。	
    ```c
    #define IMU_SERIAL ("/dev/ttyUSB0")
    #define BAUD       (115200)
    ```
    
    注意修改後需要回到 serial_imu_ws 目錄下，重新執行`catkin_make`命令

### Step 4. 顯示數據
本範例提供了三種查看數據方式：

1. 顯示所有的數據資訊，便於查看數據。
2. 打印 ROS 標準 mu_msg 數據
3. rviz 工具實現可視化

#### 4.1：輸出 IMU 原始數據

1. 打開另一個終端，執行：

    ```shell
    $ roslaunch imu_launch imu_msg.launch imu_package:=0x91
    ```
2. 如果執行失敗，提示找不到相應的 launch 文件，則需要配置環境，在當前終端執行：

    ```shell
    $source <serial_imu_ws_dir>/devel/setup.bash
    ```

3. 執行成功後，就可以看到所有的資訊：

    ```txt

         Devie ID:     0
        Run times: 0 days  3:26:10:468
      Frame Rate:   100Hz
           Acc(G):   0.933    0.317    0.248
       Gyr(deg/s):   -0.02     0.30    -0.00
          Mag(uT):    0.00     0.00     0.00
       Eul(R P Y):   52.01   -66.63   -60.77
    Quat(W X Y Z):   0.770    0.066   -0.611   -0.172

    ```



#### 4.2：輸出 ROS 標準 Imu.msg

1. 在 Windows 系統下進行配置模組，使能四元數輸出。

2. 使用 Windows 下 CHCenter 軟體進行配置 : 

    1. 把模組連接到PC後，使用 CHCenter 進行連接對應的 COM串口，點擊**模組設定**。
    2. 在協議配置區域，可以選擇單獨勾選加速度、角速度、四元數，或者是選擇建議的 **IMUSOL**(0x91)。各自寫入接收區最後顯示 **OK**，代表成功寫入在關閉配置窗口上，回到數據顯示區域，確認重啟生效。同時檢查加速度、角速度、四元數是否正確輸出。
    
3. 執行命令
   
    ```
    roslaunch imu_launch imu_msg.launch
    ```
    
    
    執行成功後，就可以看到 ROS 定義的 IMU 訊息：
    
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

#### 4.3：rviz 可視化

1. 安裝 ROS rviz 插件

2. 同上節，使模組能輸出四元數

3. 進入 serial_imu_ws，執行命令，

    ```
    roslaunch imu_launch imu_rviz.launch
    ```

    執行成功後，rviz 工具便被打開。

4. 先點擊左下角的 **Add** 標籤，然後在彈出窗口中，選擇 **By display type** 標籤，尋找 rviz_imu_plugin；找到之後，選擇它下面的 IMU標籤，點擊 **OK**, 這時，我們可以看到 rviz 的左側的展示窗口中已經成功添加上了 IMU 的標籤。在 **FixedFrame** 中填入 **base_link** 。**topic** 中添加 **/IMU_data**。這時，可以看到坐標系隨感測器改變而改變。

5. 可以從這裡下載 rviz 的工具,這是 ROS 官方的一個 rviz 插件：

    ```shell
    git clone -b indigo https://github.com/ccny-ros-pkg/imu_tools.git
    ```
![](ROSFigures/4.png)

### Step 5. 常見錯誤
如果是第一次裝 ROS serial 包，很可能會失敗，因為我們在裝的時候，遇到了這個問題，這裡把解決方法提供出來，節約大家的時間。當在終端執行`sudo apt-get install ros-kinetic-serial`這條命令的時候，有可能會提示

![](ROSFigures/5.png)

為了提供素材，serial 故意輸錯的。

一個解決辦法是：

```shell
$cd /etc/apt/sources.list.d
$sudo vi ros-latest.list
```

打開這個文件之後，一般這個文件中只有一個可用的源，就是指沒有被註釋的，現在把它註釋掉，在它的開頭輸入__#__即可註釋。

然後另起一行輸入： 

```
deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ xenial main
```

保存，關閉文件。打開終端，執行:

```
sudo apt-get update
sduo apt-get install ros-kinetic-serial
```