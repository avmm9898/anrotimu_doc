Ubuntu 系統自帶CP210x的驅動，預設不需要安裝串列埠驅動。將除錯版連線到電腦上時，會自動識別裝置。識別成功后，會在dev目錄下出現一個對應的裝置:ttyUSBx

檢查USB-UART裝置是否被Ubantu識別：

1. 打開終端，輸入`ls /dev`,先檢視已經存在的串列埠裝置。
2. 檢視是否已經存在  ttyUSBx 這個裝置檔案，便於確認對應的埠號。x表示USB裝置號，由於Ubuntu USB裝置號為從零開始依次累加，所以多個裝置每次開機後設備號是不固定的，需要確定裝置的裝置號。
4. 接下來插入USB線，連線除錯板，然後再次執行`ls /dev`。 dev目錄下多了一個裝置, 如圖：

    ![](./img/2.png)

    **ttyUSB0** 檔案就是除錯版在ubuntu系統中產生的裝置(後面的數字是不固定的，有可能為 ttyUSB1  或 ttyUSB2)

    如果不是 **ttyUSB0** ，需要修改 src/serial_port.cpp 當中 ttyUSB0 為 ttyUSBx 看是多少。

    ```
    #define IMU_SERIAL  ("/dev/ttyUSBx")
	```



5. 打開USB裝置的可執行許可權：
    ```shell
    $ sudo chmod 777 /dev/ttyUSB0
	```


6. 編譯超核IMU ROS package：
    ```shell
    $ cd workspace_of_imu
    $ colcon build
	```

7. 接收IMU數據(目前:四元數、加速度、角速度)
	publisher : 
   
    ```shell
    $ ros2 run serial_imu talker
    ```
   
   subscriber : 
    ```shell
    $ ros2 topic echo /Imu_data
    ```
