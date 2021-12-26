# 使用 Matlab 讀取 IMU 數據

[Matlab Sample Code 下載](https://github.com/avmm9898/hipnuctw_doc/raw/master/03_Examples/matlab/CH_demo_matlab.zip)

環境 : 請使用 Matlab 2020 以上版本

範例包含 : crc16.m, imu_read.m

解壓縮後使用 Matlab 2020 以上版本開啟，

修改下圖紅框位置資訊: 波特率和串口號碼:

<img src="https://raw.githubusercontent.com/avmm9898/hipnuctw_doc/master/03_Examples/matlab/img/1.png">

點擊 Run，即可在輸出區域獲得 IMU 資訊，可在程式中自行修改要打印出的數值

- raw.imu.id

- raw.imu.acc

- raw.imu.gyr

- raw.imu.mag (九軸模式才有效)

- raw.imu.eul

- raw.imu.quat

<img src="https://raw.githubusercontent.com/avmm9898/hipnuctw_doc/master/03_Examples/matlab/img/.png">