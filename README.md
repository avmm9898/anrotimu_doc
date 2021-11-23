<h1 align="center">超核IMU 資料與軟體中心<br>
(Software Center of HiPNUC)
</h1>


<br>


## 軟體中心簡介

本公司持續維護、更新產品與軟體，為用戶提供更良好的使用體驗與功能，<br>
包含所有產品資料，包含說明書、驅動程式，開源範例與官方姿態記錄軟體，<br>
用戶可參考範例與使用手冊自行創造更多應用，本公司並不免費提供額外的應用支援。<br>

聯繫我們

Email : [hipnuctw@gmail.com](mailto:hipnuctw@gmail.com)

<br>
</h3>

| 資料夾            | 說明               |
| ---------------- | ------------------ |
| 01_UserManual | IMU 產品用戶手冊 (CH110、HI221、HI226、HI229) |
| 02_GUI |使用者介面，呈現圖表、數值、記錄功能，僅支援 Windows |
| 03_Examples |   資料接收程式原始碼及各語言範例       |
| 04_UsbDrivers | 提供 Windows/Linux 的 USB 驅動，一般已經內建。<br/>CH110/HI226/HI229 請安裝 CP2104.zip。<br/>HI221/221GW 請安裝 CH341SER.EXE。 |
| 05_OpenHardware | HI226/HI229的硬體尺寸圖 |

[下載全部](https://github.com/avmm9898/hipnuctw_doc/archive/master.zip)

<br>

# 簡介

超核電子 IMU/ARU/AHRS 提供客戶豐富的支援，包含官方軟體、使用教學、詳細的產品說明書。

### [前往常見問題](https://hipnuc.com/mkdocs_tc/site/FAQ/FAQ/)

### 產品說明書

- HI221/HI221 Dongle 無線慣性感測器([PDF](https://github.com/avmm9898/hipnuctw_doc/raw/master/01_UserManual/hi221.pdf))
- HI226 六軸慣性感測器([PDF](https://github.com/avmm9898/hipnuctw_doc/raw/master/01_UserManual/hi226.pdf))
- HI229 九軸慣性感測器([PDF](https://github.com/avmm9898/hipnuctw_doc/raw/master/01_UserManual/hi229.pdf))
- CH100 六軸工業級貼片式高精度慣性感測器([PDF](https://github.com/avmm9898/hipnuctw_doc/raw/master/01_UserManual/ch110.pdf))
- CH110 六軸IP67外殼，RS232工業級高精度慣性感測器([PDF](https://github.com/avmm9898/hipnuctw_doc/raw/master/01_UserManual/ch110.pdf))



### Windows 官方姿態軟體 : CHCenter 

<iframe width="560" height="315" src="https://www.youtube.com/embed/BMr5ByL2h8w" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
### [→下載最新版CHCenter](https://github.com/avmm9898/hipnuctw_doc/raw/master/02_GUI/CHCenter_v1.2.2_win32.zip)



包含所有產品的 :

- X Y Z 多軸即時數據、波形圖
- 3D 姿態顯示
- CSV 數據紀錄
- 加速度與陀螺儀FFT分析、低通濾波功能，協助分析振動
- 模組參數設定，如採樣率、鮑率、ID、無線節點 GWID


如有發現問題請聯絡 [hipnuctw@gmail.com](mailto:hipnuctw@gmail.com) 

- 如遇上執行 GUI 提示缺少 msvcpXXX.dll 的狀況，請安裝 [VC_redist.x86.exe](https://github.com/avmm9898/hipnuctw_doc/raw/master/02_GUI/VC_redist.x86.exe)
- 如 Windows 系統未成功識別 IMU 裝置，請安裝 [USB驅動包](https://github.com/avmm9898/hipnuctw_doc/raw/master/04_UsbDrivers/win/IMU_2in1_drivers.zip)



### 範例程式與教學

基本接收資料的範例包含以下程式語言與環境(恕不提供免費額外的程式修改服務):
- C#
- Python
- QT C++
- ROS
- STM32
- Ubuntu

Example Code 下載: [連結](https://github.com/avmm9898/hipnuctw_doc/tree/master/03_Examples)



### 更多詳細資料請至 [Github](https://github.com/avmm9898/hipnuctw_doc):

