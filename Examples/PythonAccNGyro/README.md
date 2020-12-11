# HIPNUC Python lib #

本路徑提供了一個Python範例代碼，用於通過Python讀取HIPNUC模組數據。

<u>**當前支持的Python版本為3.6及以上。**</u>

已測試環境:

Windows 10

ubuntu 16.04

IMU模組使用新傳輸協議

## 使用方法 ##

- 安裝依賴

  cd到當前目錄，執行：

```
pip install -r requirements.txt
```

- import hipnuc並使用

  請參考 demo.py 中的代碼：

  ```python
  #demo.py
  
  from hipnuc_module import *
  import time
  
  if __name__ == '__main__':
  
      # m_IMU_serial= class of hipnuc_module 
      m_IMU_serial = hipnuc_module('./config.json')
  
      while True:
          try:
              #receive data
              data = m_IMU_serial.get_module_data(1)
          except:
              print("Serial closed.")
              m_IMU_serial.close()
              break
  
          try:
              #this prints all : print(data)#
  
              #id, timestamp, acc, gyr, quat, mag, euler#
              print(data['euler'])
          except:
              print("Print error.")
              m_IMU_serial.close()
              break
  ```
  
  

## API說明

*class* `hipnuc_module(*path_configjson=None*)`

超核模組類，用於接收、處理超核模組資訊。

參數:**path_configjson** (*str*) – json配置文件的路徑.

- `get_module_data(*timeout=None*)`

  獲取已接收到的模組數據.
  **timeout** – 可選參數。若為None(默認值),將會阻塞直至有有效值; 若timeout為正數，將會嘗試等待有效數據並阻塞 timeout 秒,若阻塞時間到，但仍未有有效數據,將會拋出Empty異常.

  返回 : data** (模組數據，類型為 dict)

    > 返回數據格式說明:
    >
    > **返回為字典。字典的key為數據類型，value為所有節點該數據的list。**
    >
    > 例如，返回數據中，第1個模組的加速度數據為 data["acc"]\[0]；返回數據中，第16個模組的四元數資訊為 data["quat"]\[15].

- `get_module_data_size()`

​			獲取已接收到的模組數據的數量. 注意:返回長度大於0,不保證`get_module_data`時不會被阻塞.

​			參數:**無**

​			返回:**size** – 返回模組數據，類型為字典

​			返回類型:int

- `close()`

​			關閉指定的模組.

​			參數:**無** 

​			返回:**無**

## JSON 配置文件說明

在初始化 hipnuc_module 時，需要傳入 JSON 配置文件的路徑，並從配置文件中獲取序列埠端口、波特率、數據類型等資訊：

```json
//config.json

{
  "port": "COM8",
  "baudrate": 115200,
  "report_datatype": {
    "imusol": true,
    "gwsol": true,
    "id": true,
    "acc": true,
    "gyr": true,
    "mag": true,
    "euler": true,
    "quat": true
  }
}
```

配置含義如下：

`"port"`:

序列埠端口，類型為**字符串**。在Windows下為`"COM*"`，例如`"COM11"`;Linux下,一般為`"/dev/tty*"`，例如`"/dev/ttyUSB0"`。請根據設備的實際情況進行配置。

`"baudrate"`:

序列埠波特率，類型為**整型**。請根據模組實際參數進行設置。

`"report_datatype"`：

匯報數據種類。模組將會回傳多種數據資訊，建議如範例所設置即可。



## 數據協議為 0x91(預設)

0x91 為 HI221、HI226、HI229、CH110、CH100 等的預設傳輸協議。

模組回傳內容為:

```
temp_dic = {
        "id":id_temp_list,
        "timestamp":timestamp_temp_list,
        "acc":acc_temp_list,
        "gyr":gyr_temp_list,
        "mag":mag_temp_list,
        "euler":int_eul_temp_list,
        "quat":quat_temp_list
}
return temp_dic
```

一幀 data 內含以下資料 : `id, timestamp, acc, gyr, mag, euler, quat`

- **id**: 節點 ID
- **timestamp** : 自開機起傳送的幀數
- **acc** : 加速度，順序為 X Y Z 軸，單位=1G (1G = 1x重力加速度)
- **gyr**  : 角速度，順序為 X Y Z 軸，單位=deg/s  
- **mag** : 磁場強度，順序為 X Y Z 軸，單位=µT (10^-6 T)
- **euler** : 歐拉角，順序為 橫滾(X-Roll)/俯仰(Y-Pitch)/航向(Z-Yaw)，單位=degree
- **quat** : 四元數，順序為 W X Y Z 軸

## 數據協議為 0x62

0x62 目前僅為 HI221GW (無線接收器) 預設傳輸協議。

```
temp_dic = {
    "GWD":gwid,
    "CNT":cnt,
    "id":id_temp_list,
    "timestamp":timestamp_temp_list,
    "acc":acc_temp_list,
    "gyr":gyr_temp_list,
    "mag":mag_temp_list,
    "euler":eul_temp_list,
    "quat":quat_temp_list
}
return temp_dic
```

一幀 data 內含以下資料 : `GWD, CNT, id, timestamp, acc, gyr, mag, euler, quat`

- **GWD **: 無線接收機 ID (GWID)
- **CNT **: 在線的無線節點 (HI221) 數量
- **id **: 節點 ID
- **timestamp** : 自開機起傳送的幀數
- **acc** : 加速度，順序為 X Y Z 軸，單位=1G (1G = 1x重力加速度)
- **gyr**  : 角速度，順序為 X Y Z 軸，單位=°/s
- **mag** : 磁場強度，順序為 X Y Z 軸，單位=µT (10^-6 T)
- **euler** : 歐拉角，順序為 橫滾(X-Roll)/俯仰(Y-Pitch)/航向(Z-Yaw)，單位=degree
- **quat** : 四元數，順序為 W X Y Z 軸

## 數據協議包含 0x90, A0, B0, C0, D0, D1

一幀 data 依照自己硬體模組的協議設定，可包含以下資料 : `id, acc, gyr, mag, euler, quat`

- **id** : 當協議包含 0x90，傳輸節點 ID
- **acc** : 當協議包含 0xA0，傳輸加速度，順序為 X Y Z 軸，單位=0.001G (1G = 1x重力加速度)
- **gyr** : 當協議包含 0xB0，傳輸角速度，順序為 X Y Z 軸，單位=0.1°/s
- **mag** : 當協議包含 0xC0，傳輸磁場，順序為 X Y Z 軸，單位=0.001G(=0.1 µT=10^-4 T)
- **euler** : 當協議包含 0xD0，傳輸歐拉角，順序為 X Y Z 軸，單位=°/s
- **quat** : 當協議包含 0xD1， 傳輸四元數，順序為 W X Y Z 軸

## 提醒 : 若發現數據傳輸不正常或迴圈中斷:

請檢查並重新設定:

- 數據傳輸協議是否設定正確
- Baud 是否與模組匹配
- COM 是否與模組匹配

若有其他問題請聯繫我們 : 