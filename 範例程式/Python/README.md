# HIPNUC Python lib #

本路徑提供了一個Python範例代碼，用於通過Python讀取HIPNUC模組數據。

<u>**當前支持的Python版本為3.6及以上。**</u>

已測試環境:

Windows 10

ubuntu 16.04

## 使用方法 ##

- 安裝依賴

  cd到當前目錄，執行：

```
pip install -r requirements.txt
```

- import hipnuc並使用

  請參考demo.py中的代碼：

  ```python
  from hipnuc_module import *
  import time
  
  
  if __name__ == '__main__':
  
      #my_IMU 變數名稱
      my_IMU = hipnuc_module('./config.json')
      
      while True:
          try:
              data = my_IMU.get_module_data()
          except:
              my_IMU.close()
            break
              
          #id, timestamp, acc, gyr, quat, id, linacc, float_eul, int_eul
          #print(data)
          print(data['int_eul'])
  ```
  
  

## API說明

*class* `hipnuc_module`(*path_configjson=None*)

​	超核模組類，用於接收、處理超核模組資訊。

​	參數:**path_configjson** (*str*) – json配置文件的路徑.

​	`get_module_data`(*timeout=None*)

​			獲取已接收到的模組數據.

​			參數

​			**timeout** – 可選參數。若為None(默認值),將會阻塞直至有有效值; 若timeout為正數，將會嘗試等待有效數據並阻塞timeout秒,若阻塞時間到，但仍未有有效數據,將會拋出Empty異常.

​			返回：**data** – 返回模組數據，類型為字典

​			返回類型:dict

> 返回數據格式說明:
>
> **返回為字典。字典的key為數據類型，value為所有節點該數據的list。**
>
> 例如，返回數據中，第1個模組的加速度數據為data["acc"]\[0];返回數據中，第16個模組的四元數資訊為data["quat"]\[15].

​	`get_module_data_size`()

​			獲取已接收到的模組數據的數量. 注意:返回長度大於0,不保證`get_module_data`時不會被阻塞.

​			參數:**無**

​			返回:**size** – 返回模組數據，類型為字典

​			返回類型:int

​	`close`()

​			關閉指定的模組.

​			參數:**無** 

​			返回:**無**



## JSON 配置文件說明

在初始化hipnuc_module時，需要傳入JSON配置文件的路徑，並從配置文件中獲取序列埠端口、波特率、數據類型等資訊。配置文件如下所示：

```json
{
    "port": "COM11",
    "baudrate": 115200,
    "report_datatype": {
      "Expanding Information": false,
      "acc": true,
      "gyr": false,
      "quat": false
    }
}
```

配置含義如下：

`"port"`:

序列埠端口，類型為**字符串**。在Windows下為`"COM*"`，例如`"COM11"`;Linux下,一般為`"/dev/tty*"`，例如`"/dev/ttyUSB0"`。請根據設備的實際情況進行配置。

`"baudrate"`:

序列埠波特率，類型為**整型**。請根據模組實際參數進行設置。

`"report_datatype"`：

匯報數據種類。模組將會上報多種數據資訊，可通過本設置項，配置hipnuc_module實際解析的數據類型。

​	`"Expanding Information"`:

​		數據包ID 0x61，數據幀擴展資訊，包含接收機ID和無線節點數目。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​	`"acc"`：

​		數據包ID 0x75 ，<u>HI221GW接收機專用</u>，加速度資訊集合。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​		或，

​		數據包ID 0xA0，加速度資訊。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​		<u>同一個模組不會同時發送這兩個ID的數據包。</u>

​	`"gyr"`：

​		數據包ID 0x78 ，<u>HI221GW接收機專用</u>，角速度資訊集合。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​		或，

​		數據包ID 0xB0 ，角速度資訊。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​		<u>同一個模組不會同時發送這兩個ID的數據包。</u>

​	`"quat"`：

​		數據包ID 0x71 ，<u>HI221GW接收機專用</u>，四元數資訊集合。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​		或，

​		數據包ID 0xD1 ，四元數資訊。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​		<u>同一個模組不會同時發送這兩個ID的數據包。</u>

​	`"id"`：

​		數據包ID 0x90 ，用戶ID。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​	`"linacc"`：

​		數據包ID 0xA5，線性加速度資訊。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​		`"mag"`：

​		數據包ID 0xC0，磁場強度資訊。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​	`"int_eul"`：

​		數據包ID 0x72，<u>HI221GW接收機專用</u>，歐拉角整形格式集合。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​		或，

​		數據包ID 0xD0，歐拉角整形格式。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​	`"float_eul"`：

​		數據包ID 0xD9，歐拉角浮點格式。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。

​	`"test_8f"`：

​		數據包ID 0x60，測試數據。類型為**布爾型**。

​		若為`true`，hipnuc_module將會返回該數據，若為`false`，hipnuc_module不會返回該數據。



## 附： ##

若在使用中遇到任何問題或建議，請與超核聯繫，謝謝。