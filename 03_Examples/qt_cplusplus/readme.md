# Qt C++ 範例

原始碼(瀏覽器下載並解壓 ): [anrot_demo_qtc++.zip](https://github.com/avmm9898/anrotimu_doc/raw/master/03_Examples/qt_cplusplus/anrot_demo_qtc%2B%2B.zip)

測試環境:

- QT 5.9.9 + Win10
- 僅適用ANROTIMU預設協議(0x91，0x62，詳見說明書)
- 型號:Hi221/226/229, CH100/104/108

### Step 1. include 與 .pro 設定

1. include 資料夾底下分別有 packet、imu_data_decode 的 header 與 cpp 檔，
   在 QT 加入這4檔案。

2. QT .pro 檔加入`QT += serialport`才能使用序列埠，並於在主視窗當中 include header
	```C++
     #ifndef MAINWINDOW_H
     #define MAINWINDOW_H
    
     #include "include/imu_data_decode.h"
     #include "include/packet.h"
   ```

### Step 2. main function

在開始接收前初始化模組

```
imu_data_decode_init();
```

`packet_decode(c)`接收到來自序列的單個 uint8_t 字元並解析，在 loop 中持續接收並解析:

```C++
void MainWindow::read_serial()
{
    auto NumberOfBytesToRead = m_reader.bytesAvailable();

    if(NumberOfBytesToRead > 0 && m_reader.isReadable())
    {
        QByteArray arr = m_reader.readAll();

        for (int i=0;i<NumberOfBytesToRead;i++) {
            uint8_t c=arr[i];
            packet_decode(c);
        }
    }
}
```

在解碼的同時，`receive_imusol_packet_t`會根據IMU接收的數據進行即時更新。
如果使用的是HI221Dongle(無線接收器)，則`receive_gwsol_packet_t`將會是接收到的數據，它是多個`receive_imusol_packet_t`的組合。
例如，要取得歐拉角 :

```
int euler_x,euler_y,euler_z;
euler_x=receive_imusol_packet_t.eul[0];
euler_y=receive_imusol_packet_t.eul[1];
euler_z=receive_imusol_packet_t.eul[2];
```
以下是`receive_imusol_packet_t`和`receive_imusol_packet_t`的宣告結構 : 

```
   typedef struct receive_imusol_packet_t{
    	uint8_t tag;
    	uint8_t id;
    	float acc[3];
    	float gyr[3];
    	float mag[3];
    	float eul[3];
    	float quat[4];
   }receive_imusol_packet_t;

   typedef struct receive_gwsol_packet_t{
       uint8_t tag;
    	uint8_t gw_id;
    	uint8_t n;
    	receive_imusol_packet_t receive_imusol[MAX_LENGTH];
   }receive_gwsol_packet_t;
```

### Step3. 了解數據包種類

0X91( IMUSOL)

共76字節，新加入的數據包，用於替代A0,B0,C0,D0,D1等數據包。集成了IMU的傳感器原始輸出和姿態解算數據。

| 字節偏移 | 類型     | bytes | 單位                 | 說明                                                         |
| -------- | -------- | ----- | -------------------- | ------------------------------------------------------------ |
| 0        | uint8_t  | 1     | -                    | 數據包標籤:0x91                                              |
| 1        | uint8_t  | 1     | -                    | ID                                                           |
| 2        | -        | 6     | -                    | 保留                                                         |
| 8        | uint32_t | 4     | ms                   | 時間戳資訊，從系統開機開始累加，每毫秒增加1                  |
| 12       | float    | 12    | 1G(1G = 1重力加速度) | X,Y,Z軸的加速度，注意單位和0xA0不同                          |
| 24       | float    | 12    | deg/s                | X,Y,Z軸的角速度，注意單位和0xB0不同                          |
| 36       | float    | 12    | uT                   | X,Y,Z軸的磁場強度(HI229支持,注意單位和0xC0不同)              |
| 48       | float    | 12    | deg                  | 節點歐拉角集合, 順序為：橫滾角(Roll)，俯仰角(Pitch)，航向角(Yaw)(注意順序和單位與0xD0數據包不同) |
| 60       | float    | 16    | -                    | 節點四元數集合,順序為WXYZ                                    |

0x62(GWSOL)

HI221Dongle(無線接收器)數據包前8個字節為接收機資訊。後面分為N個數據塊。每個數據塊描述一個節點的姿態數據(最大支持16個節點)。每個數據塊大小為76字節，數據結構同0x91。此協議數據量較大，建議將波特率調整至460800以上獲得穩定的幀率輸出。

協議結構如下:

| 字節偏移                 | bytes | 類型    | 單位 | 說明                      |
| ------------------------ | ----- | ------- | ---- | ------------------------- |
| 0                        | 1     | uint8_t | -    | 數據包標籤:0x62           |
| 1                        | 1     | uint8_t | -    | GWID, 接收機網絡ID        |
| 2                        | 1     | uint8_t | -    | N, 此幀包含節點數據塊個數 |
| 3                        | 5     | -       | -    | 保留                      |
| *----節點數據塊開始----* | -     | -       | -    | *數據結構同0x91*          |
| 8+76*N(N=0-15)           | 1     | uint8_t | -    | 數據包標籤:0x91           |
| 9+76*N(N=0-15)           | 1     | uint8_t | -    | 節點N的ID                 |
| 10+76*N                  | 10    | -       | -    | 保留                      |
| 20+76*N                  | 12    | float   | -    | 節點N三軸加速度           |
| 32+76*N                  | 12    | float   | -    | 節點N三軸角速度           |
| 44+76*N                  | 12    | float   | -    | 節點N軸磁場強度           |
| 56+76*N                  | 12    | float   | -    | 節點N歐拉角               |
| 68+76*N                  | 16    | float   | -    | 節點N四元數               |
| *----節點數據塊結束----* | -     | -       | -    | -----------               |