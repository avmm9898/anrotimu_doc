# STM32 範例



原始碼(瀏覽器下載並解壓 ): [CH_demo_stm32.zip](https://github.com/avmm9898/hipnuctw_doc/raw/master/03_Examples/STM32/CH_demo_stm32.zip)

本範例提供了C 語言範例代碼，用戶接收HI226/HI229及HI221數據。

- 測試環境：Windows10_x64 操作系統
	
- 編譯器：keil_V5.28
- 開發板：正點原子-戰艦V3 STM32F103ZET6
	
- 測試設備：HI226/HI229

### 硬體連接

將Hi226/Hi229 正確插入到超核調試板上。

| 超核調試板 | 正點原子開發板 |
| ---------- | -------------- |
| RXD        | PA2(TXD)       |
| TXD        | PA3(RXD)       |
| 3.3V       | 3V3            |
| GND        | GND            |

用杜邦線將上表相對應的引腳連接起來。
	
用USB線插到開發板的 __USB_232__ 插口上，另一端插到電腦上。

### 觀察輸出

打開序列埠調試助手，打開開發板對應的序列埠號，觀察數據輸出
