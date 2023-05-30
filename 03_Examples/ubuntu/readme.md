## 單元 : Ubuntu 範例

原始碼(瀏覽器下載並解壓 ): [anrot_demo_ubuntu.zip](https://github.com/avmm9898/anrotimu_doc/raw/master/03_Examples/ubuntu/anrot_demo_ubuntu.zip)

介紹如何在 Ubuntu 環境中讀取 HI221/HI221Dongle/HI226/HI229/CH100/CH110/CH10x 的數據，提供了c代碼，用於讀取模組的數據。

- 測試環境：Ubuntu 16.04

- 測試設備：HI226	HI229

### Step 1. 尋找 USB-UART 設備

因為 Ubuntu 系統自帶 CP210x 的驅動，所以不用專門去安裝相應序列埠驅動。將調試版連接到電腦上時，會自動識別設備。識別成功後，會在`dev`目錄下出現一個對應的設備文件。

檢查系統是否識別到 USB-UART 設備：

1. 打開 Ubuntu 系統，按下 __ctrl + alt + t__ 打開終端機。

2. 終端機切換到`dev`目錄下，並列出所有檔案

   ```
   cd /dev
   ls
   ```

   如下圖，在這些文件名稱中，主要關心`ttyUSB`這個設備文件。後面數字代表 USB 設備號，由於 Ubuntu USB 設備號為從零開始依次累加，所以多個設備每次開機設備號是不固定的，需要確定設備的設備號 。

    <img src="https://raw.githubusercontent.com/avmm9898/anrotimu_doc/master/03_Examples/ubuntu/img/1.png">

    上圖為沒有插入USB設備的情況，這個時候，`dev`目錄下並沒有名為`ttyUSB`文件，插入 USB 線，連接模組，然後再次執行`ls`：

    `dev`目錄下多了幾個文件名稱, 如圖 :
   
    <img src="https://raw.githubusercontent.com/avmm9898/anrotimu_doc/master/03_Examples/ubuntu/img/2.png">
   

 `ttyUSB0`文件就是調試版在 ubuntu 系統中生成的設備文件，對它進行讀寫，就可以完成序列埠通信。這個文件名稱我們把它記下來。後面的數字是不固定的，有可能為 `ttyUSB1`或`ttyUSB2`等。

### Step 2. 波特率設置

在 Ubuntu 環境中，波特率支援到 115200/460800/921600，本範例使用的是 115200。

如果需要輸出幀率超過100Hz，則需要需要修改 main.c 文件中的 options.c_cflag 參數，改為更高的波特率。

<img src="https://raw.githubusercontent.com/avmm9898/anrotimu_doc/master/03_Examples/ubuntu/img/5.png">

如圖，在第83行，將B115200修改為B460800或者是B921600。

### Step 3. 編譯並執行

我們開始在 Ubuntu 環境下生成一個可執行文件，專門用來解析模組的數據：

首先在 Ubuntu 系統中，按下打開終端機，到下載好的原始碼目錄下

```
cd CH_demo_ubuntu
make
sudo ./main ttyUSB0
```

執行成功後，會出現這個畫面：

<img src="https://raw.githubusercontent.com/avmm9898/anrotimu_doc/master/03_Examples/ubuntu/img/3.png">

這個畫面上的數字會隨著模組位置的改變而發生變化。

- 如果後期修改了這些文件，需要清理之前生成的舊 __.o__ 和 __main__ 文件，重新生成 __main__ 這個可執行文件。

  ```
  make clean
  make
  ```

- 如果後期您需要在本路徑上添加其他文件，配合使用，請打開 __Makefile__ 文件，在第一行的後面加上後添加文件的鏈接文件名，例如添加 append_file.c 文件，那麼在 __Makefile__ 中第一行後面追加 append_file.o 文件名。如果後加的文件還需要鏈接第三方的庫，請在第二行的後面添加庫名字。格式為 __-l+lib_name__  ("l" 是「L"的小寫的英文字母)。

- 如果出現：
  
  <img src="https://raw.githubusercontent.com/avmm9898/anrotimu_doc/master/03_Examples/ubuntu/img/4.png">
  
  表示未能找到序列埠，需要回到**《尋找USB-UART設備》一節** 確認 USB-UART 設備已經被 Ubuntu 識別。
