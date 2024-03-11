
# 01_AutoFeeder.X
![image](image/overview.png)
This is a test project for an auto-feeder for CompactPnP. When the switch is pressed, the feeder moves one frame. It includes a program and a schematic. The program was written in PIC MPLAB X IDE. The schematic is written in KiCAD and the printed circuit board is made. The structure was made with a 3D printer.

---
これはCompactPnP用のオートフィーダーのテスト用プロジェクトです。スイッチが押されるとフィーダーが一コマ移動します。プログラムと回路図を含みます。プログラムはPIC MPLAB X IDEで書きました。回路図はKiCADで書き、プリント基板を作っています。構造物は3Dプリンターで作りました。

---
### Rev.01
* [demo](image/ezgif.com-video-to-gif-converted.gif)
* [schematics](schematics/AutoFeeder_Drum-type_new/20231123_R1/AutoFeeder_Drum-type.pdf) 
* block chart
```mermaid
graph LR
  A[Timer2]-->B[PWM3]-->C
  subgraph BA6211
  C[FWD]
  E[REV]
  end
  A-->D[PWM4]-->E
  F[Sensor0]-->|-|M[CMP2]-->N[LED Green]
    L[FVR 1.024V]-->|+|M
    L-->|+|Q
  I[Sensor1]-->|-|Q[CMP1]-->K[LED Red]
```
---

### Rev.02
フォトセンサーの感度を切り替える回路と、ドラムの角度を取得するためのAS5600という磁気センサーと、I2CのLCDを接続できるようにしました。Add a circuit to switch the sensitivity of the photo sensor, a magnetic sensor named AS5600 to get the angle of the drum, and the LCD.
![image2](image/P0002221.JPG)
* [demo](image/20240212_AutoFeederTest-ezgif.com-video-to-gif-converter.gif)
* [schematics](schematics/AutoFeeder_Drum-type_new/20240107_Rev02/AutoFeeder_Drum-type.pdf) 
* block chart

```mermaid
graph LR
  G[Senror1 Radder detector]--> S
  F[Sensor0 Tape-hole detector]--> S
  S{Switch} -->| M- |M
  subgraph BA6211
  C[FWD]
  E[REV]
  end
  BA6211 --- Z[Motor]
  subgraph PIC16F1705
  A[Timer2 50KHz]-->D
  A-->B
  B[PWM3 -- RC5]---C
  D[PWM4 -- RC4]---E
  L[FVR 2.048V]-->| P+ |M[CMP2 -- RA0]
  RA4[GPIO RA4 Open Drain]
  RA5[GPIO RA5 Open Drain]
  end
  M-->N[LED]
  AS5600---|I2C|PIC16F1705
  PIC16F1705---|I2C|LCD
  
  H[Optical Sensor Sensitivity Selector]---RA4
  H---RA5
```

* High-Endurance Flash memory (HEF) usage 0x1F80-0x1F9F
  | Offset Address | note (1 word = 14-bit wide) |
  |:--:|---- 
  | 25 .. 31 | No Use |
  | 2 .. 24 | Tape hole position data (12-bit wide) |
  | 1 | Tape-Color data (White/Transparent/Black)|
  | 0 | AS5600 Magnetic data of origin (12-bit wide) |
* HEF 0x1FA0-0x1FFF is not use.

---
---
### Tips

* PIC16F1705でI2C を設定する。何故か400kHzに設定するとハングします。「Registers」で SSP1ADD = 0x9 (=400KHz)に設定することは可能。
  
  Setting I2C on the PIC16F1705. It hangs when set to 400kHz. It is possible to set SSP1ADD = 0x9 (=400KHz) in "Registers".
  ![image](image/01_I2C_setting.png)

* このI2Cのサンプルコードがよく出来ていてそのままincludeして使える。
  This I2C sample code is well done and can be included and used as is.
  ![image](image/02_I2C_setting.png)

* HEF-block範囲にプログラムが書かれないようにする設定。
  Settings to prevent programs from being written in the HEF-block range
  ![image](image/03_HFE-block-tips.png)

* PICに書いた時にHEFブロックが初期化されない設定。
  Settings to prevent the HEF-block from being initialized when writing to the PIC
  ![image](image/04_HFE-block-tips.png)

