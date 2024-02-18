
# 01_AutoFeeder.X
![image](image/overview.png)
This is a test project for an auto-feeder for CompactPnP. When the switch is pressed, the feeder moves one frame. It includes a program and a schematic. The program was written in PIC MPLAB X IDE. The schematic is written in KiCAD and the printed circuit board is made. The structure was made with a 3D printer.

---
これはCompactPnP用のオートフィーダーのテスト用プロジェクトです。スイッチが押されるとフィーダーが一コマ移動します。プログラムと回路図を含みます。プログラムはPIC MPLAB X IDEで書きました。回路図はKiCADで書き、プリント基板を作っています。構造物は3Dプリンターで作りました。

---
### Rev.01
* [demo](image/ezgif.com-video-to-gif-converted.gif)
* [schematics](schematics/AutoFeeder_Drum-type_new/20231123_R1/AutoFeeder_Drum-type.pdf) 

---
### Rev.02
* [demo](image/20240212_AutoFeederTest-ezgif.com-video-to-gif-converter.gif)
* [schematics](schematics/AutoFeeder_Drum-type_new/20240107_Rev02/AutoFeeder_Drum-type.pdf) 
* block chart
```mermaid
graph LR
  A[Timer2 50KHz]-->B[PWM3 --> RC5]---C
  subgraph BA6211
  C[FWD]
  E[REV]
  end
  BA6211 --- Z[Motor]
  A-->D[PWM4 --> RC4]---E
  F[Sensor0 Tape-hole detector]--> S{Switch}
  G[Senror1 Radder detector]--> S
  S -->| < - > |M[CMP2 --> RA0]-->N[LED]
    L[FVR 2.048V]-->|< + >|M
  RA4[GPIO RA4 Open Drain]---H[Optical Sensor Sensitivity Selector]
  RA5[GPIO RA5 Open Drain]---H
  subgraph SW1 TMHU26
  CW[Switch CW]
  CCW[Switch CCW]
  P[Switch Push]
  end
  CW---RC3[GPIO RC3]
  CCW---RA1[GPIO RA1]
  P---RA2[GPIO RA2]
  subgraph I2C
  SCL[SCL]
  SDA[SDA]
  end
  SCL---LCD[LCD 0x3E]
  SDA---LCD
  SCL---MAG[Magnetic Sensor 0x36]
  SDA---MAG
```

* variables
  | HEF_buffer | note |
  ----|---- 
  | 2 | Tape Color |
  | 1 | Mag Position (High 4bit)|
  | 0 | Mag Position (Low 8bit) |

---
---
### Tips

* PIC16F1705でI2C を設定する。何故か400kHzに設定するとハングします。「Registers」で SSP1ADD = 0x9 (=400KHz)に設定することは可能。
  
  Setting I2C on the PIC16F1705. It hangs when set to 400kHz. It is possible to set SSP1ADD = 0x9 (=400KHz) in "Registers".
  ![image](image/01_I2C_setting.png)

* このサンプルコードがよく出来ていてそのままincludeして使える。
  This sample code is well done and can be included and used as is.
  ![image](image/02_I2C_setting.png)

* HEF-block範囲にプログラムが書かれないようにする設定。
  Settings to prevent programs from being written in the HEF-block range
  ![image](image/03_HFE-block-tips.png)

* PICに書いた時にHEFブロックが初期化されない設定。
  Settings to prevent the HEF-block from being initialized when writing to the PIC
  ![image](image/04_HFE-block-tips.png)

