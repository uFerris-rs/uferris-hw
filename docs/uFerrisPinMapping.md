# uFerris Pin Mapping

The table below provides a comprehensive mapping of the various components and their connections to the Xiao, I/O Expander, Header H3, Header H4, ESP32-C3, and ESP32-C6 pins. 

Note that the enclosure labels are in refrence to the 3D printed enclosure for the uFerris alarm clock project. 

| Enclosure Label   | Component Connection   | Xiao Pin  | I/O Expander Pin   | Header H3 Pin   | Header H4 Pin   | Direction   | ESP32-C3 Pin   | ESP32-C6 Pin   |
|:------------------|:-------------|:-------|:---------------|:---------------|:---------------|:------------|:---------------|:---------------|
| Alarm             | LED 1        | D1/A1  | -              | -              | -              | Output      | GPIO3          | GPIO1          |
| LED               | LED 2        | -      | P14            | -              | -              | Output      | -              | -              |
| PM                | LED 3        | -      | P15            | -              | -              | Output      | -              | -              |
| -                 | Buzzer       | D2/A2  | -              | -              | -              | Output      | GPIO4          | GPIO2          |
| Hour              | SW1          | -      | P07            | -              | -              | Input       | -              | -              |
| Minute            | SW2          | -      | P06            | -              | -              | Input       | -              | -              |
| Time              | SW3          | -      | P05            | -              | -              | Input       | -              | -              |
| Alarm             | SW4          | -      | P04            | -              | -              | Input       | -              | -              |
| Snooze            | SW5          | D3     | -              | -              | -              | Input       | GPIO5          | GPIO21         |
| 12                | SW6 Pos 1    | -      | P16            | -              | -              | Input       | -              | -              |
| 24                | SW6 Pos 2    | -      | P17            | -              | -              | Input       | -              | -              |
| On                | SW7 Pos 1    | -      | P00            | -              | -              | Input       | -              | -              |
| Off               | SW7 Pos 2    | -      | P01            | -              | -              | Input       | -              | -              |
| -                 | SDA          | D4     | -              | -              | P2             | Comms       | GPIO6          | GPIO22         |
| -                 | SCL          | D5     | -              | -              | P3             | Comms       | GPIO7          | GPIO23         |
| -                 | LDR          | D0/A0  | -              | -              | -              | Analog      | GPIO2          | GPIO0          |
| -                 | Digit 1      | -      | P10            | -              | -              | Output      | -              | -              |
| -                 | Digit 2      | -      | P11            | -              | -              | Output      | -              | -              |
| -                 | Digit 3      | -      | P12            | -              | -              | Output      | -              | -              |
| -                 | Digit 4      | -      | P13            | -              | -              | Output      | -              | -              |
| -                 | Seg A        | -      | P20            | -              | -              | Output      | -              | -              |
| -                 | Seg B        | -      | P21            | -              | -              | Output      | -              | -              |
| -                 | Seg C        | -      | P22            | -              | -              | Output      | -              | -              |
| -                 | Seg D        | -      | P23            | -              | -              | Output      | -              | -              |
| -                 | Seg E        | -      | P24            | -              | -              | Output      | -              | -              |
| -                 | Seg F        | -      | P25            | -              | -              | Output      | -              | -              |
| -                 | Seg G        | -      | P27            | -              | -              | Output      | -              | -              |
| -                 | DP           | -      | P26            | -              | -              | Output      | -              | -              |
| -                 | XiaoTx       | D6     | nINT           | P2             | -              | -           | GPIO21         | GPIO16         |
| -                 | XiaoMosi     | D10    | -              | P3             | -              | -           | GPIO10         | GPIO18         |
| -                 | XiaoMiso     | D9     | -              | P4             | -              | -           | GPIO9          | GPIO20         |
| -                 | XiaoScl      | D8     | -              | P5             | -              | -           | GPIO8          | GPIO19         |
| -                 | XiaoRx       | D7     | -              | P6             | -              | -           | GPIO20         | GPIO17         |