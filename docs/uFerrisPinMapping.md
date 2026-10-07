# uFerris Pin Mapping

The table below provides a comprehensive mapping of the various components and their connections to the Xiao, I/O Expander, Expansion Headers H3/H4, Telemetry Headers U2/U6/U7, the SWD debug header H5 and pogo pins U4/U5, and the ESP32-C3 and ESP32-C6 pins.

Note that the enclosure labels are in refrence to the 3D printed enclosure for the uFerris alarm clock project.

`Signal (Net)` is the schematic net name and is the canonical signal name shared with the telemetry extension board. Header pins are written as `<RefDes>-<Pin>`.

| Enclosure Label | Component Connection | Signal (Net) | Xiao Pin       | I/O Expander Pin | Header H3 Pin | Header H4 Pin | Telemetry Header Pin | Debug H5 Pin | SWD Pogo Pin | Direction | ESP32-C3 Pin | ESP32-C6 Pin |
|:----------------|:---------------------|:-------------|:---------------|:-----------------|:--------------|:--------------|:---------------------|:-------------|:-------------|:----------|:-------------|:-------------|
| Alarm           | LED 1                | LED1WIRE     | D1/A1          | -                | -             | -             | U7-7                 | -            | -            | Output    | GPIO3        | GPIO1        |
| LED             | LED 2                | LED2WIRE     | -              | P14              | -             | -             | U2-6                 | -            | -            | Output    | -            | -            |
| PM              | LED 3                | LED3WIRE     | -              | P15              | -             | -             | U2-2                 | -            | -            | Output    | -            | -            |
| -               | Buzzer               | BUZZERWIRE   | D2/A2          | -                | -             | -             | U6-2                 | -            | -            | Output    | GPIO4        | GPIO2        |
| Hour            | SW1                  | SW1WIRE      | -              | P07              | -             | -             | U2-7                 | -            | -            | Input     | -            | -            |
| Minute          | SW2                  | SW2WIRE      | -              | P06              | -             | -             | U7-10                | -            | -            | Input     | -            | -            |
| Time            | SW3                  | SW3WIRE      | -              | P05              | -             | -             | U7-6                 | -            | -            | Input     | -            | -            |
| Alarm           | SW4                  | SW4WIRE      | -              | P04              | -             | -             | U7-1                 | -            | -            | Input     | -            | -            |
| Snooze          | SW5                  | SW5WIRE      | D3             | -                | -             | -             | U7-3                 | -            | -            | Input     | GPIO5        | GPIO21       |
| 12/24           | SW6                  | LSWITCH      | -              | P16              | -             | -             | U2-1                 | -            | -            | Input     | -            | -            |
| On/Off          | SW7                  | RSWITCH      | -              | P01              | -             | -             | U7-2                 | -            | -            | Input     | -            | -            |
| -               | SDA                  | SDA          | D4             | -                | -             | P2            | -                    | -            | -            | Comms     | GPIO6        | GPIO22       |
| -               | SCL                  | SCL          | D5             | -                | -             | P3            | -                    | -            | -            | Comms     | GPIO7        | GPIO23       |
| -               | LDR                  | LDRWIRE      | D0/A0          | -                | -             | -             | U7-5                 | -            | -            | Analog    | GPIO2        | GPIO0        |
| -               | Digit 1              | DIGIT1       | -              | P10              | -             | -             | U2-5                 | -            | -            | Output    | -            | -            |
| -               | Digit 2              | DIGIT2       | -              | P11              | -             | -             | U2-10                | -            | -            | Output    | -            | -            |
| -               | Digit 3              | DIGIT3       | -              | P12              | -             | -             | U2-8                 | -            | -            | Output    | -            | -            |
| -               | Digit 4              | DIGIT4       | -              | P13              | -             | -             | U2-4                 | -            | -            | Output    | -            | -            |
| -               | Seg A                | A            | -              | P20              | -             | -             | U6-9                 | -            | -            | Output    | -            | -            |
| -               | Seg B                | B            | -              | P21              | -             | -             | U6-7                 | -            | -            | Output    | -            | -            |
| -               | Seg C                | C            | -              | P22              | -             | -             | U6-10                | -            | -            | Output    | -            | -            |
| -               | Seg D                | D            | -              | P23              | -             | -             | U6-8                 | -            | -            | Output    | -            | -            |
| -               | Seg E                | E            | -              | P24              | -             | -             | U6-6                 | -            | -            | Output    | -            | -            |
| -               | Seg F                | F            | -              | P25              | -             | -             | U6-4                 | -            | -            | Output    | -            | -            |
| -               | Seg G                | G            | -              | P27              | -             | -             | U7-8                 | -            | -            | Output    | -            | -            |
| -               | DP                   | DP           | -              | P26              | -             | -             | U7-9                 | -            | -            | Output    | -            | -            |
| -               | XiaoTx               | XIAOTX       | D6             | nINT             | P2            | -             | -                    | -            | -            | -         | GPIO21       | GPIO16       |
| -               | XiaoMosi             | XIAOMOSI     | D10            | -                | P3            | -             | -                    | -            | -            | -         | GPIO10       | GPIO18       |
| -               | XiaoMiso             | XIAOMISO     | D9             | -                | P4            | -             | -                    | -            | -            | -         | GPIO9        | GPIO20       |
| -               | XiaoScl              | XIAOSCL      | D8             | -                | P5            | -             | -                    | -            | -            | -         | GPIO8        | GPIO19       |
| -               | XiaoRx               | XIAORX       | D7             | -                | P6            | -             | -                    | -            | -            | -         | GPIO20       | GPIO17       |
| -               | SWCLK                | SWCLK        | SWD pad (pogo) | -                | -             | -             | U6-1                 | H5-4         | U5-2         | Debug     | -            | -            |
| -               | SWDIO                | SWIO         | SWD pad (pogo) | -                | -             | -             | U6-5                 | H5-5         | U4-2         | Debug     | -            | -            |
| -               | RST                  | RST          | SWD pad (pogo) | -                | -             | -             | U6-3                 | H5-6         | U4-1         | Debug     | -            | -            |

## Header Pinouts

Pin-ordered view of the telemetry and debug headers (schematic Rev A.2.0). `NC` = not connected.

### Telemetry Headers (U2, U6, U7 — PZ254-2-05-S, 2×5, 2.54 mm)

**U2**

| Pin | Signal (Net) | Component | Source       |
|:----|:-------------|:----------|:-------------|
| 1   | LSWITCH      | SW6       | Expander P16 |
| 2   | LED3WIRE     | LED 3     | Expander P15 |
| 3   | NC           | -         | -            |
| 4   | DIGIT4       | Digit 4   | Expander P13 |
| 5   | DIGIT1       | Digit 1   | Expander P10 |
| 6   | LED2WIRE     | LED 2     | Expander P14 |
| 7   | SW1WIRE      | SW1       | Expander P07 |
| 8   | DIGIT3       | Digit 3   | Expander P12 |
| 9   | NC           | -         | -            |
| 10  | DIGIT2       | Digit 2   | Expander P11 |

**U6**

| Pin | Signal (Net) | Component | Source              |
|:----|:-------------|:----------|:--------------------|
| 1   | SWCLK        | SWCLK     | Xiao SWD pad (pogo) |
| 2   | BUZZERWIRE   | Buzzer    | Xiao D2/A2          |
| 3   | RST          | RST       | Xiao SWD pad (pogo) |
| 4   | F            | Seg F     | Expander P25        |
| 5   | SWIO         | SWDIO     | Xiao SWD pad (pogo) |
| 6   | E            | Seg E     | Expander P24        |
| 7   | B            | Seg B     | Expander P21        |
| 8   | D            | Seg D     | Expander P23        |
| 9   | A            | Seg A     | Expander P20        |
| 10  | C            | Seg C     | Expander P22        |

**U7**

| Pin | Signal (Net) | Component | Source       |
|:----|:-------------|:----------|:-------------|
| 1   | SW4WIRE      | SW4       | Expander P04 |
| 2   | RSWITCH      | SW7       | Expander P01 |
| 3   | SW5WIRE      | SW5       | Xiao D3      |
| 4   | NC           | -         | -            |
| 5   | LDRWIRE      | LDR       | Xiao D0/A0   |
| 6   | SW3WIRE      | SW3       | Expander P05 |
| 7   | LED1WIRE     | LED 1     | Xiao D1/A1   |
| 8   | G            | Seg G     | Expander P27 |
| 9   | DP           | DP        | Expander P26 |
| 10  | SW2WIRE      | SW2       | Expander P06 |


### SWD Debug Header (H5 — PH2.54-2X3P, 2×3, 2.54 mm)

**H5**

| Pin | Signal (Net) | Component | Source              |
|:----|:-------------|:----------|:--------------------|
| 1   | VCC          | Power     | -                   |
| 2   | GND          | Power     | -                   |
| 3   | GND          | Power     | -                   |
| 4   | SWCLK        | SWCLK     | Xiao SWD pad (pogo) |
| 5   | SWIO         | SWDIO     | Xiao SWD pad (pogo) |
| 6   | RST          | RST       | Xiao SWD pad (pogo) |


### SWD Pogo Pins (U4, U5 — 2-pin pogo, 2.50 mm)
These contact the SWD pads on the underside of XIAO variants that expose them.

**U4**

| Pin | Signal (Net) | Component | Source              |
|:----|:-------------|:----------|:--------------------|
| 1   | RST          | RST       | Xiao SWD pad (pogo) |
| 2   | SWIO         | SWDIO     | Xiao SWD pad (pogo) |

**U5**

| Pin | Signal (Net) | Component | Source              |
|:----|:-------------|:----------|:--------------------|
| 1   | GND          | Power     | -                   |
| 2   | SWCLK        | SWCLK     | Xiao SWD pad (pogo) |

### Notes
- The telemetry board also mates with Expansion Headers H3 and H4, which supply power/GND, SDA/SCL and the XIAO serial/SPI lines (XIAOTX, XIAORX, XIAOMOSI, XIAOMISO, XIAOSCL).
- SWCLK, SWIO and RST are shared between U6, H5 and the pogo pins.
