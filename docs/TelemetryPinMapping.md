# uFerris Megalops Telemetry Board Pin Mapping

The telemetry board (Rev A.0.0) carries an RP2040 that reads and drives the uFerris baseboard lines. It mates with the baseboard's Telemetry Headers (U2, U6, U7) and Expansion Headers (H3, H4).

`Signal` columns use the baseboard net names (see `uFerrisPinMapping.md`), so names are identical across both boards. Where the telemetry schematic uses a different net name, it is shown in `Telemetry Net`.

Eight baseboard signals share four RP2040 pins each through two TMUX1574 2:1 muxes (U4, U9), switched by `MODESEL` (GPIO0). With `MODESEL = 0` the RP2040 sees the XIAO-side signals; with `MODESEL = 1` it sees the 7-segment signals. Both mux enables (EN#) are tied to GND.

## RP2040 GPIO Map

| RP2040 GPIO | RP2040 Pin | Function      | Telemetry Net | Path                         | Signal (MODESEL=0) | Signal (MODESEL=1) | Telemetry Header Pin | Baseboard Header Pin | Baseboard Source          | Notes                                                                       |
|:------------|:-----------|:--------------|:--------------|:-----------------------------|:-------------------|:-------------------|:---------------------|:---------------------|:--------------------------|:----------------------------------------------------------------------------|
| GPIO0       | 2          | GPIO0         | MODESEL       | Mux select                   | -                  | -                  | -                    | -                    | -                         | Drives SEL on U4 and U9. 0 = XIAO-side signals, 1 = 7-segment signals       |
| GPIO1       | 3          | GPIO1         | XIAOMOSI/E    | Mux U9 D1                    | XIAOMOSI           | E                  | H2-3 / H5-8          | H3-3 / U6-6          | Xiao D10 / Expander P24   |                                                                             |
| GPIO2       | 4          | GPIO2         | SWCLK_XIAO    | Direct                       | SWCLK              | SWCLK              | H5-5                 | U6-1                 | Xiao SWD pad (pogo)       | Net name differs from baseboard (SWCLK)                                     |
| GPIO3       | 5          | GPIO3         | SWIO_XIAO     | Direct                       | SWIO               | SWIO               | H5-3                 | U6-5                 | Xiao SWD pad (pogo)       | Net name differs from baseboard (SWIO)                                      |
| GPIO4       | 6          | GPIO4         | XIAORX/B      | Mux U9 D2                    | XIAORX             | B                  | H2-6 / H5-2          | H3-6 / U6-7          | Xiao D7 / Expander P21    |                                                                             |
| GPIO5       | 7          | GPIO5         | XIAOMISO/DP   | Mux U9 D3                    | XIAOMISO           | DP                 | H2-4 / H6-1          | H3-4 / U7-9          | Xiao D9 / Expander P26    |                                                                             |
| GPIO6       | 8          | GPIO6         | XIAOTX/G      | Mux U9 D4                    | XIAOTX             | G                  | H2-2 / H6-9          | H3-2 / U7-8          | Xiao D6 / Expander P27    |                                                                             |
| GPIO7       | 9          | GPIO7         | BUZZERWIRE/A  | Mux U4 D1                    | BUZZERWIRE         | A                  | H5-6 / H5-1          | U6-2 / U6-9          | Xiao D2/A2 / Expander P20 |                                                                             |
| GPIO8       | 11         | GPIO8         | LED1WIRE/F    | Mux U4 D2                    | LED1WIRE           | F                  | H6-2 / H5-7          | U7-7 / U6-4          | Xiao D1/A1 / Expander P25 |                                                                             |
| GPIO9       | 12         | GPIO9         | SW5WIRE/D     | Mux U4 D3                    | SW5WIRE            | D                  | H6-4 / H5-9          | U7-3 / U6-8          | Xiao D3 / Expander P23    |                                                                             |
| GPIO10      | 13         | GPIO10        | XIAOSCL/C     | Mux U4 D4                    | XIAOSCL            | C                  | H2-5 / H5-10         | H3-5 / U6-10         | Xiao D8 / Expander P22    |                                                                             |
| GPIO11      | 14         | GPIO11        | RST_XIAO      | Direct                       | RST                | RST                | H5-4                 | U6-3                 | Xiao SWD pad (pogo)       | Net name differs from baseboard (RST)                                       |
| GPIO12      | 15         | GPIO12        | SCL           | Direct                       | SCL                | SCL                | H3-3                 | H4-3                 | Xiao D5                   |                                                                             |
| GPIO13      | 16         | GPIO13        | SDA           | Direct                       | SDA                | SDA                | H3-2                 | H4-2                 | Xiao D4                   |                                                                             |
| GPIO14      | 17         | GPIO14        | SW1WIRE       | Direct                       | SW1WIRE            | SW1WIRE            | H4-2                 | U2-7                 | Expander P07              |                                                                             |
| GPIO15      | 18         | GPIO15        | SW2WIRE       | Direct                       | SW2WIRE            | SW2WIRE            | H6-10                | U7-10                | Expander P06              |                                                                             |
| GPIO16      | 27         | GPIO16        | SW3WIRE       | Direct                       | SW3WIRE            | SW3WIRE            | H6-8                 | U7-6                 | Expander P05              |                                                                             |
| GPIO17      | 28         | GPIO17        | SW4WIRE       | Direct                       | SW4WIRE            | SW4WIRE            | H6-5                 | U7-1                 | Expander P04              |                                                                             |
| GPIO18      | 29         | GPIO18        | LSWITCH       | Direct                       | LSWITCH            | LSWITCH            | H4-5                 | U2-1                 | Expander P16              |                                                                             |
| GPIO19      | 30         | GPIO19        | RSWITCH       | Direct                       | RSWITCH            | RSWITCH            | H6-6                 | U7-2                 | Expander P01              |                                                                             |
| GPIO20      | 31         | GPIO20        | DIGIT1        | Direct                       | DIGIT1             | DIGIT1             | H4-3                 | U2-5                 | Expander P10              |                                                                             |
| GPIO21      | 32         | GPIO21        | DIGIT2        | Direct                       | DIGIT2             | DIGIT2             | H4-10                | U2-10                | Expander P11              |                                                                             |
| GPIO22      | 34         | GPIO22        | DIGIT3        | Direct                       | DIGIT3             | DIGIT3             | H4-9                 | U2-8                 | Expander P12              |                                                                             |
| GPIO23      | 35         | GPIO23        | DIGIT4        | Direct                       | DIGIT4             | DIGIT4             | H4-7                 | U2-4                 | Expander P13              |                                                                             |
| GPIO24      | 36         | GPIO24        | LED2WIRE      | Direct                       | LED2WIRE           | LED2WIRE           | H4-8                 | U2-6                 | Expander P14              |                                                                             |
| GPIO25      | 37         | GPIO25        | LED3WIRE      | Direct                       | LED3WIRE           | LED3WIRE           | H4-6                 | U2-2                 | Expander P15              |                                                                             |
| GPIO26      | 38         | GPIO26 / ADC0 | LDRWIREBUF    | Buffer U5 (MCP6002 follower) | LDRWIRE            | LDRWIRE            | H6-3                 | U7-5                 | Xiao D0/A0                | Analog: unity-gain buffered into ADC                                        |
| GPIO27      | 39         | GPIO27 / ADC1 | XIAO3V3BUF    | Buffer U5 (MCP6002 follower) | VCC                | VCC                | H3-1                 | H4-1                 | Baseboard 3V3             | Analog: unity-gain buffered into ADC. Net name differs from baseboard (VCC) |
| GPIO28      | 40         | GPIO28 / ADC2 | NC            | -                            | -                  | -                  | -                    | -                    | -                         | Unused                                                                      |
| GPIO29      | 41         | GPIO29 / ADC3 | NC            | -                            | -                  | -                  | -                    | -                    | -                         | Unused                                                                      |

## Header Mating

The 2×5 headers use different pin numbering on each board: the baseboard numbers odd/even across the rows, while the telemetry board numbers 1–5 down one column and 6–10 back up the other. The table gives the pin-to-pin correspondence, checked net-by-net against both netlists.

| Telemetry Header | Pin | Telemetry Net | Mates With (Baseboard) | Baseboard Signal (Net) |
|:-----------------|:----|:--------------|:-----------------------|:-----------------------|
| H4               | 1   | NC            | U2-9                   | NC                     |
| H4               | 2   | SW1WIRE       | U2-7                   | SW1WIRE                |
| H4               | 3   | DIGIT1        | U2-5                   | DIGIT1                 |
| H4               | 4   | NC            | U2-3                   | NC                     |
| H4               | 5   | LSWITCH       | U2-1                   | LSWITCH                |
| H4               | 6   | LED3WIRE      | U2-2                   | LED3WIRE               |
| H4               | 7   | DIGIT4        | U2-4                   | DIGIT4                 |
| H4               | 8   | LED2WIRE      | U2-6                   | LED2WIRE               |
| H4               | 9   | DIGIT3        | U2-8                   | DIGIT3                 |
| H4               | 10  | DIGIT2        | U2-10                  | DIGIT2                 |
| H5               | 1   | A             | U6-9                   | A                      |
| H5               | 2   | B             | U6-7                   | B                      |
| H5               | 3   | SWIO_XIAO     | U6-5                   | SWIO                   |
| H5               | 4   | RST_XIAO      | U6-3                   | RST                    |
| H5               | 5   | SWCLK_XIAO    | U6-1                   | SWCLK                  |
| H5               | 6   | BUZZERWIRE    | U6-2                   | BUZZERWIRE             |
| H5               | 7   | F             | U6-4                   | F                      |
| H5               | 8   | E             | U6-6                   | E                      |
| H5               | 9   | D             | U6-8                   | D                      |
| H5               | 10  | C             | U6-10                  | C                      |
| H6               | 1   | DP            | U7-9                   | DP                     |
| H6               | 2   | LED1WIRE      | U7-7                   | LED1WIRE               |
| H6               | 3   | LDRWIRE       | U7-5                   | LDRWIRE                |
| H6               | 4   | SW5WIRE       | U7-3                   | SW5WIRE                |
| H6               | 5   | SW4WIRE       | U7-1                   | SW4WIRE                |
| H6               | 6   | RSWITCH       | U7-2                   | RSWITCH                |
| H6               | 7   | NC            | U7-4                   | NC                     |
| H6               | 8   | SW3WIRE       | U7-6                   | SW3WIRE                |
| H6               | 9   | G             | U7-8                   | G                      |
| H6               | 10  | SW2WIRE       | U7-10                  | SW2WIRE                |
| H2               | 1   | NC            | H3-1                   | +5V                    |
| H2               | 2   | XIAOTX        | H3-2                   | XIAOTX                 |
| H2               | 3   | XIAOMOSI      | H3-3                   | XIAOMOSI               |
| H2               | 4   | XIAOMISO      | H3-4                   | XIAOMISO               |
| H2               | 5   | XIAOSCL       | H3-5                   | XIAOSCL                |
| H2               | 6   | XIAORX        | H3-6                   | XIAORX                 |
| H3               | 1   | XIAO3V3       | H4-1                   | VCC                    |
| H3               | 2   | SDA           | H4-2                   | SDA                    |
| H3               | 3   | SCL           | H4-3                   | SCL                    |
| H3               | 4   | NC            | H4-4                   | NC                     |
| H3               | 5   | NC            | H4-5                   | NC                     |
| H3               | 6   | GND           | H4-6                   | GND                    |

## Other Connectors

| Connector              | Pin | Net             | Function                              |
|:-----------------------|:----|:----------------|:--------------------------------------|
| H1 (SWD, RP2040 debug) | 1   | VCC             | Telemetry 3V3                         |
| H1 (SWD, RP2040 debug) | 2   | SWCLK           | RP2040 SWCLK (pin 24)                 |
| H1 (SWD, RP2040 debug) | 3   | SWD             | RP2040 SWDIO (pin 25)                 |
| H1 (SWD, RP2040 debug) | 4   | GND             | -                                     |
| SW1                    | -   | RUN             | RP2040 RUN (pin 26), reset button     |
| SW3                    | -   | QSPI_SS         | BOOTSEL button (via R3 1k)            |
| USB1                   | -   | USB_D+ / USB_D- | RP2040 USB_DP/DM (pins 47/46) via 27Ω |
| USB1                   | -   | VUSB            | AMS1117-3.3 input → telemetry VCC     |
| LED1                   | -   | VCC             | Power LED (470Ω)                      |

## Notes
- **Hardware peripherals don't line up with the routed pins; plan on PIO for bus capture.**
  - I²C: SCL is on GPIO12 and SDA on GPIO13. In the RP2040's fixed I²C mapping GPIO12 is I2C0 SDA and GPIO13 is I2C0 SCL, so the pair is swapped for the hardware block.
  - UART: XIAOTX lands on GPIO6, which has no UART RX function. XIAORX is on GPIO4 (UART1 TX), so the RP2040 can drive the XIAO's RX with hardware UART but can't receive the XIAO's TX that way.
  - SPI: XIAOMOSI (GPIO1), XIAOMISO (GPIO5) and XIAOSCL (GPIO10) don't form a hardware SPI set.
- XIAO serial/SPI lines and the 7-segment lines are never visible at the same time; see `MODESEL`.
- `XIAO3V3` is the baseboard `VCC` rail (H4-1), buffered into ADC1 for supply monitoring. The telemetry board's own `VCC` is a separate 3V3 from USB; the boards share only GND (H4-6).
- Baseboard H3-1 (+5V) is not connected on the telemetry side.
- GPIO28 and GPIO29 are unused.
- The telemetry net names `SWCLK` and `SWD` belong to the RP2040's own debug port (H1), **not** the baseboard XIAO SWD lines (`SWCLK_XIAO`, `SWIO_XIAO`, `RST_XIAO`).
