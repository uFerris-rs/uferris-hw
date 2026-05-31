# 🦀 uFerris Embedded Rust Learner Board

<div align="center">
  <img src="assets/uFerris_bb.png" width="400">
</div>
<br>

**uFerris** is a versatile, all-in-one learner board designed to:

- Support multiple controllers on a single reference platform.
- Enable programming of standard peripherals (GPIO, Timers/Counters, Analog, PWM, Serial Comms).
- Allow learners to build a complete embedded product replica.
- Serve as a centralized reference for Rust embedded beginners.

## 📦 Purchase

For pre-built hardware, uFerris can be acquired from [The Embedded Rustacean Store](https://shop.theembeddedrustacean.com/).

## 🧱 Board Variants

### Megalops Baseboard
The baseboard is equipped with components that let you practice and apply your skills across all standard peripherals. The baseboard also features a SeeedStudio XIAO platform header, giving the flexibility to work with a wide range of controllers.

<div align="center">
  <img src="assets/uFerris_Components.png" width="500">
</div>
<br>

<div align="center">
  <img src="assets/baseboardblockdiag.png" width="500">
</div>
<br>

### Megalops Power Expansion Board

The Power Expansion Board lets the uFerris baseboard run independently of USB power. It features a 2×AAA battery holder with a current measurement circuit, making it easy to measure current consumption for your projects. It also includes an SD card holder—perfect for learning SPI and storing or logging data directly from your applications.

<div align="center">
  <img src="assets/uferris_pb.png" width="500">
</div>
<br>



## 📁 Repository Structure
- [`boards/`](boards/) → Hardware designs (schematic, PCB, BOM)
- [`docs/`](docs/) → Documentation and certifications
- [`firmware/`](firmware/) → Example Rust firmware projects
- [`assets/`](assets/) → Images and media assets

## 🏁 Getting Started
To get started, refer to the [getting-started.md](https://github.com/uFerris-rs/uferris-hw/blob/main/docs/getting-started.md) document.

## 📄 Certifications

uFerris is certified open source hardware by the Open Source Hardware Association.

<div align="center">
<a href="https://certification.oshwa.org/jo000001.html">
  <img src="assets/certification-mark-JO000001-stacked.png" alt="OSHW Certified JO000001" width="200">
</a>
</div>


- **OSHWA UID:** [JO000001](https://certification.oshwa.org/jo000001.html)
- **Certification Date:** May 8, 2026
- **Hardware License:** CC-BY-SA-4.0
- **Software License:** MIT

uFerris is the first OSHWA-certified project from Jordan.

See [`docs/`](docs/) for more compliance details.

## 📜 License

All uFerris boards and their derivatives are licensed
under the Creative Commons Attribution-ShareAlike 4.0 International License (CC BY-SA 4.0).

All software and firmware examples are licensed under the MIT License.


## 🌐 Learn More

To stay updated and learn more about uFerris, subscribe to The Embedded Rustacean newsletter: https://www.theembeddedrustacean.com/subscribe 


---

Developed and maintained by **The Embedded Rustacean**  🦀
