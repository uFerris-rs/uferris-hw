#![no_std]
#![no_main]

use core::cell::RefCell;

use bitmask_enum::bitmask;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal_bus::i2c::{self, AtomicDevice, RefCellDevice};
use embedded_hal_bus::util::AtomicCell;
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, InputPin, Io, Output, OutputPin, Pin};
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::ledc::{
    channel, channel::ChannelIFace, timer, timer::TimerIFace, LSGlobalClkSource, Ledc, LowSpeed,
};
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Blocking;
use esp_println::println;
extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// TCA6424 Commands
const TCA6424_ADDR: u8 = 0x22; // I2C address of the TCA6424
const IN_PORT0: u8 = 0x80; // Input Port 0 Register
const IN_PORT1: u8 = 0x81; // Input Port 1 Register
const IN_PORT2: u8 = 0x82; // Input Port 2 Register
const OUT_PORT0: u8 = 0x84; // Output Port 0 Register
const OUT_PORT1: u8 = 0x85; // Output Port 1 Register
const OUT_PORT2: u8 = 0x86; // Output Port 2 Register
const POL_INV_PORT0: u8 = 0x88; // Polarity Inversion Port 0 Register
const POL_INV_PORT1: u8 = 0x89; // Polarity Inversion Port 1 Register
const POL_INV_PORT2: u8 = 0x8A; // Polarity Inversion Port 2 Register
const CONFIG_PORT0: u8 = 0x8C; // Configuration Port 0 Register
const CONFIG_PORT1: u8 = 0x8D; // Configuration Port 1 Register
const CONFIG_PORT2: u8 = 0x8E; // Configuration Port 2 Register

#[bitmask(u8)]
pub enum IoExpPort0 {
    Sw7Pos1, // Port 0 Pin 0
    Sw7Pos2, // Port 0 Pin 1
    P02,     // Port 0 Pin 2 (Unused)
    Sw4,     // Port 0 Pin 3
    Sw3,     // Port 0 Pin 4
    Sw2,     // Port 0 Pin 5
    Sw1,     // Port 0 Pin 6
    SegG,    // Port 0 Pin 7
}

#[bitmask(u8)]
pub enum IoExpPort1 {
    Dp,   // Port 1 Pin 0
    SegA, // Port 1 Pin 1
    SegB, // Port 1 Pin 2
    SegC, // Port 1 Pin 3
    SegD, // Port 1 Pin 4
    SegE, // Port 1 Pin 5
    SegF, // Port 1 Pin 6
    P17,  // Port 1 Pin 7 (Unused)
}

#[bitmask(u8)]
pub enum IoExpPort2 {
    Digit4,  // Port 2 Pin 0
    Digit3,  // Port 2 Pin 1
    Digit2,  // Port 2 Pin 2
    Digit1,  // Port 2 Pin 3
    Led3,    // Port 2 Pin 4
    Led2,    // Port 2 Pin 5
    Sw6Pos1, // Port 2 Pin 6
    Sw6Pos2, // Port 2 Pin 7
}

// Direction Configuration for TCA6424
// Port 0: All outputs except for SW7 and SW4
// Port 1: All outputs except for DP, SegA, SegB, SegC
// Port 2: All outputs except for Digit4, Digit3, Digit2, Digit
const PORT0_DIR: u8 = 0x7F;
const PORT1_DIR: u8 = 0x00;
const PORT2_DIR: u8 = 0xC0;

// Test Sequence:
// Spawn LED Task to Blink all LEDs
// Spawn Seven Segment Display Task to Cycle through all segments
// Spawn Buzzer Task to Buzz at 2 second intervals
// Read LDR (currently unavailable)
// Set RTC to some value (e.g., 2025-08-01 12:00:00)
// Wait 2 seconds (show message)
// Read RTC value and print it
// User Interaction Tests: Prompt user to press any button or slide switch
// On every user interaction, print the button pressed or switch toggled and show LDR (currently unavailable) and RTC values

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    // XIAO Configurations
    let mut led1 = Output::new(
        peripherals.GPIO3,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let sw5 = Input::new(
        peripherals.GPIO5,
        InputConfig::default().with_pull(esp_hal::gpio::Pull::Up),
    );

    // Master I2C Configuration
    let mut i2c0 = I2c::new(
        peripherals.I2C0,
        Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_scl(peripherals.GPIO7)
    .with_sda(peripherals.GPIO6);

    // Configure I/O Expander pins
    // Write I/O Direction to TCA6424 Config Registers
    i2c0.write(
        TCA6424_ADDR,
        &[CONFIG_PORT0, PORT0_DIR, PORT1_DIR, PORT2_DIR],
    )
    .unwrap();
    // Reset all output ports to low
    i2c0.write(TCA6424_ADDR, &[OUT_PORT0, 0, 0, 0]).unwrap();

    // Since more than one task will share the same I2C
    // It should be configured is a shared type (e.g. Mutex) for shared access

    spawner.spawn(led_task(led1)).unwrap();

    // Configure and Test LDR (miswired on v0.1)
    // Create handle for ADC configuration parameters
    // let mut adc_config = AdcConfig::new();

    // // Configure ADC pin
    // let mut ldr = adc_config.enable_pin(peripherals.GPIO20, Attenuation::_11dB);

    // // Create ADC Driver
    // let mut adc = Adc::new(peripherals.ADC1, adc_config);

    // Configure Buzzer PWM
    // Create LEDC instance with low speed global clock
    let mut buzz = Ledc::new(peripherals.LEDC);
    buzz.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // Configure LEDC timer
    let mut timer = buzz.timer::<LowSpeed>(timer::Number::Timer0);
    timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(2700u32),
        })
        .unwrap();

    // Configure LEDC Channel Attaching Timer and Pin
    let mut buzz_channel = buzz.channel(channel::Number::Channel0, peripherals.GPIO2);
    buzz_channel
        .configure(channel::config::Config {
            timer: &timer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    // Testing LEDs
    println!("Testing LEDs...");
    println!("Turning On LED1");

    // Turn on LED1
    led1.set_high();
    Timer::after(Duration::from_secs(1)).await;

    // Turn off LED1
    println!("Turning Off LED1");
    led1.set_low();
    Timer::after(Duration::from_secs(1)).await;
    println!("LED1 Test Completed!");

    // Turn on LED3
    println!("Turning On LED3");
    i2c0.write(TCA6424_ADDR, &[OUT_PORT2, IoExpPort2::Led3.bits()])
        .unwrap();
    Timer::after(Duration::from_secs(1)).await;
    // Turn off LED3
    println!("Turning Off LED3");
    i2c0.write(TCA6424_ADDR, &[OUT_PORT2, 0]).unwrap();
    Timer::after(Duration::from_secs(1)).await;
    println!("LED3 Test Completed!");

    // Turn on LED2
    println!("Turning On LED2");
    i2c0.write(TCA6424_ADDR, &[OUT_PORT2, IoExpPort2::Led2.bits()])
        .unwrap();
    Timer::after(Duration::from_secs(1)).await;
    // Turn off LED2
    println!("Turning Off LED2");
    i2c0.write(TCA6424_ADDR, &[OUT_PORT2, 0]).unwrap();
    Timer::after(Duration::from_secs(1)).await;
    println!("LED2 Test Completed!");

    // Testing Buzzer
    println!("Testing Buzzer...");
    // Set Buzzer to 50% duty cycle
    buzz_channel.set_duty(50_u8).unwrap(); // 50% of 16384 (14-bit resolution)
    println!("Buzzer On for 1 second...");
    Timer::after(Duration::from_secs(1)).await;
    // Turn off Buzzer
    println!("Turning Off Buzzer");
    buzz_channel.set_duty(0).unwrap();
    Timer::after(Duration::from_secs(1)).await;
    println!("Buzzer Test Completed!");

    // Button Tests
    println!("Testing Buttons...");
    // SW1 Test
    let mut rbuf: [u8; 1] = [0xFF];
    println!("Press SW1 to Proceed");
    while rbuf[0] & IoExpPort0::Sw1.bits == IoExpPort0::Sw1.bits {
        i2c0.write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf)
            .unwrap();
    }
    rbuf[0] = 0xFF; // Clear buffer for next read
    println!("SW1 Pressed! Proceeding...");
    // SW2 Test
    println!("Press SW2 to Proceed");
    while rbuf[0] & IoExpPort0::Sw2.bits == IoExpPort0::Sw2.bits {
        i2c0.write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf)
            .unwrap();
    }
    rbuf[0] = 0xFF; // Clear buffer for next read
    println!("SW2 Pressed! Proceeding...");
    // SW3 Test
    println!("Press SW3 to Proceed");
    while rbuf[0] & IoExpPort0::Sw3.bits == IoExpPort0::Sw3.bits {
        i2c0.write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf)
            .unwrap();
    }
    rbuf[0] = 0xFF; // Clear buffer for next read
    println!("SW3 Pressed! Proceeding...");
    // SW4 Test
    println!("Press SW4 to Proceed");
    while rbuf[0] & IoExpPort0::Sw4.bits == IoExpPort0::Sw4.bits {
        i2c0.write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf)
            .unwrap();
    }
    rbuf[0] = 0xFF; // Clear buffer for next read
    println!("SW4 Pressed! Proceeding...");
    // SW5 Test
    println!("Press SW5 to Proceed");
    while sw5.is_low() {
        Timer::after(Duration::from_millis(100)).await;
    }
    println!("SW5 Pressed! Proceeding...");

    // LDR Tests
    // println!("Testing LDR...");

    // Slide Switch Tests
    println!("Testing Slide Switches...");

    // Seven Segment Display Tests
    println!("Testing Seven Segment Display...");
    // Turn on one segment at a time from all digits
    for segment in [
        IoExpPort1::SegA,
        IoExpPort1::SegB,
        IoExpPort1::SegC,
        IoExpPort1::SegD,
        IoExpPort1::SegE,
        IoExpPort1::SegF,
        IoExpPort1::Dp,
    ] {
        // Set the digits to high
        i2c0.write(
            TCA6424_ADDR,
            &[
                OUT_PORT2,
                IoExpPort2::Digit1
                    .or(IoExpPort2::Digit2)
                    .or(IoExpPort2::Digit3)
                    .or(IoExpPort2::Digit4)
                    .bits(),
            ],
        )
        .unwrap();
        // Set the segment to high
        i2c0.write(TCA6424_ADDR, &[OUT_PORT1, segment.bits()])
            .unwrap();
        Timer::after(Duration::from_millis(300)).await;
        // Turn off the segment
        i2c0.write(TCA6424_ADDR, &[OUT_PORT1, 0]).unwrap();
    }
    // Set the segment to high
    i2c0.write(TCA6424_ADDR, &[OUT_PORT0, IoExpPort0::SegG.bits()])
        .unwrap();
    Timer::after(Duration::from_secs(1)).await;
    // Turn off the segment
    i2c0.write(TCA6424_ADDR, &[OUT_PORT0, 0]).unwrap();

    // RTC Test
    println!("Testing RTC...");

    // Test Finished Press Button X to Repeat
    println!("Test Finished! Press SW5 to Repeat...");

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn led_task(led: Output<'static>) {}
