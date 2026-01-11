#![no_std]
#![no_main]

use bitmask_enum::bitmask;
use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::ledc::{
    channel, channel::ChannelIFace, timer, timer::Timer as LedcTimer, timer::TimerIFace,
    LSGlobalClkSource, Ledc, LowSpeed,
};
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
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

const RTC_ADDR: u8 = 0x68;

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

// Mutex to Share same I2C singleton across threads
static SHARED_I2C: Mutex<CriticalSectionRawMutex, RefCell<Option<I2c<'static, Blocking>>>> =
    Mutex::new(RefCell::new(None));

// Function to convert to BCD from decimal (for RTC)
fn to_bcd(val: u8) -> u8 {
    ((val / 10) << 4) | (val % 10)
}

// Function to convert from BCD
fn from_bcd(val: u8) -> u8 {
    10 * (val >> 4) + (val & 0x0F)
}

// Function to set RTC device parameters
fn set_rtc(
    i2c: &mut I2c<'static, Blocking>,
    year: u16,
    month: u8,
    day: u8,
    hour: u8,
    min: u8,
    sec: u8,
) {
    let year_u8 = (year - 2000) as u8;
    let sec_bcd = to_bcd(sec);
    let min_bcd = to_bcd(min);
    let hour_bcd = to_bcd(hour);
    let day_bcd = to_bcd(day);
    let weekday_bcd = to_bcd(5);
    let month_bcd = to_bcd(month);
    let year_bcd = to_bcd(year_u8);
    i2c.write(
        RTC_ADDR,
        &[
            0x00,
            sec_bcd,
            min_bcd,
            hour_bcd,
            weekday_bcd,
            day_bcd,
            month_bcd,
            year_bcd,
        ],
    )
    .unwrap();
}

// Function to read RTC parameters
fn read_rtc(i2c: &mut I2c<Blocking>) -> (u16, u8, u8, u8, u8, u8) {
    let mut buf = [0u8; 7];
    i2c.write_read(RTC_ADDR, &[0x00], &mut buf).unwrap();
    let sec = from_bcd(buf[0] & 0x7F);
    let min = from_bcd(buf[1] & 0x7F);
    let hour = from_bcd(buf[2] & 0x3F);
    let day = from_bcd(buf[4] & 0x3F);
    let month = from_bcd(buf[5] & 0x1F);
    let year = from_bcd(buf[6]) as u16 + 2000;
    (year, month, day, hour, min, sec)
}

// Test Sequence:
// Spawn LED Task to Blink all LEDs
// Spawn Seven Segment Display Task to Cycle through all segments
// Spawn Buzzer Task to Buzz at 2 second intervals
// Read LDR (currently unavailable)
// Set RTC to some value (e.g., 2025-08-01 12:00:00)
// Wait 2 seconds (show message)
// Read RTC value and print it
// User Interaction Tests: Prompt user to press any button or slide switch
// On every user (button/switch) interaction, print the button pressed or switch toggled

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    // XIAO GPIO Pin Configuration
    let led1 = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    let sw5 = Input::new(
        peripherals.GPIO5,
        InputConfig::default().with_pull(Pull::Up),
    );

    // Master I2C Configuration
    // Bus Addresses:
    // RTC -> 0x68
    // I/O Expander -> 0x22
    // Pins:
    // SCL -> GPIO7 on XIAO
    // SDA -> GPIO6 on XIAO
    let mut i2c0 = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_scl(peripherals.GPIO7)
    .with_sda(peripherals.GPIO6);

    // Configure I/O Expander pins before moving I2C to shared state
    // Write I/O Direction to TCA6424 Config Registers
    i2c0.write(
        TCA6424_ADDR,
        &[CONFIG_PORT0, PORT0_DIR, PORT1_DIR, PORT2_DIR],
    )
    .unwrap();
    // Reset all output ports to low
    i2c0.write(TCA6424_ADDR, &[OUT_PORT0, 0, 0, 0]).unwrap();

    // Move I2C to shared state to share among tasks
    SHARED_I2C.lock(|i2c| i2c.borrow_mut().replace(i2c0));

    // Configure Buzzer PWM
    // Buzzer connected to GPIO2 on XIAO
    // Create LEDC instance with low speed global clock
    let mut buzz = Ledc::new(peripherals.LEDC);
    buzz.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // Configure LEDC timer
    let timer = buzz.timer::<LowSpeed>(timer::Number::Timer0);

    // Configure LEDC Channel Attaching Timer and Pin
    let buzz_channel = buzz.channel(channel::Number::Channel0, peripherals.GPIO2);

    // Spawn LED, Seven Seg, and Buzzer tasks
    spawner.spawn(led_task(led1)).unwrap();
    spawner.spawn(sseg_task()).unwrap();
    spawner.spawn(buzzer_task(buzz_channel, timer)).unwrap();

    // Read LDR (currently unavailable)
    println!("Reading LDR (currently unavailable)");

    // Set RTC to some value (e.g., 2025-08-01 12:00:00)
    SHARED_I2C.lock(|cell| {
        let mut i2c_ref = cell.borrow_mut();
        let i2c = i2c_ref.as_mut().unwrap();
        set_rtc(i2c, 2025, 8, 1, 12, 0, 0);
    });
    println!("RTC set to 2025-08-01 12:00:00");

    // Wait 2 seconds (show message)
    println!("Waiting 2 seconds...");
    Timer::after(Duration::from_secs(2)).await;

    // Read RTC value and print it
    SHARED_I2C.lock(|cell| {
        let mut i2c_ref = cell.borrow_mut();
        let i2c = i2c_ref.as_mut().unwrap();
        let (y, m, d, h, min, s) = read_rtc(i2c);
        println!(
            "RTC value: {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
            y, m, d, h, min, s
        );
    });

    println!("To Test Buttons, Press Any Button or Slide a Switch");

    #[derive(PartialEq, Debug, Clone, Copy)]
    enum Swstate {
        Top,
        Bottom,
        Undefined,
    }

    let mut crswstate = Swstate::Undefined;
    let mut clswstate = Swstate::Undefined;
    let mut prswstate = Swstate::Undefined;
    let mut plswstate = Swstate::Undefined;
    let mut port0 = 0u8;
    let mut port2 = 0u8;

    SHARED_I2C.lock(|cell| {
        let mut i2c_ref = cell.borrow_mut();
        let i2c = i2c_ref.as_mut().unwrap();
        let mut rbuf = [0u8];
        i2c.write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf)
            .unwrap();
        port0 = rbuf[0];
        i2c.write_read(TCA6424_ADDR, &[IN_PORT2], &mut rbuf)
            .unwrap();
        port2 = rbuf[0];
    });

    if port0 & IoExpPort0::Sw7Pos1.bits() != 0 {
        crswstate = Swstate::Top;
        prswstate = Swstate::Top;
    }
    if port0 & IoExpPort0::Sw7Pos2.bits() != 0 {
        crswstate = Swstate::Bottom;
        prswstate = Swstate::Bottom;
    }
    if port2 & IoExpPort2::Sw6Pos1.bits() != 0 {
        clswstate = Swstate::Bottom;
        plswstate = Swstate::Bottom;
    }
    if port2 & IoExpPort2::Sw6Pos2.bits() != 0 {
        clswstate = Swstate::Top;
        plswstate = Swstate::Top;
    }

    loop {
        let mut interaction_detected = false;

        SHARED_I2C.lock(|cell| {
            let mut i2c_ref = cell.borrow_mut();
            let i2c = i2c_ref.as_mut().unwrap();
            let mut rbuf = [0u8];
            i2c.write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf)
                .unwrap();
            port0 = rbuf[0];
            i2c.write_read(TCA6424_ADDR, &[IN_PORT2], &mut rbuf)
                .unwrap();
            port2 = rbuf[0];
        });

        if port0 & IoExpPort0::Sw1.bits() == 0 {
            println!("SW1 pressed");
            interaction_detected = true;
        }
        if port0 & IoExpPort0::Sw2.bits() == 0 {
            println!("SW2 pressed");
            interaction_detected = true;
        }
        if port0 & IoExpPort0::Sw3.bits() == 0 {
            println!("SW3 pressed");
            interaction_detected = true;
        }
        if port0 & IoExpPort0::Sw4.bits() == 0 {
            println!("SW4 pressed");
            interaction_detected = true;
        }
        if port0 & IoExpPort0::Sw7Pos1.bits() != 0 {
            crswstate = Swstate::Top;
        }
        if port0 & IoExpPort0::Sw7Pos2.bits() != 0 {
            crswstate = Swstate::Bottom;
        }
        if crswstate != prswstate {
            prswstate = crswstate;
            interaction_detected = true;
        }
        if port2 & IoExpPort2::Sw6Pos1.bits() != 0 {
            clswstate = Swstate::Bottom;
        }
        if port2 & IoExpPort2::Sw6Pos2.bits() != 0 {
            clswstate = Swstate::Top;
        }
        if clswstate != plswstate {
            plswstate = clswstate;
            interaction_detected = true;
        }

        if sw5.is_low() {
            interaction_detected = true;
            println!("SW5 pressed");
        }

        if interaction_detected {
            SHARED_I2C.lock(|cell| {
                let mut i2c_ref = cell.borrow_mut();
                let i2c = i2c_ref.as_mut().unwrap();
                let (y, m, d, h, min, s) = read_rtc(i2c);
                println!(
                    "At time: {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                    y, m, d, h, min, s
                );
            });
            println!("Current Right Switch State: {:?}", crswstate);
            println!("Current Left Switch State: {:?}", clswstate);
            println!("Current LDR Reading Unavailable");
            println!("");
            // println!("");
        }

        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn led_task(mut led1: Output<'static>) {
    let led_bits = IoExpPort2::Led2.or(IoExpPort2::Led3).bits();
    loop {
        // Turn on all LEDs
        led1.set_high();
        SHARED_I2C.lock(|cell| {
            let mut i2c_ref = cell.borrow_mut();
            let i2c = i2c_ref.as_mut().unwrap();
            let mut rbuf = [0u8];
            i2c.write_read(TCA6424_ADDR, &[OUT_PORT2], &mut rbuf)
                .unwrap();
            let current = rbuf[0];
            let new = current | led_bits;
            i2c.write(TCA6424_ADDR, &[OUT_PORT2, new]).unwrap();
        });
        Timer::after(Duration::from_secs(2)).await;

        // Turn off all LEDs
        led1.set_low();
        SHARED_I2C.lock(|cell| {
            let mut i2c_ref = cell.borrow_mut();
            let i2c = i2c_ref.as_mut().unwrap();
            let mut rbuf = [0u8];
            i2c.write_read(TCA6424_ADDR, &[OUT_PORT2], &mut rbuf)
                .unwrap();
            let current = rbuf[0];
            let new = current & !led_bits;
            i2c.write(TCA6424_ADDR, &[OUT_PORT2, new]).unwrap();
        });
        Timer::after(Duration::from_secs(2)).await;
    }
}

#[embassy_executor::task]
async fn sseg_task() {
    let segments = [
        IoExpPort1::SegA,
        IoExpPort1::SegB,
        IoExpPort1::SegC,
        IoExpPort1::SegD,
        IoExpPort1::SegE,
        IoExpPort1::SegF,
        IoExpPort1::Dp,
    ];

    // Set all digits on
    SHARED_I2C.lock(|cell| {
        let mut i2c_ref = cell.borrow_mut();
        let i2c = i2c_ref.as_mut().unwrap();
        let mut rbuf = [0u8];
        i2c.write_read(TCA6424_ADDR, &[OUT_PORT2], &mut rbuf)
            .unwrap();
        let current = rbuf[0];
        let digits = IoExpPort2::Digit1
            .or(IoExpPort2::Digit2)
            .or(IoExpPort2::Digit3)
            .or(IoExpPort2::Digit4)
            .bits();
        let new = current | digits;
        i2c.write(TCA6424_ADDR, &[OUT_PORT2, new]).unwrap();
    });

    loop {
        for &seg in segments.iter() {
            SHARED_I2C.lock(|cell| {
                let mut i2c_ref = cell.borrow_mut();
                let i2c = i2c_ref.as_mut().unwrap();
                i2c.write(TCA6424_ADDR, &[OUT_PORT1, !seg.bits()]).unwrap();
            });
            Timer::after(Duration::from_millis(300)).await;
            SHARED_I2C.lock(|cell| {
                let mut i2c_ref = cell.borrow_mut();
                let i2c = i2c_ref.as_mut().unwrap();
                i2c.write(TCA6424_ADDR, &[OUT_PORT1, 0xFF]).unwrap();
            });
        }

        // Cycle SegG separately
        SHARED_I2C.lock(|cell| {
            let mut i2c_ref = cell.borrow_mut();
            let i2c = i2c_ref.as_mut().unwrap();
            i2c.write(TCA6424_ADDR, &[OUT_PORT0, !IoExpPort0::SegG.bits()])
                .unwrap();
        });
        Timer::after(Duration::from_millis(300)).await;
        SHARED_I2C.lock(|cell| {
            let mut i2c_ref = cell.borrow_mut();
            let i2c = i2c_ref.as_mut().unwrap();
            i2c.write(TCA6424_ADDR, &[OUT_PORT0, 0xFF]).unwrap();
        });
    }
}

#[embassy_executor::task]
async fn buzzer_task(
    mut buzz_channel: channel::Channel<'static, LowSpeed>,
    mut timer: LedcTimer<'static, LowSpeed>,
) {
    timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(2700u32),
        })
        .unwrap();

    buzz_channel
        .configure(channel::config::Config {
            timer: &timer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    loop {
        buzz_channel.set_duty(50u8).unwrap();
        Timer::after(Duration::from_millis(500)).await;
        buzz_channel.set_duty(0u8).unwrap();
        Timer::after(Duration::from_millis(1500)).await;
    }
}
