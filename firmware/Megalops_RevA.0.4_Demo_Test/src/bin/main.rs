#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_println::println;
use uferris_bsp::{SevenSegDigit, uferris_init};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let mut uferris = uferris_init(peripherals);

    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u64 = 200;

    // Set RTC to Known Time for Testing
    uferris.set_rtc_time(2026, 1, 29, 9, 16, 0).unwrap();

    loop {
        // ---------------------------------------
        // LED Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing LED Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );
        uferris.led1_on();
        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris.led1_off();
        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris.led2_on().unwrap();
        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris.led2_off().unwrap();
        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris.led3_on().unwrap();
        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris.led3_off().unwrap();
        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        // ---------------------------------------
        // Seven Segment Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing Seven Segment Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit1, Some(8))
            .unwrap();

        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit1, None)
            .unwrap();

        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit2, Some(8))
            .unwrap();

        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit2, None)
            .unwrap();

        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris.seven_segment_display_colon_en(true).unwrap();

        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris.seven_segment_display_colon_en(false).unwrap();

        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit3, Some(8))
            .unwrap();

        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit3, None)
            .unwrap();

        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit4, Some(8))
            .unwrap();

        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit4, None)
            .unwrap();

        // ---------------------------------------
        // Buzzer Test
        // ---------------------------------------

        let datetime = uferris.rtc.read_time().unwrap();
        println!(
            "Commencing Buzzer Testing at {}:{}:{} on {}/{}/{}",
            datetime.2, datetime.1, datetime.0, datetime.5, datetime.4, datetime.3
        );

        Timer::after(Duration::from_millis(TEST_DELAY_MS)).await;

        uferris.buzz_on(1024);

        Timer::after(Duration::from_millis(2000)).await;

        uferris.buzz_off();

        // ---------------------------------------
        // LDR Testing
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing LDR Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );
        println!("Taking Three Readings with 3s Delay Between Readings");
        let ldr_value = uferris.read_ldr();
        println!("First LDR Reading: {}", ldr_value);
        println!("Waiting 3 Seconds...");
        Timer::after(Duration::from_millis(3000)).await;
        let ldr_value = uferris.read_ldr();
        println!("Second LDR Reading: {}", ldr_value);
        println!("Waiting 3 Seconds...");
        Timer::after(Duration::from_millis(3000)).await;
        let ldr_value = uferris.read_ldr();
        println!("Third LDR Reading: {}", ldr_value);
        println!("LDR Reading Test Complete.");

        // ---------------------------------------
        // Button Press Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing Button Press Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        println!("Button Press Testing");
        println!("Press Button 1 to Proceed...");
        while !uferris.read_sw1().unwrap() {}
        println!("Button 1 Pressed! Continuing...");
        println!("Press Button 2 to Proceed...");
        while !uferris.read_sw2().unwrap() {}
        println!("Button 2 Pressed! Continuing...");
        println!("Press Button 3 to Proceed...");
        while !uferris.read_sw3().unwrap() {}
        println!("Button 3 Pressed! Continuing...");
        println!("Press Button 4 to Proceed...");
        while !uferris.read_sw4().unwrap() {}
        println!("Button 4 Pressed! Continuing...");
        println!("Press Button 5 to Proceed...");
        while !uferris.read_sw5() {}
        println!("Button 5 Pressed! Continuing...");

        // ---------------------------------------
        // Slide Switch Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing Slide Switch Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        println!("Slide Switch Testing");
        println!("Starting with Switch 6 (Left Switch)...");
        let current_sw6_state = uferris.read_sw6().unwrap();
        let mut new_sw6_state = uferris.read_sw6().unwrap();
        println!("Current Slide Switch 6 State: {:?}", current_sw6_state);
        println!("Toggle Slide Switch 6 to Proceed...");
        while current_sw6_state == new_sw6_state {
            new_sw6_state = uferris.read_sw6().unwrap();
        }
        println!("Slide Switch 6 State Changed");
        println!("Current Slide Switch 6 State: {:?}", new_sw6_state);

        Timer::after(Duration::from_millis(2000)).await;

        println!("Moving to Switch 7 (Right Switch)...");
        let current_sw7_state = uferris.read_sw7().unwrap();
        let mut new_sw7_state = uferris.read_sw7().unwrap();
        println!("Current Slide Switch 7 State: {:?}", current_sw7_state);
        println!("Toggle Slide Switch 7 to Proceed...");
        while current_sw7_state == new_sw7_state {
            new_sw7_state = uferris.read_sw7().unwrap();
        }
        println!("Slide Switch 7 State Changed");
        println!("Current Slide Switch 7 State: {:?}", new_sw7_state);
        Timer::after(Duration::from_millis(2000)).await;

        // ---------------------------------------
        // Powerboard Testing
        // Reading INA219 Voltage, Current, Power
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing Power Measurement Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        let voltage = uferris.read_system_voltage();
        let current = uferris.read_system_current();
        let power = uferris.read_system_power();
        println!("Battery Voltage: {:?} mV", voltage);
        println!("Battery Current: {:?} uA", current);
        println!("Battery Power: {:?} uW", power);

        // ---------------------------------------
        // Powerboard Testing
        // Reading from and Writing to SD Card
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing SD Card Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        let sd_card = uferris.init_sd_card();

        match sd_card {
            Ok(vol_mgr) => {
                println!("SD Card Initialized Successfully.");
                println!("SD Card Size: {} Bytes", vol_mgr);

                uferris.write_to_file_in_root("TEST.TXT", "hello".as_bytes());
                println!("Wrote 'hello' to 'TEXT.TXT' file in root directory.");

                println!("Reading 'TEST.TXT' file from root directory:");
                let result = uferris.read_file_chunked("TEST.TXT", |chunk| {
                    println!("Chunk: {:?}", core::str::from_utf8(chunk).unwrap_or("??"));
                });

                if result.is_err() {
                    println!("Read error occurred!");
                }
            }
            Err(e) => {
                println!("Failed to initialize SD Card: {:?}", e);
            }
        }

        println!("Testing Loop Complete. Restarting...");
    }
}
