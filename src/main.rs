#![no_std]
#![no_main]

use alloc::format;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    delay::Delay,
    gpio::Io,
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

extern crate alloc;
use core::mem::MaybeUninit;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);
    init_heap();

    esp_println::logger::init_logger_from_env();

    let timer = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None).timer0;
    let _init = esp_wifi::initialize(
        esp_wifi::EspWifiInitFor::Wifi,
        timer,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio25, // SDA
        io.pins.gpio26, // SCL
        400u32.kHz(),
        &clocks,
        None,
    );

    // Start Scan at Address 1 going up to 127
    // for addr in 1..=127 {
    //     log::info!("Reading {}", addr as u8);

    //     let res = i2c.read(addr as u8, &mut [0]);
    //     if res.is_ok() {
    //         log::info!("Device Found at Address {}", addr as u8);
    //     } else {
    //         log::info!("error reading {}", addr as u8);
    //     }
    // }

    // 0x3c
    let interface = I2CDisplayInterface::new(i2c);

    let driver = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0);

    // let mut display = driver.into_terminal_mode();
    let mut display = driver.into_buffered_graphics_mode();

    log::debug!("config");
    display.set_display_on(true).unwrap();
    display.set_brightness(Brightness::BRIGHTEST).unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let sensor_1_pin = io.pins.gpio32;
    let sensor_2_pin = io.pins.gpio33;

    // Create ADC instances
    let mut adc1_config = AdcConfig::new();
    let mut pin1_adc1_pin = adc1_config.enable_pin(sensor_1_pin, Attenuation::Attenuation11dB);
    let mut pin2_adc1_pin = adc1_config.enable_pin(sensor_2_pin, Attenuation::Attenuation11dB);

    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    fn linear_interpolation(adc: f64) -> f64 {
        let psi_at_min = 0.0;
        let adc_at_min = 310.0; // 0.5 v / 2

        let adc_at_max = 2792.0; // 4.5 v / 2
        let psi_at_max = 100.0;

        psi_at_min + ((adc - adc_at_min) * (psi_at_max - psi_at_min)) / (adc_at_max - adc_at_min)
    }

    loop {
        let pin1_value: u16 = nb::block!(adc1.read_oneshot(&mut pin1_adc1_pin)).unwrap();
        let pin2_value: u16 = nb::block!(adc1.read_oneshot(&mut pin2_adc1_pin)).unwrap();

        let psi1 = linear_interpolation(pin1_value as f64);
        let psi2 = linear_interpolation(pin2_value as f64);

        log::info!(
            "PIN32 {:.0} PSI ({pin1_value}) | PIN33 {:.0} PSI ({pin2_value})",
            psi1,
            psi2,
        );

        Text::with_baseline(
            &format!("Pin 32 {:.0} PSI", psi1),
            Point::zero(),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_baseline(
            &format!("Pin 33 {:.0} PSI", psi2),
            Point::zero(),
            text_style,
            Baseline::Bottom,
        )
        .draw(&mut display)
        .unwrap();

        display.flush().unwrap();

        delay.delay(500.millis());
    }
}
