#![no_std]
#![no_main]

use alloc::{
    format,
    string::{String, ToString},
};
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    text::Text,
};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
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
    display.init().unwrap();
    display.set_display_on(true).unwrap();
    display.set_brightness(Brightness::BRIGHTEST).unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(BinaryColor::On)
        .build();

    let sensor_1_pin = io.pins.gpio32;
    let sensor_2_pin = io.pins.gpio33;

    // Create ADC instances
    let mut adc1_config = AdcConfig::new();
    let mut pin1_adc1_pin = adc1_config.enable_pin(sensor_1_pin, Attenuation::Attenuation11dB);
    let mut pin2_adc1_pin = adc1_config.enable_pin(sensor_2_pin, Attenuation::Attenuation11dB);

    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    loop {
        display
            .clear(embedded_graphics::pixelcolor::BinaryColor::Off)
            .unwrap();

        let pin1_value = nb::block!(adc1.read_oneshot(&mut pin1_adc1_pin)).unwrap();
        let pin2_value = nb::block!(adc1.read_oneshot(&mut pin2_adc1_pin)).unwrap();

        let psi1_reading = linear_interpolation(pin1_value as f64);
        let psi2_reading = linear_interpolation(pin2_value as f64);

        let psi1 = format_feading(psi1_reading);
        let psi2 = format_feading(psi2_reading);

        log::info!("PIN32 {psi1} ({pin1_value}) | PIN33 {psi2} ({pin2_value})");

        Text::new(&format!("32 {psi1}"), Point::new(0, 30), text_style)
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(0, 35), Point::new(bar_width(psi1_reading), 35))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 5))
            .draw(&mut display)
            .unwrap();

        Text::new(&format!("33 {psi2}"), Point::new(0, 55), text_style)
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(0, 60), Point::new(bar_width(psi2_reading), 60))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 5))
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();
    }
}

fn linear_interpolation(adc: f64) -> Option<f64> {
    let psi_at_min = 0.0;
    let adc_at_min = 310.0; // 0.5 v / 2

    let adc_at_max = 2792.0; // 4.5 v / 2
    let psi_at_max = 100.0;

    // Value is clipped (not plugged in etc)
    if adc < adc_at_min || adc > adc_at_max {
        return None;
    }

    Some(psi_at_min + ((adc - adc_at_min) * (psi_at_max - psi_at_min)) / (adc_at_max - adc_at_min))
}

fn format_feading(val: Option<f64>) -> String {
    if let Some(value) = val {
        format!("{:.0} PSI", value).to_string()
    } else {
        "--".to_string()
    }
}

fn bar_width(val: Option<f64>) -> i32 {
    if let Some(value) = val {
        value as i32 * 128 / 100
    } else {
        0
    }
}
