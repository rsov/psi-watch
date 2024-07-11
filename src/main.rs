#![no_std]
#![no_main]

use alloc::{
    format,
    string::{String, ToString},
};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
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
use ssd1306::{command::Command, prelude::*, I2CDisplayInterface, Ssd1306};
use u8g2_fonts::{
    fonts,
    types::{FontColor, HorizontalAlignment, VerticalPosition},
    FontRenderer,
};

extern crate alloc;
use core::{borrow::Borrow, mem::MaybeUninit};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

const PSI_MAX: f64 = 100.0;
const PSI_MIN: f64 = 0.0;
const PSI_WARN: f64 = 15.0;

const YELLOW_HEIGHT: i32 = 16; // first few pixels are in different color

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
    let mut interface = I2CDisplayInterface::new(i2c);

    // Might solve brightness issue?
    Command::ChargePump(true).send(&mut interface).unwrap();

    let driver = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0);

    // let mut display = driver.into_terminal_mode();
    let mut display = driver.into_buffered_graphics_mode();

    log::debug!("config");
    display.init().unwrap();
    display.set_display_on(true).unwrap();
    display.set_brightness(Brightness::BRIGHTEST).unwrap();
    display.flush().unwrap();

    let font = FontRenderer::new::<fonts::u8g2_font_timB24_tf>();

    let sensor_1_pin = io.pins.gpio32;
    let sensor_2_pin = io.pins.gpio33;

    // Create ADC instances
    let mut adc1_config = AdcConfig::new();
    let mut pin1_adc1_pin = adc1_config.enable_pin(sensor_1_pin, Attenuation::Attenuation11dB);
    let mut pin2_adc1_pin = adc1_config.enable_pin(sensor_2_pin, Attenuation::Attenuation11dB);

    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    let warn_min = PSI_MIN + PSI_WARN;
    let warn_max = PSI_MAX - PSI_WARN;

    loop {
        display
            .clear(embedded_graphics::pixelcolor::BinaryColor::Off)
            .unwrap();

        let pin1_value = nb::block!(adc1.read_oneshot(&mut pin1_adc1_pin)).unwrap();
        let pin2_value = nb::block!(adc1.read_oneshot(&mut pin2_adc1_pin)).unwrap();

        let psi1_reading = linear_interpolation(pin1_value as f64);
        let psi2_reading = linear_interpolation(pin2_value as f64);

        let psi1 = format_reading(psi1_reading);
        let psi2 = format_reading(psi2_reading);

        if let Some(value) = psi1_reading {
            if (PSI_MIN..warn_min).contains(&value) || (warn_max..PSI_MAX).contains(&value) {
                Line::new(Point::new(0, 0), Point::new(128 / 2, 0))
                    .into_styled(PrimitiveStyle::with_stroke(
                        BinaryColor::On,
                        YELLOW_HEIGHT as u32,
                    ))
                    .draw(&mut display)
                    .unwrap();
            }

            Line::new(Point::new(4, 64), Point::new(4, bar_y(value)))
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 8))
                .draw(&mut display)
                .unwrap();
        }

        if let Some(value) = psi2_reading {
            if (PSI_MIN..warn_min).contains(&value) || (warn_max..PSI_MAX).contains(&value) {
                Line::new(Point::new(128 / 2, 0), Point::new(128, 0))
                    .into_styled(PrimitiveStyle::with_stroke(
                        BinaryColor::On,
                        YELLOW_HEIGHT as u32,
                    ))
                    .draw(&mut display)
                    .unwrap();
            }

            Line::new(Point::new(122, 64), Point::new(122, bar_y(value)))
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 8))
                .draw(&mut display)
                .unwrap();
        }

        font.render_aligned(
            psi1.borrow(),
            Point::new(60, 24),
            VerticalPosition::Top,
            HorizontalAlignment::Right,
            FontColor::Transparent(BinaryColor::On),
            &mut display,
        )
        .unwrap();

        font.render_aligned(
            psi2.borrow(),
            Point::new(110, 24),
            VerticalPosition::Top,
            HorizontalAlignment::Right,
            FontColor::Transparent(BinaryColor::On),
            &mut display,
        )
        .unwrap();

        display.flush().unwrap();
        delay.delay_millis(100u32);
    }
}

fn linear_interpolation(adc: f64) -> Option<f64> {
    let psi_at_min = PSI_MIN;
    let adc_at_min = 310.0; // 0.5 v / 2

    let adc_at_max = 2792.0; // 4.5 v / 2
    let psi_at_max = PSI_MAX;

    // Value is clipped (not plugged in etc)
    if adc < adc_at_min || adc > adc_at_max {
        return None;
    }

    Some(psi_at_min + ((adc - adc_at_min) * (psi_at_max - psi_at_min)) / (adc_at_max - adc_at_min))
}

fn format_reading(val: Option<f64>) -> String {
    if let Some(value) = val {
        format!("{:.0}", value).to_string()
    } else {
        "--".to_string()
    }
}

fn bar_y(val: f64) -> i32 {
    64 - (val as i32 * (64 - YELLOW_HEIGHT) / 100)
}
