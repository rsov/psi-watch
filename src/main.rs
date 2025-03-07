#![no_std]
#![no_main]

extern crate alloc;

use alloc::{
    format,
    string::{String, ToString},
};
use core::borrow::Borrow;
use embedded_graphics::{
    image::Image,
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::CpuClock,
    delay::Delay,
    i2c::master::I2c,
    time::RateExtU32,
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use tinybmp::Bmp;
use u8g2_fonts::{
    fonts,
    types::{FontColor, HorizontalAlignment, VerticalPosition},
    FontRenderer,
};

const PSI_MAX: f64 = 100.0;
const PSI_MIN: f64 = 0.0;
const PSI_WARN: f64 = 15.0;

const YELLOW_HEIGHT: i32 = 16; // first few pixels are in different color

#[esp_hal::main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::default());
    let peripherals = esp_hal::init(config);

    let delay = Delay::new();

    esp_alloc::heap_allocator!(size: 32 * 1024);

    esp_println::logger::init_logger_from_env();

    // Create a new peripheral object with the described wiring and standard
    // I2C clock speed:
    let i2c = I2c::new(peripherals.I2C0, {
        esp_hal::i2c::master::Config::default().with_frequency(400u32.kHz())
    })
    .unwrap()
    .with_sda(peripherals.GPIO25)
    .with_scl(peripherals.GPIO26);

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

    display.set_invert(true).unwrap();
    display.flush().unwrap();

    let font = FontRenderer::new::<fonts::u8g2_font_timB24_tf>();
    let bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("./truck.bmp")).unwrap();
    let image = Image::new(&bmp, Point::new(0, 0));
    image.draw(&mut display).unwrap();
    display.flush().unwrap();
    delay.delay_millis(1500u32);

    display.set_invert(false).unwrap();
    display.flush().unwrap();

    let sensor_1_pin = peripherals.GPIO32;
    let sensor_2_pin = peripherals.GPIO33;

    // Create ADC instances
    let mut adc1_config = AdcConfig::new();
    let mut pin1_adc1_pin = adc1_config.enable_pin(sensor_1_pin, Attenuation::_11dB);
    let mut pin2_adc1_pin = adc1_config.enable_pin(sensor_2_pin, Attenuation::_11dB);

    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    let warn_min = PSI_MIN + PSI_WARN;
    let warn_max = PSI_MAX - PSI_WARN;

    loop {
        display
            .clear(embedded_graphics::pixelcolor::BinaryColor::Off)
            .unwrap();

        // TODO: May need block!
        let pin1_value = adc1.read_oneshot(&mut pin1_adc1_pin).unwrap();
        let pin2_value = adc1.read_oneshot(&mut pin2_adc1_pin).unwrap();

        let pin1_vol = adc_to_v(pin1_value as f64);
        let pin2_vol = adc_to_v(pin2_value as f64);

        let psi1_reading = linear_interpolation(pin1_vol);
        let psi2_reading = linear_interpolation(pin2_vol);

        let psi1 = format_reading(psi1_reading);
        let psi2 = format_reading(psi2_reading);

        log::debug!(
            "PSI1 {}v {} | PSI2 {}v {}",
            pin1_vol,
            pin1_value,
            pin2_vol,
            pin2_value
        );

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

fn linear_interpolation(vol: f64) -> Option<f64> {
    let psi_at_min = PSI_MIN;
    let vol_at_min = 0.35;

    let psi_at_max = PSI_MAX;
    let vol_at_max = 3.2;

    // Value is clipped (not plugged in etc)
    if vol < vol_at_min || vol > vol_at_max {
        return None;
    }

    Some(psi_at_min + ((vol - vol_at_min) * (psi_at_max - psi_at_min)) / (vol_at_max - vol_at_min))
}

// Copy - pasta, hope it works
fn adc_to_v(adc: f64) -> f64 {
    (adc * 3.3) / 4095.0
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
