#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::ascii::FONT_8X13;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Baseline;
use embedded_graphics::text::Text;
use embedded_graphics::text::TextStyleBuilder;
use hal::gpio::Level;
use hal::prelude::OutputPin;
use hal::prelude::PinState;
use hal::{Delay, Spim};
// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;

// use nrf52840_hal::gpio::p0::Parts;
// use nrf52840_hal::gpio::p1::Parts;

use nrf52840_hal as _;
use nrf52840_hal as hal;
use st7789::ST7789;

#[entry]
fn main() -> ! {
    let core = cortex_m::Peripherals::take().unwrap();
    let device = nrf52840_hal::pac::Peripherals::take().unwrap();

    let mut delay = Delay::new(core.SYST);

    let port0 = hal::gpio::p0::Parts::new(device.P0);
    let port1 = hal::gpio::p1::Parts::new(device.P1);

    let tft_reset = port1.p1_03.into_push_pull_output(Level::Low);
    let tft_backlight = port1.p1_05.into_push_pull_output(Level::Low);
    let _tft_cs = port0.p0_12.into_push_pull_output(Level::Low);
    let tft_dc = port0.p0_13.into_push_pull_output(Level::Low);
    let tft_sck = port0.p0_14.into_push_pull_output(Level::Low).degrade();
    let tft_mosi = port0.p0_15.into_push_pull_output(Level::Low).degrade();

    let pins = hal::spim::Pins {
        sck: Some(tft_sck),
        miso: None,
        mosi: Some(tft_mosi),
    };
    // https://github.com/almindor/st7789-examples/blob/master/examples/image.rs
    let spi = Spim::new(
        device.SPIM0,
        pins,
        hal::spim::Frequency::M8,
        hal::spim::MODE_3,
        122,
    );
    // Display interface from SPI and DC
    let display_interface = SPIInterfaceNoCS::new(spi, tft_dc);
    // Create driver
    let mut display = ST7789::new(
        display_interface,
        Some(tft_reset),
        Some(tft_backlight),
        240,
        240,
    );

    // initialize
    display.init(&mut delay).unwrap();
    // set default orientation
    display
        .set_orientation(st7789::Orientation::Landscape)
        .unwrap();

    display.clear(Rgb565::BLACK).unwrap();

    let bounding_box = display.bounding_box();

    let character_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(RgbColor::WHITE)
        .build();

    let left_aligned = TextStyleBuilder::new()
        .alignment(Alignment::Left)
        .baseline(Baseline::Top)
        .build();

    let center_aligned = TextStyleBuilder::new()
        .alignment(Alignment::Center)
        .baseline(Baseline::Middle)
        .build();

    let right_aligned = TextStyleBuilder::new()
        .alignment(Alignment::Right)
        .baseline(Baseline::Bottom)
        .build();

    Text::with_text_style(
        "Left aligned text, origin top left",
        bounding_box.top_left,
        character_style,
        left_aligned,
    )
    .draw(&mut display)
    .unwrap();

    Text::with_text_style(
        "Center aligned text, origin center center",
        bounding_box.center(),
        character_style,
        center_aligned,
    )
    .draw(&mut display)
    .unwrap();

    Text::with_text_style(
        "Right aligned text, origin bottom right",
        bounding_box.bottom_right().unwrap(),
        character_style,
        right_aligned,
    )
    .draw(&mut display)
    .unwrap();

    // delay.delay_ms(6000_u32);

    let mut speaker = port1.p1_00.into_push_pull_output(Level::Low);

    let mut state = false;
    loop {
        display.clear(Rgb565::BLACK).unwrap();
        Text::with_text_style(
            if state { "High" } else { "Low" },
            bounding_box.center(),
            character_style,
            center_aligned,
        )
        .draw(&mut display)
        .unwrap();

        // if state {
        //     speaker.set_high().unwrap();
        // } else {
        //     speaker.set_low().unwrap();
        // }

        state = !state;

        delay.delay_ms(2000_u32);
    }
}
