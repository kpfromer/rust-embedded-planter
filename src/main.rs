#![no_std]
#![no_main]

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use hal::gpio::Level;
use hal::prelude::OutputPin;
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
    // let mut monotonic = DwtSystick::new(
    //     &mut debug_control_block,
    //     cx.core.DWT,
    //     cx.core.SYST,
    //     hal::clocks::HFCLK_FREQ,
    // );

    let port0 = hal::gpio::p0::Parts::new(device.P0);
    let port1 = hal::gpio::p1::Parts::new(device.P1);

    let tft_reset = port1.p1_03.into_push_pull_output(Level::Low);
    let tft_backlight = port1.p1_05.into_push_pull_output(Level::Low);
    let _tft_cs = port0.p0_12.into_push_pull_output(Level::Low);
    let tft_dc = port0.p0_13.into_push_pull_output(Level::Low);
    let tft_sck = port0.p0_14.into_push_pull_output(Level::Low).degrade();
    let tft_mosi = port0.p0_15.into_push_pull_output(Level::Low).degrade();

    let mut led = port0.p0_10.into_push_pull_output(Level::Low);
    led.set_high().unwrap();

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

    let circle1 =
        Circle::new(Point::new(128, 64), 64).into_styled(PrimitiveStyle::with_fill(Rgb565::RED));
    let circle2 = Circle::new(Point::new(64, 64), 64)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1));

    let blue_with_red_outline = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLUE)
        .stroke_color(Rgb565::RED)
        .stroke_width(1) // > 1 is not currently supported in embedded-graphics on triangles
        .build();
    let triangle = Triangle::new(
        Point::new(40, 120),
        Point::new(40, 220),
        Point::new(140, 120),
    )
    .into_styled(blue_with_red_outline);

    let line = Line::new(Point::new(180, 160), Point::new(239, 239))
        .into_styled(PrimitiveStyle::with_stroke(RgbColor::WHITE, 10));

    // draw two circles on black background
    display.clear(Rgb565::BLACK).unwrap();
    circle1.draw(&mut display).unwrap();
    circle2.draw(&mut display).unwrap();
    triangle.draw(&mut display).unwrap();
    line.draw(&mut display).unwrap();

    loop {
        // your code goes here
        cortex_m::asm::nop();
    }
}
