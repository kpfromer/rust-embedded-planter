#![no_main]
#![no_std]

use hal::gpio::p0::P0_13;
use hal::gpio::p1::P1_03;
use hal::gpio::p1::P1_05;
use hal::gpio::Output;
use hal::gpio::Pin;
use hal::gpio::PushPull;
use hal::pac::SPIM0;
use hal::Spim;
use panic_halt as _;

use nrf52840_hal as _;

use rtic::app;

use dwt_systick_monotonic::DwtSystick;
use dwt_systick_monotonic::ExtU32;

use nrf52840_hal as hal;
use nrf52840_hal::clocks::HFCLK_FREQ;
use nrf52840_hal::gpio::Level;
use nrf52840_hal::prelude::*;

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;

use st7789::ST7789;

type Display = ST7789<
    SPIInterfaceNoCS<Spim<SPIM0>, P0_13<Output<PushPull>>>,
    P1_03<Output<PushPull>>,
    P1_05<Output<PushPull>>,
>;

pub struct Led(Pin<Output<PushPull>>);

impl Led {
    fn new<Mode>(pin: Pin<Mode>) -> Self {
        Led(pin.into_push_pull_output(Level::Low))
    }

    /// Turn the LED on
    pub fn on(&mut self) {
        self.0.set_high().unwrap()
    }

    /// Turn the LED off
    pub fn off(&mut self) {
        self.0.set_low().unwrap()
    }
}

#[app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: hal::gpio::p1::P1_01<hal::gpio::Output<hal::gpio::PushPull>>,
        white_led: Led,
        state: bool,
        display: Display,
    }

    // 64_000_000 matches hal::clocks::HFCLK_FREQ
    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<HFCLK_FREQ>;

    struct DelayWrapper<'a>(&'a mut MonoTimer);

    impl embedded_hal::blocking::delay::DelayUs<u32> for DelayWrapper<'_> {
        fn delay_us(&mut self, us: u32) {
            let wait_until = self.0.now() + (us as u32).micros();
            while self.0.now() < wait_until {
                /* spin */
                cortex_m::asm::nop();
            }
        }
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut debug_control_block = cx.core.DCB;
        let _clocks = hal::Clocks::new(cx.device.CLOCK).enable_ext_hfosc();
        let mut monotonic = DwtSystick::new(
            &mut debug_control_block,
            cx.core.DWT,
            cx.core.SYST,
            hal::clocks::HFCLK_FREQ,
        );

        let port0 = hal::gpio::p0::Parts::new(cx.device.P0);
        let port1 = hal::gpio::p1::Parts::new(cx.device.P1);

        let tft_reset = port1.p1_03.into_push_pull_output(Level::Low);
        let tft_backlight = port1.p1_05.into_push_pull_output(Level::Low);
        let _tft_cs = port0.p0_12.into_push_pull_output(Level::Low);
        let tft_dc = port0.p0_13.into_push_pull_output(Level::Low);
        let tft_sck = port0.p0_14.into_push_pull_output(Level::Low).degrade();
        let tft_mosi = port0.p0_15.into_push_pull_output(Level::Low).degrade();

        let mut led = port1.p1_01.into_push_pull_output(Level::Low);
        led.set_high().unwrap();

        let white_led = Led::new(port0.p0_10.degrade());

        let pins = hal::spim::Pins {
            sck: Some(tft_sck),
            miso: None,
            mosi: Some(tft_mosi),
        };
        // https://github.com/almindor/st7789-examples/blob/master/examples/image.rs
        let spi = Spim::new(
            cx.device.SPIM0,
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
        display.init(&mut DelayWrapper(&mut monotonic)).unwrap();
        // set default orientation
        display
            .set_orientation(st7789::Orientation::Landscape)
            .unwrap();

        let circle1 = Circle::new(Point::new(128, 64), 64)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::RED));
        // let circle2 = Circle::new(Point::new(64, 64), 64)
        //     .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1));

        display.clear(Rgb565::BLACK).unwrap();
        circle1.draw(&mut display).unwrap();

        // let blue_with_red_outline = PrimitiveStyleBuilder::new()
        //     .fill_color(Rgb565::BLUE)
        //     .stroke_color(Rgb565::RED)
        //     .stroke_width(1) // > 1 is not currently supported in embedded-graphics on triangles
        //     .build();
        // let triangle = Triangle::new(
        //     Point::new(40, 120),
        //     Point::new(40, 220),
        //     Point::new(140, 120),
        // )
        // .into_styled(blue_with_red_outline);

        // let line = Line::new(Point::new(180, 160), Point::new(239, 239))
        //     .into_styled(PrimitiveStyle::with_stroke(RgbColor::WHITE, 10));

        // // draw two circles on black background
        // display.clear(Rgb565::BLACK).unwrap();
        // circle1.draw(&mut display).unwrap();
        // circle2.draw(&mut display).unwrap();
        // triangle.draw(&mut display).unwrap();
        // line.draw(&mut display).unwrap();

        // TODO: https://github.com/Jarrod-Bennett/rust-nrf52-bluetooth/blob/cda7d9cb181e3dbf6e3afb1c27427a0ece20cbb0/src/main.rs

        // blink::spawn_after(Duration::<u32, 1, 64000000>::from_ticks(1000)).unwrap();
        blink::spawn_after(1.secs()).unwrap();

        (
            Shared {},
            Local {
                led,
                white_led,
                state: false,
                display,
            },
            init::Monotonics(monotonic),
        )
    }

    #[task(local = [led, white_led, display, state])]
    fn blink(cx: blink::Context) {
        if *cx.local.state {
            cx.local.led.set_high().unwrap();
            cx.local.white_led.on();
            *cx.local.state = false;
        } else {
            cx.local.led.set_low().unwrap();
            cx.local.white_led.off();
            *cx.local.state = true;
        }

        let circle = Circle::new(Point::new(128, 64), 64).into_styled(PrimitiveStyle::with_fill(
            if *cx.local.state {
                Rgb565::RED
            } else {
                Rgb565::BLUE
            },
        ));

        circle.draw(&mut *cx.local.display).unwrap();

        blink::spawn_after(2.secs()).unwrap();
    }

    // #[idle]
    // fn idle(_: idle::Context) -> ! {
    //     loop {
    //         cortex_m::asm::nop();
    //     }
    // }
}
