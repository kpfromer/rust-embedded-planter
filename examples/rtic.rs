#![no_main]
#![no_std]

use panic_halt as _;

use nrf52840_hal as _;

use rtic::app;

use dwt_systick_monotonic::DwtSystick;
use dwt_systick_monotonic::ExtU32;

use nrf52840_hal as hal;
use nrf52840_hal::gpio::Level;
use nrf52840_hal::prelude::*;

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;

#[app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [])]
mod app {
    use hal::Spim;
    use st7789::ST7789;

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    // 64_000_000 matches hal::clocks::HFCLK_FREQ
    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<64_000_000>;

    struct DelayWrapper<'a>(&'a mut MonoTimer);

    impl embedded_hal::blocking::delay::DelayUs<u32> for DelayWrapper<'_> {
        fn delay_us(&mut self, us: u32) {
            let wait_until = self.0.now() + (us as u32).micros();
            while self.0.now() < wait_until { /* spin */ }
        }
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut debug_control_block = cx.core.DCB;
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

        let mut led = port0.p0_10.into_push_pull_output(Level::Low);
        led.set_high().unwrap();

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

        // TODO: https://github.com/Jarrod-Bennett/rust-nrf52-bluetooth/blob/cda7d9cb181e3dbf6e3afb1c27427a0ece20cbb0/src/main.rs

        (Shared {}, Local {}, init::Monotonics(monotonic))
    }
}

// extern crate alloc;
// extern crate no_std_compat as std;
// use panic_halt as _;

// use cortex_m::asm;
// use nrf52840_hal as hal;

// #[rtic::app(device = hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3, SWI4_EGU4])]
// mod app {
//     use crate::*;

//     use cortex_m_semihosting::{debug, hprintln};

//     #[shared]
//     struct Shared {}

//     #[local]
//     struct Local {}

//     #[init(local = [x: u32 = 0])]
//     fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
//         // Cortex-M peripherals
//         let _core: cortex_m::Peripherals = cx.core;

//         // Device specific peripherals
//         let _device: lm3s6965::Peripherals = cx.device;

//         // Locals in `init` have 'static lifetime
//         let _x: &'static mut u32 = cx.local.x;

//         // Access to the critical section token,
//         // to indicate that this is a critical seciton
//         let _cs_token: bare_metal::CriticalSection = cx.cs;

//         hprintln!("init").unwrap();

//         debug::exit(debug::EXIT_SUCCESS); // Exit QEMU simulator

//         (Shared {}, Local {}, init::Monotonics())
//     }

//     // #[monotonic(binds = TIMER1, default = true)]
//     // type MyMono = MonoTimer<TIMER1>;

//     // #[shared]
//     // struct Shared {
//     //     usb_dev: UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
//     //     serial: MyUsbSerial,
//     //     radio: ieee802154::Radio<'static>,
//     //     #[lock_free]
//     //     led1: Pin<Output<PushPull>>,
//     //     #[lock_free]
//     //     led1_on: bool,
//     //     led2: Pin<Output<PushPull>>,
//     //     led2_on: bool,
//     //     led3: Pin<Output<PushPull>>,
//     //     led3_on: bool,
//     //     led4: Pin<Output<PushPull>>,
//     //     led4_on: bool,
//     // }

//     // #[local]
//     // struct Local {
//     //     wdh0: wdt::WatchdogHandle<wdt::handles::Hdl0>,
//     // }

//     // #[init]
//     // fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {}
// }
