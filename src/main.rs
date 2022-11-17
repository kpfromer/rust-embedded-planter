#![no_main]
#![no_std]

pub mod error;
pub mod led;
pub mod soil;
pub mod usb_serial;

pub mod prelude {
    pub use crate::error::*;
}

use hal::gpio::p0::P0_13;
use hal::gpio::p1::P1_03;
use hal::gpio::p1::P1_05;
use hal::gpio::Output;
use hal::gpio::PushPull;
use hal::pac::SPIM0;
use hal::Spim;
use hal::Timer;

use heapless::{String, Vec};

use nrf52840_hal as _;

use rtic::app;

use dwt_systick_monotonic::DwtSystick;
use dwt_systick_monotonic::ExtU32;

use nrf52840_hal as hal;
use nrf52840_hal::clocks::HFCLK_FREQ;
use nrf52840_hal::clocks::{ExternalOscillator, Internal, LfOscStopped};
use nrf52840_hal::gpio::Level;
use nrf52840_hal::gpiote::Gpiote;
use nrf52840_hal::Clocks;

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::{
    image::Image, mono_font::MonoTextStyleBuilder, prelude::*, primitives::Rectangle, text::Text,
};
use embedded_layout::layout::linear::LinearLayout;
use embedded_layout::prelude::*;
use st7789::BacklightState;
use tinybmp::Bmp;

use st7789::ST7789;

use nrf52840_hal::usbd::{UsbPeripheral, Usbd};
use usb_device::bus::UsbBusAllocator;

use profont::PROFONT_24_POINT;

use core::fmt::Write;

use led::Led;
use usb_serial::UsbSerialDevice;

use crate::soil::Soil;

type Display = ST7789<
    SPIInterfaceNoCS<Spim<SPIM0>, P0_13<Output<PushPull>>>,
    P1_03<Output<PushPull>>,
    P1_05<Output<PushPull>>,
>;

pub struct DisplayDevice<T> {
    timer: Timer<T>,
    is_on: bool,
    display: Display,
}

impl<T> DisplayDevice<T>
where
    T: nrf52840_hal::timer::Instance,
{
    pub fn new(mut display: Display, mut timer: Timer<T>) -> DisplayDevice<T> {
        display
            .set_backlight(BacklightState::On, &mut timer)
            .unwrap();
        DisplayDevice {
            display,
            is_on: true,
            timer,
        }
    }

    #[inline]
    pub fn on(&mut self) {
        self.display
            .set_backlight(st7789::BacklightState::On, &mut self.timer)
            .unwrap();
        self.is_on = true;
    }

    #[inline]
    pub fn off(&mut self) {
        self.display
            .set_backlight(st7789::BacklightState::Off, &mut self.timer)
            .unwrap();
        self.is_on = false;
    }

    #[inline]
    pub fn toggle(&mut self) {
        if self.is_on {
            self.off()
        } else {
            self.on()
        }
    }
}

const MOISTURE_THRESHOLD: u8 = 50;

#[app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [PWM3, SPIM3, SPIM2_SPIS2_SPI2])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        display_device: DisplayDevice<nrf52840_hal::pac::TIMER1>,
    }

    #[local]
    struct Local {
        white_led: Led,
        usb_serial_device: UsbSerialDevice<'static, Usbd<UsbPeripheral<'static>>>,
        soil: Soil,
        motor: Led,
        gpiote: Gpiote,
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

    #[init(local=[clocks: Option<Clocks<ExternalOscillator, Internal, LfOscStopped>> = None, usb_bus: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut debug_control_block = cx.core.DCB;
        let clocks = cx.local.clocks;
        clocks.replace(hal::Clocks::new(cx.device.CLOCK).enable_ext_hfosc());
        let monotonic = DwtSystick::new(
            &mut debug_control_block,
            cx.core.DWT,
            cx.core.SYST,
            hal::clocks::HFCLK_FREQ,
        );

        let port0 = hal::gpio::p0::Parts::new(cx.device.P0);
        let port1 = hal::gpio::p1::Parts::new(cx.device.P1);

        let white_led = Led::new(port0.p0_10.degrade());
        let soil = Soil::new(cx.device.SAADC, port0.p0_05);
        let motor = Led::new(port0.p0_03.degrade());

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
        let mut timer = Timer::new(cx.device.TIMER4);
        display.init(&mut timer).unwrap();
        // set default orientation
        display
            .set_orientation(st7789::Orientation::LandscapeSwapped)
            // .set_orientation(st7789::Orientation::Landscape)
            .unwrap();
        display.clear(Rgb565::BLACK).unwrap();
        let display_device = DisplayDevice::new(display, Timer::new(cx.device.TIMER1));

        let usb_bus = cx.local.usb_bus;
        usb_bus.replace(UsbBusAllocator::new(Usbd::new(UsbPeripheral::new(
            cx.device.USBD,
            clocks.as_ref().unwrap(),
        ))));

        let usb_serial_device = UsbSerialDevice::new(usb_bus.as_ref().unwrap());

        // GPIO interrupts
        let btn = port1.p1_02.into_pullup_input().degrade();
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        gpiote.port().input_pin(&btn).low();
        gpiote.port().enable_interrupt();

        soil_measure::spawn().unwrap();

        (
            Shared { display_device },
            Local {
                white_led,
                usb_serial_device,
                soil,
                motor,
                gpiote,
            },
            init::Monotonics(monotonic),
        )
    }

    #[task(binds = GPIOTE, local = [gpiote], priority = 5)]
    fn on_gpiote(cx: on_gpiote::Context) {
        cx.local.gpiote.reset_events();
        toggle_display::spawn().unwrap();
    }

    #[task(shared=[display_device], priority = 4)]
    fn toggle_display(mut cx: toggle_display::Context) {
        cx.shared
            .display_device
            .lock(|display_device| display_device.toggle());
    }

    #[task(local=[string: String<32> = String::new()], shared = [display_device], priority = 4)]
    fn render(mut cx: render::Context, soil_moisture: u8) {
        let string = cx.local.string;

        let style = MonoTextStyleBuilder::new()
            .font(&PROFONT_24_POINT)
            .text_color(Rgb565::WHITE)
            .background_color(Rgb565::BLACK)
            .build();

        write!(string, " Soil {}% ", soil_moisture).unwrap();

        let display_area = Rectangle::new(Point::new(80, 0), Size::new(240, 240));

        let title_text = Text::new("PWAT 5000", Point::zero(), style);
        let soil_text = Text::new(
            string.as_str(),
            Point::zero(),
            if soil_moisture > MOISTURE_THRESHOLD {
                MonoTextStyleBuilder::new()
                    .font(&PROFONT_24_POINT)
                    .text_color(Rgb565::GREEN)
                    .background_color(Rgb565::BLACK)
                    .build()
            } else {
                MonoTextStyleBuilder::new()
                    .font(&PROFONT_24_POINT)
                    .text_color(Rgb565::RED)
                    .background_color(Rgb565::BLACK)
                    .build()
            },
        );
        // TODO: motor status

        cx.shared.display_device.lock(|display_device| {
            if display_device.is_on {
                // The layout
                LinearLayout::vertical(Chain::new(title_text).append(soil_text))
                    .with_alignment(horizontal::Center)
                    .arrange()
                    .align_to(&display_area, horizontal::Center, vertical::Center)
                    .draw(&mut display_device.display)
                    .unwrap();
            }
        });

        // let bmp_data = include_bytes!("../images/rust-social.bmp");
        // // Parse the BMP file.
        // let bmp = Bmp::from_slice(bmp_data).unwrap();
        // // Draw the image with the top left corner at (10, 20) by wrapping it in
        // // an embedded-graphics `Image`.
        // Image::new(&bmp, Point::new(100, 50)).draw(display).unwrap();

        string.clear();
    }

    #[task(local = [motor], priority = 3)]
    fn motor(cx: motor::Context, on: bool) {
        if on {
            cx.local.motor.on();
            // Turn off motor in .5 seconds
            motor::spawn_after(500.millis(), false).unwrap();
        } else {
            cx.local.motor.off()
        }
    }

    #[task(local = [soil])]
    fn soil_measure(cx: soil_measure::Context) {
        let soil = cx.local.soil;

        let soil_moisture = soil.soil_moisture_percentage().unwrap();

        // render dipslay
        render::spawn(soil_moisture).unwrap();
        // control motor
        motor::spawn(soil_moisture < MOISTURE_THRESHOLD).unwrap();

        // spawn again in a bit
        soil_measure::spawn_after(1.secs()).unwrap();
    }

    #[idle(local = [usb_serial_device, white_led])]
    fn idle(cx: idle::Context) -> ! {
        let usb_serial_device = cx.local.usb_serial_device;
        let mut chars: Vec<u8, 64> = Vec::new();

        loop {
            // Read line and clear chars if there's an error
            if usb_serial_device.read_line(&mut chars).is_err() {
                chars.clear();
            }

            match &chars[..] {
                b"on" => cx.local.white_led.on(),
                b"off" => cx.local.white_led.off(),
                b"motor" => motor::spawn(true).unwrap(),
                b"display" => toggle_display::spawn().unwrap(),
                _ => {}
            }

            chars.clear();
        }
    }

    // TODO: interrupt on usb
    // https://github.com/Jarrod-Bennett/rust-nrf52-bluetooth/blob/cda7d9cb181e3dbf6e3afb1c27427a0ece20cbb0/src/main.rs
}

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    let device = unsafe { hal::pac::Peripherals::steal() };

    let port0 = hal::gpio::p0::Parts::new(device.P0);
    let port1 = hal::gpio::p1::Parts::new(device.P1);

    let mut motor = Led::new(port0.p0_03.degrade());
    let mut red_led = Led::new(port1.p1_01.degrade());

    // MAKE SURE TO TURN OFF MOTOR IF ANYTHING GOES WRONG
    motor.off();
    red_led.on();

    loop {
        cortex_m::asm::nop();
    }
}
