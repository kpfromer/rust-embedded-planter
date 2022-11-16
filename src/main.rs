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
// use panic_halt as _;

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

use nrf52840_hal::usbd::{UsbPeripheral, Usbd};
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usb_device::prelude::UsbDevice;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use nrf52840_hal::clocks::{ExternalOscillator, Internal, LfOscStopped};
use nrf52840_hal::Clocks;

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

#[app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [PWM3])]
mod app {
    use hal::Timer;

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        white_led: Led,
        display: Display,
        serial: SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
        usb_dev: UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
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

        let mut white_led = Led::new(port0.p0_10.degrade());

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
            .set_orientation(st7789::Orientation::Landscape)
            .unwrap();
        display.clear(Rgb565::BLACK).unwrap();

        let usb_bus = cx.local.usb_bus;
        usb_bus.replace(UsbBusAllocator::new(Usbd::new(UsbPeripheral::new(
            cx.device.USBD,
            clocks.as_ref().unwrap(),
        ))));

        let serial = SerialPort::new(usb_bus.as_ref().unwrap());
        let usb_dev = UsbDeviceBuilder::new(usb_bus.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .max_packet_size_0(64) // (makes control transfers 8x faster)
            .build();

        // blink::spawn_after(1.secs()).unwrap();
        render::spawn_after(2.secs()).unwrap();

        (
            Shared {},
            Local {
                white_led,
                display,
                usb_dev,
                serial,
            },
            init::Monotonics(monotonic),
        )
    }

    // #[task(local = [white_led, state: bool = false])]
    // fn blink(cx: blink::Context) {
    //     if *cx.local.state {
    //         cx.local.white_led.on();
    //         *cx.local.state = false;
    //     } else {
    //         cx.local.white_led.off();
    //         *cx.local.state = true;
    //     }

    //     blink::spawn_after(2.secs()).unwrap();
    // }

    #[task(local = [display])]
    fn render(cx: render::Context) {
        let circle = Circle::new(Point::new(128, 64), 64)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::RED));
        circle.draw(cx.local.display).unwrap();
    }

    #[idle(local = [usb_dev, serial ])]
    fn idle(cx: idle::Context) -> ! {
        let usb_dev = cx.local.usb_dev;
        let serial = cx.local.serial;
        loop {
            if !usb_dev.poll(&mut [serial]) {
                continue;
            }
            let mut buf = [0u8; 64];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Echo back characters that came in
                    let mut write_offset = 0;
                    while write_offset < count {
                        match serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }

            // match serial.write(&[0x3a, 0x29]) {
            //     Ok(_count) => {
            //         // count bytes were written
            //     }
            //     _ => {}
            // };
        }
    }

    // TODO: interrupt on usb
    // https://github.com/Jarrod-Bennett/rust-nrf52-bluetooth/blob/cda7d9cb181e3dbf6e3afb1c27427a0ece20cbb0/src/main.rs
}

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    let device = unsafe { hal::pac::Peripherals::steal() };

    let port1 = hal::gpio::p1::Parts::new(device.P1);
    let mut red_led = Led::new(port1.p1_01.degrade());

    red_led.on();

    loop {
        cortex_m::asm::nop();
    }
}
