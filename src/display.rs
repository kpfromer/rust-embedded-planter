use hal::gpio::p0::P0_13;
use hal::gpio::p1::P1_03;
use hal::gpio::p1::P1_05;
use hal::gpio::Output;
use hal::gpio::PushPull;
use hal::pac::SPIM0;
use hal::Spim;
use hal::Timer;

use nrf52840_hal as _;

use nrf52840_hal as hal;

use display_interface_spi::SPIInterfaceNoCS;
use st7789::BacklightState;

use st7789::ST7789;

pub type Display = ST7789<
    SPIInterfaceNoCS<Spim<SPIM0>, P0_13<Output<PushPull>>>,
    P1_03<Output<PushPull>>,
    P1_05<Output<PushPull>>,
>;

pub struct DisplayDevice<T> {
    timer: Timer<T>,
    pub is_on: bool,
    pub display: Display,
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
