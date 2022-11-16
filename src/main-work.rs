#![no_main]
#![no_std]

use cortex_m_rt;
use embedded_hal::blocking::delay::DelayMs;
use nrf52840_hal::{gpio::Level, prelude::OutputPin, Timer};

#[cortex_m_rt::entry]
fn main() -> ! {
    let core = cortex_m::Peripherals::take().unwrap();
    let device = nrf52840_hal::pac::Peripherals::take().unwrap();
    let mut timer = Timer::new(device.TIMER4);
    let port1 = nrf52840_hal::gpio::p1::Parts::new(device.P1);
    let mut red = port1.p1_01.degrade().into_push_pull_output(Level::Low);

    loop {
        // Blink some LEDs
        red.set_high().unwrap();
        timer.delay_ms(500 as u32);
        red.set_low().unwrap();
        timer.delay_ms(100 as u32);
    }
}

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
