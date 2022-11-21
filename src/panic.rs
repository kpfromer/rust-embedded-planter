use crate::led::Led;

use nrf52840_hal as hal;

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
