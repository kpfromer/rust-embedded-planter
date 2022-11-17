use nrf52840_hal::{
    gpio::{Level, Output, Pin, PushPull},
    prelude::OutputPin,
};

pub struct Led(Pin<Output<PushPull>>);

impl Led {
    pub fn new<Mode>(pin: Pin<Mode>) -> Self {
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
