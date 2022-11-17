use nrf52840_hal::{
    gpio::{Level, Output, Pin, PushPull},
    prelude::OutputPin,
};

pub struct Led(Pin<Output<PushPull>>, bool);

impl Led {
    pub fn new<Mode>(pin: Pin<Mode>) -> Self {
        Led(pin.into_push_pull_output(Level::Low), false)
    }

    /// Turn the LED on
    pub fn on(&mut self) {
        if !self.1 {
            self.0.set_high().unwrap();
            self.1 = true;
        }
    }

    /// Turn the LED off
    pub fn off(&mut self) {
        if self.1 {
            self.0.set_low().unwrap();
            self.1 = false;
        }
    }
}
