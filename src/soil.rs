use crate::prelude::*;

use cortex_m::prelude::_embedded_hal_adc_OneShot;
use nrf52840_hal::{
    gpio::{p0::P0_05, Disconnected, Floating, Input},
    pac::SAADC,
    saadc::SaadcConfig,
    Saadc,
};

pub struct Soil {
    adc: Saadc,
    pin: P0_05<Input<Floating>>,
}

impl Soil {
    pub fn new(saadc: SAADC, pin: P0_05<Disconnected>) -> Soil {
        // set up ADC and analog pin to read
        // read more here: https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Fsaadc.html
        let adc = Saadc::new(saadc, SaadcConfig::default());
        Soil {
            adc,
            pin: pin.into_floating_input(),
        }
    }

    pub fn soil_moisture_percentage(&mut self) -> Result<u8, AppError> {
        // self.adc
        //     .read(&mut self.pin)
        //     .map_err(|_| AppError::SoilReadingError)
        self.adc
            .read(&mut self.pin)
            .map(|value| (((value as f32 / 16384_f32) * 100_f32) as u8).clamp(0, 100))
            .map_err(|_| AppError::SoilReadingError)
    }

    // TODO: take 10 samples to average?

    // fn soil_measure_percentage<const N: usize>(&mut self) -> Result<u8, AppError> {
    //     let mut items = [0_f32; N];
    // }
}
