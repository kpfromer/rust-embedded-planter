use heapless::Vec;

use usb_device::{bus, prelude::*};
use usbd_serial::{DefaultBufferStore, SerialPort};

use crate::prelude::*;

pub struct UsbSerialDevice<'a, B>
where
    B: bus::UsbBus,
{
    serial_port: SerialPort<'a, B, DefaultBufferStore, DefaultBufferStore>,
    usb_device: UsbDevice<'a, B>,
}

impl<B> UsbSerialDevice<'_, B>
where
    B: bus::UsbBus,
{
    /// New usb serial
    pub fn new(usb_bus: &bus::UsbBusAllocator<B>) -> UsbSerialDevice<B> {
        let serial_port = SerialPort::new(usb_bus);

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Adafruit")
            .product("CLUE")
            .serial_number("1")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        UsbSerialDevice {
            serial_port,
            usb_device,
        }
    }

    /// Poll periodically
    // #[inline]
    // pub fn poll(&mut self) {
    //     if !self.usb_device.poll(&mut [&mut self.serial_port]) {
    //         // https://github.com/mvirkkunen/usb-device/issues/32
    //         usb_device::class::UsbClass::poll(&mut self.serial_port);
    //     }
    // }

    // /// Serial read append to the given vector, non-blocking
    // #[inline]
    // pub fn read<const S: usize>(&mut self, data: &mut Vec<u8, S>) -> Result<(), AppError> {
    //     self.poll();

    //     let mut buf = [0u8; 1];
    //     while self.serial_port.dtr() && (data.capacity() > data.len()) {
    //         match self.serial_port.read(&mut buf) {
    //             Ok(_) => {
    //                 data.push(buf[0]).map_err(|_| AppError::UsbSerialError)?;
    //             }
    //             Err(WouldBlock) => break,
    //             e => e.map(|_| ())?,
    //         }

    //         self.poll();
    //     }

    //     Ok(())
    // }

    // /// Serial write all bytes out, blocking
    // #[inline]
    // pub fn write(&mut self, data: &[u8]) -> Result<(), AppError> {
    //     self.poll();

    //     if self.serial_port.dtr() {
    //         let mut n = 0;
    //         while n < data.len() - 1 {
    //             match self.serial_port.write(&data[n..]) {
    //                 Ok(s) => {
    //                     n += s;
    //                 }
    //                 Err(WouldBlock) => {}
    //                 e => {
    //                     e?;
    //                 }
    //             }
    //         }
    //     }

    //     Ok(())
    // }

    // #[inline]
    // pub fn read_line(&mut self, chars: &mut Vec<u8, 64>) -> Result<(), AppError> {
    //     chars.clear();

    //     loop {
    //         if !self.usb_device.poll(&mut [&mut self.serial_port]) {
    //             continue;
    //         }

    //         let mut buf = [0u8; 1];

    //         match self.serial_port.read(&mut buf) {
    //             Ok(count) if count > 0 => {
    //                 // If enter is entered
    //                 if buf[0] == b'\n' || buf[0] == b'\r' {
    //                     break;
    //                 } else {
    //                     chars.push(buf[0]).map_err(|_| AppError::UsbSerialError)?;
    //                 }
    //             }
    //             _ => {}
    //         }
    //     }

    //     Ok(())
    // }

    #[inline]
    pub fn read_line(&mut self, chars: &mut Vec<u8, 64>) -> Result<(), AppError> {
        chars.clear();

        loop {
            if !self.usb_device.poll(&mut [&mut self.serial_port]) {
                continue;
            }

            let mut buf = [0u8; 1];

            match self.serial_port.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // If enter is entered
                    if buf[0] == b'\n' || buf[0] == b'\r' {
                        break;
                    } else {
                        chars.push(buf[0]).map_err(|_| AppError::UsbSerialError)?;

                        // Write back characters
                        let mut write_offset = 0;
                        while write_offset < count {
                            match self.serial_port.write(&buf[..]) {
                                Ok(write_count) if write_count > 0 => write_offset += buf.len(),
                                _ => {}
                            }
                        }
                    }
                }
                _ => {}
            }
        }

        Ok(())
    }

    #[inline]
    pub fn write_chars(&mut self, chars: &[u8]) -> Result<(), AppError> {
        // wait for ability to write
        while !self.usb_device.poll(&mut [&mut self.serial_port]) {}

        let mut position = 0;
        while position < chars.len() {
            match self.serial_port.write(&chars[position..]) {
                Ok(count) if count > 0 => {
                    position += count;
                }
                _ => {}
            }
        }

        Ok(())
    }

    // #[inline]
    // pub fn wait_for_input(&mut self) -> Result<(), AppError> {
    //     // wait for read or write
    //     while !self.usb_device.poll(&mut [&mut self.serial_port]) {}

    //     Ok(())
    // }
}

impl From<UsbError> for AppError {
    fn from(_: UsbError) -> Self {
        AppError::UsbSerialError
    }
}
