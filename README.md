# rust-planter-waterer

## Installation

Just run `cargo build` to build the binary.

Other things to install:

- minicom (serial comms)
- cargo size
- cargo objcopy

## Deploying To Board

`./deploy.sh`

The better way to is to run `./deploy.sh` to build the binary, convert it to uf2 file (via the [uf2conv.py](https://github.com/microsoft/uf2/blob/master/utils/uf2conv.py) file created by microsoft) and upload it to the board (on macos).

Make sure that the clue is in uf2 file mode by **double clicking** the reset button (the neopixel should be green).

## Notes

## Why uf2

Note I am using the [uf2](https://github.com/microsoft/uf2) (developed by microsoft) since it makes it easy to upload a file and run. I tried to use `openocd` and did so successfully. However, you need to have pins connect to flash the board and it's a pain in the ass to upload a file to the raspberry pi and then run openocd and then flash and then check for errors, etc.

I tried to use cargo embed with a jtag thing but it didn't work on mac. Also jtags and seggar are really expensive and out of stock, so this was not the way to go.

- `memory.x`
  - Used in the `build.rs` file to handle putting the rust binary data in the correct locations for the embedded system. This file will change per chipset based on it's memory structure.
- `build.rs`

  - builds the binary with `memory.x` and makes sure to link it in.

## Resources

- nrf pinout
  - clue pinout: https://learn.adafruit.com/adafruit-clue/pinouts
  - https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpin.html
- interrupts
  - https://docs.rs/nrf52840-hal/latest/nrf52840_hal/pac/enum.Interrupt.html
- notion (for me)
  - https://kpfromer.notion.site/Plant-Waterer-ceaf3c83a06943e5b5f615dedf74861a
