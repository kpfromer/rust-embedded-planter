# rust-planter-waterer

## Installation

- cargo install
- minicom (serial comms)
- cargo size
- cargo objcopy

## Deploying To Board

- information about cargo embed
  - what jtag and seggar are, and why they don't work
- using uf2 for deployments
  - What is uf2
  - how to put board in to uf2 load state (double click reset button)
- explain deploy.sh, build.rs, .cargo

## Notes

### Memory

Explain nrf memory model and `model.x`

## TODO

- [ ] Clean up display
  - [ ] show moisture
  - [ ] show motor status
  - [ ] Title
  - [ ] Plant image?
    - [ ] Store image in other memory
- [ ] ability to turn off display
- [ ] Trigger motor on low mositure
- [ ] usbd interrupt instead of idle (idle for polling though)

## Resources

- nrf pinout
- interrupts
- notion, etc
