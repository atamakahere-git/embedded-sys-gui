# Slint UI for embedded system

## About

This repo contains a bare metal MCU Rust application with Slint as toolkit for the user interface.
It implements the `slint::platform::Platform` trait, and displays a simple `.slint` design on the screen.


For a template about using Slint with an operating system (Desktop, or Embedded Linux), check out the
classic template at https://github.com/slint-ui/slint-rust-template.
## Usage

1. Install **Nightly** Rust by following the [Rust Getting Started Guide](https://www.rust-lang.org/learn/get-started).
   Once this is done, you should have the ```rustc``` compiler and the ```cargo``` build system installed in your path.
2. Install [`cargo-generate`](https://github.com/cargo-generate/cargo-generate)
    ```
    cargo install cargo-generate
    ```
3. Set up a sample project with this template
    ```
    cargo generate --git https://github.com/slint-ui/slint-mcu-rust-template --name my-project
    cd my-project
    ```
3. Run on the Desktop (Simulator)
    ```
    cargo run --features simulator
    ```
4. If you have a [RaspberryPi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) with a [2.8 inch Waveshare Touch Screen](https://www.waveshare.com/pico-restouch-lcd-2.8.htm), run on the device with
    ```
    cargo +nightly build --target=thumbv6m-none-eabi --features=pico --release && elf2uf2-rs -d target/thumbv6m-none-eabi/release/project-name
    ```

Porting to other device : https://slint-ui.com/snapshots/master/docs/rust/slint/docs/mcu/index.html

