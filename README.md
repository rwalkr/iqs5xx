# IQS5xx

IQS5xx capacitive touchpad controller device 

This crate provides a device driver for the Azoteq IQS5xx capacitive
touchpad controller.

The IQS5xx touchpad connects to the target via I2C and two GPIO pins. The
[`embedded_hal`](https://docs.rs/embedded-hal) `blocking::i2c` and
`digital::v2` interfaces are used, so should work with any target that
provides these.


# Documentation

[API documentation](https://docs.rs/iqs5xx)


# Crate

[iqs5xx](https://crates.io/iqs5xx)


# Examples

Examples using the Raspberry PI Pico to report touchpad events via
`defmt_rtt` and to act as a basic USB mouse are located in the
![`examples/rp-pico`](examples/rp-pico) directory.
