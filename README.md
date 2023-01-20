# IQS5xx

[![Github](https://img.shields.io/badge/iqs5xx-github%2Frwalkr-blue?&logo=github)](https://github.com/rwalkr/iqs5xx)
[![Crates.io](https://img.shields.io/crates/v/iqs5xx.svg)](https://crates.io/crates/iqs5xx)
[![Documentation](https://img.shields.io/docsrs/iqs5xx)](https://docs.rs/iqs5xx)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

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

[iqs5xx](https://crates.io/crates/iqs5xx)


# Examples

Examples using the Raspberry PI Pico to report touchpad events via
`defmt_rtt` and to act as a basic USB mouse are located in the
[`examples/rp-pico`](https://github.com/rwalkr/iqs5xx/tree/main/examples/rp-pico) directory.
