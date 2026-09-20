AD936X driver
========

A `no_std` Rust driver for the Analog Devices AD936X RF transceivers, ported from the AD9361
driver of the [no-OS](https://github.com/analogdevicesinc/no-OS) repository. It also drives the
[AXI AD9361 IP core](https://analogdevicesinc.github.io/hdl/library/axi_ad9361/index.html) in the
FPGA (with the `axi-ad9361` crate), which the digital interface is tuned with.

# Usage

```rust,ignore
let ad9361 = Uninitialized::new(spi, resetb, ReferenceClock::new(40.MHz())?, axi)
    .configure(Ad9361Settings::default())?
    .init()
    .await?;
```

`Ad9361Settings::default()` is the setup from the no-OS `main.c` (2R2T, FDD, 30.72 MSPS, 2.4 GHz).
See the crate docs for a complete example, the requirements (SPI mode 1, a time driver for
`embassy-time`) and the caveats.

# Features

- Setup like `ad9361_setup()`: clock chain, RF synths, ports, gain control, calibrations, FIRs
  and the digital interface tuning with the AXI AD9361 core.
- Change the sample rate, LO frequencies, bandwidths, TX attenuation, RX gain mode and FIRs
  while running.
- Types for values with a limited range, like `TxAttenuation` and `RxLoFrequency`.
- Host tests against a fake register file.

Only 2R2T, FDD, LVDS and the full gain table were tested on hardware.

# License

This crate is a port of the AD9361 driver and the AXI ADC/DAC core drivers of the Analog Devices
[no-OS](https://github.com/analogdevicesinc/no-OS) repository, and stays under the 3-clause BSD
license of the files it is derived from, see [`LICENSE-ADI-BSD`](./LICENSE-ADI-BSD), which also
lists the copyright notices. The changes and additions of this crate are licensed under the MIT
license, see [`LICENSE-MIT`](./LICENSE-MIT). The crate is therefore `MIT AND BSD-3-Clause`.

This crate is not affiliated with or endorsed by Analog Devices, Inc.
