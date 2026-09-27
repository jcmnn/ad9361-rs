AD936X driver
========

A `no_std` Rust driver for the Analog Devices AD936X RF transceivers, ported from the AD9361
driver of the [no-OS](https://github.com/analogdevicesinc/no-OS) repository.

The FPGA side of the data port is a trait, `DataInterface`. `AxiCores` implements it for the
[AXI AD9361 IP core](https://analogdevicesinc.github.io/hdl/library/axi_ad9361/index.html) with
the `axi-ad9361` crate (`axi` feature, on by default).

# Usage

```rust,ignore
let config = Ad9361Config::new(Ad9361Settings::default(), ReferenceClock::new(40.MHz())?)?;
let cores = AxiCores::new(axi.take_adc_no_init().unwrap(), axi.take_dac_no_init().unwrap());
let ad9361 = Ad9361::init(spi, resetb, cores, &config).await?;
```

`Ad9361Settings::default()` is 2R2T, FDD, 30.72 MSPS with both LOs at 2.4 GHz.
See the crate docs for a complete example, the requirements (SPI mode 1, a time driver for
`embassy-time`) and the caveats.

# Features

- Setup like `ad9361_setup()`: clock chain, RF synths, ports, gain control, calibrations, FIRs
  and the digital interface tuning with the AXI AD9361 core.
- Change the sample rate, LO frequencies, bandwidths, TX attenuation, RX gain, DCXO trim and
  FIRs while running.
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
