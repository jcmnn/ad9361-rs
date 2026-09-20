//! # AD9361 driver
//!
//! `no_std` driver for the Analog Devices AD9361 RF transceiver, ported from the
//! [no-OS](https://github.com/analogdevicesinc/no-OS) driver. It also drives the AXI AD9361 core
//! in the FPGA (through the `axi-ad9361` crate), which is needed to tune the digital interface.
//!
//! The port follows no-OS closely, so the no-OS sources are the best reference for what the chip
//! does at each step. Docs name the no-OS function they mirror, like `ad9361_gc_setup()`, and
//! mention where this crate differs.
//!
//! # Usage
//!
//! Start with [`Uninitialized`], call [`configure`](Uninitialized::configure) with the
//! [`Ad9361Settings`], then [`init`](Configured::init). The result is an [`Ad9361`].
//!
//! ```no_run
//! use ad9361::{Ad9361Settings, AxiCores, ReferenceClock, RxLoFrequency, Uninitialized};
//! use embedded_hal::{digital::OutputPin, spi::SpiDevice};
//! use fugit::HertzU32;
//!
//! async fn bring_up<S: SpiDevice<u8>, R: OutputPin>(spi: S, resetb: R, axi: AxiCores)
//! where
//!     S::Error: core::fmt::Debug,
//!     R::Error: core::fmt::Debug,
//! {
//!     // 40 MHz crystal, like on the Pluto
//!     let ref_clk = ReferenceClock::new(HertzU32::MHz(40)).expect("reference clock out of range");
//!
//!     // Default settings are the ones from the no-OS `main.c`: 2R2T, FDD, 30.72 MSPS, 2.4 GHz.
//!     let settings = Ad9361Settings {
//!         rx_synth_freq: RxLoFrequency::from_hz(2_450_000_000),
//!         ..Ad9361Settings::default()
//!     };
//!
//!     let mut ad9361 = Uninitialized::new(spi, resetb, ref_clk, axi)
//!         .configure(settings)
//!         .expect("invalid settings")
//!         .init()
//!         .await
//!         .expect("chip init failed");
//!
//!     let _ = ad9361.revision();
//! }
//! ```
//!
//! [`configure`](Uninitialized::configure) checks the settings against each other and against the
//! reference clock, without touching the chip. [`init`](Configured::init) resets the chip and
//! runs the whole setup. The DAC plays its DDS tones after that.
//!
//! The AXI cores come from `axi-ad9361`. Take them with `take_adc_no_init()` and
//! `take_dac_no_init()`, because the cores have no clock until the AD9361 runs.
//!
//! Once the chip is up, [`Ad9361`] can change the sample rate, LO frequencies, bandwidths, TX
//! attenuation and RX gain mode, and load and enable the FIRs.
//!
//! Values with a limited range have their own types ([`TxAttenuation`], [`RxLoFrequency`],
//! [`TxLoFrequency`], [`RfBandwidth`], [`SampleRate`], [`ReferenceClock`], [`RxInput`]). The
//! constructors return an error for out of range values.
//!
//! # Requirements
//!
//! - A blocking [`embedded_hal::spi::SpiDevice`] with chip select handled by the device. The
//!   AD9361 needs SPI mode 1 (same as no-OS) and 10 MHz at most.
//! - An [`embedded_hal::digital::OutputPin`] for the reset line. Hold it low before init.
//!   [`init`](Configured::init) pulses it, and the [`Ad9361`] keeps holding the pin.
//! - A time driver for [`embassy_time`]. Setup, calibrations, and tuning wait with
//!   [`embassy_time::Timer`], so those functions are `async`. Any executor works.
//!
//! The crate logs a few info messages through the `log` facade.
//!
//! # Caveats
//!
//! - [`init`](Configured::init) takes a while. Calibrations and interface tuning wait on the
//!   chip.
//! - If [`init`](Configured::init) fails, the SPI device and reset pin are consumed and the chip
//!   is left half configured. Retrying needs the peripherals again, and the reset at the start
//!   of `init` brings the chip back to a known state.
//! - All methods take `&mut self`. Share the driver between tasks with a mutex.
//! - SPI access is blocking.
//! - Only 2R2T, FDD, LVDS, and the full gain table (the `main.c` defaults) were run on hardware.
//!   1R1T, TDD, CMOS, and the split gain table are ported but untested. Manual gain is not
//!   implemented for the split table ([`Ad9361Error::UnsupportedGainTable`]).
//! - Not ported from no-OS: fastlock, external LO and band switching, LO power down,
//!   IDELAY/ODELAY tuning, the RSSI factory table and the FIR coefficient read-back.
//! - The AXI core needs an HDL version with the per-channel DAC data source mux (v8 and later).
//! - The ENSM is not exposed. After init the chip runs in FDD, or in RX when configured for TDD.
//!
//! # License
//!
//! This is a port of the AD9361 driver in [no-OS](https://github.com/analogdevicesinc/no-OS), so
//! it stays under the BSD-3 license of the files it comes from (`LICENSE-ADI-BSD`). The changes
//! and additions are MIT (`LICENSE-MIT`). Not affiliated with or endorsed by Analog Devices.
#![cfg_attr(not(test), no_std)]

use arbitrary_int::{traits::Integer, u2, u3, u4, u5, u6, u7, u10, u13};
use embassy_time::Timer;
use embedded_hal::spi::SpiDevice;
use fugit::{HertzU32, HertzU64};

mod api;
mod calibration;
mod clocks;
mod config;
mod ports;
mod setup;
mod spi;
mod state;
mod synth;
mod clock_chain;
mod axi_init;
mod control;
mod error;
mod dig_tune;
mod fir;
mod gain_control;
mod gain_tables;
mod monitor;
pub mod registers;
mod synth_lut;
mod units;

pub use calibration::{DcOffsetConfig, RxPhase, TrackingConfig};
use calibration::find_opt;
pub use axi_init::AxiInitError;
pub use dig_tune::{AxiCores, BistLoopback, BistMode, DigTuneFlags};
pub use fir::{
    DEFAULT_FIR_COEFFICIENTS, FirChannels, FirCoefficients, FirFactor, RxFirConfig, RxFirGain,
    TxFirConfig, TxFirGain,
};
pub use control::{ClkoutMode, EnsmState, ForcedEnsmState, RequestedEnsmState};
pub use units::{
    AuxAdcDecimation, ElnaGain, OutOfRange, RfBandwidth, RxInput, RxLoFrequency, SampleRate, TxAttenuation, TxLoFrequency,
};
pub use monitor::{RssiConfiguration, RssiRestartMode, TxMonitorConfig};
pub use gain_control::{FastAgcTargetGain, GainControl, GainMode, GainTableDest};
pub use api::{Ad9361, Configured, ManualRxGain, RxFir, TxFir, Uninitialized};
pub use clocks::*;
pub use clock_chain::PathClocks;
pub use config::{
    Ad9361Config, Ad9361Settings, ChannelMode, ConfigError, DigInterfaceTune, Duplex,
    GainTableKind, RateGovernor, ReferenceClock,
};
pub use ports::*;
pub use error::{Ad9361Error, InitError};
pub use registers::*;
use synth_lut::{SYNTH_LUT_FDD, SYNTH_LUT_SIZE, SYNTH_LUT_TDD};

pub const RFPLL_MODULUS: u32 = 8388593;
pub const BBPLL_MODULUS: u32 = 2088960;

pub const MAX_BBPLL_FREF: HertzU32 = HertzU32::Hz(70_007_000);
pub const MIN_BBPLL_FREQ: HertzU32 = HertzU32::Hz(714_928_500);
pub const MAX_BBPLL_FREQ: HertzU32 = HertzU32::Hz(1_430_143_000);

pub const MAX_ADC_CLK: HertzU32 = HertzU32::Hz(640_000_000);
pub const MAX_DAC_CLK: HertzU32 = HertzU32::Hz(MAX_ADC_CLK.to_raw() / 2);
pub const MAX_RX_HB1: HertzU32 = HertzU32::Hz(245_760_000);
pub const MAX_RX_HB2: HertzU32 = HertzU32::Hz(320_000_000);
pub const MAX_RX_HB3: HertzU32 = HertzU32::Hz(640_000_000);
pub const MAX_TX_HB1: HertzU32 = HertzU32::Hz(160_000_000);
pub const MAX_TX_HB2: HertzU32 = HertzU32::Hz(320_000_000);
pub const MAX_TX_HB3: HertzU32 = HertzU32::Hz(320_000_000);
pub const MAX_BASEBAND_RATE: HertzU32 = HertzU32::Hz(61_440_000);

/// Lowest VCO frequency. The LO is the VCO divided by a power of two.
const MIN_VCO_FREQ_HZ: u64 = 6_000_000_000;
pub const MAX_CARRIER_FREQ_HZ: u64 = 6_000_000_000;
pub const MIN_RX_CARRIER_FREQ_HZ: u64 = 70_000_000;
pub const MIN_TX_CARRIER_FREQ_HZ: u64 = 46_875_001;

pub const MAX_SYNTH_FREF: HertzU32 = HertzU32::Hz(80_008_000);
pub const MIN_SYNTH_FREF: HertzU32 = HertzU32::Hz(9_999_000);

// TODO: Implement async SpiBus

/// Register-level driver core plus the state it needs. Not public, go through [`Ad9361`] so
/// setup is guaranteed to have run.
pub(crate) struct Engine<S: SpiDevice<u8>> {
    spi: S,
    axi: AxiCores,
    ref_clk_in: HertzU32,
    /// channel/duplex/data interface modes
    mode: state::ModeState,
    /// clock tree
    clk: state::ClockState,
    /// RX gain control, gain table
    gain: state::GainState,
    /// FIRs
    fir: state::FirState,
    /// calibrations and tracking
    cal: state::CalibrationState,
    /// digital interface tuning
    tune: state::TuneState,
    /// ENSM
    ensm: state::EnsmTracking,
    /// saved while TX is muted
    tx1_atten_cached: TxAttenuation,
    tx2_atten_cached: TxAttenuation,
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    /// Builds the state for a chip that was just reset and identified. `ensm_state` is what the
    /// chip reported after the reset. Reads the clock rates from the chip.
    pub(crate) fn new(
        spi: S,
        axi: AxiCores,
        config: &Ad9361Config,
        ensm_state: u8,
    ) -> Result<Self, S::Error> {
        let mut engine = Engine {
            spi,
            axi,
            ref_clk_in: config.ref_clk.get(),
            mode: state::ModeState::new(config),
            clk: state::ClockState::new(config),
            gain: state::GainState::new(config),
            fir: state::FirState::default(),
            cal: state::CalibrationState::new(config),
            tune: state::TuneState::new(config),
            ensm: state::EnsmTracking::new(config, ensm_state),
            tx1_atten_cached: TxAttenuation::MIN,
            tx2_atten_cached: TxAttenuation::MIN,
        };
        // no-OS calls this clks_resync
        engine.init_clocks()?;
        Ok(engine)
    }
}

#[cfg(test)]
mod tests;
