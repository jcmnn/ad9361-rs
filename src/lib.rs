//! # AD9361 driver
//!
//! `no_std` driver for the Analog Devices AD9361 RF transceiver, ported from the
//! [no-OS](https://github.com/analogdevicesinc/no-OS) driver.
//!
//! The chip sends and receives samples over a parallel data port to an FPGA, and the driver
//! needs the FPGA side to bring that port up and tune its timing. That side is the
//! [`DataInterface`] trait. [`AxiCores`] implements it for ADI's AXI AD9361 IP core through the
//! `axi-ad9361` crate (`axi` feature, on by default).
//!
//! The port follows no-OS closely, so the no-OS sources are the best reference for what the chip
//! does at each step. Docs name the no-OS function they mirror, like `ad9361_gc_setup()`, and
//! mention where this crate differs.
//!
//! # Usage
//!
//! Check the [`Ad9361Settings`] with [`Ad9361Config::new`], then hand the peripherals and the
//! config to [`Ad9361::init`].
//!
//! ```no_run
//! # #[cfg(feature = "axi")] mod example {
//! use ad9361::{Ad9361, Ad9361Config, Ad9361Settings, AxiCores, ReferenceClock, RxLoFrequency};
//! use axi_ad9361::{AxiAd9361, dds::{DdsScale, IqPair}};
//! use embedded_hal::digital::OutputPin;
//! use embedded_hal_async::spi::SpiDevice;
//! use fugit::HertzU32;
//!
//! async fn bring_up<S: SpiDevice<u8>, R: OutputPin>(spi: S, resetb: R, axi: &mut AxiAd9361)
//! where
//!     S::Error: core::fmt::Debug,
//!     R::Error: core::fmt::Debug,
//! {
//!     // 40 MHz crystal, like on the Pluto
//!     let ref_clk = ReferenceClock::new(HertzU32::MHz(40)).expect("reference clock out of range");
//!
//!     // The defaults are 2R2T, FDD, 30.72 MSPS and 2.4 GHz, see `Ad9361Settings`.
//!     let settings = Ad9361Settings {
//!         rx_lo_frequency: RxLoFrequency::from_hz(2_450_000_000),
//!         ..Ad9361Settings::default()
//!     };
//!     let config = Ad9361Config::new(settings, ref_clk).expect("invalid settings");
//!
//!     // the cores have no clock until the AD9361 runs, `init` brings them up
//!     let cores = AxiCores::new(axi.take_adc_no_init().unwrap(), axi.take_dac_no_init().unwrap());
//!     let mut ad9361 = Ad9361::init(spi, resetb, cores, &config).await.expect("init failed");
//!
//!     // a tone 1 MHz above the TX LO on TX1, at a quarter of full scale
//!     let sample_rate = ad9361.sample_rate();
//!     ad9361
//!         .interface_mut()
//!         .dac
//!         .set_tone(IqPair::First, HertzU32::MHz(1), DdsScale::from_fraction(0.25), sample_rate)
//!         .expect("1 MHz is below Nyquist");
//! }
//! # }
//! ```
//!
//! [`Ad9361Config::new`] checks the settings against each other and against the reference clock,
//! without any hardware. [`Ad9361::init`] resets the chip and runs the whole setup. With
//! [`AxiCores`], the DAC then plays a DDS tone 3 MHz above the TX LO at 10% of full scale, with
//! no DMA or CPU time. [`Ad9361::set_tx_source`] switches it to sample data.
//!
//! Once the chip is up, [`Ad9361`] can change the sample rate, LO frequencies, bandwidths, TX
//! attenuation, RX gain mode and DCXO trim, and load and enable the FIRs.
//!
//! # Units
//!
//! Frequencies are [`fugit`] rates (`HertzU32::MHz(1)`). Values with a limited range have their
//! own types: [`TxAttenuation`], [`RxLoFrequency`], [`TxLoFrequency`], [`RfBandwidth`],
//! [`SampleRate`], [`ReferenceClock`] and [`DcxoTrim`]. The `new` constructors return an error
//! for out of range values. The `const fn` ones that don't return a `Result` (like
//! [`RxLoFrequency::from_hz`]) panic instead, which is a compile error in a `const`.
//!
//! Every frequency the chip makes comes from the reference clock, so a crystal that is off by
//! some ppm moves the LOs, the sample rate and with it the DDS tones by the same ppm.
//! [`DcxoTrim`] corrects that.
//!
//! # Navigating the crate
//!
//! The crate root has what bring up and run time need: [`Ad9361`] and its handles,
//! [`Ad9361Settings`] and [`Ad9361Config`], the value types above, the FIR types and the errors.
//! The types that only go into [`Ad9361Settings`] fields are in [`settings`], and the FPGA side
//! of the data port in [`interface`].
//!
//! # Requirements
//!
//! - An [`embedded_hal_async::spi::SpiDevice`] with chip select handled by the device. The
//!   AD9361 needs SPI mode 1 and 10 MHz at most.
//! - An [`embedded_hal::digital::OutputPin`] for the reset line. Hold it low before init.
//!   [`Ad9361::init`] pulses it, and the [`Ad9361`] keeps holding the pin.
//! - A [`DataInterface`], like [`AxiCores`]. The AXI core needs an HDL version with the
//!   per-channel DAC data source mux (v8 and later).
//! - A time driver for [`embassy_time`]. Setup, calibrations, and tuning wait with
//!   [`embassy_time::Timer`], so those functions are `async`. Any executor works.
//!
//! The crate logs a few info messages through the `log` facade.
//!
//! # Caveats
//!
//! - [`Ad9361::init`] takes a while. Calibrations and interface tuning wait on the chip. If it
//!   fails, the [`InitFailure`] hands the peripherals back, and the reset at the start of the
//!   next `init` brings the chip back to a known state.
//! - All methods take `&mut self`. Share the driver between tasks with a mutex.
//! - Everything is `async`. A blocking SPI driver needs an adapter that implements the async
//!   trait, and then every transfer blocks the executor.
//! - Only 2R2T, FDD, LVDS, and the full gain table (the defaults) were run on hardware.
//!   1R1T, TDD, CMOS, and the split gain table are ported but untested. Manual gain is not
//!   implemented for the split table ([`Ad9361Error::UnsupportedGainTable`]).
//! - Not ported from no-OS: fastlock, external LO and band switching, LO power down,
//!   IDELAY/ODELAY tuning, the RSSI factory table and the FIR coefficient read-back.
//! - The ENSM is not exposed. After init the chip runs in FDD, or in RX when configured for TDD.
//!
//! # License
//!
//! This is a port of the AD9361 driver in [no-OS](https://github.com/analogdevicesinc/no-OS), so
//! it stays under the BSD-3 license of the files it comes from (`LICENSE-ADI-BSD`). The changes
//! and additions are MIT (`LICENSE-MIT`). Not affiliated with or endorsed by Analog Devices.
#![cfg_attr(not(test), no_std)]
// the docs link to `AxiCores` in places
#![cfg_attr(not(feature = "axi"), allow(rustdoc::broken_intra_doc_links))]

use arbitrary_int::{traits::Integer, u2, u3, u4, u5, u6, u7, u10, u13};
use embassy_time::Timer;
use embedded_hal_async::spi::SpiDevice;
use fugit::{HertzU32, HertzU64};

mod api;
#[cfg(feature = "axi")]
mod axi;
mod calibration;
mod clock_chain;
mod clocks;
mod config;
mod control;
mod dig_tune;
mod error;
mod fir;
mod gain_control;
mod gain_tables;
mod init;
pub mod interface;
mod monitor;
mod ports;
mod registers;
mod setup;
mod spi;
mod state;
mod synth;
mod synth_lut;
mod units;

pub use api::{Ad9361, ManualRxGain, RxFir, TxFir};
#[cfg(feature = "axi")]
pub use axi::AxiCores;
pub use clocks::Ad9361ClockRates;
pub use config::{Ad9361Config, Ad9361Settings, ConfigError, DcxoTrim, ReferenceClock};
pub use error::{Ad9361Error, InitError, InitFailure};
pub use fir::{
    DEFAULT_FIR_COEFFICIENTS, FirCoefficients, FirFactor, RxFirConfig, RxFirGain, TxFirConfig,
    TxFirGain,
};
pub use gain_control::GainMode;
pub use interface::DataInterface;
pub use ports::{Channel, Channels};
pub use units::{OutOfRange, RfBandwidth, RxLoFrequency, SampleRate, TxAttenuation, TxLoFrequency};

/// The types that only appear inside [`Ad9361Settings`]: channel and duplex modes, the reference
/// source, clock chain, ports, gain control, calibration tracking, monitors and the small blocks.
/// Their defaults are the ones [`Ad9361Settings::default`] uses.
pub mod settings {
    pub use crate::calibration::{DcOffsetConfig, TrackingConfig};
    pub use crate::clock_chain::PathClocks;
    pub use crate::config::{
        ChannelMode, DigInterfaceTune, Duplex, GainTableKind, MAX_SYNTH_FREF, MIN_SYNTH_FREF,
        RateGovernor, ReferenceSource,
    };
    pub use crate::control::ClkoutMode;
    pub use crate::gain_control::{FastAgcTargetGain, GainControl};
    pub use crate::monitor::{RssiConfiguration, RssiRestartMode, TxMonitorConfig};
    pub use crate::ports::{
        AuxAdcConfig, AuxDacConfig, ChannelInversion, CmosConfig, CmosPorts, CtrlOutsConfig,
        ElnaConfig, GpoConfig, LvdsBias, LvdsConfig, PortConfig, PortDelays, PortMode,
    };
    pub use crate::units::{AuxAdcDecimation, ElnaGain, RxInput, TxOutput};
}

use calibration::{RxPhase, find_opt};
use clocks::*;
use control::{ForcedEnsmState, RequestedEnsmState};
use dig_tune::{BistLoopback, DigTuneFlags};
use gain_control::GainTableDest;
use registers::*;
use settings::*;
use synth_lut::{SYNTH_LUT_FDD, SYNTH_LUT_SIZE, SYNTH_LUT_TDD};

const RFPLL_MODULUS: u32 = 8388593;
const BBPLL_MODULUS: u32 = 2088960;

const MAX_BBPLL_FREF: HertzU32 = HertzU32::Hz(70_007_000);
const MIN_BBPLL_FREQ: HertzU32 = HertzU32::Hz(714_928_500);
const MAX_BBPLL_FREQ: HertzU32 = HertzU32::Hz(1_430_143_000);

const MAX_ADC_CLK: HertzU32 = HertzU32::Hz(640_000_000);
const MAX_DAC_CLK: HertzU32 = HertzU32::Hz(MAX_ADC_CLK.to_raw() / 2);
const MAX_RX_HB1: HertzU32 = HertzU32::Hz(245_760_000);
const MAX_RX_HB2: HertzU32 = HertzU32::Hz(320_000_000);
const MAX_RX_HB3: HertzU32 = HertzU32::Hz(640_000_000);
const MAX_TX_HB1: HertzU32 = HertzU32::Hz(160_000_000);
const MAX_TX_HB2: HertzU32 = HertzU32::Hz(320_000_000);
const MAX_TX_HB3: HertzU32 = HertzU32::Hz(320_000_000);
const MAX_BASEBAND_RATE: HertzU32 = HertzU32::Hz(61_440_000);

/// Lowest VCO frequency. The LO is the VCO divided by a power of two.
const MIN_VCO_FREQ_HZ: u64 = 6_000_000_000;
const MAX_CARRIER_FREQ_HZ: u64 = 6_000_000_000;
const MIN_RX_CARRIER_FREQ_HZ: u64 = 70_000_000;
const MIN_TX_CARRIER_FREQ_HZ: u64 = 46_875_001;

/// Register-level driver core plus the state it needs. Not public, go through [`Ad9361`] so
/// setup is guaranteed to have run.
pub(crate) struct Engine<S: SpiDevice<u8>, I: DataInterface> {
    spi: S,
    interface: I,
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

impl<S, I> Engine<S, I>
where
    S: SpiDevice<u8>,
    I: DataInterface,
{
    /// The state for a chip that was just reset and identified. `ensm_state` is what the chip
    /// reported after the reset. Call [`Self::init_clocks`] before anything else.
    pub(crate) fn new(spi: S, interface: I, config: &Ad9361Config, ensm_state: u8) -> Self {
        Engine {
            spi,
            interface,
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
        }
    }

    pub(crate) fn into_parts(self) -> (S, I) {
        (self.spi, self.interface)
    }
}

#[cfg(test)]
mod tests;
