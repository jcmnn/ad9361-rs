//! Settings of the chip.
//!
//! [`Ad9361Settings`] is plain data with public fields and a `Default`. [`Ad9361Config::new`]
//! checks it.
//! Settings that only apply to one mode live in the mode's enum ([`ChannelMode`], [`Duplex`]).

use super::clock_chain::validate_trx_clock_chain;
use super::*;

/// Frequency of the clock that feeds the chip, 10 to 80 MHz.
///
/// Has to be one the chip's dividers can bring into the BBPLL range. Whether it also fits the synth
/// reference window from the settings is checked in [`Ad9361Config::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReferenceClock(HertzU32);

impl ReferenceClock {
    /// 10 to 80 MHz.
    pub fn new(frequency: HertzU32) -> Result<Self, OutOfRange> {
        let mhz = frequency.to_raw() / 1_000_000;
        if !(10..=80).contains(&mhz) || ref_div_sel(frequency, MAX_BBPLL_FREF).to_raw() == 0 {
            return Err(OutOfRange);
        }
        Ok(Self(frequency))
    }

    pub const fn get(self) -> HertzU32 {
        self.0
    }
}

/// Highest synth reference, the limit for [`Ad9361Settings::trx_synth_max_fref`].
pub const MAX_SYNTH_FREF: HertzU32 = HertzU32::Hz(80_008_000);
/// Lowest synth reference, the limit for [`Ad9361Settings::trx_synth_max_fref`].
pub const MIN_SYNTH_FREF: HertzU32 = HertzU32::Hz(9_999_000);

/// How the reference clock gets into the chip.
///
/// A crystal goes across XTALP and XTALN and runs on the chip's DCXO, which can pull it by about
/// ±60 ppm with a [`DcxoTrim`]. An external oscillator drives XTALN on its own, and needs its own
/// frequency correction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReferenceSource {
    Crystal(DcxoTrim),
    External,
}

/// DCXO trim, pulls the crystal frequency by about ±60 ppm in total.
///
/// `coarse` (0..=63) steps are about 12 ppm and change the load capacitance, `fine` (0..=8191)
/// steps are small enough to track temperature. The resolution is 0.0125 ppm or better. Higher
/// values lower the frequency. The right values depend on the crystal on the board, so measure a
/// known tone (or the LO) and adjust, at run time with
/// [`Ad9361::set_dcxo_trim`](crate::Ad9361::set_dcxo_trim).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DcxoTrim {
    pub coarse: u6,
    pub fine: u13,
}

impl DcxoTrim {
    /// The trim the default settings use. Measure the crystal on the board and adjust from
    /// there.
    pub const DEFAULT: Self = Self {
        coarse: u6::new(8),
        fine: u13::new(5920),
    };
}

impl Default for DcxoTrim {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// 2R2T (both receivers and transmitters) or 1R1T (one of each).
///
/// 2R2T is the default and what was tested. In 1R1T the other channels are switched
/// off and the unused TX attenuation is set to the maximum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelMode {
    /// Both RX and both TX channels
    TwoByTwo,
    /// One RX and one TX channel
    OneByOne { rx: Channel, tx: Channel },
}

impl ChannelMode {
    pub(super) fn is_two_by_two(self) -> bool {
        matches!(self, ChannelMode::TwoByTwo)
    }

    /// RX channel for 1R1T, RX1 in 2R2T.
    pub(super) fn one_by_one_rx(self) -> Channel {
        match self {
            ChannelMode::OneByOne { rx, .. } => rx,
            ChannelMode::TwoByTwo => Channel::Ch1,
        }
    }

    /// TX channel for 1R1T, TX1 in 2R2T.
    pub(super) fn one_by_one_tx(self) -> Channel {
        match self {
            ChannelMode::OneByOne { tx, .. } => tx,
            ChannelMode::TwoByTwo => Channel::Ch1,
        }
    }
}

/// FDD (RX and TX at the same time) or TDD (they take turns).
///
/// After init the chip runs in FDD, or in RX for TDD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Duplex {
    Fdd {
        /// RX and TX LO are set independently
        independent_mode: bool,
    },
    Tdd {
        /// Skip VCO cal on the way from TX/RX to alert
        skip_vco_cal: bool,
        /// One synth each for RX and TX
        dual_synth: bool,
    },
}

impl Duplex {
    pub(super) fn is_fdd(self) -> bool {
        matches!(self, Duplex::Fdd { .. })
    }

    pub(super) fn fdd_independent_mode(self) -> bool {
        matches!(
            self,
            Duplex::Fdd {
                independent_mode: true
            }
        )
    }

    pub(super) fn tdd_skip_vco_cal(self) -> bool {
        matches!(
            self,
            Duplex::Tdd {
                skip_vco_cal: true,
                ..
            }
        )
    }

    pub(super) fn tdd_dual_synth(self) -> bool {
        matches!(
            self,
            Duplex::Tdd {
                dual_synth: true,
                ..
            }
        )
    }
}

/// What to do about the digital interface delays.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DigInterfaceTune {
    /// Tune RX and TX
    RxAndTx,
    /// Tune RX only
    RxOnly,
    /// Don't tune, use the delays from [`PortConfig`](crate::settings::PortConfig). For a second
    /// chip that shares its timing with the first, for example
    UseConfigured,
}

/// Oversampling to pick when the sample rate changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RateGovernor {
    /// as much as possible
    Highest = 0,
    /// the usual amount
    Nominal = 1,
}

/// Gain table layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GainTableKind {
    /// One gain index for all gain stages
    Full,
    /// LMT and LPF gains separate
    Split,
}

/// Invalid settings, from [`Ad9361Config::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// Path clocks over the chip's limits, or none of them matches the data clock
    ClockChain,
    /// Can't get a synth reference below `trx_synth_max_fref` out of this reference clock
    SynthReference,
    /// ENSM pin control and FDD independent mode can't be combined
    EnsmPinControlInFddIndependentMode,
    /// BBPLL can't be divided down to the AuxADC clock (divider has to be under 64)
    AuxAdcClockRate,
    /// Doesn't fit its register field. The string names the setting.
    OutOfRange(&'static str),
}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::ClockChain => f.write_str("the path clocks are beyond the chip's limits"),
            ConfigError::SynthReference => {
                f.write_str("the reference clock has no synthesizer reference in the window")
            }
            ConfigError::EnsmPinControlInFddIndependentMode => {
                f.write_str("ENSM pin control doesn't work in FDD independent mode")
            }
            ConfigError::AuxAdcClockRate => {
                f.write_str("the BBPLL can't be divided down to the AuxADC clock rate")
            }
            ConfigError::OutOfRange(name) => write!(f, "{name} is out of range"),
        }
    }
}

impl core::error::Error for ConfigError {}

/// Catches values that would get truncated on their way into a register field.
fn validate_ranges(settings: &Ad9361Settings) -> Result<(), ConfigError> {
    macro_rules! check {
        ($name:literal, $value:expr, $range:expr) => {
            if !($range).contains(&($value)) {
                return Err(ConfigError::OutOfRange($name));
            }
        };
    }

    let g = &settings.gain_ctrl;
    check!(
        "gain_ctrl.adc_ovr_sample_size",
        g.adc_ovr_sample_size,
        1..=8
    );
    check!(
        "gain_ctrl.lmt_overload_high_thresh",
        g.lmt_overload_high_thresh,
        16..=800
    );
    check!(
        "gain_ctrl.lmt_overload_low_thresh",
        g.lmt_overload_low_thresh,
        16..=800
    );
    check!(
        "gain_ctrl.dec_pow_measurement_duration",
        g.dec_pow_measurement_duration,
        16..=u32::MAX
    );
    check!("gain_ctrl.low_power_thresh", g.low_power_thresh, 0..=64);
    check!("gain_ctrl.max_dig_gain", g.max_dig_gain, 0..=31);
    check!("gain_ctrl.mgc_inc_gain_step", g.mgc_inc_gain_step, 1..=8);
    check!("gain_ctrl.mgc_dec_gain_step", g.mgc_dec_gain_step, 1..=8);
    check!(
        "gain_ctrl.mgc_split_table_ctrl_inp_gain_mode",
        g.mgc_split_table_ctrl_inp_gain_mode,
        0..=2
    );
    check!(
        "gain_ctrl.agc_attack_delay_extra_margin_us",
        g.agc_attack_delay_extra_margin_us,
        0..=31
    );
    check!(
        "gain_ctrl.agc_outer_thresh_high_dec_steps",
        g.agc_outer_thresh_high_dec_steps,
        0..=15
    );
    check!(
        "gain_ctrl.agc_outer_thresh_low_inc_steps",
        g.agc_outer_thresh_low_inc_steps,
        0..=15
    );
    check!(
        "gain_ctrl.agc_inner_thresh_high",
        g.agc_inner_thresh_high,
        0..=127
    );
    check!(
        "gain_ctrl.agc_inner_thresh_low",
        g.agc_inner_thresh_low,
        0..=127
    );
    check!(
        "gain_ctrl.agc_inner_thresh_high_dec_steps",
        g.agc_inner_thresh_high_dec_steps,
        0..=7
    );
    check!(
        "gain_ctrl.agc_inner_thresh_low_inc_steps",
        g.agc_inner_thresh_low_inc_steps,
        0..=7
    );
    check!(
        "gain_ctrl.adc_small_overload_exceed_counter",
        g.adc_small_overload_exceed_counter,
        0..=15
    );
    check!(
        "gain_ctrl.adc_large_overload_exceed_counter",
        g.adc_large_overload_exceed_counter,
        0..=15
    );
    check!(
        "gain_ctrl.adc_large_overload_inc_steps",
        g.adc_large_overload_inc_steps,
        0..=15
    );
    check!(
        "gain_ctrl.lmt_overload_large_exceed_counter",
        g.lmt_overload_large_exceed_counter,
        0..=15
    );
    check!(
        "gain_ctrl.lmt_overload_small_exceed_counter",
        g.lmt_overload_small_exceed_counter,
        0..=15
    );
    check!(
        "gain_ctrl.lmt_overload_large_inc_steps",
        g.lmt_overload_large_inc_steps,
        0..=7
    );
    check!(
        "gain_ctrl.dig_saturation_exceed_counter",
        g.dig_saturation_exceed_counter,
        0..=15
    );
    check!("gain_ctrl.dig_gain_step_size", g.dig_gain_step_size, 0..=7);
    check!(
        "gain_ctrl.f_agc_dec_pow_measurement_duration",
        g.f_agc_dec_pow_measurement_duration,
        16..=u32::MAX
    );
    check!(
        "gain_ctrl.f_agc_lp_thresh_increment_steps",
        g.f_agc_lp_thresh_increment_steps,
        1..=8
    );
    check!(
        "gain_ctrl.f_agc_lock_level_gain_increase_upper_limit",
        g.f_agc_lock_level_gain_increase_upper_limit,
        0..=63
    );
    check!(
        "gain_ctrl.f_agc_lpf_final_settling_steps",
        g.f_agc_lpf_final_settling_steps,
        0..=3
    );
    check!(
        "gain_ctrl.f_agc_lmt_final_settling_steps",
        g.f_agc_lmt_final_settling_steps,
        0..=3
    );
    check!(
        "gain_ctrl.f_agc_final_overrange_count",
        g.f_agc_final_overrange_count,
        0..=7
    );
    check!(
        "gain_ctrl.f_agc_optimized_gain_offset",
        g.f_agc_optimized_gain_offset,
        0..=15
    );
    check!(
        "gain_ctrl.f_agc_rst_gla_stronger_sig_thresh_above_ll",
        g.f_agc_rst_gla_stronger_sig_thresh_above_ll,
        0..=63
    );
    check!(
        "gain_ctrl.f_agc_rst_gla_energy_lost_sig_thresh_below_ll",
        g.f_agc_rst_gla_energy_lost_sig_thresh_below_ll,
        0..=63
    );
    check!(
        "gain_ctrl.f_agc_energy_lost_stronger_sig_gain_lock_exit_cnt",
        g.f_agc_energy_lost_stronger_sig_gain_lock_exit_cnt,
        0..=63
    );
    check!(
        "gain_ctrl.f_agc_power_measurement_duration_in_state5",
        g.f_agc_power_measurement_duration_in_state5,
        16..=u32::MAX
    );
    check!(
        "gain_ctrl.f_agc_large_overload_inc_steps",
        g.f_agc_large_overload_inc_steps,
        0..=7
    );

    check!(
        "dc_offset.update_events",
        settings.dc_offset.update_events,
        0..=7
    );
    check!("rssi.duration", settings.rssi.duration, 1..=u32::MAX);

    let t = &settings.txmon;
    check!("txmon.delay", t.delay, 0..=1023);
    check!("txmon.low_gain_db", t.low_gain_db, 0..=31);
    check!("txmon.high_gain_db", t.high_gain_db, 0..=31);
    check!("txmon.tx1_front_end_gain", t.tx1_front_end_gain, 0..=3);
    check!("txmon.tx2_front_end_gain", t.tx2_front_end_gain, 0..=3);
    check!("txmon.tx1_lo_cm", t.tx1_lo_cm, 0..=63);
    check!("txmon.tx2_lo_cm", t.tx2_lo_cm, 0..=63);
    check!(
        "txmon.low_high_gain_threshold_mdb",
        t.low_high_gain_threshold_mdb,
        0..=63_999
    );
    Ok(())
}

/// All settings of the chip.
///
/// `Default` is a crystal reference with [`DcxoTrim::DEFAULT`], 2R2T, FDD, LVDS, 30.72 MSPS, both LOs at 2.4 GHz, 18 MHz bandwidths, the `A`
/// differential RX inputs and TX outputs, slow attack AGC and 10 dB TX attenuation. The 64 tap example FIR is loaded in both directions and stays bypassed until
/// enabled.
///
/// Change what is needed and keep the rest:
///
/// ```
/// use ad9361::{Ad9361Settings, RxLoFrequency, TxAttenuation, TxLoFrequency};
///
/// let settings = Ad9361Settings {
///     rx_lo_frequency: RxLoFrequency::from_hz(915_000_000),
///     tx_lo_frequency: TxLoFrequency::from_hz(915_000_000),
///     tx_attenuation: TxAttenuation::from_db(20),
///     ..Ad9361Settings::default()
/// };
/// # let _ = settings;
/// ```
///
/// Most fields match the no-OS `AD9361_InitParam` fields, look there for what they do on the chip.
/// Numbers that go into a register field (most of [`GainControl`], for example) are range checked
/// by [`Ad9361Config::new`]. A value that doesn't fit is an error, not silently truncated.
///
/// `rx_path_clks` and `tx_path_clks` are the full clock chain, and default to
/// 30.72 MSPS. For another rate, keep them and call [`Ad9361::set_sample_rate`] after init, or fill
/// them in by hand.
pub struct Ad9361Settings {
    /// Crystal with DCXO trim, or an external oscillator
    pub reference_source: ReferenceSource,
    pub channels: ChannelMode,
    pub rx_path_clks: PathClocks,
    pub tx_path_clks: PathClocks,
    pub rx_input: RxInput,
    pub tx_output: TxOutput,
    pub port: PortConfig,
    pub auxadc: AuxAdcConfig,
    pub ctrl_outs: CtrlOutsConfig,
    pub elna: ElnaConfig,
    /// Highest reference frequency for the RX/TX synths, clamped to
    /// [`MIN_SYNTH_FREF`]..=[`MAX_SYNTH_FREF`]. Lower means worse phase noise but fewer
    /// fractional spurs.
    pub trx_synth_max_fref: HertzU32,
    pub duplex: Duplex,
    pub rx_lo_frequency: RxLoFrequency,
    pub tx_lo_frequency: TxLoFrequency,
    pub gain_ctrl: GainControl,
    pub gain_table: GainTableKind,
    pub rx_bandwidth: RfBandwidth,
    pub tx_bandwidth: RfBandwidth,
    pub dc_offset: DcOffsetConfig,
    pub tracking: TrackingConfig,
    /// ENSM on pins instead of SPI
    pub ensm_pin_ctrl: bool,
    /// TX attenuation at startup
    pub tx_attenuation: TxAttenuation,
    pub clkout_mode: ClkoutMode,
    pub rssi: RssiConfiguration,
    pub txmon: TxMonitorConfig,
    /// ENSM enable pin is a pulse, not a level
    pub ensm_pin_pulse_mode: bool,
    pub dig_interface_tune: DigInterfaceTune,
    pub dig_interface_tune_fir_disable: bool,
    /// Retune the digital interface whenever the baseband clocks change
    pub bb_clk_change_dig_tune_en: bool,
    /// AXI DAC at half rate
    pub axi_half_dac_rate_en: bool,
    pub rate_governor: RateGovernor,
    /// FIRs to load after init
    pub rx_fir: Option<RxFirConfig>,
    pub tx_fir: Option<TxFirConfig>,
    pub auxdac: AuxDacConfig,
    pub gpo: GpoConfig,
}

impl Default for Ad9361Settings {
    fn default() -> Self {
        Self {
            reference_source: ReferenceSource::Crystal(DcxoTrim::DEFAULT),
            channels: ChannelMode::TwoByTwo,
            rx_path_clks: PathClocks::DEFAULT_RX,
            tx_path_clks: PathClocks::DEFAULT_TX,
            rx_input: RxInput::DifferentialA,
            tx_output: TxOutput::A,
            port: PortConfig::default(),
            auxadc: AuxAdcConfig::default(),
            ctrl_outs: CtrlOutsConfig::default(),
            elna: ElnaConfig::default(),
            trx_synth_max_fref: MAX_SYNTH_FREF,
            duplex: Duplex::Fdd {
                independent_mode: false,
            },
            rx_lo_frequency: RxLoFrequency::from_hz(2_400_000_000),
            tx_lo_frequency: TxLoFrequency::from_hz(2_400_000_000),
            gain_ctrl: GainControl::default(),
            gain_table: GainTableKind::Full,
            rx_bandwidth: RfBandwidth::from_hz(18_000_000),
            tx_bandwidth: RfBandwidth::from_hz(18_000_000),
            dc_offset: DcOffsetConfig::default(),
            tracking: TrackingConfig::default(),
            ensm_pin_ctrl: false,
            tx_attenuation: TxAttenuation::from_db(10),
            clkout_mode: ClkoutMode::Disable,
            rssi: RssiConfiguration::default(),
            txmon: TxMonitorConfig::default(),
            ensm_pin_pulse_mode: false,
            dig_interface_tune: DigInterfaceTune::RxAndTx,
            dig_interface_tune_fir_disable: false,
            bb_clk_change_dig_tune_en: false,
            axi_half_dac_rate_en: false,
            rate_governor: RateGovernor::Nominal,
            rx_fir: Some(RxFirConfig::default()),
            tx_fir: Some(TxFirConfig::default()),
            auxdac: AuxDacConfig::default(),
            gpo: GpoConfig::default(),
        }
    }
}

/// [`Ad9361Settings`] and the reference clock, checked against each other. The only thing
/// [`Ad9361::init`](crate::Ad9361::init) takes, so bad settings show up here instead of half way
/// through the setup. Nothing here needs the hardware.
///
/// ```
/// use ad9361::{Ad9361Config, Ad9361Settings, ReferenceClock};
/// use fugit::HertzU32;
///
/// let ref_clk = ReferenceClock::new(HertzU32::MHz(40)).unwrap();
/// let config = Ad9361Config::new(Ad9361Settings::default(), ref_clk).unwrap();
/// # let _ = config;
/// ```
///
/// [`Self::new`] checks that:
///
/// - the path clocks are within the chip's limits and one of them matches DATA_CLK
/// - the reference clock can be divided into the synth reference window (`trx_synth_max_fref`)
/// - the BBPLL rate can be divided down to the AuxADC clock
/// - ENSM pin control isn't combined with FDD independent mode
/// - every number fits its register field ([`ConfigError::OutOfRange`] names the setting)
pub struct Ad9361Config {
    pub(super) settings: Ad9361Settings,
    pub(super) ref_clk: ReferenceClock,
    /// Reference clock scaled into the BBPLL's range
    pub(super) bbpll_ref: HertzU32,
    /// RX/TX synth reference
    pub(super) synth_ref: HertzU32,
}

impl Ad9361Config {
    pub fn new(settings: Ad9361Settings, ref_clk: ReferenceClock) -> Result<Self, ConfigError> {
        validate_ranges(&settings)?;

        // pin control doesn't work with separate RX/TX LOs
        if settings.ensm_pin_ctrl && settings.duplex.fdd_independent_mode() {
            return Err(ConfigError::EnsmPinControlInFddIndependentMode);
        }

        let lvds_mode = settings.port.is_lvds();
        validate_trx_clock_chain::<()>(
            &settings.rx_path_clks,
            &settings.tx_path_clks,
            settings.channels.is_two_by_two(),
            lvds_mode,
        )
        .map_err(|_| ConfigError::ClockChain)?;

        let max_fref = settings
            .trx_synth_max_fref
            .clamp(MIN_SYNTH_FREF, MAX_SYNTH_FREF);
        let synth_ref = ref_div_sel(ref_clk.get(), max_fref);
        if synth_ref.to_raw() == 0 {
            return Err(ConfigError::SynthReference);
        }
        // can't fail, ReferenceClock already checked it
        let bbpll_ref = ref_div_sel(ref_clk.get(), MAX_BBPLL_FREF);

        // BBPLL has to divide down to the AuxADC clock
        settings
            .rx_path_clks
            .bbpll
            .to_raw()
            .checked_div(settings.auxadc.auxadc_clock_rate.to_raw())
            .and_then(|divider| u8::try_from(divider).ok())
            .filter(|divider| *divider < 64)
            .ok_or(ConfigError::AuxAdcClockRate)?;

        Ok(Self {
            settings,
            ref_clk,
            bbpll_ref,
            synth_ref,
        })
    }

    pub fn settings(&self) -> &Ad9361Settings {
        &self.settings
    }

    pub fn reference_clock(&self) -> ReferenceClock {
        self.ref_clk
    }
}
