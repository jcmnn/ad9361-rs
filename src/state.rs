//! What the driver remembers about the chip, split up by subsystem. All of it starts from the
//! checked config, so there are no placeholder values. `Option` only where the chip really
//! has nothing yet (FIR not loaded, no calibration result).

use super::*;

/// Channel, duplex and data interface modes.
pub(crate) struct ModeState {
    /// 2R2T instead of 1R1T
    pub rx2tx2: bool,
    /// FDD, not TDD
    pub fdd: bool,
    /// LVDS data interface, not CMOS
    pub lvds_mode: bool,
    /// Lock detector off while the RFPLL gets programmed (TDD)
    pub tdd_skip_vco_cal: bool,
    /// FDD with separate RX and TX LOs
    pub fdd_independent_mode: bool,
    pub tdd_use_dual_synth: bool,
    /// RX channel in 1R1T
    pub rx1tx1_use_rx: Channel,
    /// TX channel in 1R1T
    pub rx1tx1_use_tx: Channel,
    /// Parallel port conf 3, with the combos the chip can't do already fixed up
    pub pp_conf3: ParallelPortConf3,
    /// TX monitor in TDD
    pub txmon_tdd_en: bool,
}

impl ModeState {
    pub fn new(config: &Ad9361Config) -> Self {
        let settings = &config.settings;
        Self {
            rx2tx2: settings.channels.is_two_by_two(),
            fdd: settings.duplex.is_fdd(),
            lvds_mode: settings.port.sanitized_conf3().lvds_mode(),
            tdd_skip_vco_cal: settings.duplex.tdd_skip_vco_cal(),
            fdd_independent_mode: settings.duplex.fdd_independent_mode(),
            tdd_use_dual_synth: settings.duplex.tdd_dual_synth(),
            rx1tx1_use_rx: settings.channels.one_by_one_rx(),
            rx1tx1_use_tx: settings.channels.one_by_one_tx(),
            pp_conf3: settings.port.sanitized_conf3(),
            txmon_tdd_en: false,
        }
    }
}

/// Clock tree.
pub(crate) struct ClockState {
    /// Rate cache, same idea as `phy->clks[]->rate` in no-OS
    pub rates: Ad9361ClockRates,
    pub rx_path_clks: PathClocks,
    pub tx_path_clks: PathClocks,
    /// 1 = nominal oversampling, 0 = highest
    pub rate_governor: u32,
    /// external reference instead of the crystal
    pub use_extclk: bool,
    /// LO in Hz, once programmed
    pub current_rx_lo_freq: Option<u64>,
    pub current_tx_lo_freq: Option<u64>,
    pub current_rx_use_tdd_table: bool,
    pub current_tx_use_tdd_table: bool,
}

impl ClockState {
    pub fn new(config: &Ad9361Config) -> Self {
        let settings = &config.settings;
        Self {
            rates: Ad9361ClockRates::uninitialized(config.ref_clk.get()),
            rx_path_clks: settings.rx_path_clks,
            tx_path_clks: settings.tx_path_clks,
            rate_governor: settings.rate_governor as u32,
            use_extclk: settings.use_external_clock,
            current_rx_lo_freq: None,
            current_tx_lo_freq: None,
            current_rx_use_tdd_table: false,
            current_tx_use_tdd_table: false,
        }
    }
}

/// RX gain control and gain table.
pub(crate) struct GainState {
    /// as of the last `gc_setup`
    pub gain_ctrl: GainControl,
    pub agc_mode: [GainMode; 2],
    pub elna_settling_delay_ns: u32,
    pub elna_in_gaintable_all_index_en: bool,
    /// split gain table
    pub split_gt: bool,
    /// index of the table that's in the chip
    pub current_table: usize,
    /// Entry whose LPF/TIA word the TX quad cal uses. There may be none, no-OS just prints an
    /// error then.
    pub tx_quad_lpf_tia_match: Option<u8>,
}

impl GainState {
    pub fn new(config: &Ad9361Config) -> Self {
        let settings = &config.settings;
        let split_gt = settings.gain_table == GainTableKind::Split;
        Self {
            gain_ctrl: settings.gain_ctrl.clone(),
            agc_mode: [settings.gain_ctrl.rx1_mode, settings.gain_ctrl.rx2_mode],
            elna_settling_delay_ns: settings.elna.settling_delay_ns,
            elna_in_gaintable_all_index_en: settings.elna.elna_in_gaintable_all_index_en,
            split_gt,
            current_table: gain_control::gain_table_index(
                split_gt,
                settings.rx_synth_freq.get().to_raw(),
            ),
            tx_quad_lpf_tia_match: None,
        }
    }
}

/// FIR with coefficients in the chip.
#[derive(Clone, Copy)]
pub(crate) struct LoadedFir {
    /// decimation (RX) or interpolation (TX)
    pub factor: FirFactor,
    pub ntaps: u32,
    /// Stays bypassed after loading until someone enables it. no-OS does the same.
    pub bypassed: bool,
}

/// RX and TX FIR. `None` until coefficients are loaded.
#[derive(Default)]
pub(crate) struct FirState {
    pub rx: Option<LoadedFir>,
    pub tx: Option<LoadedFir>,
}

impl FirState {
    /// 1 unless there's an enabled FIR
    pub fn rx_decimation(&self) -> u32 {
        self.rx.filter(|fir| !fir.bypassed).map_or(1, |fir| fir.factor as u32)
    }

    /// 1 unless there's an enabled FIR
    pub fn tx_interpolation(&self) -> u32 {
        self.tx.filter(|fir| !fir.bypassed).map_or(1, |fir| fir.factor as u32)
    }

    pub fn rx_bypassed(&self) -> bool {
        self.rx.is_none_or(|fir| fir.bypassed)
    }

    pub fn tx_bypassed(&self) -> bool {
        self.tx.is_none_or(|fir| fir.bypassed)
    }
}

/// Calibrations and tracking.
pub(crate) struct CalibrationState {
    pub rssi_ctrl: RssiConfiguration,
    pub auxadc_config: AuxAdcConfig,
    pub tracking: TrackingConfig,
    pub dc_offset: DcOffsetConfig,
    /// redo TX quad cal after a big TX LO move
    pub auto_cal_en: bool,
    pub cal_threshold_freq: u64,
    pub last_tx_quad_cal_freq: u64,
    /// RF bandwidths from setup
    pub current_rx_bw: HertzU32,
    pub current_tx_bw: HertzU32,
    /// RX NCO phase offset of the last TX quad cal that converged
    pub last_tx_quad_cal_phase: Option<u32>,
    /// RX1/RX2 phase inversion on
    pub rx_phase_inversion: bool,
}

impl CalibrationState {
    pub fn new(config: &Ad9361Config) -> Self {
        let settings = &config.settings;
        Self {
            rssi_ctrl: settings.rssi,
            auxadc_config: settings.auxadc,
            tracking: settings.tracking,
            dc_offset: settings.dc_offset,
            auto_cal_en: false,
            cal_threshold_freq: 100_000_000,
            last_tx_quad_cal_freq: settings.tx_synth_freq.get().to_raw(),
            current_rx_bw: settings.rf_rx_bandwidth.get(),
            current_tx_bw: settings.rf_tx_bandwidth.get(),
            last_tx_quad_cal_phase: None,
            rx_phase_inversion: settings.port.rx1rx2_phase_inversion
                || settings.port.conf2.invert_rx2(),
        }
    }
}

/// Digital interface tuning.
pub(crate) struct TuneState {
    pub bist_loopback_mode: BistLoopback,
    pub bist_config: BistConfig,
    /// DAC sources stashed while ADC data loops back in the FPGA
    pub scratch_dac_source: [axi_ad9361::regs::dac::regs::ChannelDataSource; 4],
    pub dig_interface_tune: DigInterfaceTune,
    pub dig_interface_tune_fir_disable: bool,
    /// retune on every baseband clock change
    pub bb_clk_change_dig_tune_en: bool,
    pub axi_half_dac_rate_en: bool,
    /// from the config, or whatever tuning found last
    pub rx_clk_data_delay: RxClockDataDelay,
    pub tx_clk_data_delay: TxClockDataDelay,
}

impl TuneState {
    pub fn new(config: &Ad9361Config) -> Self {
        let settings = &config.settings;
        Self {
            bist_loopback_mode: BistLoopback::Off,
            bist_config: BistConfig::default(),
            scratch_dac_source: [axi_ad9361::regs::dac::regs::ChannelDataSource::default(); 4],
            dig_interface_tune: settings.dig_interface_tune,
            dig_interface_tune_fir_disable: settings.dig_interface_tune_fir_disable,
            bb_clk_change_dig_tune_en: settings.bb_clk_change_dig_tune_en,
            axi_half_dac_rate_en: settings.axi_half_dac_rate_en,
            rx_clk_data_delay: settings.port.rx_clk_data_delay,
            tx_clk_data_delay: settings.port.tx_clk_data_delay,
        }
    }
}

/// Enable state machine.
pub(crate) struct EnsmTracking {
    /// pins instead of SPI
    pub pin_ctrl: bool,
    /// enable pin is a pulse, not a level
    pub pin_pulse_mode: bool,
    /// state after the last change
    pub current: u8,
    /// was pin control on when the state was last forced?
    pub saved_pin_ctrl_enable: bool,
}

impl EnsmTracking {
    pub fn new(config: &Ad9361Config, current: u8) -> Self {
        let settings = &config.settings;
        Self {
            // config already rejects pin control + separate LOs
            pin_ctrl: settings.ensm_pin_ctrl,
            pin_pulse_mode: settings.ensm_pin_pulse_mode,
            current,
            saved_pin_ctrl_enable: false,
        }
    }
}
