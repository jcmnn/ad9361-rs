//! Gain control: mixer GM sub table, manual/slow/fast AGC setup, and the timing that depends on
//! the clock rates (`ad9361_gc_setup()`, `ad9361_gc_update()`).

use arbitrary_int::{u2, u3, u4, u5, u6, u7};
use embedded_hal::spi::SpiDevice;

use super::gain_tables::GAIN_TABLES;
use super::{
    Channel, RxEnableFilterControl,
    AdcLargeOverloadThresh, AdcOverloadCounters, AdcSmallOverloadThresh, AgcAttackDelay,
    AgcConfig1, AgcConfig2, AgcConfig3, AgcInnerLowThresh, AgcLockLevel, Engine, Ad9361Error,
    DecPowerMeasureDuration0, DigitalGain, DigitalSatCounter, FastAgcllUpperLimit, FastConfig1,
    FastConfig2SettlingDelay, FastEnergyDetectCount, FastEnergyLostThresh,
    FastFinalOverRangeAndOptGain, FastGainLockExitCount, FastIncrementTime,
    FastInitialLmtGainLimit, FastLowPowerThresh, FastStrongSignalFreeze, FastStrongerSignalThresh,
    GainStp1, GainStp2, GainStpConfig1, GainStpConfig2, GainUpdateCounter1, GainUpdateCounter2,
    GmSubTableAddress, GmSubTableBiasWrite, GmSubTableConfig, GmSubTableControlWrite,
    GainTableAddress, GainTableConfig, GainTableReadData1, GainTableWriteData1,
    GainTableWriteData2, GainTableWriteData3, GmSubTableGainRead, GmSubTableGainWrite,
    MaxLmtFullGain, Rx2ManualLmtFullGain, LargeLmtOverloadThresh, LmtOverloadCounters,
    OuterPowerThreshs, PeakWaitTime, Register, Rx1ManualDigitalforcedGain, Rx1ManualLmtFullGain,
    Rx1ManualLpfGain, Rx2ManualDigitalforcedGain, Rx2ManualLpfGain, SmallLmtOverloadThresh,
    TxSymbolAttenConfig,
};

/// mixer GM sub table gains
const GM_ST_GAIN: [u8; 16] = [
    0x78, 0x74, 0x70, 0x6C, 0x68, 0x64, 0x60, 0x5C, 0x58, 0x54, 0x50, 0x4C, 0x48, 0x30, 0x18, 0x0,
];
/// mixer GM sub table controls
const GM_ST_CTRL: [u8; 16] = [
    0x0, 0xD, 0x15, 0x1B, 0x21, 0x25, 0x29, 0x2C, 0x2F, 0x31, 0x33, 0x34, 0x35, 0x3A, 0x3D, 0x3E,
];

/// How an RX channel picks its gain. Each channel has its own mode.
///
/// - `Manual`: the gain stays where it was set (see `Ad9361::manual_gain`).
/// - `FastAttackAgc`: reacts within a few microseconds. Meant for bursty signals like TDD,
///   where the gain has to settle at the start of a burst and then hold.
/// - `SlowAttackAgc`: keeps the average power inside a window and follows it slowly. This is the
///   default and the one for continuous or slowly changing signals such as FDD LTE.
/// - `HybridAgc`: slow attack AGC, but gain updates happen when the CTRL_IN2 pin goes high
///   instead of on the update timer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GainMode {
    Manual = 0,
    FastAttackAgc = 1,
    SlowAttackAgc = 2,
    HybridAgc = 3,
}

/// Gain the fast AGC goes to when it leaves gain lock. The fast AGC settings in
/// [`GainControl`] pick which of these is used and when.
///
/// `MaxGain` is the top of the gain table, `SetGain` is the gain the AGC had last time it
/// locked, `OptimizedGain` is that gain plus an offset that leaves some headroom, and
/// `NoGainChange` leaves the gain alone.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FastAgcTargetGain {
    MaxGain = 0,
    SetGain = 1,
    OptimizedGain = 2,
    NoGainChange = 3,
}

/// Gain control settings for both RX channels. Field names match `struct gain_control` in no-OS,
/// and `Default` is the no-OS slow attack setup.
///
/// The defaults are fine for most uses. Only the fields for the mode in `rx1_mode` and `rx2_mode`
/// matter: the `mgc_` ones for `Manual`, the `agc_` ones for slow attack, the `f_agc_` ones for
/// fast attack, and the "Common" ones for all of them. Hybrid mode uses both AGC groups.
///
/// The overload and power thresholds apply to both channels. To change gain at run time use
/// `Ad9361::set_gain_mode` or `Ad9361::manual_gain`, not these.
///
/// ```
/// use ad9361::{GainControl, GainMode};
///
/// let gain = GainControl {
///     rx1_mode: GainMode::FastAttackAgc,
///     rx2_mode: GainMode::FastAttackAgc,
///     ..GainControl::default()
/// };
/// ```
#[derive(Clone, Debug)]
pub struct GainControl {
    pub rx1_mode: GainMode,
    pub rx2_mode: GainMode,

    // Common
    /// ADC samples summed for the overload check, 1..=8
    pub adc_ovr_sample_size: u8,
    pub adc_small_overload_thresh: u8,
    pub adc_large_overload_thresh: u8,
    /// 16..=800 mV
    pub lmt_overload_high_thresh: u16,
    /// 16..=800 mV
    pub lmt_overload_low_thresh: u16,
    /// Length of one power measurement, in RX samples. Both AGC modes and the low power check
    /// use it.
    pub dec_pow_measurement_duration: u32,
    /// Signal below this is "low power", 0..=64 (-dBFS)
    pub low_power_thresh: u8,
    pub use_rx_fir_out_for_dec_pwr_meas: bool,
    /// leave it off, the ADI gain tables don't use digital gain
    pub dig_gain_en: bool,
    /// 0..=31
    pub max_dig_gain: u8,

    // MGC
    /// RX1 gain steps come from the CTRL_IN pins instead of SPI
    pub mgc_rx1_ctrl_inp_en: bool,
    /// RX2 gain steps come from the CTRL_IN pins instead of SPI
    pub mgc_rx2_ctrl_inp_en: bool,
    /// Table steps per pin pulse when going up, 1..=8
    pub mgc_inc_gain_step: u8,
    /// Table steps per pin pulse when going down, 1..=8
    pub mgc_dec_gain_step: u8,
    /// Split table only: 0: AGC decides, 1: only in LPF, 2: only in LMT
    pub mgc_split_table_ctrl_inp_gain_mode: u8,

    // AGC (slow attack)
    /// Added to the computed AGC attack delay, 0..=31 us
    pub agc_attack_delay_extra_margin_us: u8,
    pub agc_outer_thresh_high: u8,
    pub agc_outer_thresh_high_dec_steps: u8,
    pub agc_inner_thresh_high: u8,
    pub agc_inner_thresh_high_dec_steps: u8,
    pub agc_inner_thresh_low: u8,
    pub agc_inner_thresh_low_inc_steps: u8,
    pub agc_outer_thresh_low: u8,
    pub agc_outer_thresh_low_inc_steps: u8,
    pub adc_small_overload_exceed_counter: u8,
    pub adc_large_overload_exceed_counter: u8,
    pub adc_large_overload_inc_steps: u8,
    pub adc_lmt_small_overload_prevent_gain_inc: bool,
    pub lmt_overload_large_exceed_counter: u8,
    pub lmt_overload_small_exceed_counter: u8,
    pub lmt_overload_large_inc_steps: u8,
    pub dig_saturation_exceed_counter: u8,
    pub dig_gain_step_size: u8,
    pub sync_for_gain_counter_en: bool,
    pub gain_update_interval_us: u32,
    pub immed_gain_change_if_large_adc_overload: bool,
    pub immed_gain_change_if_large_lmt_overload: bool,

    // Fast AGC
    /// Power measurement length while the fast AGC hunts for a level, in RX samples
    pub f_agc_dec_pow_measurement_duration: u32,
    pub f_agc_state_wait_time_ns: u32,
    pub f_agc_allow_agc_gain_increase: bool,
    pub f_agc_lp_thresh_increment_time: u8,
    pub f_agc_lp_thresh_increment_steps: u8,
    pub f_agc_lock_level_lmt_gain_increase_en: bool,
    pub f_agc_lock_level_gain_increase_upper_limit: u8,
    pub f_agc_lpf_final_settling_steps: u8,
    pub f_agc_lmt_final_settling_steps: u8,
    pub f_agc_final_overrange_count: u8,
    pub f_agc_gain_increase_after_gain_lock_en: bool,
    pub f_agc_gain_index_type_after_exit_rx_mode: FastAgcTargetGain,
    pub f_agc_use_last_lock_level_for_set_gain_en: bool,
    pub f_agc_optimized_gain_offset: u8,
    pub f_agc_rst_gla_stronger_sig_thresh_exceeded_en: bool,
    pub f_agc_rst_gla_stronger_sig_thresh_above_ll: u8,
    pub f_agc_rst_gla_energy_lost_sig_thresh_exceeded_en: bool,
    pub f_agc_rst_gla_energy_lost_goto_optim_gain_en: bool,
    pub f_agc_rst_gla_energy_lost_sig_thresh_below_ll: u8,
    pub f_agc_energy_lost_stronger_sig_gain_lock_exit_cnt: u8,
    pub f_agc_rst_gla_large_adc_overload_en: bool,
    pub f_agc_rst_gla_large_lmt_overload_en: bool,
    pub f_agc_rst_gla_en_agc_pulled_high_en: bool,
    pub f_agc_rst_gla_if_en_agc_pulled_high_mode: FastAgcTargetGain,
    /// Power measurement length once the fast AGC has locked, in RX samples
    pub f_agc_power_measurement_duration_in_state5: u32,
    pub f_agc_large_overload_inc_steps: u8,
}

impl Default for GainControl {
    /// no-OS defaults, slow attack AGC
    fn default() -> Self {
        Self {
            rx1_mode: GainMode::SlowAttackAgc,
            rx2_mode: GainMode::SlowAttackAgc,
            adc_ovr_sample_size: 4,
            adc_small_overload_thresh: 47,
            adc_large_overload_thresh: 58,
            lmt_overload_high_thresh: 800,
            lmt_overload_low_thresh: 704,
            dec_pow_measurement_duration: 8192,
            low_power_thresh: 24,
            use_rx_fir_out_for_dec_pwr_meas: false,
            dig_gain_en: false,
            max_dig_gain: 15,
            mgc_rx1_ctrl_inp_en: false,
            mgc_rx2_ctrl_inp_en: false,
            mgc_inc_gain_step: 2,
            mgc_dec_gain_step: 2,
            mgc_split_table_ctrl_inp_gain_mode: 0,
            agc_attack_delay_extra_margin_us: 1,
            agc_outer_thresh_high: 5,
            agc_outer_thresh_high_dec_steps: 2,
            agc_inner_thresh_high: 10,
            agc_inner_thresh_high_dec_steps: 1,
            agc_inner_thresh_low: 12,
            agc_inner_thresh_low_inc_steps: 1,
            agc_outer_thresh_low: 18,
            agc_outer_thresh_low_inc_steps: 2,
            adc_small_overload_exceed_counter: 10,
            adc_large_overload_exceed_counter: 10,
            adc_large_overload_inc_steps: 2,
            adc_lmt_small_overload_prevent_gain_inc: false,
            lmt_overload_large_exceed_counter: 10,
            lmt_overload_small_exceed_counter: 10,
            lmt_overload_large_inc_steps: 2,
            dig_saturation_exceed_counter: 3,
            dig_gain_step_size: 4,
            sync_for_gain_counter_en: false,
            gain_update_interval_us: 1000,
            immed_gain_change_if_large_adc_overload: false,
            immed_gain_change_if_large_lmt_overload: false,
            f_agc_dec_pow_measurement_duration: 64,
            f_agc_state_wait_time_ns: 260,
            f_agc_allow_agc_gain_increase: false,
            f_agc_lp_thresh_increment_time: 5,
            f_agc_lp_thresh_increment_steps: 1,
            f_agc_lock_level_lmt_gain_increase_en: true,
            f_agc_lock_level_gain_increase_upper_limit: 5,
            f_agc_lpf_final_settling_steps: 1,
            f_agc_lmt_final_settling_steps: 1,
            f_agc_final_overrange_count: 3,
            f_agc_gain_increase_after_gain_lock_en: false,
            f_agc_gain_index_type_after_exit_rx_mode: FastAgcTargetGain::MaxGain,
            f_agc_use_last_lock_level_for_set_gain_en: true,
            f_agc_optimized_gain_offset: 5,
            f_agc_rst_gla_stronger_sig_thresh_exceeded_en: true,
            f_agc_rst_gla_stronger_sig_thresh_above_ll: 10,
            f_agc_rst_gla_energy_lost_sig_thresh_exceeded_en: true,
            f_agc_rst_gla_energy_lost_goto_optim_gain_en: true,
            f_agc_rst_gla_energy_lost_sig_thresh_below_ll: 10,
            f_agc_energy_lost_stronger_sig_gain_lock_exit_cnt: 8,
            f_agc_rst_gla_large_adc_overload_en: true,
            f_agc_rst_gla_large_lmt_overload_en: true,
            f_agc_rst_gla_en_agc_pulled_high_en: false,
            f_agc_rst_gla_if_en_agc_pulled_high_mode: FastAgcTargetGain::MaxGain,
            f_agc_power_measurement_duration_in_state5: 64,
            f_agc_large_overload_inc_steps: 2,
        }
    }
}

/// Which receiver(s) a gain table goes to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GainTableDest {
    Rx1 = 1,
    Rx2 = 2,
    Both = 3,
}

fn ilog2(value: u32) -> u32 {
    value.checked_ilog2().unwrap_or(0)
}

/// Gain table for an RX LO in Hz (`ad9361_gt_tableindex()`). First table if nothing matches.
pub(super) fn gain_table_index(split_gt: bool, freq: u64) -> usize {
    GAIN_TABLES
        .iter()
        .position(|t| t.split_table == split_gt && t.start < freq && freq <= t.end)
        .unwrap_or(0)
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    /// `ad9361_load_mixer_gm_subtable()`.
    pub fn load_mixer_gm_subtable(&mut self) -> Result<(), S::Error> {
        let start = GmSubTableConfig::default().with_start_gm_sub_table_clock(true);
        // dummy reads, the table needs a moment
        let delay = |this: &mut Self| -> Result<(), S::Error> {
            this.write_reg(GmSubTableGainRead::default())?;
            this.write_reg(GmSubTableGainRead::default())
        };

        self.write_reg(start)?; // Start clock
        for (i, (gain, ctrl)) in GM_ST_GAIN.iter().zip(GM_ST_CTRL).enumerate() {
            // index counts down
            self.write_reg(GmSubTableAddress((GM_ST_CTRL.len() - 1 - i) as u8))?;
            self.write_reg(GmSubTableBiasWrite::default())?;
            self.write_reg(GmSubTableGainWrite::default().with_gm_sub_table_gain_write(u7::new(*gain)))?;
            self.write_reg(GmSubTableControlWrite::default().with_gm_sub_table_ctrl_write(u6::new(ctrl)))?;
            self.write_reg(start.with_write_gm_sub_table(true))?; // Write words
            delay(self)?;
        }

        self.write_reg(start)?; // Clear write
        delay(self)?;
        self.write_reg(GmSubTableConfig::default()) // Stop clock
    }

    /// `ad9361_gc_setup()`. Common part for MGC and AGC.
    pub fn gc_setup(&mut self, ctrl: &GainControl) -> Result<(), Ad9361Error<S::Error>> {
        self.gain.agc_mode = [ctrl.rx1_mode, ctrl.rx2_mode];
        self.gain.gain_ctrl = ctrl.clone();
        let fast = self.gain.agc_mode.contains(&GainMode::FastAttackAgc);
        let hybrid = self.gain.agc_mode.contains(&GainMode::HybridAgc);

        self.write_reg(
            AgcConfig1::default()
                .with_dec_pwr_for_gain_lock_exit(true)
                .with_dec_pwr_for_lock_level(true)
                .with_dec_pwr_for_low_pwr(true)
                .with_slow_attack_hybrid_mode(hybrid)
                .with_rx1_gain_ctrl_setup(u2::new(ctrl.rx1_mode as u8))
                .with_rx2_gain_ctrl_setup(u2::new(ctrl.rx2_mode as u8)),
        )?;

        // AGC_USE_FULL_GAIN_TABLE is set when the table gets loaded
        self.modify_reg::<AgcConfig2>(|reg| {
            reg.with_man_gain_ctrl_rx1(ctrl.mgc_rx1_ctrl_inp_en)
                .with_man_gain_ctrl_rx2(ctrl.mgc_rx2_ctrl_inp_en)
                .with_dig_gain_en(ctrl.dig_gain_en)
        })?;

        let sample_size = ctrl.adc_ovr_sample_size.clamp(1, 8);
        let mut agc3 = AgcConfig3::default().with_adc_overrange_sample_size(u3::new(sample_size - 1));
        if self.gain.split_gt && (ctrl.mgc_rx1_ctrl_inp_en || ctrl.mgc_rx2_ctrl_inp_en) {
            agc3 = match ctrl.mgc_split_table_ctrl_inp_gain_mode {
                1 => agc3.with_incdec_lmt_gain(false),
                2 => agc3.with_incdec_lmt_gain(true),
                _ => agc3.with_use_agc_for_lmtlpf_gain(true),
            };
        }
        let inc_step = ctrl.mgc_inc_gain_step.clamp(1, 8);
        self.write_reg(agc3.with_manual_incr_step_size(u3::new(inc_step - 1)))?;

        let dec_step = ctrl.mgc_dec_gain_step.clamp(1, 8);
        self.write_reg(
            PeakWaitTime::default().with_manual_ctrl_in_decr_gain_stp_size(u3::new(dec_step - 1)),
        )?;

        if ctrl.dig_gain_en {
            self.write_reg(
                DigitalGain::default()
                    .with_maximum_digital_gain(u5::new(ctrl.max_dig_gain & 0x1F))
                    .with_dig_gain_stp_size(u3::new(ctrl.dig_gain_step_size & 0x7)),
            )?;
        }

        let (small, large) = if ctrl.adc_large_overload_thresh >= ctrl.adc_small_overload_thresh {
            (ctrl.adc_small_overload_thresh, ctrl.adc_large_overload_thresh)
        } else {
            (ctrl.adc_large_overload_thresh, ctrl.adc_small_overload_thresh)
        };
        self.write_reg(AdcSmallOverloadThresh(small))?;
        self.write_reg(AdcLargeOverloadThresh(large))?;

        let lmt = |mv: u16| ((mv as u32 / 16).wrapping_sub(1)).clamp(0, 63) as u8;
        self.write_reg(
            LargeLmtOverloadThresh::default()
                .with_large_lmt_overload_thresh(u6::new(lmt(ctrl.lmt_overload_high_thresh))),
        )?;
        self.modify_reg::<SmallLmtOverloadThresh>(|reg| {
            reg.with_small_lmt_overload_thresh(u6::new(lmt(ctrl.lmt_overload_low_thresh)))
        })?;

        if self.gain.split_gt {
            // REVISIT in no-OS too: RX LPF gain indices and initial LMT gain limit
            self.write_reg(Rx1ManualLpfGain::from_raw(0x58))?;
            self.write_reg(Rx2ManualLpfGain::from_raw(0x18))?;
            self.write_reg(FastInitialLmtGainLimit::from_raw(0x27))?;
        }

        self.write_reg(Rx1ManualDigitalforcedGain::default())?;
        self.write_reg(Rx2ManualDigitalforcedGain::default())?;

        // can spill into the top bit (DONT_UNLOCK_GAIN_IF_ADC_OVRG), no-OS does the same
        self.write_reg(FastLowPowerThresh::from_raw(ctrl.low_power_thresh.min(64) * 2))?;
        self.write_reg(TxSymbolAttenConfig::default())?;

        self.modify_reg::<DecPowerMeasureDuration0>(|reg| {
            reg.with_use_hb1_out_for_dec_pwr_meas(!ctrl.use_rx_fir_out_for_dec_pwr_meas)
        })?;
        self.modify_reg::<DecPowerMeasureDuration0>(|reg| reg.with_enable_dec_pwr_meas(true))?;
        let duration = if fast {
            ctrl.f_agc_dec_pow_measurement_duration
        } else {
            ctrl.dec_pow_measurement_duration
        };
        self.modify_reg::<DecPowerMeasureDuration0>(|reg| {
            reg.with_dec_power_measurement_duration(u4::new((ilog2(duration / 16) & 0xF) as u8))
        })?;

        // AGC
        let inner_high = ctrl.agc_inner_thresh_high.min(127);
        self.modify_reg::<AgcLockLevel>(|reg| {
            reg.with_agc_lock_level_fast_agc_inner_high_thresh_slow(u7::new(inner_high))
        })?;

        let inner_low = ctrl.agc_inner_thresh_low.min(127);
        self.write_reg(
            AgcInnerLowThresh::default()
                .with_agc_inner_low_thresh(u7::new(inner_low))
                .with_prevent_gain_inc(ctrl.adc_lmt_small_overload_prevent_gain_inc),
        )?;

        self.write_reg(
            OuterPowerThreshs::default()
                .with_agc_outer_high_thresh(u4::new(
                    (inner_high as u32).wrapping_sub(ctrl.agc_outer_thresh_high as u32) as u8 & 0xF,
                ))
                .with_agc_outer_low_thresh(u4::new(
                    (ctrl.agc_outer_thresh_low as u32).wrapping_sub(inner_low as u32) as u8 & 0xF,
                )),
        )?;

        self.write_reg(
            GainStp2::default()
                .with_agc_outer_high_thresh_exed_stp_size(u4::new(
                    ctrl.agc_outer_thresh_high_dec_steps & 0xF,
                ))
                .with_agc_outer_low_thresh_exed_stp_size(u4::new(
                    ctrl.agc_outer_thresh_low_inc_steps & 0xF,
                )),
        )?;

        self.write_reg(
            GainStp1::default()
                .with_immed_gain_change_if_lg_adc_overload(ctrl.immed_gain_change_if_large_adc_overload)
                .with_immed_gain_change_if_lg_lmt_overload(ctrl.immed_gain_change_if_large_lmt_overload)
                .with_agc_inner_high_thresh_exed_stp_size(u3::new(
                    ctrl.agc_inner_thresh_high_dec_steps & 0x7,
                ))
                .with_agc_inner_low_thresh_exed_stp_size(u3::new(
                    ctrl.agc_inner_thresh_low_inc_steps & 0x7,
                )),
        )?;

        self.write_reg(
            AdcOverloadCounters::default()
                .with_large_adc_overload_exed_counter(u4::new(
                    ctrl.adc_large_overload_exceed_counter & 0xF,
                ))
                .with_small_adc_overload_exed_counter(u4::new(
                    ctrl.adc_small_overload_exceed_counter & 0xF,
                )),
        )?;

        self.write_reg(
            GainStpConfig2::default()
                .with_decrement_stp_size_for_small_lpf_gain_change(u3::new(
                    ctrl.f_agc_large_overload_inc_steps & 0x7,
                ))
                .with_large_lpf_gain_step(u4::new(ctrl.adc_large_overload_inc_steps & 0xF)),
        )?;

        self.write_reg(
            LmtOverloadCounters::default()
                .with_large_lmt_overload_exed_counter(u4::new(
                    ctrl.lmt_overload_large_exceed_counter & 0xF,
                ))
                .with_small_lmt_overload_exed_counter(u4::new(
                    ctrl.lmt_overload_small_exceed_counter & 0xF,
                )),
        )?;

        self.modify_reg::<GainStpConfig1>(|reg| {
            reg.with_dec_stp_size_for_large_lmt_overload(u3::new(
                ctrl.lmt_overload_large_inc_steps & 0x7,
            ))
        })?;

        self.write_reg(
            DigitalSatCounter::default()
                .with_dig_saturation_exed_counter(u4::new(ctrl.dig_saturation_exceed_counter & 0xF))
                .with_enable_sync_for_gain_counter(ctrl.sync_for_gain_counter_en),
        )?;

        // Fast AGC - low power
        self.modify_reg::<FastConfig1>(|reg| {
            reg.with_enable_incr_gain(ctrl.f_agc_allow_agc_gain_increase)
        })?;
        self.write_reg(FastIncrementTime(ctrl.f_agc_lp_thresh_increment_time))?;
        let steps = (ctrl.f_agc_lp_thresh_increment_steps as u32).wrapping_sub(1).clamp(0, 7) as u8;
        self.modify_reg::<FastEnergyDetectCount>(|reg| {
            reg.with_increment_gain_stp_lpflmt(u3::new(steps))
        })?;

        // Fast AGC - lock level (shared with agc_inner_thresh_high)
        self.modify_reg::<FastConfig2SettlingDelay>(|reg| {
            reg.with_enable_lmt_gain_inc_for_lock_level(ctrl.f_agc_lock_level_lmt_gain_increase_en)
        })?;
        self.modify_reg::<FastAgcllUpperLimit>(|reg| {
            reg.with_agcll_max_increase(u6::new(
                ctrl.f_agc_lock_level_gain_increase_upper_limit.min(63),
            ))
        })?;

        // Fast AGC - peak detectors and final settling
        self.modify_reg::<FastEnergyLostThresh>(|reg| {
            reg.with_post_lock_level_stp_size_for_lpf_table_full_table(u2::new(
                ctrl.f_agc_lpf_final_settling_steps.min(3),
            ))
        })?;
        self.modify_reg::<FastStrongerSignalThresh>(|reg| {
            reg.with_post_lock_level_stp_for_lmt_table(u2::new(
                ctrl.f_agc_lmt_final_settling_steps.min(3),
            ))
        })?;
        self.modify_reg::<FastFinalOverRangeAndOptGain>(|reg| {
            reg.with_final_over_range_count(u3::new(ctrl.f_agc_final_overrange_count.min(7)))
        })?;

        // Fast AGC - final power test
        self.modify_reg::<FastConfig1>(|reg| {
            reg.with_enable_gain_inc_after_gain_lock(ctrl.f_agc_gain_increase_after_gain_lock_en)
        })?;

        // Fast AGC - unlocking the gain
        let after_exit = ctrl.f_agc_gain_index_type_after_exit_rx_mode;
        self.modify_reg::<FastConfig1>(|reg| {
            reg.with_goto_set_gain_if_exit_rx_state(after_exit == FastAgcTargetGain::SetGain)
                .with_goto_optimized_gain_if_exit_rx_state(
                    after_exit == FastAgcTargetGain::OptimizedGain,
                )
        })?;
        self.modify_reg::<FastConfig2SettlingDelay>(|reg| {
            reg.with_use_last_lock_level_for_set_gain(ctrl.f_agc_use_last_lock_level_for_set_gain_en)
        })?;
        self.modify_reg::<FastFinalOverRangeAndOptGain>(|reg| {
            reg.with_optimize_gain_offset(u4::new(ctrl.f_agc_optimized_gain_offset.min(15)))
        })?;

        let unlock_ctrl = !ctrl.f_agc_rst_gla_stronger_sig_thresh_exceeded_en
            || !ctrl.f_agc_rst_gla_energy_lost_sig_thresh_exceeded_en
            || !ctrl.f_agc_rst_gla_large_adc_overload_en
            || !ctrl.f_agc_rst_gla_large_lmt_overload_en
            || ctrl.f_agc_rst_gla_en_agc_pulled_high_en;
        self.modify_reg::<AgcConfig2>(|reg| reg.with_agc_gain_unlock_ctrl(unlock_ctrl))?;

        self.modify_reg::<FastStrongSignalFreeze>(|reg| {
            reg.with_dont_unlock_gain_if_stronger_signal(
                !ctrl.f_agc_rst_gla_stronger_sig_thresh_exceeded_en,
            )
        })?;
        self.modify_reg::<FastStrongerSignalThresh>(|reg| {
            reg.with_stronger_signal_thresh(u6::new(
                ctrl.f_agc_rst_gla_stronger_sig_thresh_above_ll.min(63),
            ))
        })?;
        self.modify_reg::<FastEnergyLostThresh>(|reg| {
            reg.with_energy_lost_thresh(u6::new(
                ctrl.f_agc_rst_gla_energy_lost_sig_thresh_below_ll.min(63),
            ))
        })?;
        self.modify_reg::<FastConfig1>(|reg| {
            reg.with_goto_opt_gain_if_energy_lost_or_en_agc_high(
                ctrl.f_agc_rst_gla_energy_lost_goto_optim_gain_en,
            )
        })?;
        self.modify_reg::<FastConfig1>(|reg| {
            reg.with_dont_unlock_gain_if_energy_lost(
                !ctrl.f_agc_rst_gla_energy_lost_sig_thresh_exceeded_en,
            )
        })?;
        self.modify_reg::<FastGainLockExitCount>(|reg| {
            reg.with_gain_lock_exit_count(u6::new(
                ctrl.f_agc_energy_lost_stronger_sig_gain_lock_exit_cnt.min(63),
            ))
        })?;
        self.modify_reg::<FastConfig1>(|reg| {
            reg.with_dont_unlock_gain_if_lg_adc_or_lmt_ovrg(
                !ctrl.f_agc_rst_gla_large_adc_overload_en
                    || !ctrl.f_agc_rst_gla_large_lmt_overload_en,
            )
        })?;
        self.modify_reg::<FastLowPowerThresh>(|reg| {
            reg.with_dont_unlock_gain_if_adc_ovrg(!ctrl.f_agc_rst_gla_large_adc_overload_en)
        })?;

        // what to do when EN_AGC goes high with the gain locked
        if ctrl.f_agc_rst_gla_en_agc_pulled_high_en {
            // (goto max/opt gain, goto set gain, goto opt gain). `None` = leave the last one
            let (max_or_opt, set, opt) = match ctrl.f_agc_rst_gla_if_en_agc_pulled_high_mode {
                FastAgcTargetGain::MaxGain => (true, false, Some(false)),
                FastAgcTargetGain::SetGain => (false, true, None),
                FastAgcTargetGain::OptimizedGain => (true, false, Some(true)),
                FastAgcTargetGain::NoGainChange => (false, false, None),
            };
            self.set_agc_pulled_high(max_or_opt, set, opt)?;
        } else {
            self.set_agc_pulled_high(false, false, None)?;
        }

        let state5 = ilog2(ctrl.f_agc_power_measurement_duration_in_state5 / 16).min(15);
        self.modify_reg::<Rx1ManualLpfGain>(|reg| {
            reg.with_power_meas_in_state_5(u3::new((state5 & 0x7) as u8))
        })?;
        self.modify_reg::<Rx1ManualLmtFullGain>(|reg| {
            reg.with_power_meas_in_state_5_msb(state5 >> 3 != 0)
        })?;

        self.gc_update()
    }

    fn set_agc_pulled_high(
        &mut self,
        goto_max_or_opt: bool,
        goto_set: bool,
        goto_opt: Option<bool>,
    ) -> Result<(), S::Error> {
        self.modify_reg::<FastConfig2SettlingDelay>(|reg| {
            reg.with_goto_max_gain_or_opt_gain_if_en_agc_high(goto_max_or_opt)
        })?;
        self.modify_reg::<FastConfig1>(|reg| reg.with_goto_set_gain_if_en_agc_high(goto_set))?;
        if let Some(opt) = goto_opt {
            self.modify_reg::<FastConfig1>(|reg| {
                reg.with_goto_opt_gain_if_energy_lost_or_en_agc_high(opt)
            })?;
        }
        Ok(())
    }

    /// Redoes the timing that depends on CLKRF and the RX rate (`ad9361_gc_update()`). Call it
    /// after baseband clock changes.
    pub fn gc_update(&mut self) -> Result<(), Ad9361Error<S::Error>> {
        let clkrf = self.clk.rates.clkrf.to_raw();
        let rx_sampl = self.clk.rates.rx_sampl.to_raw();
        if clkrf < 2000 || rx_sampl == 0 {
            return Err(Ad9361Error::InvalidRate);
        }
        let ctrl = self.gain.gain_ctrl.clone();
        let delay_lna = self.gain.elna_settling_delay_ns;
        let div_round_up = |n: u32, d: u32| n.div_ceil(d);
        let div_round_closest = |n: u32, d: u32| (n + d / 2) / d;

        // AGC attack delay (us) = ceil(((0.2 + Delay_LNA) * ClkRF + 14) / (2 * ClkRF)) + 1
        let reg = (200 + delay_lna) / 2 + 14_000_000 / (clkrf / 500);
        let reg = div_round_up(reg, 1000) + ctrl.agc_attack_delay_extra_margin_us as u32;
        self.modify_reg::<AgcAttackDelay>(|r| r.with_agc_attack_delay(u6::new(reg.min(31) as u8)))?;

        // Peak overload wait time (ClkRF cycles) = ceil((0.1 + Delay_LNA) * ClkRF + 1)
        let reg = (delay_lna + 100).wrapping_mul(clkrf / 1000);
        let reg = div_round_up(reg, 1_000_000) + 1;
        self.modify_reg::<PeakWaitTime>(|r| r.with_peak_overload_wait_time(u5::new(reg.min(31) as u8)))?;

        // Settling delay applies to all gain control modes:
        // ceil((0.2 + Delay_LNA) * ClkRF)
        let reg = (delay_lna + 200).wrapping_mul(clkrf / 2000);
        let settling_delay = (div_round_up(reg, 1_000_000) + 7).min(31);
        self.modify_reg::<FastConfig2SettlingDelay>(|r| {
            r.with_settling_delay(u5::new(settling_delay as u8))
        })?;

        // Gain update counter[15:0] = round(((time * ClkRF - settling * 2) - 2) / 2)
        let reg = ctrl
            .gain_update_interval_us
            .wrapping_mul(clkrf / 1000)
            .wrapping_sub(settling_delay * 2000)
            .wrapping_sub(2000);
        let reg = div_round_closest(reg, 2000).min(131_071);

        let dec_pow_meas_dur = if self.gain.agc_mode.contains(&GainMode::FastAttackAgc) {
            ctrl.f_agc_dec_pow_measurement_duration
        } else {
            let fir_div = div_round_closest(clkrf, rx_sampl).max(1);
            let dur = ctrl.dec_pow_measurement_duration;
            if dur == 0 || (reg * 2 / fir_div) / dur < 2 {
                reg / fir_div
            } else {
                dur
            }
        };
        if dec_pow_meas_dur < 16 {
            return Err(Ad9361Error::GainControlTiming);
        }
        self.modify_reg::<DecPowerMeasureDuration0>(|r| {
            r.with_dec_power_measurement_duration(u4::new((ilog2(dec_pow_meas_dur / 16) & 0xF) as u8))
        })?;

        self.modify_reg::<DigitalSatCounter>(|r| r.with_double_gain_counter(reg > 65_535))?;
        let reg = if reg > 65_535 { reg / 2 } else { reg };
        self.write_reg(GainUpdateCounter1(reg as u8))?;
        self.write_reg(GainUpdateCounter2((reg >> 8) as u8))?;

        // Fast AGC state wait time - energy detect count
        let reg = div_round_closest(
            ctrl.f_agc_state_wait_time_ns.wrapping_mul(clkrf / 1000),
            1_000_000,
        )
        .min(31);
        self.modify_reg::<FastEnergyDetectCount>(|r| r.with_energy_detect_count(u5::new(reg as u8)))?;
        Ok(())
    }

    /// Current mode.
    pub fn gain_mode(&self, channel: Channel) -> GainMode {
        self.gain.agc_mode[channel as usize]
    }

    /// `ad9361_set_gain_ctrl_mode()`. The channel is off while the mode changes.
    pub fn set_gain_mode(
        &mut self,
        channel: Channel,
        mode: GainMode,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let rx = self.read_reg::<RxEnableFilterControl>()?.rx_channel_enable().value();
        let mask = 1 << channel as u8;
        self.modify_reg::<RxEnableFilterControl>(|reg| {
            reg.with_rx_channel_enable(u2::new(rx & !mask))
        })?;

        let setup = u2::new(mode as u8);
        self.modify_reg::<AgcConfig1>(|reg| {
            let reg = match channel {
                Channel::Ch1 => reg.with_rx1_gain_ctrl_setup(setup),
                Channel::Ch2 => reg.with_rx2_gain_ctrl_setup(setup),
            };
            reg.with_slow_attack_hybrid_mode(mode == GainMode::HybridAgc)
        })?;
        self.gain.agc_mode[channel as usize] = mode;

        self.modify_reg::<RxEnableFilterControl>(|reg| {
            reg.with_rx_channel_enable(u2::new(rx))
        })?;
        self.gc_update()
    }

    /// `ad9361_set_rx_gain()` for a full table, channel has to be in manual mode. Returns the gain
    /// of the entry it picked.
    pub(super) fn set_manual_rx_gain(
        &mut self,
        channel: Channel,
        gain_db: i8,
    ) -> Result<i8, S::Error> {
        let table = self.gain.current_table;
        let index = Self::find_table_index(table, gain_db);
        let index7 = u7::new(index as u8);
        match channel {
            Channel::Ch1 => self.modify_reg::<Rx1ManualLmtFullGain>(|reg| {
                reg.with_rx1_manual_full_table_lmt_table_gain_index(index7)
            })?,
            Channel::Ch2 => self.modify_reg::<Rx2ManualLmtFullGain>(|reg| {
                reg.with_rx2_manual_full_table_lmt_table_gain_index(index7)
            })?,
        }
        Ok(GAIN_TABLES[table].abs_gain[index])
    }

    /// Entry of `table` closest to `gain` in dB (`find_table_index()`).
    fn find_table_index(table: usize, gain: i8) -> usize {
        let abs = GAIN_TABLES[table].abs_gain;
        // above the table = last entry
        let Some(i) = abs.iter().position(|g| *g >= gain) else {
            return abs.len() - 1;
        };
        let previous = abs[i.saturating_sub(1)].abs_diff(gain);
        let current = abs[i].abs_diff(gain);
        if previous < current { i.saturating_sub(1) } else { i }
    }

    /// Loads the table for an RX LO in Hz (`ad9361_load_gt()`), keeping the current gains. No-op
    /// if that table is already in.
    pub fn load_gain_table(
        &mut self,
        freq: u64,
        dest: GainTableDest,
    ) -> Result<(), S::Error> {
        let band = gain_table_index(self.gain.split_gt, freq);
        if self.gain.current_table == band {
            return Ok(());
        }
        self.write_gain_table(band, dest)
    }

    /// Writes table `band` into the chip, keeps the current gains. Also finds the entry the TX
    /// quad cal needs.
    pub(super) fn write_gain_table(
        &mut self,
        band: usize,
        dest: GainTableDest,
    ) -> Result<(), S::Error> {
        let table = &GAIN_TABLES[band];
        let index_max = table.entries.len();

        let full_table = !self.gain.split_gt;
        self.modify_reg::<AgcConfig2>(|reg| reg.with_agc_use_full_gain_table(full_table))?;
        self.write_reg(
            MaxLmtFullGain::default()
                .with_maximum_full_tablelmt_table_index(u7::new((index_max - 1) as u8)),
        )?;

        // absolute gain of the current RX1/RX2 indices, so it can be restored in the new table
        let abs_gain = |this: &Self, index: u8| {
            let abs = GAIN_TABLES[this.gain.current_table].abs_gain;
            abs[(index as usize).min(abs.len() - 1)]
        };
        let rx1_index = self
            .read_reg::<Rx1ManualLmtFullGain>()?
            .rx1_manual_full_table_lmt_table_gain_index()
            .value();
        let rx1_gain = abs_gain(self, rx1_index);
        let rx2_index = self
            .read_reg::<Rx2ManualLmtFullGain>()?
            .rx2_manual_full_table_lmt_table_gain_index()
            .value();
        let rx2_gain = abs_gain(self, rx2_index);

        let ext_lna = if self.gain.elna_in_gaintable_all_index_en { 0x80 } else { 0 };
        let select = u2::new(dest as u8);
        let config = GainTableConfig::default()
            .with_start_gain_table_clock(true)
            .with_receiver_select(select);
        let delay = |this: &mut Self| -> Result<(), S::Error> {
            // dummy writes, about 1 us / 3 ADCCLK/16 cycles
            this.write_reg(GainTableReadData1::default())?;
            this.write_reg(GainTableReadData1::default())
        };

        self.write_reg(config)?; // Start gain table clock

        // TX quad cal wants the last entry with this LPF/TIA word
        let lpf_tia_mask = if self.gain.split_gt { 0x20 } else { 0x3F };
        self.gain.tx_quad_lpf_tia_match = None;

        for (i, word) in table.entries.iter().enumerate() {
            self.write_reg(GainTableAddress::default().with_gain_table_address(u7::new(i as u8)))?;
            self.write_reg(GainTableWriteData1::from_raw(word[0] | ext_lna))?;
            self.write_reg(GainTableWriteData2::from_raw(word[1]))?;
            self.write_reg(GainTableWriteData3(word[2]))?;
            self.write_reg(config.with_write_gain_table(true))?;
            delay(self)?;

            if word[1] & lpf_tia_mask == 0x20 {
                self.gain.tx_quad_lpf_tia_match = Some(i as u8);
            }
        }

        self.write_reg(config)?; // Clear write bit
        delay(self)?;
        self.write_reg(GainTableConfig::default())?; // Stop gain table clock

        self.gain.current_table = band;

        let rx1 = Self::find_table_index(band, rx1_gain) as u8;
        self.modify_reg::<Rx1ManualLmtFullGain>(|reg| {
            reg.with_rx1_manual_full_table_lmt_table_gain_index(u7::new(rx1))
        })?;
        let rx2 = Self::find_table_index(band, rx2_gain) as u8;
        self.write_reg(
            Rx2ManualLmtFullGain::default()
                .with_rx2_manual_full_table_lmt_table_gain_index(u7::new(rx2)),
        )
    }
}
