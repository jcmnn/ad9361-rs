//! Baseband filter, TIA and ADC calibration and setup, DC offset, tracking, TX quad cal.
//! Function names follow `ad9361_rx_bb_analog_filter_calib()` and friends in no-OS.

use arbitrary_int::{u2, u3, u4, u5, u6};
use core::num::NonZeroU32;

use embedded_hal::spi::SpiDevice;
use fugit::HertzU32;

use super::{
ForcedEnsmState,     Engine, Ad9361Error, BbDcOffsetAtten, BbDcOffsetCount, BbDcOffsetShift, CalibrationConfig1,
    CalibrationConfig2, CalibrationConfig3, CalibrationControl, Capacitor, Channel, Config0,
    DcOffsetConfig2, InvertBits, Kexp1, Kexp2, MagFtestThresh, MagFtestThresh2,
    ParallelPortConf2, QuadCalControl, QuadCalCount, QuadCalNcoFreqPhaseOffset, QuadCalStatusTx1,
    QuadCalStatusTx2, QuadSettleCount, Register, Resistor, RfDcOffsetAtten, RfDcOffsetConfig1,
    RfDcOffsetCount, RxQuadGain2, TxEnableFilterControl, TxQuadFullLmtGain, TxQuadLpfGain,
    WaitCount,
    Rx1TuneControl, Rx2TuneControl, RxBbbwKhz, RxBbbwMhz, RxBbfC3Lsb, RxBbfC3Msb, RxBbfR2346,
    RxBbfTuneConfig, RxBbfTuneDivide, RxMixGmConfig, RxMixLoCm, RxTiaConfig, Tia1CLsb, Tia1CMsb,
    Tia2CLsb, Tia2CMsb, TxBbfTuneDivider, TxBbfTuneMode, TxTuneControl,
};

/// RX DC offset cal and tracking.
#[derive(Clone, Copy, Debug)]
pub struct DcOffsetConfig {
    /// tracking update events
    pub update_events: u8,
    pub attenuation_high: u8,
    pub attenuation_low: u8,
    pub count_high: u8,
    pub count_low: u8,
}

impl Default for DcOffsetConfig {
    /// no-OS defaults
    fn default() -> Self {
        Self {
            update_events: 5,
            attenuation_high: 6,
            attenuation_low: 5,
            count_high: 0x28,
            count_low: 0x32,
        }
    }
}

/// Which RX tracking cals are on.
#[derive(Clone, Copy, Debug)]
pub struct TrackingConfig {
    pub bbdc: bool,
    pub rfdc: bool,
    pub rx_quad: bool,
    /// slow mode for the quadrature error correction tracking
    pub qec_slow_mode: bool,
}

impl Default for TrackingConfig {
    fn default() -> Self {
        Self {
            bbdc: true,
            rfdc: true,
            rx_quad: true,
            qec_slow_mode: false,
        }
    }
}

/// RX NCO phase offset the TX quad cal starts from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RxPhase {
    /// the one that matches the clock config
    Auto,
    /// a specific one, 0..=31
    Fixed(u8),
    /// skip the first attempt, search all of them
    ForceSearch,
}

/// LO leakage and single sideband both converged.
const TX_QUAD_CONVERGED: u8 = 0b11;

/// Longest run of zeros in `field`: (length, start). `ad9361_find_opt()`.
pub(super) fn find_opt(field: &[u8]) -> (usize, usize) {
    let (mut count, mut max_count) = (0, 0);
    let (mut start, mut max_start) = (None, 0);
    for (i, value) in field.iter().enumerate() {
        if *value == 0 {
            start.get_or_insert(i);
            count += 1;
        } else {
            if count > max_count {
                max_count = count;
                max_start = start.unwrap_or(0);
            }
            start = None;
            count = 0;
        }
    }
    if count > max_count {
        max_count = count;
        max_start = start.unwrap_or(0);
    }
    (max_count, max_start)
}

fn div_round_closest(n: u32, d: u32) -> u32 {
    n.wrapping_add(d / 2) / d
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    /// Runs the cals in `mask` and waits (`ad9361_run_calibration()`).
    pub(super) async fn run_calibration(&mut self, mask: u8) -> Result<(), Ad9361Error<S::Error>> {
        self.write_reg(CalibrationControl::from_raw(mask))?;
        self.wait_until::<CalibrationControl>(
            CalibrationControl::ADDRESS,
            |reg| reg.to_raw() & mask == 0,
            1200,
        )
        .await
    }

    /// RX baseband analog filter cal. Returns the tune divider, [`Self::rx_adc_setup`] wants it.
    pub async fn rx_bb_analog_filter_calib(
        &mut self,
        rx_bb_bw: HertzU32,
        bbpll: HertzU32,
    ) -> Result<NonZeroU32, Ad9361Error<S::Error>> {
        let bw = rx_bb_bw.to_raw().clamp(200_000, 28_000_000);

        // 1.4 * BBBW * 2PI / ln(2)
        let target = 126_906 * (bw / 10_000);
        let div = 511.min(bbpll.to_raw().div_ceil(target));
        let divider = NonZeroU32::new(div).ok_or(Ad9361Error::InvalidRate)?;

        self.write_reg(RxBbfTuneDivide(div as u8))?;
        self.modify_reg::<RxBbfTuneConfig>(|reg| reg.with_rx_bbf_tune_divide(div >> 8 != 0))?;

        self.write_reg(RxBbbwMhz::from_raw((bw / 1_000_000) as u8))?;
        let khz = div_round_closest((bw % 1_000_000) * 128, 1_000_000).min(127);
        self.write_reg(RxBbbwKhz::from_raw(khz as u8))?;

        self.write_reg(RxMixLoCm::default().with_rx_mix_lo_cm(u6::new(0x3F)))?;
        self.write_reg(RxMixGmConfig::default().with_rx_mix_gm_pload(u2::new(3)))?;

        self.write_reg(Rx1TuneControl::default().with_rx1_tune_resample(true))?;
        self.write_reg(Rx2TuneControl::default().with_rx2_tune_resample(true))?;

        // done when RX_BB_TUNE_CAL clears itself
        let result = self.run_calibration(CalibrationControl::default().with_rx_bb_tune_cal(true).raw_value()).await;

        self.write_reg(
            Rx1TuneControl::default()
                .with_rx1_tune_resample(true)
                .with_rx1_pd_tune(true),
        )?;
        self.write_reg(
            Rx2TuneControl::default()
                .with_rx2_tune_resample(true)
                .with_rx2_pd_tune(true),
        )?;

        result.map(|()| divider)
    }

    /// TX baseband analog filter cal.
    pub async fn tx_bb_analog_filter_calib(
        &mut self,
        tx_bb_bw: HertzU32,
        bbpll: HertzU32,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let bw = tx_bb_bw.to_raw().clamp(625_000, 20_000_000);

        // 1.6 * BBBW * 2PI / ln(2)
        let target = 145_036 * (bw / 10_000);
        let div = 511.min(bbpll.to_raw().div_ceil(target));

        self.write_reg(TxBbfTuneDivider(div as u8))?;
        self.modify_reg::<TxBbfTuneMode>(|reg| reg.with_tx_bbf_tune_divider(div >> 8 != 0))?;

        let tune = TxTuneControl::default()
            .with_tuner_resample(true)
            .with_tune_ctrl(u2::new(1));
        self.write_reg(tune)?;

        let result = self.run_calibration(CalibrationControl::default().with_tx_bb_tune_cal(true).raw_value()).await;

        self.write_reg(tune.with_pd_tune(true))?;

        result
    }

    /// TX secondary filter (`ad9361_tx_bb_second_filter_calib()`).
    pub fn tx_bb_second_filter_calib(&mut self, tx_bb_bw: HertzU32) -> Result<(), S::Error> {
        let bw = tx_bb_bw.to_raw().clamp(530_000, 20_000_000);

        // BBBW * 5PI
        let corner = 15_708 * (bw / 10_000);

        let mut res = 1u32;
        let mut cap = 0u64;
        for _ in 0..4 {
            let div = (corner * res) as u64;
            cap = (500_000_000 + (div >> 1)) / div;
            cap = cap.wrapping_sub(12);
            if cap < 64 {
                break;
            }
            res <<= 1;
        }
        let cap = cap.min(63) as u8;

        let conf = if bw <= 4_500_000 {
            0x59
        } else if bw <= 12_000_000 {
            0x56
        } else {
            0x57
        };
        let resistor = match res {
            1 => 0x0C,
            2 => 0x04,
            4 => 0x03,
            _ => 0x01,
        };

        self.write_reg(Config0::from_raw(conf))?;
        self.write_reg(Resistor::from_raw(resistor))?;
        self.write_reg(Capacitor::from_raw(cap))
    }

    /// RX TIA setup off the baseband filter settings (`ad9361_rx_tia_calib()`).
    pub fn rx_tia_calib(&mut self, rx_bb_bw: HertzU32) -> Result<(), S::Error> {
        let c3_msb = self.read_reg::<RxBbfC3Msb>()?.to_raw() as u32;
        let c3_lsb = self.read_reg::<RxBbfC3Lsb>()?.to_raw() as u32;
        let r2346 = (self.read_reg::<RxBbfR2346>()?.to_raw() & 0x7) as u32;

        let bw = rx_bb_bw.to_raw().clamp(200_000, 20_000_000);

        let cbbf = c3_msb * 160 + c3_lsb * 10 + 140; // fF
        let r2346 = 18_300 * r2346;
        let ctia_ff = cbbf as u64 * r2346 as u64 * 560 / 3_500_000;

        let tia_config = if bw <= 3_000_000 {
            0xE0
        } else if bw <= 10_000_000 {
            0x60
        } else {
            0x20
        };

        let (c_lsb, c_msb) = if ctia_ff > 2920 {
            let msb = 127.min(div_round_closest((ctia_ff as u32).wrapping_sub(400), 320)) as u8;
            (0x40, msb)
        } else {
            // truncated to a byte, no-OS does it too
            let lsb = div_round_closest((ctia_ff as u32).wrapping_sub(400), 40).wrapping_add(0x40);
            (lsb as u8, 0)
        };

        self.write_reg(RxTiaConfig::from_raw(tia_config))?;
        self.write_reg(Tia1CLsb::from_raw(c_lsb))?;
        self.write_reg(Tia1CMsb::from_raw(c_msb))?;
        self.write_reg(Tia2CLsb::from_raw(c_lsb))?;
        self.write_reg(Tia2CMsb::from_raw(c_msb))
    }

    /// RX ADC setup, registers 0x200..0x227 (`ad9361_rx_adc_setup()`). `rxbbf_div` comes from
    /// [`Self::rx_bb_analog_filter_calib`].
    pub fn rx_adc_setup(
        &mut self,
        bbpll: HertzU32,
        adc_sampl_freq: HertzU32,
        rxbbf_div: NonZeroU32,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let c3_msb = self.read_reg::<RxBbfC3Msb>()?.to_raw() as u64;
        let c3_lsb = self.read_reg::<RxBbfC3Lsb>()?.to_raw() as u64;
        let r2346 = self.read_reg::<RxBbfR2346>()?.to_raw() as u64;
        let adc = adc_sampl_freq.to_raw();
        if adc < 1000 {
            return Err(Ad9361Error::InvalidRate);
        }

        // BBBW = (BBPLL / RxTuneDiv) * ln(2) / (1.4 * 2PI)
        let bb_bw = (bbpll.to_raw() as u64 * 10_000 / (126_906 * rxbbf_div.get() as u64)) as u32;
        let bb_bw = bb_bw.clamp(200_000, 28_000_000);

        let scale_snr_1e3: u64 = if adc < 80_000_000 { 1000 } else { 1585 }; // 10^(scale_snr_dB/10)

        // same maths as no-OS, integer widths and wrapping included
        let filter_c = 160 * c3_msb + 10 * c3_lsb + 140;
        let base = 160_975u64
            .wrapping_mul(r2346)
            .wrapping_mul(filter_c)
            .wrapping_mul(bb_bw as u64);
        let mut invrc_tconst_1e6 = if bb_bw >= 18_000_000 {
            base.wrapping_mul(1000 + (10 * (bb_bw - 18_000_000) / 1_000_000) as u64) / 1000
        } else {
            base
        };
        invrc_tconst_1e6 /= 1_000_000_000;
        if invrc_tconst_1e6 == 0 {
            return Err(Ad9361Error::CalibrationResult);
        }

        let sqrt_inv_rc_tconst_1e3 = (invrc_tconst_1e6 as u32).isqrt();
        let maxsnr = 640 / 160;
        let scaled_adc_clk_1e6 = div_round_closest(adc, 640);
        let inv_scaled_adc_clk_1e3 =
            div_round_closest(640_000_000, div_round_closest(adc, 1000));
        let tmp_1e3 = div_round_closest(
            980_000 + 20 * 1000.max(div_round_closest(inv_scaled_adc_clk_1e3, maxsnr)),
            1000,
        );
        let sqrt_term_1e3 = scaled_adc_clk_1e6.isqrt();
        let min_sqrt_term_1e3 = 1000.min((maxsnr * scaled_adc_clk_1e6).isqrt());

        let mut data = [0u8; 40];
        data[3] = 0x24;
        data[4] = 0x24;

        let sqrt_min = sqrt_inv_rc_tconst_1e3 as u64 * min_sqrt_term_1e3 as u64;
        let neg = |value: i64| value as u64;
        let tmp = neg(-50_000_000).wrapping_add(8 * scale_snr_1e3 * sqrt_min) / 100_000_000;
        data[7] = tmp.min(124) as u8;

        let tmp = ((invrc_tconst_1e6 >> 1)
            + (20u32
                .wrapping_mul(inv_scaled_adc_clk_1e3)
                .wrapping_mul(data[7] as u32)
                / 80) as u64
                * 1000)
            / invrc_tconst_1e6;
        data[8] = tmp.min(255) as u8;

        let tmp = neg(-500_000).wrapping_add(77 * sqrt_min) / 1_000_000;
        data[10] = tmp.min(127) as u8;

        data[9] = 127.min(800 * data[10] as u32 / 1000) as u8;
        let tmp = ((invrc_tconst_1e6 >> 1)
            + 20u32
                .wrapping_mul(inv_scaled_adc_clk_1e3)
                .wrapping_mul(data[10] as u32) as u64
                * 1000)
            / (invrc_tconst_1e6 * 77);
        data[11] = tmp.min(255) as u8;
        data[12] = 127.min(
            0u32.wrapping_sub(500_000)
                .wrapping_add(
                    80u32
                        .wrapping_mul(sqrt_inv_rc_tconst_1e3)
                        .wrapping_mul(min_sqrt_term_1e3),
                )
                / 1_000_000,
        ) as u8;

        let neg_half = (-3i32).wrapping_mul((invrc_tconst_1e6 >> 1) as u32 as i32) as i64 as u64;
        let tmp = neg_half.wrapping_add(
            inv_scaled_adc_clk_1e3.wrapping_mul(data[12] as u32) as u64 * (1000 * 20 / 80),
        ) / invrc_tconst_1e6;
        data[13] = tmp.min(255) as u8;

        data[14] = (21 * (inv_scaled_adc_clk_1e3 / 10_000)) as u8;
        data[15] = 127.min((500 + 1025 * data[7] as u32) / 1000) as u8;
        data[16] = 127.min(data[15] as u32 * tmp_1e3 / 1000) as u8;
        data[17] = data[15];
        data[18] = 127.min((500 + 975 * data[10] as u32) / 1000) as u8;
        data[19] = 127.min(data[18] as u32 * tmp_1e3 / 1000) as u8;
        data[20] = data[18];
        data[21] = 127.min((500 + 975 * data[12] as u32) / 1000) as u8;
        data[22] = 127.min(data[21] as u32 * tmp_1e3 / 1000) as u8;
        data[23] = data[21];
        data[24] = 0x2E;
        data[25] = (128 + 63_000.min(div_round_closest(63 * scaled_adc_clk_1e6, 1000)) / 1000) as u8;
        data[26] = 63.min(
            63 * scaled_adc_clk_1e6 / 1_000_000 * (920 + 80 * inv_scaled_adc_clk_1e3 / 1000) / 1000,
        ) as u8;
        data[27] = 63.min(32 * sqrt_term_1e3 / 1000) as u8;
        data[28] = data[25];
        data[29] = data[26];
        data[30] = data[27];
        data[31] = data[25];
        data[32] = data[26];
        data[33] = 63.min(63 * sqrt_term_1e3 / 1000) as u8;
        data[34] = 127.min(64 * sqrt_term_1e3 / 1000) as u8;
        data[35] = 0x40;
        data[36] = 0x40;
        data[37] = 0x2C;

        for (i, byte) in data.iter().enumerate() {
            self.write_bytes(&[*byte], arbitrary_int::u10::new(0x200 + i as u16))?;
        }
        Ok(())
    }

    /// `ad9361_bb_dc_offset_calib()`.
    pub async fn bb_dc_offset_calib(&mut self) -> Result<(), Ad9361Error<S::Error>> {
        self.write_reg(BbDcOffsetCount(0x3F))?;
        self.write_reg(BbDcOffsetShift::default().with_bb_dc_m_shift(u5::new(0xF)))?;
        self.write_reg(BbDcOffsetAtten::default().with_bb_dc_offset_atten(u4::new(1)))?;

        self.run_calibration(CalibrationControl::default().with_bbdc_cal(true).raw_value())
            .await
    }

    /// `ad9361_rf_dc_offset_calib()`, `rx_freq` in Hz.
    pub async fn rf_dc_offset_calib(&mut self, rx_freq: u64) -> Result<(), Ad9361Error<S::Error>> {
        let dc = self.cal.dc_offset;
        self.write_reg(WaitCount(0x20))?;

        let (count, dac_fs, atten) = if rx_freq <= 4_000_000_000 {
            (dc.count_low, 2, dc.attenuation_low)
        } else {
            (dc.count_high, 3, dc.attenuation_high)
        };
        self.write_reg(RfDcOffsetCount(count))?;
        self.write_reg(
            RfDcOffsetConfig1::default()
                .with_rf_dc_calibration_count(u4::new(4))
                .with_dac_fs(u2::new(dac_fs)),
        )?;
        self.write_reg(RfDcOffsetAtten::default().with_rf_dc_offset_atten(u5::new(atten & 0x1F)))?;

        self.write_reg(
            DcOffsetConfig2::default()
                .with_use_wait_counter_for_rf_dc_init_cal(true)
                .with_dc_offset_update(u3::new(3)),
        )?;

        let inv = InvertBits::default().with_invert_rx1_rf_dc_cgout_word(true);
        self.write_reg(if self.cal.rx_phase_inversion {
            inv
        } else {
            inv.with_invert_rx2_rf_dc_cgout_word(true)
        })?;

        self.run_calibration(CalibrationControl::default().with_rfdc_cal(true).raw_value())
            .await
    }

    /// `ad9361_tracking_control()`.
    pub fn tracking_control(&mut self, tracking: &TrackingConfig) -> Result<(), S::Error> {
        self.write_reg(
            CalibrationConfig2::default()
                .with_calibration_config2_dflt(u2::new(3))
                .with_k_exp_phase(u5::new(0x15)),
        )?;
        self.write_reg(
            CalibrationConfig3::default()
                .with_prevent_pos_loop_gain(true)
                .with_k_exp_amplitude(u5::new(0x15)),
        )?;
        self.write_reg(
            DcOffsetConfig2::default()
                .with_use_wait_counter_for_rf_dc_init_cal(true)
                .with_dc_offset_update(u3::new(self.cal.dc_offset.update_events & 0x7))
                .with_enable_bb_dc_offset_tracking(tracking.bbdc)
                .with_enable_rf_offset_tracking(tracking.rfdc),
        )?;
        self.modify_reg::<RxQuadGain2>(|reg| {
            reg.with_correction_word_decimation_m(u3::new(if tracking.qec_slow_mode { 4 } else { 0 }))
        })?;

        let (ch1, ch2) = match (tracking.rx_quad, self.mode.rx2tx2, self.mode.rx1tx1_use_rx) {
            (false, _, _) => (false, false),
            (true, true, _) => (true, true),
            (true, false, Channel::Ch1) => (true, false),
            (true, false, Channel::Ch2) => (false, true),
        };
        self.write_reg(
            CalibrationConfig1::default()
                .with_enable_phase_corr(true)
                .with_enable_gain_corr(true)
                .with_free_run_mode(true)
                .with_enable_corr_word_decimation(true)
                .with_enable_tracking_mode_ch1(ch1)
                .with_enable_tracking_mode_ch2(ch2),
        )
    }

    /// Redoes the baseband filter and ADC cals for new bandwidths (`__ad9361_update_rf_bandwidth()`).
    pub async fn update_rf_bandwidth_filters(
        &mut self,
        rf_rx_bw: HertzU32,
        rf_tx_bw: HertzU32,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let (rx, tx) = (rf_rx_bw / 2, rf_tx_bw / 2);
        let bbpll = self.clk.rates.bbpll;
        let rxbbf_div = self.rx_bb_analog_filter_calib(rx, bbpll).await?;
        self.tx_bb_analog_filter_calib(tx, bbpll).await?;
        self.rx_tia_calib(rx)?;
        self.tx_bb_second_filter_calib(tx)?;
        self.rx_adc_setup(bbpll, self.clk.rates.adc, rxbbf_div)
    }

    /// One TX quad cal run (`__ad9361_tx_quad_calib()`). Bit 1 = LO leakage converged, bit 0 =
    /// single sideband.
    async fn tx_quad_calib_once(
        &mut self,
        phase: u8,
        rxnco_word: u8,
        decim: u8,
    ) -> Result<u8, Ad9361Error<S::Error>> {
        self.write_reg(
            QuadCalNcoFreqPhaseOffset::default()
                .with_rx_nco_freq(u2::new(rxnco_word & 0x3))
                .with_rx_nco_phase_offset(u5::new(phase & 0x1F)),
        )?;
        let control = QuadCalControl::default()
            .with_settle_main_enable(true)
            .with_dc_offset_enable(true)
            .with_gain_enable(true)
            .with_phase_enable(true)
            .with_m_decim(u2::new(decim & 0x3));
        self.write_reg(control.with_quad_cal_soft_reset(true))?;
        self.write_reg(control)?;

        self.run_calibration(CalibrationControl::default().with_tx_quad_cal(true).raw_value())
            .await?;

        let status = if self.mode.rx1tx1_use_tx == Channel::Ch2 {
            self.read_reg::<QuadCalStatusTx2>()?.to_raw()
        } else {
            self.read_reg::<QuadCalStatusTx1>()?.to_raw()
        } & TX_QUAD_CONVERGED;
        Ok(if self.mode.rx2tx2 {
            status & self.read_reg::<QuadCalStatusTx2>()?.to_raw()
        } else {
            status
        } & TX_QUAD_CONVERGED)
    }

    /// Tries every phase offset when the cal doesn't converge (`ad9361_tx_quad_phase_search()`).
    async fn tx_quad_phase_search(
        &mut self,
        rxnco_word: u8,
        decim: u8,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let mut field = [0u8; 64];
        for i in 0..32 {
            let val = self.tx_quad_calib_once(i as u8, rxnco_word, decim).await?;
            // 360/0 wraps
            let failed = (val != TX_QUAD_CONVERGED) as u8;
            field[i] = failed;
            field[i + 32] = failed;
        }

        let (count, start) = find_opt(&field);
        let phase = ((start + count / 2) & 0x1F) as u32;
        self.cal.last_tx_quad_cal_phase = Some(phase);
        self.tx_quad_calib_once(phase as u8, rxnco_word, decim).await?;
        Ok(())
    }

    /// `ad9361_tx_quad_calib()`, bandwidths in Hz. Skips the LO power down handling from the C
    /// driver, we have no LO power down.
    pub async fn tx_quad_calib(
        &mut self,
        bw_rx: u32,
        bw_tx: u32,
        rx_phase: RxPhase,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let clkrf = self.clk.rates.clkrf.to_raw();
        let clktf = self.clk.rates.clktf.to_raw();
        if clktf == 0 {
            return Err(Ad9361Error::InvalidRate);
        }

        // want BW / 4 = Rx NCO freq = Tx NCO freq, with
        // Rx NCO = ClkRF * (rxNCO<1:0> + 1) / 32 and Tx NCO = ClkTF * (txNCO<1:0> + 1) / 32
        let mut txnco_word = (div_round_closest(bw_tx * 8, clktf) as i32 - 1).clamp(0, 3);
        let mut rxnco_word = txnco_word;
        let decim = if clktf <= 4_000_000 { 2 } else { 3 };

        let mut phase = 0u8;
        if clkrf == 2 * clktf {
            phase = 0x0E;
            match txnco_word {
                0 => txnco_word += 1,
                1 => rxnco_word -= 1,
                2 => {
                    rxnco_word -= 2;
                    txnco_word -= 1;
                }
                _ => {
                    rxnco_word -= 2; // REVISIT (as in no-OS)
                    phase = 0x08;
                }
            }
        } else if clkrf == clktf {
            phase = match txnco_word {
                0 | 3 => 0x15,
                2 => 0x1F,
                _ => {
                    let filter = self.read_reg::<TxEnableFilterControl>()?.to_raw() & 0x3F;
                    if filter == 0x22 { 0x15 } else { 0x1A } // REVISIT (as in no-OS)
                }
            };
        }
        // other ratios aren't handled in no-OS either, phase offset stays 0

        if let RxPhase::Fixed(fixed) = rx_phase {
            phase = fixed;
        }

        let txnco_freq = clktf * (txnco_word as u32 + 1) / 32;
        // bandwidth has to be wide enough during the cal
        let widen = txnco_freq > bw_rx / 4 || txnco_freq > bw_tx / 4;
        if widen {
            let bw = HertzU32::Hz(txnco_freq * 8);
            self.update_rf_bandwidth_filters(bw, bw).await?;
        }

        let inverted = self.cal.rx_phase_inversion;
        let mut saved_invert_bits = None;
        if inverted {
            self.modify_reg::<ParallelPortConf2>(|reg| reg.with_invert_rx2(false))?;
            saved_invert_bits = Some(self.read_reg::<InvertBits>()?);
            self.write_reg(
                InvertBits::default()
                    .with_invert_rx1_rf_dc_cgout_word(true)
                    .with_invert_rx2_rf_dc_cgout_word(true),
            )?;
        }

        self.modify_reg::<Kexp2>(|reg| reg.with_tx_nco_freq(u2::new(txnco_word as u8)))?;
        self.write_reg(QuadCalCount(0xFF))?;
        self.write_reg(
            Kexp1::default()
                .with_kexp_tx(u2::new(1))
                .with_kexp_tx_comp(u2::new(3))
                .with_kexp_dc_i(u2::new(3))
                .with_kexp_dc_q(u2::new(3)),
        )?;
        self.write_reg(MagFtestThresh(0x03))?;
        self.write_reg(MagFtestThresh2(0x03))?;

        // no match is just an error print in no-OS
        if let Some(matched) = self.gain.tx_quad_lpf_tia_match {
            self.write_reg(TxQuadFullLmtGain::from_raw(matched))?;
        }
        self.write_reg(QuadSettleCount(0xF0))?;
        self.write_reg(TxQuadLpfGain::default())?;

        let result = self
            .tx_quad_calibrate(phase, rxnco_word as u8, decim, rx_phase)
            .await;

        // restore even if the cal failed
        if inverted {
            self.modify_reg::<ParallelPortConf2>(|reg| reg.with_invert_rx2(true))?;
            if let Some(bits) = saved_invert_bits {
                self.write_reg(bits)?;
            }
        }
        if widen {
            let (rx, tx) = (self.cal.current_rx_bw, self.cal.current_tx_bw);
            self.update_rf_bandwidth_filters(rx, tx).await?;
        }

        result
    }

    /// The cal and phase search part of [`Self::tx_quad_calib`].
    async fn tx_quad_calibrate(
        &mut self,
        phase: u8,
        rxnco_word: u8,
        decim: u8,
        rx_phase: RxPhase,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let mut converged = 0;
        if rx_phase != RxPhase::ForceSearch {
            converged = self.tx_quad_calib_once(phase, rxnco_word, decim).await?;
            if converged != TX_QUAD_CONVERGED {
                // failed, try the last phase offset
                if let Some(last) = self.cal.last_tx_quad_cal_phase.filter(|phase| *phase < 31) {
                    converged = self.tx_quad_calib_once(last as u8, rxnco_word, decim).await?;
                }
            } else {
                self.cal.last_tx_quad_cal_phase = Some(phase as u32);
            }
        }
        // still failed, search all 32
        if converged != TX_QUAD_CONVERGED {
            self.tx_quad_phase_search(rxnco_word, decim).await?;
        }
        Ok(())
    }

    /// TX quad cal with tracking paused and the chip in alert (`ad9361_do_calib_run()` with
    /// `TX_QUAD_CAL`).
    pub async fn tx_quad_calib_run(&mut self) -> Result<(), Ad9361Error<S::Error>> {
        let tracking = self.cal.tracking;
        self.tracking_control(&TrackingConfig {
            bbdc: false,
            rfdc: false,
            rx_quad: false,
            ..tracking
        })?;
        let saved_ensm = self.ensm_force_state(ForcedEnsmState::Alert).await?;

        let (rx_bw, tx_bw) = (self.cal.current_rx_bw / 2, self.cal.current_tx_bw / 2);
        let result = self
            .tx_quad_calib(rx_bw.to_raw(), tx_bw.to_raw(), RxPhase::Auto)
            .await;

        self.tracking_control(&tracking)?;
        self.ensm_restore_state(saved_ensm)?;
        result
    }

    /// New RF bandwidths: redoes the baseband filter cals and the TX quad cal
    /// (`ad9361_update_rf_bandwidth()`).
    pub async fn update_rf_bandwidth(
        &mut self,
        rf_rx_bw: HertzU32,
        rf_tx_bw: HertzU32,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let tracking = self.cal.tracking;
        self.tracking_control(&TrackingConfig {
            bbdc: false,
            rfdc: false,
            rx_quad: false,
            ..tracking
        })?;
        let saved_ensm = self.ensm_force_state(ForcedEnsmState::Alert).await?;

        let result = async {
            self.update_rf_bandwidth_filters(rf_rx_bw, rf_tx_bw).await?;
            self.cal.current_rx_bw = rf_rx_bw;
            self.cal.current_tx_bw = rf_tx_bw;
            self.tx_quad_calib((rf_rx_bw / 2).to_raw(), (rf_tx_bw / 2).to_raw(), RxPhase::Auto)
                .await
        }
        .await;

        self.tracking_control(&tracking)?;
        self.ensm_restore_state(saved_ensm)?;
        result
    }
}
