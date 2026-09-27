//! RSSI setup, RSSI gain step cal, TX monitor (`ad9361_rssi_setup()`,
//! `ad9361_rssi_gain_step_calib()`, `ad9361_txmon_setup()`).

use embedded_hal_async::spi::SpiDevice;

use super::interface::DataInterface;
use arbitrary_int::{u2, u3, u4, u5, u6};
use embassy_time::Timer;

use super::{
    Ad9361Error, CalibrationControl, Config, Engine, ForcedEnsmState, GainDiffWorderrorWrite,
    GainErrorRead, LnaGain, MaxMixerCalibrationGainIndex, MeasureDuration, MeasureDuration01,
    MeasureDuration23, Register, RssiConfig, RssiDelay, RssiWaitTime, RssiWeight0, RssiWeight1,
    RssiWeight2, RssiWeight3, SettleTime, TpmModeEnable, TxAttenThresh, TxLevelThresh,
    TxMon1Config, TxMon2Config, TxMonDelay, TxMonHighGain, TxMonLowGain, WordAddress,
};

/// What restarts an RSSI measurement.
///
/// The gain change modes suit FDD, where the signal is continuous. The fast AGC lock, `EN_AGC`
/// pin and RX entry modes suit TDD, where a burst starts the measurement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RssiRestartMode {
    AgcInFastAttackModeLocksTheGain = 0,
    EnAgcPinIsPulledHigh = 1,
    EntersRxMode = 2,
    GainChangeOccurs = 3,
    SpiWriteToRegister = 4,
    GainChangeOccursOrEnAgcPinPulledHigh = 5,
}

/// RSSI measurement timing. The reading is relative to full scale and corrected for the RX
/// gain, so turning it into dBm needs a calibration with a known signal at the antenna port.
///
/// After a restart the chip waits `delay`, then alternates between measuring for `duration` and
/// waiting for `wait`. `wait` mostly matters in FDD, to line measurements up with slot
/// boundaries. The defaults measure 1000 us after every gain change.
#[derive(Clone, Copy, Debug)]
pub struct RssiConfiguration {
    pub restart_mode: RssiRestartMode,
    /// delay, wait and duration in RX samples, not microseconds
    pub unit_is_rx_samples: bool,
    pub delay: u32,
    pub wait: u32,
    pub duration: u32,
}

impl Default for RssiConfiguration {
    fn default() -> Self {
        Self {
            restart_mode: RssiRestartMode::GainChangeOccurs,
            unit_is_rx_samples: false,
            delay: 1,
            wait: 1,
            duration: 1000,
        }
    }
}

/// TX monitor (TX power detector) settings. Only usable in TDD, since it borrows the idle RX
/// path to measure the TX signal through the `TX_MON` pins. The inputs take at most +4 dBm.
///
/// The detector has a low and a high gain setting. `low_high_gain_threshold_mdb` is where it
/// switches between them. `track_en` keeps it running, and `one_shot_mode_en` does a single
/// measurement each TX burst.
#[derive(Clone, Copy, Debug)]
pub struct TxMonitorConfig {
    pub track_en: bool,
    pub one_shot_mode_en: bool,
    pub low_high_gain_threshold_mdb: u32,
    pub low_gain_db: u8,
    pub high_gain_db: u8,
    pub delay: u16,
    pub duration: u16,
    pub tx1_front_end_gain: u8,
    pub tx2_front_end_gain: u8,
    pub tx1_lo_cm: u8,
    pub tx2_lo_cm: u8,
}

impl Default for TxMonitorConfig {
    fn default() -> Self {
        Self {
            track_en: false,
            one_shot_mode_en: false,
            low_high_gain_threshold_mdb: 37_000,
            low_gain_db: 0,
            high_gain_db: 24,
            delay: 511,
            duration: 8192,
            tx1_front_end_gain: 2,
            tx2_front_end_gain: 2,
            tx1_lo_cm: 48,
            tx2_lo_cm: 48,
        }
    }
}

/// RSSI gain step cal values (LNA gain and gain step words) per LO range:
/// 600-1300, 1300-3300, 2700-4100 and 4000-6000 MHz.
const GAIN_STEP_CALIB_REG_VAL: [[u8; 5]; 4] = [
    [0xC0, 0x2E, 0x10, 0x06, 0x00],
    [0xC0, 0x2C, 0x10, 0x06, 0x00],
    [0xB8, 0x2C, 0x10, 0x06, 0x00],
    [0xA0, 0x24, 0x10, 0x06, 0x00],
];

const RSSI_MAX_WEIGHT: u32 = 255;

impl<S, I> Engine<S, I>
where
    S: SpiDevice<u8>,
    I: DataInterface,
{
    /// `ad9361_rssi_setup()`. `is_update` only refreshes what depends on the RX sample rate.
    pub(crate) async fn rssi_setup(
        &mut self,
        ctrl: &RssiConfiguration,
        is_update: bool,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let (mut delay, wait, mut duration) = if ctrl.unit_is_rx_samples {
            if is_update {
                return Ok(()); // No update required
            }
            (ctrl.delay, ctrl.wait, ctrl.duration)
        } else {
            let rate_khz = (self.clk.rates.rx_sample.to_raw() + 500) / 1000;
            let convert = |us: u32| (us * rate_khz + 500) / 1000;
            (
                convert(ctrl.delay),
                convert(ctrl.wait),
                convert(ctrl.duration),
            )
        };
        if ctrl.restart_mode == RssiRestartMode::EnAgcPinIsPulledHigh {
            delay = 0;
        }
        let delay = (delay / 8).min(255) as u8;
        let wait = (wait / 4).min(255) as u8;

        // duration becomes up to four power-of-two chunks
        let mut exponents = [0u8; 4];
        let mut count = 0;
        let mut total_duration = 0u32;
        while count < 4 && duration > 0 {
            let Some(exp) = (0..=14u8).rev().find(|e| duration >= 1 << e) else {
                break;
            };
            exponents[count] = exp;
            count += 1;
            total_duration += 1 << exp;
            duration -= 1 << exp;
        }
        if count == 0 {
            return Err(Ad9361Error::RssiDurationTooShort);
        }

        let mut weights = [0u32; 4];
        for (weight, exp) in weights.iter_mut().zip(&exponents[..count]) {
            *weight = (RSSI_MAX_WEIGHT * (1 << exp) + total_duration / 2) / total_duration;
        }
        // weights have to add up to 0xFF
        let total: u32 = weights.iter().sum();
        weights[count - 1] = weights[count - 1].wrapping_sub(total.wrapping_sub(0xFF));

        self.write_reg(
            MeasureDuration01::default()
                .with_measurement_duration_0(u4::new(exponents[0]))
                .with_measurement_duration_1(u4::new(exponents[1])),
        )
        .await?;
        self.write_reg(
            MeasureDuration23::default()
                .with_measurement_duration_2(u4::new(exponents[2]))
                .with_measurement_duration_3(u4::new(exponents[3])),
        )
        .await?;
        self.write_reg(RssiWeight0(weights[0] as u8)).await?;
        self.write_reg(RssiWeight1(weights[1] as u8)).await?;
        self.write_reg(RssiWeight2(weights[2] as u8)).await?;
        self.write_reg(RssiWeight3(weights[3] as u8)).await?;
        self.write_reg(RssiDelay(delay)).await?;
        self.write_reg(RssiWaitTime(wait)).await?;

        self.write_reg(
            RssiConfig::default()
                .with_rssi_mode_select(u3::new(ctrl.restart_mode as u8))
                .with_start_rssi_meas(ctrl.restart_mode == RssiRestartMode::SpiWriteToRegister)
                .with_default_rssi_meas_mode(duration == 0 && count == 1),
        )
        .await?;
        Ok(())
    }

    /// `ad9361_rssi_gain_step_calib()` for the current RX LO. The chip sits in alert while it
    /// runs. No-OS' factory table path (`rssi_skip_calib`) isn't supported.
    pub(crate) async fn rssi_gain_step_calib(&mut self) -> Result<(), Ad9361Error<S::Error>> {
        let lo_freq = self.clk.rates.rx_rfpll.to_raw();
        let table = &GAIN_STEP_CALIB_REG_VAL[match lo_freq {
            0..1_300_000_000 => 0,
            1_300_000_000..3_300_000_000 => 1,
            3_300_000_000..4_100_000_000 => 2,
            _ => 3,
        }];

        let saved_ensm = self.ensm_force_state(ForcedEnsmState::Alert).await?;

        let select = |config: Config| config.with_calib_table_select(u2::new(3));
        let table_clock = Config::default().with_start_calib_table_clock(true);

        self.write_reg(
            MaxMixerCalibrationGainIndex::default()
                .with_max_mixer_calibration_gain_index(u5::new(0x0F)),
        )
        .await?;
        self.write_reg(MeasureDuration::default().with_gain_cal_meas_duration(u4::new(0x0E)))
            .await?;
        self.write_reg(SettleTime::default().with_settle_time(u6::new(0x3F)))
            .await?;
        self.write_reg(
            RssiConfig::default()
                .with_rssi_mode_select(u3::new(3))
                .with_default_rssi_meas_mode(true),
        )
        .await?;
        self.write_reg(MeasureDuration01::default().with_measurement_duration_0(u4::new(0x0E)))
            .await?;
        self.write_reg(LnaGain::from_raw(table[0])).await?;

        self.write_reg(select(table_clock)).await?;
        for i in 0..4 {
            self.write_reg(WordAddress(i)).await?;
            self.write_reg(GainDiffWorderrorWrite::from_raw(table[i as usize + 1]))
                .await?;
            self.write_reg(select(table_clock).with_write_lna_gain_diff(true))
                .await?;
            Timer::after_micros(3).await; // Wait for the data to fully write to the table
        }
        self.write_reg(table_clock).await?;
        self.write_reg(Config::default()).await?;

        let result = self
            .run_calibration(
                CalibrationControl::default()
                    .with_rx_gain_step_cal(true)
                    .raw_value(),
            )
            .await;

        let mut lna_error = [0u8; 4];
        let mut mixer_error = [0u8; 15];
        self.write_reg(
            Config::default()
                .with_calib_table_select(u2::new(1))
                .with_read_select(true),
        )
        .await?;
        for (i, error) in lna_error.iter_mut().enumerate() {
            self.write_reg(WordAddress(i as u8)).await?;
            *error = self.read_reg::<GainErrorRead>().await?.to_raw();
        }
        self.write_reg(Config::default().with_calib_table_select(u2::new(1)))
            .await?;
        for (i, error) in mixer_error.iter_mut().enumerate() {
            self.write_reg(WordAddress(i as u8)).await?;
            *error = self.read_reg::<GainErrorRead>().await?.to_raw();
        }
        self.write_reg(Config::default()).await?;

        self.write_reg(select(table_clock)).await?;
        for (i, error) in lna_error.iter().enumerate() {
            self.write_reg(WordAddress(i as u8)).await?;
            self.write_reg(GainDiffWorderrorWrite::from_raw(*error))
                .await?;
            self.write_reg(select(table_clock).with_write_lna_error_table(true))
                .await?;
        }
        self.write_reg(select(table_clock)).await?;
        for (i, error) in mixer_error.iter().enumerate() {
            self.write_reg(WordAddress(i as u8)).await?;
            self.write_reg(GainDiffWorderrorWrite::from_raw(*error))
                .await?;
            self.write_reg(select(table_clock).with_write_mixer_error_table(true))
                .await?;
        }
        self.write_reg(Config::default()).await?;

        self.ensm_restore_state(saved_ensm).await?;
        result
    }

    /// `ad9361_txmon_setup()`.
    pub(crate) async fn txmon_setup(&mut self, ctrl: &TxMonitorConfig) -> Result<(), S::Error> {
        let duration = (ctrl.duration as u32 / 16).checked_ilog2().unwrap_or(0) as u8;
        self.write_reg(
            TpmModeEnable::default()
                .with_one_shot_mode(ctrl.one_shot_mode_en)
                .with_tx_mon_duration(u4::new(duration & 0xF)),
        )
        .await?;

        self.write_reg(TxMonDelay(ctrl.delay as u8)).await?;
        self.modify_reg::<TxLevelThresh>(|reg| {
            reg.with_tx_mon_delay_counter(u2::new((ctrl.delay >> 8) as u8 & 0x3))
        })
        .await?;

        self.write_reg(
            TxMon1Config::default()
                .with_tx_mon_1_lo_cm(u6::new(ctrl.tx1_lo_cm & 0x3F))
                .with_tx_mon_1_gain(u2::new(ctrl.tx1_front_end_gain & 0x3)),
        )
        .await?;
        self.write_reg(
            TxMon2Config::default()
                .with_tx_mon_2_lo_cm(u6::new(ctrl.tx2_lo_cm & 0x3F))
                .with_tx_mon_2_gain(u2::new(ctrl.tx2_front_end_gain & 0x3)),
        )
        .await?;

        self.write_reg(TxAttenThresh(
            (ctrl.low_high_gain_threshold_mdb / 250) as u8,
        ))
        .await?;
        self.write_reg(
            TxMonHighGain::default().with_tx_mon_high_gain(u5::new(ctrl.high_gain_db & 0x1F)),
        )
        .await?;
        self.write_reg(
            TxMonLowGain::default()
                .with_tx_mon_track(ctrl.track_en)
                .with_tx_mon_low_gain(u5::new(ctrl.low_gain_db & 0x1F)),
        )
        .await?;
        Ok(())
    }
}
