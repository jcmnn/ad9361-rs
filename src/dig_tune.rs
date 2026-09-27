//! Digital interface tuning (`ad9361_dig_tune()`, `ad9361_post_setup()`), using the PN checkers
//! of the [`DataInterface`].
//!
//! The IDELAY/ODELAY tuning (`DO_IDELAY`/`DO_ODELAY`) isn't here, no-OS never turns it on in its
//! own setup.

use arbitrary_int::u2;
use embassy_time::Timer;
use embedded_hal_async::spi::SpiDevice;

use fugit::HertzU32;

use super::interface::DataInterface;
use super::{
    Ad9361Error, BistConfig, Channel, DigInterfaceTune, Engine, ForcedEnsmState, ObserveConfig,
    ParallelPortConf3, Register, RxClockDataDelay, TxAttenuation, TxClockDataDelay, find_opt,
};

/// Where the data loops back during tuning.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BistLoopback {
    Off,
    /// inside the AD9361, TX to RX
    TxToRx,
}

/// Where BIST injects the PRBS.
#[allow(dead_code, reason = "full set from no-OS, not all used yet")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BistMode {
    Disable,
    InjectTx,
    InjectRx,
}

/// Options for `dig_tune`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct DigTuneFlags {
    /// don't tune, put the configured delays back
    pub restore_default: bool,
    /// don't keep the result as the new default
    pub skip_store_result: bool,
}

/// PN checker sweep result, `true` = that delay setting fails.
type Field = [bool; 16];

impl<S, I> Engine<S, I>
where
    S: SpiDevice<u8>,
    I: DataInterface,
{
    /// RX at twice the TX rate in FDD
    pub(super) fn rx_eq_2tx(&self) -> bool {
        self.mode.pp_conf3.fdd_rx_rate_2tx_rate()
    }

    /// `ad9361_bist_prbs()`.
    pub(crate) async fn bist_prbs(&mut self, mode: BistMode) -> Result<(), S::Error> {
        let config = match mode {
            BistMode::Disable => BistConfig::default(),
            BistMode::InjectTx => BistConfig::default()
                .with_bist_ctrl_point(u2::new(0))
                .with_bist_enable(true),
            BistMode::InjectRx => BistConfig::default()
                .with_bist_ctrl_point(u2::new(2))
                .with_bist_enable(true),
        };
        self.tune.bist_config = config;
        self.write_reg(config).await
    }

    /// Loopback only works TX1 -> RX1 or TX2 -> RX2. In 1R1T with different RX and TX channels,
    /// this moves TX to the RX channel while looping back, and back again after.
    async fn int_loopback_fix_ch_cross(&mut self, enable: bool) -> Result<(), S::Error> {
        if !self.mode.rx2tx2 && self.mode.rx1tx1_use_rx != self.mode.rx1tx1_use_tx {
            let ch = if enable {
                self.mode.rx1tx1_use_rx
            } else {
                self.mode.rx1tx1_use_tx
            };
            self.set_tx_channels(ch == Channel::Ch1, ch == Channel::Ch2)
                .await?;
        }
        Ok(())
    }

    /// `ad9361_bist_loopback()`.
    pub(crate) async fn bist_loopback(&mut self, mode: BistLoopback) -> Result<(), S::Error> {
        let observe = self.read_reg::<ObserveConfig>().await?;
        self.tune.bist_loopback_mode = mode;
        match mode {
            BistLoopback::Off => {
                self.int_loopback_fix_ch_cross(false).await?;
                self.write_reg(
                    observe
                        .with_data_port_sp_hd_loop_test_oe(false)
                        .with_data_port_loop_test_enable(false),
                )
                .await?;
            }
            BistLoopback::TxToRx => {
                self.int_loopback_fix_ch_cross(true).await?;
                let conf3 = self.read_reg::<ParallelPortConf3>().await?;
                self.write_reg(
                    observe
                        .with_data_port_sp_hd_loop_test_oe(
                            conf3.single_port_mode() && conf3.half_duplex_mode(),
                        )
                        .with_data_port_loop_test_enable(true),
                )
                .await?;
            }
        }
        Ok(())
    }

    /// `ad9361_get_tx_atten()`, TX2 or else TX1.
    pub(crate) async fn tx_atten(&mut self, tx2: bool) -> Result<TxAttenuation, S::Error> {
        let mut buf = [0u8; 2];
        let addr = if tx2 {
            super::Tx2Atten1::ADDRESS
        } else {
            super::Tx1Atten1::ADDRESS
        };
        self.read_bytes(&mut buf, addr).await?;
        Ok(TxAttenuation::saturating_from_quarter_db(
            u16::from_be_bytes(buf),
        ))
    }

    /// Mute by maxing the attenuation, or restore it (`ad9361_tx_mute()`).
    pub(crate) async fn tx_mute(&mut self, mute: bool) -> Result<(), S::Error> {
        if mute {
            self.tx1_atten_cached = self.tx_atten(false).await?;
            self.tx2_atten_cached = self.tx_atten(true).await?;
            return self
                .set_tx_atten(TxAttenuation::MAX, true, true, true)
                .await;
        }
        let (tx1, tx2) = (self.tx1_atten_cached, self.tx2_atten_cached);
        if tx1 == tx2 {
            return self.set_tx_atten(tx1, true, true, true).await;
        }
        self.set_tx_atten(tx1, true, false, true).await?;
        self.set_tx_atten(tx2, false, true, true).await
    }

    /// Waits `delay_ms`, then checks the PN checkers. `true` = error.
    async fn check_pn(&mut self, tx: bool, delay_ms: u64) -> bool {
        let layout = self.port_layout();
        self.interface.clear_pn_errors(layout);
        Timer::after_millis(delay_ms).await;
        self.interface.pn_errors(layout, !tx)
    }

    /// `ad9361_set_intf_delay()`.
    async fn set_intf_delay(
        &mut self,
        tx: bool,
        clock_delay: u8,
        data_delay: u8,
        clock_changed: bool,
    ) -> Result<(), Ad9361Error<S::Error>> {
        if clock_changed {
            let _ = self.ensm_force_state(ForcedEnsmState::Alert).await?;
        }
        // same layout for RX and TX
        let raw = ((clock_delay & 0xF) << 4) | (data_delay & 0xF);
        if tx {
            self.write_reg(TxClockDataDelay::from_raw(raw)).await?;
        } else {
            self.write_reg(RxClockDataDelay::from_raw(raw)).await?;
        }
        if clock_changed {
            let _ = self.ensm_force_state(ForcedEnsmState::Fdd).await?;
        }
        Ok(())
    }

    /// Sweeps clock and data delays, picks the middle of the widest window that passes
    /// (`ad9361_dig_tune_delay()`).
    async fn dig_tune_delay(
        &mut self,
        max_freq: HertzU32,
        tx: bool,
    ) -> Result<(), Ad9361Error<S::Error>> {
        const RATES: [u32; 3] = [25_000_000, 40_000_000, 61_440_000];
        let half_data_rate = !(self.mode.lvds_mode || !self.mode.rx2tx2);

        let mut field = [Field::default(); 2];
        let sweeps = if max_freq.to_raw() != 0 {
            RATES.len()
        } else {
            1
        };
        for rate in RATES.iter().take(sweeps) {
            if max_freq.to_raw() != 0 {
                let rate = if half_data_rate { rate / 2 } else { *rate };
                // no chain for that rate? carry on with the current clocks, no-OS does
                let _ = self
                    .set_trx_clock_chain_freq_no_tune(HertzU32::Hz(rate))
                    .await;
            }

            for (i, results) in field.iter_mut().enumerate() {
                for j in 0..16u8 {
                    // i == 0: clock delay 0, data delay 0..15
                    // i == 1: clock delay 15, data delay 15..0
                    let (clock, data) = if i == 1 { (15, 15 - j) } else { (0, j) };
                    self.set_intf_delay(tx, clock, data, j == 0).await?;
                    results[j as usize] |= self.check_pn(tx, 4).await;
                }
            }
        }

        let to_bytes = |f: &Field| f.map(|failed| failed as u8);
        let (c0, s0) = find_opt(&to_bytes(&field[0]));
        let (c1, s1) = find_opt(&to_bytes(&field[1]));
        if c0 == 0 && c1 == 0 {
            return Err(Ad9361Error::TuningFailed);
        }
        if c1 > c0 {
            self.set_intf_delay(tx, (s1 + c1 / 2) as u8, 0, true).await
        } else {
            self.set_intf_delay(tx, 0, (s0 + c0 / 2) as u8, true).await
        }
    }

    /// `ad9361_dig_tune_rx()`.
    async fn dig_tune_rx(&mut self, max_freq: HertzU32) -> Result<(), Ad9361Error<S::Error>> {
        self.bist_loopback(BistLoopback::Off).await?;
        self.bist_prbs(BistMode::InjectRx).await?;

        let result = self.dig_tune_delay(max_freq, false).await;
        self.interface.relatch_rx();
        result
    }

    /// `ad9361_dig_tune_tx()`. DAC sends a PN sequence, the AD9361 loops it back to the ADC PN
    /// checker.
    async fn dig_tune_tx(&mut self, max_freq: HertzU32) -> Result<(), Ad9361Error<S::Error>> {
        self.bist_prbs(BistMode::Disable).await?;
        self.bist_loopback(BistLoopback::TxToRx).await?;
        let layout = self.port_layout();
        self.interface.start_tx_pn(layout);
        let result = self.dig_tune_delay(max_freq, true).await;
        self.interface.stop_tx_pn(layout);
        result
    }

    /// `ad9361_dig_tune()`. `max_freq` 0 tunes at the current rate, anything else sweeps 25 MSPS
    /// up to 61.44 MSPS.
    ///
    pub(crate) async fn dig_tune(
        &mut self,
        max_freq: HertzU32,
        flags: DigTuneFlags,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let ensm_state = self.save_ensm_state().await?;

        let mut restore = self.tune.dig_interface_tune == DigInterfaceTune::UseConfigured
            || flags.restore_default;
        let mut result = Ok(());
        if !restore {
            let loopback = self.tune.bist_loopback_mode;
            let bist = self.tune.bist_config;

            // mute, the PRBS must not go out on the antenna
            self.tx_mute(true).await?;
            if !self.mode.fdd {
                self.set_ensm_mode(true, false).await?;
            }

            result = self.dig_tune_rx(max_freq).await;
            if result.is_ok() && self.tune.dig_interface_tune == DigInterfaceTune::RxAndTx {
                result = self.dig_tune_tx(max_freq).await;
            }

            self.bist_loopback(loopback).await?;
            self.write_reg(bist).await?;

            if matches!(result, Err(Ad9361Error::TuningFailed)) {
                restore = true;
            }
            if max_freq.to_raw() == 0 {
                result = Ok(());
            }
        }

        if restore {
            let _ = self.ensm_force_state(ForcedEnsmState::Alert).await?;
            self.write_reg(self.tune.rx_clk_data_delay).await?;
            self.write_reg(self.tune.tx_clk_data_delay).await?;
        } else if !flags.skip_store_result {
            self.tune.rx_clk_data_delay = self.read_reg::<RxClockDataDelay>().await?;
            self.tune.tx_clk_data_delay = self.read_reg::<TxClockDataDelay>().await?;
        }

        if !self.mode.fdd {
            self.set_ensm_mode(self.mode.fdd, self.ensm.pin_ctrl)
                .await?;
        }
        self.ensm_restore_state(ensm_state).await?;

        self.interface.relatch_rx();
        self.tx_mute(false).await?;
        result
    }

    /// The tuning part of `ad9361_post_setup()`, after the interface is up.
    pub(crate) async fn post_setup(&mut self) -> Result<(), Ad9361Error<S::Error>> {
        let flags = DigTuneFlags::default();
        // with bb_clk_change_dig_tune_en this is just a baseline, the real retune happens on
        // every baseband clock change
        let mode = if self.tune.bb_clk_change_dig_tune_en {
            DigInterfaceTune::UseConfigured
        } else {
            self.tune.dig_interface_tune
        };
        self.with_dig_tune_mode(mode, async |this| {
            this.dig_tune(HertzU32::Hz(61_440_000), flags).await?;
            let (rx, tx) = (this.clk.rx_path_clks, this.clk.tx_path_clks);
            this.set_trx_clock_chain(&rx, &tx).await?;
            let saved_ensm = this.ensm_force_state(ForcedEnsmState::Alert).await?;
            this.ensm_restore_state(saved_ensm).await?;
            Ok(())
        })
        .await
    }

    /// Runs `f` with tuning mode `mode` (nested tuning included), puts the configured mode back
    /// afterwards no matter what `f` returns.
    async fn with_dig_tune_mode<T>(
        &mut self,
        mode: DigInterfaceTune,
        f: impl AsyncFnOnce(&mut Self) -> T,
    ) -> T {
        let configured = core::mem::replace(&mut self.tune.dig_interface_tune, mode);
        let result = f(self).await;
        self.tune.dig_interface_tune = configured;
        result
    }
}
