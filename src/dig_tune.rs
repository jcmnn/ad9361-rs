//! Digital interface tuning (`ad9361_dig_tune()`, `ad9361_post_setup()`), using the ADC PN
//! checker and DAC data source from the `axi-ad9361` crate.
//!
//! Needs an HDL core with the per-channel DAC data source mux (version 8+). The IDELAY/ODELAY
//! tuning (`DO_IDELAY`/`DO_ODELAY`) isn't here, no-OS never turns it on in its own setup.

use arbitrary_int::{u2, u4};
use axi_ad9361::{
    adc::Adc,
    dac::Dac,
    regs::{
        adc::fields::{ChannelDataPathControl, ChannelStatus, PnSel},
        dac::regs::{ChannelDataSource, ChannelLegacyControl, DataSource},
    },
};
use embassy_time::Timer;
use embedded_hal::spi::SpiDevice;
use fugit::HertzU32;

use super::{
ForcedEnsmState, DigInterfaceTune, Engine, Ad9361Error, TxAttenuation, BistConfig, Channel, ObserveConfig, ParallelPortConf3, Register,
    RxClockDataDelay, TxClockDataDelay, find_opt,
};

/// I and Q for each of the two RX channels.
const MAX_CHANNELS: usize = 4;

/// The ADC and DAC halves of the AXI AD9361 core, as handed out by `axi-ad9361`.
///
/// Take them with `take_adc_no_init()` and `take_dac_no_init()`. The core has no clock until the
/// AD9361 runs, so [`Configured::init`](crate::Configured::init) brings them up.
pub struct AxiCores {
    pub adc: Adc,
    pub dac: Dac,
    /// Instance number of the core. Only instance 0 gets its interface tuned (same in no-OS), the
    /// others use the configured delays. Use 0 with a single chip.
    pub core_id: u32,
}

/// Where the data loops back during tuning.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BistLoopback {
    Off,
    /// inside the AD9361, TX to RX
    TxToRx,
    /// inside the FPGA, RX to TX
    RxToTx,
}

/// Where BIST injects the PRBS.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BistMode {
    Disable,
    InjectTx,
    InjectRx,
}

/// Options for `dig_tune`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DigTuneFlags {
    /// don't tune, put the configured delays back
    pub restore_default: bool,
    /// don't keep the result as the new default
    pub skip_store_result: bool,
}

/// PN checker sweep result, `true` = that delay setting fails.
type Field = [bool; 16];

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    /// 4 for 2R2T, 2 for 1R1T.
    fn num_phy_chan(&self) -> u8 {
        if self.mode.rx2tx2 { 4 } else { 2 }
    }

    /// RX at twice the TX rate in FDD
    pub(super) fn rx_eq_2tx(&self) -> bool {
        self.mode.pp_conf3.fdd_rx_rate_2tx_rate()
    }

    /// `ad9361_bist_prbs()`.
    pub fn bist_prbs(&mut self, mode: BistMode) -> Result<(), S::Error> {
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
        self.write_reg(config)
    }

    /// Loopback only works TX1 -> RX1 or RX2 -> RX2.
    fn int_loopback_fix_ch_cross(&mut self, enable: bool) -> Result<(), S::Error> {
        if !self.mode.rx2tx2 && self.mode.rx1tx1_use_rx != self.mode.rx1tx1_use_tx {
            let ch = if enable { self.mode.rx1tx1_use_rx } else { self.mode.rx1tx1_use_tx };
            self.set_tx_channels(ch == Channel::Ch1, ch == Channel::Ch2)?;
        }
        Ok(())
    }

    /// Loops ADC data back into the DAC in the FPGA, or puts the old DAC sources back
    /// (`ad9361_hdl_loopback()`).
    fn hdl_loopback(&mut self, enable: bool) {
        let num_channels = self.num_phy_chan();
        let axi = &mut self.axi;
        for chan in 0..num_channels {
            let mut channel = axi.dac.channel_mut(u4::new(chan));
            let current = channel.read_data_source();
            let loopback = ChannelDataSource::builder()
                .with_data_source(DataSource::LoopbackDataAdc)
                .build();
            if enable && current.raw_value() != loopback.raw_value() {
                self.tune.scratch_dac_source[chan as usize] = current;
                channel.write_data_source(loopback);
            } else if current.raw_value() == loopback.raw_value() {
                channel.write_data_source(self.tune.scratch_dac_source[chan as usize]);
            }
        }
    }

    /// `ad9361_bist_loopback()`.
    pub fn bist_loopback(&mut self, mode: BistLoopback) -> Result<(), S::Error> {
        let observe = self.read_reg::<ObserveConfig>()?;
        self.tune.bist_loopback_mode = mode;
        match mode {
            BistLoopback::Off => {
                self.hdl_loopback(false);
                self.int_loopback_fix_ch_cross(false)?;
                self.write_reg(
                    observe
                        .with_data_port_sp_hd_loop_test_oe(false)
                        .with_data_port_loop_test_enable(false),
                )?;
            }
            BistLoopback::TxToRx => {
                self.hdl_loopback(false);
                self.int_loopback_fix_ch_cross(true)?;
                let conf3 = self.read_reg::<ParallelPortConf3>()?;
                self.write_reg(
                    observe
                        .with_data_port_sp_hd_loop_test_oe(
                            conf3.single_port_mode() && conf3.half_duplex_mode(),
                        )
                        .with_data_port_loop_test_enable(true),
                )?;
            }
            BistLoopback::RxToTx => {
                self.hdl_loopback(true);
                self.int_loopback_fix_ch_cross(false)?;
                self.write_reg(
                    observe
                        .with_data_port_sp_hd_loop_test_oe(false)
                        .with_data_port_loop_test_enable(false),
                )?;
            }
        }
        Ok(())
    }

    /// `ad9361_get_tx_atten()`, TX2 or else TX1.
    pub fn tx_atten(&mut self, tx2: bool) -> Result<TxAttenuation, S::Error> {
        let mut buf = [0u8; 2];
        let addr = if tx2 { super::Tx2Atten1::ADDRESS } else { super::Tx1Atten1::ADDRESS };
        self.read_bytes(&mut buf, addr)?;
        Ok(TxAttenuation::saturating_from_quarter_db(u16::from_be_bytes(buf)))
    }

    /// Mute by maxing the attenuation, or restore it (`ad9361_tx_mute()`).
    pub fn tx_mute(&mut self, mute: bool) -> Result<(), S::Error> {
        if mute {
            self.tx1_atten_cached = self.tx_atten(false)?;
            self.tx2_atten_cached = self.tx_atten(true)?;
            return self.set_tx_atten(TxAttenuation::MAX, true, true, true);
        }
        let (tx1, tx2) = (self.tx1_atten_cached, self.tx2_atten_cached);
        if tx1 == tx2 {
            return self.set_tx_atten(tx1, true, true, true);
        }
        self.set_tx_atten(tx1, true, false, true)?;
        self.set_tx_atten(tx2, false, true, true)
    }

    /// Waits `delay_ms`, then checks the PN checkers. `true` = error.
    async fn check_pn(&mut self, tx: bool, delay_ms: u64) -> bool {
        let num_channels = self.num_phy_chan();
        let axi = &mut self.axi;
        let clear = ChannelStatus::default()
            .with_pn_error(true)
            .with_pn_out_of_sync(true);
        for chan in 0..num_channels {
            axi.adc.channel_mut(u4::new(chan)).write_status(clear);
        }
        Timer::after_millis(delay_ms).await;

        if !tx && !axi.adc.read_status().locked() {
            return true;
        }
        (0..num_channels).any(|chan| {
            axi.adc.channel_mut(u4::new(chan)).read_status().raw_value() != 0
        })
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
            self.write_reg(TxClockDataDelay::from_raw(raw))?;
        } else {
            self.write_reg(RxClockDataDelay::from_raw(raw))?;
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
        let sweeps = if max_freq.to_raw() != 0 { RATES.len() } else { 1 };
        for rate in RATES.iter().take(sweeps) {
            if max_freq.to_raw() != 0 {
                let rate = if half_data_rate { rate / 2 } else { *rate };
                // no chain for that rate? carry on with the current clocks, no-OS does
                let _ = self.set_trx_clock_chain_freq_no_tune(HertzU32::Hz(rate)).await;
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
        self.bist_loopback(BistLoopback::Off)?;
        self.bist_prbs(BistMode::InjectRx)?;

        let result = self.dig_tune_delay(max_freq, false).await;
        let axi = &mut self.axi;
        axi.adc.reset_pulse();
        result
    }

    /// `ad9361_dig_tune_tx()`. DAC sends a PN sequence, the AD9361 loops it back to the ADC PN
    /// checker.
    async fn dig_tune_tx(&mut self, max_freq: HertzU32) -> Result<(), Ad9361Error<S::Error>> {
        self.bist_prbs(BistMode::Disable)?;
        self.bist_loopback(BistLoopback::TxToRx)?;
        let num_channels = self.num_phy_chan();
        let axi = &mut self.axi;
        axi.dac.release_reset();

        let mut saved_adc = [ChannelDataPathControl::ZERO; MAX_CHANNELS];
        let mut saved_dac_source = [ChannelDataSource::default(); MAX_CHANNELS];
        let mut saved_dac_legacy = [ChannelLegacyControl::default(); MAX_CHANNELS];
        for chan in 0..num_channels {
            let index = chan as usize;
            let mut adc = axi.adc.channel_mut(u4::new(chan));
            saved_adc[index] = adc.read_data_path_control();
            adc.write_data_path_control(
                ChannelDataPathControl::ZERO
                    .with_format_signext(true)
                    .with_format_enable(true)
                    .with_enable(true)
                    .with_iqcor_enable(true),
            );
            adc.modify_pn_select(|reg| reg.with_pn_sel(PnSel::PnCustom));

            let mut dac = axi.dac.channel_mut(u4::new(chan));
            saved_dac_legacy[index] = dac.read_legacy_control();
            saved_dac_source[index] = dac.read_data_source();
            dac.write_data_source(
                ChannelDataSource::builder()
                    .with_data_source(DataSource::PnX)
                    .build(),
            );
            dac.write_legacy_control(ChannelLegacyControl::default()); // !IQCOR_ENB
            axi.dac.synchronize();
        }

        let result = self.dig_tune_delay(max_freq, true).await;

        let axi = &mut self.axi;
        for chan in 0..num_channels {
            let index = chan as usize;
            let mut adc = axi.adc.channel_mut(u4::new(chan));
            adc.write_data_path_control(saved_adc[index]);
            adc.modify_pn_select(|reg| reg.with_pn_sel(PnSel::Pn9));

            let mut dac = axi.dac.channel_mut(u4::new(chan));
            dac.write_data_source(saved_dac_source[index]);
            axi.dac.synchronize();
            axi.dac
                .channel_mut(u4::new(chan))
                .write_legacy_control(saved_dac_legacy[index]);
        }
        result
    }

    /// `ad9361_dig_tune()`. `max_freq` 0 tunes at the current rate, anything else sweeps 25 MSPS
    /// up to 61.44 MSPS.
    ///
    pub async fn dig_tune(
        &mut self,
        max_freq: HertzU32,
        flags: DigTuneFlags,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let ensm_state = self.save_ensm_state()?;

        let mut restore = self.tune.dig_interface_tune == DigInterfaceTune::UseConfigured || flags.restore_default;
        let mut result = Ok(());
        if !restore {
            let loopback = self.tune.bist_loopback_mode;
            let bist = self.tune.bist_config;

            // mute, the PRBS must not go out on the antenna
            self.tx_mute(true)?;
            if !self.mode.fdd {
                self.set_ensm_mode(true, false)?;
            }

            result = self.dig_tune_rx(max_freq).await;
            if result.is_ok() && self.tune.dig_interface_tune == DigInterfaceTune::RxAndTx {
                result = self.dig_tune_tx(max_freq).await;
            }

            self.bist_loopback(loopback)?;
            self.write_reg(bist)?;

            if matches!(result, Err(Ad9361Error::TuningFailed)) {
                restore = true;
            }
            if max_freq.to_raw() == 0 {
                result = Ok(());
            }
        }

        if restore {
            let _ = self.ensm_force_state(ForcedEnsmState::Alert).await?;
            self.write_reg(self.tune.rx_clk_data_delay)?;
            self.write_reg(self.tune.tx_clk_data_delay)?;
        } else if !flags.skip_store_result {
            self.tune.rx_clk_data_delay = self.read_reg::<RxClockDataDelay>()?;
            self.tune.tx_clk_data_delay = self.read_reg::<TxClockDataDelay>()?;
        }

        if !self.mode.fdd {
            self.set_ensm_mode(self.mode.fdd, self.ensm.pin_ctrl)?;
        }
        self.ensm_restore_state(ensm_state)?;

        let axi = &mut self.axi;
        axi.adc.reset_pulse();
        self.tx_mute(false)?;
        result
    }

    /// `ad9361_post_setup()`: configures the AXI cores and tunes.
    pub async fn post_setup(&mut self) -> Result<(), Ad9361Error<S::Error>> {
        let (rx2tx2, lvds, half_rate) = (self.mode.rx2tx2, self.mode.lvds_mode, self.tune.axi_half_dac_rate_en);
        let num_channels = self.num_phy_chan();
        let axi = &mut self.axi;

        axi.adc
            .regs_mut()
            .modify_adc_control1(|reg| reg.with_r1_mode(r1_mode(rx2tx2)));
        axi.dac
            .regs()
            .modify_control2(|reg| reg.with_r1_mode(r1_mode(rx2tx2)));
        let rate = match (rx2tx2, half_rate, lvds) {
            (false, false, lvds) => lvds as u8,
            (false, true, _) => 0,
            (true, false, true) => 3,
            (true, false, false) | (true, true, _) => 1,
        };
        axi.dac
            .set_rate_div(core::num::NonZero::new(rate + 1).expect("rate divider is nonzero"));

        for chan in 0..num_channels {
            // no DC filter offset. enabling the channel also sets the IQ correction coefficients
            axi.adc.channel_mut(u4::new(chan)).write_control1(0);
            axi.adc.enable_channel(u4::new(chan));
        }

        let core_id = self.axi.core_id;
        let flags = DigTuneFlags {
            restore_default: core_id != 0,
            ..Default::default()
        };
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
            this.ensm_restore_state(saved_ensm)?;
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

fn r1_mode(rx2tx2: bool) -> axi_ad9361::regs::fields::R1Mode {
    if rx2tx2 {
        axi_ad9361::regs::fields::R1Mode::TwoChannels
    } else {
        axi_ad9361::regs::fields::R1Mode::OneChannel
    }
}
