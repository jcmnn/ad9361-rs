//! ENSM, TX attenuation, CLKOUT.

use arbitrary_int::u3;
use embassy_time::Timer;
use embedded_hal::spi::SpiDevice;

use super::{
    Engine, Ad9361Error, BbPll, ClockEnable, EnsmConfig1, EnsmConfig2, EnsmMode, GainMode,
    Register, RxCpOverrangeVcoLock, RxPfdConfig, SmallLmtOverloadThresh, State, Tx1Atten1,
    Tx2Atten1, Tx2DigAtten, TxAttenuation,
};

/// ENSM state to go back to later.
#[derive(Clone, Copy, Debug)]
#[must_use = "restore the state with `ensm_restore_state`"]
pub(crate) struct SavedEnsmState(u8);

/// States the driver can force.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ForcedEnsmState {
    Tx,
    Fdd,
    Rx,
    Alert,
}

impl From<ForcedEnsmState> for EnsmState {
    fn from(state: ForcedEnsmState) -> Self {
        match state {
            ForcedEnsmState::Tx => EnsmState::Tx,
            ForcedEnsmState::Fdd => EnsmState::Fdd,
            ForcedEnsmState::Rx => EnsmState::Rx,
            ForcedEnsmState::Alert => EnsmState::Alert,
        }
    }
}

/// States that can be requested. The flush states are transient.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RequestedEnsmState {
    Sleep,
    SleepWait,
    Alert,
    Tx,
    Rx,
    Fdd,
}

impl From<RequestedEnsmState> for EnsmState {
    fn from(state: RequestedEnsmState) -> Self {
        match state {
            RequestedEnsmState::Sleep => EnsmState::Sleep,
            RequestedEnsmState::SleepWait => EnsmState::SleepWait,
            RequestedEnsmState::Alert => EnsmState::Alert,
            RequestedEnsmState::Tx => EnsmState::Tx,
            RequestedEnsmState::Rx => EnsmState::Rx,
            RequestedEnsmState::Fdd => EnsmState::Fdd,
        }
    }
}

/// ENSM state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EnsmState {
    /// Clocks off
    Sleep,
    SleepWait,
    Alert,
    Tx,
    TxFlush,
    Rx,
    RxFlush,
    Fdd,
    FddFlush,
}

impl EnsmState {
    /// Value of the `ENSM_STATE` field. The chip never reports `Sleep`.
    pub(crate) const fn raw(self) -> u8 {
        match self {
            EnsmState::Sleep => 0x80,
            EnsmState::SleepWait => 0x0,
            EnsmState::Alert => 0x5,
            EnsmState::Tx => 0x6,
            EnsmState::TxFlush => 0x7,
            EnsmState::Rx => 0x8,
            EnsmState::RxFlush => 0x9,
            EnsmState::Fdd => 0xA,
            EnsmState::FddFlush => 0xB,
        }
    }
}

/// What the CLKOUT pin outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClkoutMode {
    Disable = 0,
    BufferedXtalnDcxo = 1,
    AdcClkDiv2 = 2,
    AdcClkDiv3 = 3,
    AdcClkDiv4 = 4,
    AdcClkDiv8 = 5,
    AdcClkDiv16 = 6,
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    /// `ad9361_set_ensm_mode()`.
    pub fn set_ensm_mode(&mut self, fdd: bool, pinctrl: bool) -> Result<(), S::Error> {
        self.write_reg(EnsmMode::default().with_fdd_mode(fdd))?;

        // keep the synth power down / ready mask bits
        let current = self.read_reg::<EnsmConfig2>()?;
        let kept = EnsmConfig2::default()
            .with_power_down_rx_synth(current.power_down_rx_synth())
            .with_power_down_tx_synth(current.power_down_tx_synth())
            .with_rx_synth_ready_mask(current.rx_synth_ready_mask())
            .with_tx_synth_ready_mask(current.tx_synth_ready_mask());

        self.write_reg(if fdd {
            kept.with_dual_synth_mode(true)
                .with_fdd_external_ctrl_enable(self.mode.fdd_independent_mode)
        } else if self.mode.tdd_use_dual_synth {
            kept.with_dual_synth_mode(true)
        } else {
            kept.with_synth_enable_pin_ctrl_mode(pinctrl)
        })
    }

    /// `ad9361_set_tx_atten()`. `immed` = apply now, don't wait for the update.
    pub fn set_tx_atten(
        &mut self,
        atten: TxAttenuation,
        tx1: bool,
        tx2: bool,
        immed: bool,
    ) -> Result<(), S::Error> {
        // 0.25 dB per LSB, and the burst counts addresses down, so MSB goes first
        let buf = atten.quarter_db().to_be_bytes();

        self.modify_reg::<Tx2DigAtten>(|reg| reg.with_immediately_update_tpc_atten(false))?;
        if tx1 {
            self.write_bytes(&buf, Tx1Atten1::ADDRESS)?;
        }
        if tx2 {
            self.write_bytes(&buf, Tx2Atten1::ADDRESS)?;
        }
        if immed {
            self.modify_reg::<Tx2DigAtten>(|reg| reg.with_immediately_update_tpc_atten(true))?;
        }
        Ok(())
    }

    /// `ad9361_clkout_control()`.
    pub fn clkout_control(&mut self, mode: ClkoutMode) -> Result<(), S::Error> {
        self.modify_reg::<BbPll>(|reg| {
            if mode == ClkoutMode::Disable {
                reg.with_clkout_enable(false)
            } else {
                reg.with_clkout_enable(true)
                    .with_clkout_select(u3::new(mode as u8 - 1))
            }
        })
    }

    /// RX (`tx == false`) or TX VCO cal on/off. `ad9361_trx_vco_cal_control()`.
    pub fn trx_vco_cal_control(&mut self, tx: bool, enable: bool) -> Result<(), S::Error> {
        self.modify_synth_reg::<RxPfdConfig>(tx, |reg| reg.with_bypass_ld_synth(!enable))
    }

    /// Raw state from the chip.
    pub fn ensm_state(&mut self) -> Result<u8, S::Error> {
        Ok(self.read_reg::<State>()?.ensm_state().value())
    }

    /// `ad9361_ensm_force_state()`. Returns the old state for restoring.
    pub async fn ensm_force_state(
        &mut self,
        state: ForcedEnsmState,
    ) -> Result<SavedEnsmState, S::Error> {
        let device_state = self.ensm_state()?;
        let saved = SavedEnsmState(device_state);
        if device_state == EnsmState::from(state).raw() {
            return Ok(saved);
        }

        // SPI control on, and out of alert
        let mut config = self.read_reg::<EnsmConfig1>()?;
        self.ensm.saved_pin_ctrl_enable = config.enable_ensm_pin_ctrl();
        config = config.with_enable_ensm_pin_ctrl(false);
        if device_state != 0 {
            config = config.with_to_alert(false);
        }
        config = match state {
            ForcedEnsmState::Tx | ForcedEnsmState::Fdd => config.with_force_tx_on(true),
            ForcedEnsmState::Rx => config.with_force_rx_on(true),
            ForcedEnsmState::Alert => config
                .with_force_tx_on(false)
                .with_force_rx_on(false)
                .with_to_alert(true)
                .with_force_alert_state(true),
        };

        self.write_reg(EnsmConfig1::default().with_to_alert(true).with_force_alert_state(true))?;
        self.write_reg(config)?;

        // takes a moment. Timing out is not an error, no-OS doesn't care either
        for _ in 0..10 {
            if self.ensm_state()? == EnsmState::from(state).raw() {
                break;
            }
            Timer::after_millis(1).await;
        }
        Ok(saved)
    }

    /// Current state, for [`Self::ensm_restore_state`].
    pub fn save_ensm_state(&mut self) -> Result<SavedEnsmState, S::Error> {
        Ok(SavedEnsmState(self.ensm_state()?))
    }

    /// Goes back to a saved state if it's one that can be restored (`ad9361_ensm_restore_state()`).
    pub fn ensm_restore_state(&mut self, saved: SavedEnsmState) -> Result<(), S::Error> {
        let previous = saved.0;
        // clear whatever forcing set
        let mut config = self
            .read_reg::<EnsmConfig1>()?
            .with_force_tx_on(false)
            .with_force_rx_on(false)
            .with_force_alert_state(false)
            .with_to_alert(true);
        match previous {
            0x6 | 0xA => config = config.with_force_tx_on(true),
            0x8 => config = config.with_force_rx_on(true),
            0x5 => {}
            _ => return Ok(()), // Not a state that can be restored
        }

        self.write_reg(EnsmConfig1::default().with_to_alert(true).with_force_alert_state(true))?;
        self.write_reg(config)?;
        if self.ensm.saved_pin_ctrl_enable {
            self.write_reg(config.with_enable_ensm_pin_ctrl(true))?;
        }
        Ok(())
    }

    /// `ad9361_ensm_set_state()`.
    pub async fn ensm_set_state(
        &mut self,
        state: RequestedEnsmState,
        pinctrl: bool,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let state = EnsmState::from(state);
        let sleeping = self.ensm.current == EnsmState::Sleep.raw();
        if sleeping {
            self.write_reg(
                ClockEnable::default()
                    .with_digital_power_up(true)
                    .with_clock_enable_dflt(true)
                    .with_bbpll_enable(true)
                    .with_xo_bypass(self.clk.use_extclk),
            )?; // Enable clocks
            Timer::after_micros(20).await;
            self.write_reg(
                EnsmConfig1::default().with_to_alert(true).with_force_alert_state(true),
            )?;
            self.trx_vco_cal_control(false, true)?;
            self.trx_vco_cal_control(true, true)?;
        }

        let mut val = EnsmConfig1::default()
            .with_level_mode(!self.ensm.pin_pulse_mode)
            .with_enable_ensm_pin_ctrl(pinctrl)
            .with_enable_rx_data_port_for_cal(self.mode.txmon_tdd_en)
            .with_to_alert(true);
        // transition not allowed from where we are
        let mut invalid = false;
        let from_alert = self.ensm.current == EnsmState::Alert.raw();

        match state {
            EnsmState::Tx => {
                val = val.with_force_tx_on(true);
                invalid = self.mode.fdd || !from_alert;
            }
            EnsmState::Rx => {
                val = val.with_force_rx_on(true);
                invalid = self.mode.fdd || !from_alert;
            }
            EnsmState::Fdd => {
                val = val.with_force_tx_on(true);
                invalid = !self.mode.fdd;
            }
            EnsmState::Alert => {
                val = val
                    .with_force_tx_on(false)
                    .with_force_rx_on(false)
                    .with_to_alert(true)
                    .with_force_alert_state(true);
            }
            EnsmState::SleepWait => {}
            EnsmState::Sleep => {
                self.trx_vco_cal_control(false, false)?;
                self.trx_vco_cal_control(true, false)?;
                self.write_reg(EnsmConfig1::default())?; // Clear to alert
                self.write_reg(if self.mode.fdd {
                    EnsmConfig1::default().with_force_tx_on(true)
                } else {
                    EnsmConfig1::default().with_force_rx_on(true)
                })?;
                // flush takes 384 ADC clock cycles
                let adc = self.clk.rates.adc.to_raw().max(1);
                Timer::after_micros(384_000_000u64 / adc as u64).await;
                self.write_reg(EnsmConfig1::default())?; // Move to wait
                Timer::after_micros(1).await; // Wait for the ENSM to settle
                self.write_reg(ClockEnable::default().with_xo_bypass(self.clk.use_extclk))?; // Clocks off
                self.ensm.current = state.raw();
                return Ok(());
            }
            EnsmState::TxFlush | EnsmState::RxFlush | EnsmState::FddFlush => {
                unreachable!("a RequestedEnsmState is never a flush state")
            }
        }

        if invalid {
            if !from_alert && (val.force_rx_on() || val.force_tx_on()) {
                // has to pass through alert first
                let alert = val
                    .with_force_tx_on(false)
                    .with_force_rx_on(false)
                    .with_to_alert(true)
                    .with_force_alert_state(true);
                self.write_reg(alert)?;
                self.wait_until::<State>(
                    State::ADDRESS,
                    |reg| reg.ensm_state().value() == EnsmState::Alert.raw(),
                    120,
                )
                .await?;
            } else {
                return Err(Ad9361Error::InvalidEnsmTransition);
            }
        }

        if !self.mode.fdd
            && !pinctrl
            && !self.mode.tdd_use_dual_synth
            && matches!(state, EnsmState::Tx | EnsmState::Rx)
        {
            let tx = state == EnsmState::Tx;
            self.modify_reg::<EnsmConfig2>(|reg| reg.with_txnrx_spi_ctrl(tx))?;
            self.wait_until::<RxCpOverrangeVcoLock>(
                Self::synth_addr::<RxCpOverrangeVcoLock>(tx),
                |reg| reg.vco_lock(),
                120,
            )
            .await?;
        }

        self.write_reg(val)?;

        let manual = |mode: GainMode| mode == GainMode::Manual;
        if val.force_rx_on() && self.gain.agc_mode.iter().any(|m| manual(*m)) {
            let raw = self.read_reg::<SmallLmtOverloadThresh>()?.to_raw() & 0x3F;
            let base = SmallLmtOverloadThresh::from_raw(raw);
            self.write_reg(
                base.with_force_pd_reset_rx1(manual(self.gain.agc_mode[0]))
                    .with_force_pd_reset_rx2(manual(self.gain.agc_mode[1])),
            )?;
            self.write_reg(base)?;
        }

        self.ensm.current = state.raw();
        Ok(())
    }
}
