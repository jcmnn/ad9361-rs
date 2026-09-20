//! Ports and the small blocks: RF/data ports, channel enables, AuxADC/AuxDAC, GPO, control
//! outputs, external LNA.

use super::*;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Dac {
    Dac1,
    Dac2,
}

impl Dac {
    /// Bit for this DAC in the two-bit register fields.
    const fn index(self) -> u8 {
        match self {
            Dac::Dac1 => 0,
            Dac::Dac2 => 1,
        }
    }
}

/// AuxADC and temperature sensor.
#[derive(Clone, Copy, Debug)]
pub struct AuxAdcConfig {
    /// signed
    pub offset: i8,
    /// temperature measurement interval
    pub temp_time_interval_ms: u32,
    pub temp_sensor_decimation: AuxAdcDecimation,
    pub periodic_temp_measurement: bool,
    pub auxadc_clock_rate: HertzU32,
    pub auxadc_decimation: AuxAdcDecimation,
}

impl Default for AuxAdcConfig {
    /// no-OS defaults
    fn default() -> Self {
        Self {
            offset: 0xCEu8 as i8,
            temp_time_interval_ms: 1000,
            temp_sensor_decimation: AuxAdcDecimation::D256,
            periodic_temp_measurement: true,
            auxadc_clock_rate: HertzU32::Hz(40_000_000),
            auxadc_decimation: AuxAdcDecimation::D256,
        }
    }
}

/// Control outputs.
pub struct CtrlOutsConfig {
    pub index: u8,
    /// one bit per output
    pub en_mask: u8,
}

impl Default for CtrlOutsConfig {
    fn default() -> Self {
        Self {
            index: 0,
            en_mask: 0xFF,
        }
    }
}

/// External LNA.
pub struct ElnaConfig {
    /// high gain
    pub gain: ElnaGain,
    /// bypass loss
    pub bypass_loss: ElnaGain,
    pub settling_delay_ns: u32,
    /// LNA 1 on GPO0
    pub elna_1_control_en: bool,
    /// LNA 2 on GPO1
    pub elna_2_control_en: bool,
    pub elna_in_gaintable_all_index_en: bool,
}

impl Default for ElnaConfig {
    fn default() -> Self {
        Self {
            gain: ElnaGain::ZERO,
            bypass_loss: ElnaGain::ZERO,
            settling_delay_ns: 0,
            elna_1_control_en: false,
            elna_2_control_en: false,
            elna_in_gaintable_all_index_en: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    Ch1 = 0,
    Ch2 = 1,
}

/// Parallel port (the digital data interface).
pub struct PortConfig {
    pub conf1: ParallelPortConf1,
    pub conf2: ParallelPortConf2,
    pub conf3: ParallelPortConf3,
    pub rx_clk_data_delay: RxClockDataDelay,
    pub tx_clk_data_delay: TxClockDataDelay,
    pub lvds_bias: LvdsBiasControl,
    pub lvds_invert1: u8,
    pub lvds_invert2: u8,
    pub rx1rx2_phase_inversion: bool,
}

impl PortConfig {
    /// conf 3 with the combos the chip can't do fixed up
    pub(super) fn sanitized_conf3(&self) -> ParallelPortConf3 {
        let mut conf3 = self.conf3;
        if conf3.lvds_mode() {
            conf3 = conf3
                .with_half_duplex_mode(false)
                .with_single_data_rate(false)
                .with_single_port_mode(false);
        }
        if conf3.full_port() {
            conf3 = conf3.with_half_duplex_mode(false).with_single_port_mode(false);
        }
        conf3
    }
}

impl Default for PortConfig {
    /// no-OS default: LVDS, 150 mV bias, on-chip RX termination
    fn default() -> Self {
        Self {
            conf1: ParallelPortConf1::default()
                .with_pp_tx_swap_iq(true)
                .with_pp_rx_swap_iq(true)
                .with_rx_frame_pulse_mode(true),
            conf2: ParallelPortConf2::default(),
            conf3: ParallelPortConf3::default().with_lvds_mode(true),
            rx_clk_data_delay: RxClockDataDelay::default()
                .with_data_clk_delay(u4::new(0))
                .with_rx_data_delay(u4::new(4)),
            tx_clk_data_delay: TxClockDataDelay::default()
                .with_fb_clk_delay(u4::new(7))
                .with_tx_data_delay(u4::new(0)),
            // (150 mV - 75 mV) / 75 mV = 1
            lvds_bias: LvdsBiasControl::default()
                .with_lvds_bias(u3::new(1))
                .with_rx_on_chip_term(true),
            lvds_invert1: 0xFF,
            lvds_invert2: 0x0F,
            rx1rx2_phase_inversion: false,
        }
    }
}

#[derive(Default)]
pub struct GpoConfig {
    pub gpo_manual_mode_en: bool,

    pub gpo0_slave_rx_en: bool,
    pub gpo1_slave_rx_en: bool,
    pub gpo2_slave_rx_en: bool,
    pub gpo3_slave_rx_en: bool,
    pub gpo0_slave_tx_en: bool,
    pub gpo1_slave_tx_en: bool,
    pub gpo2_slave_tx_en: bool,
    pub gpo3_slave_tx_en: bool,

    pub gpo_manual_mode_enable_mask: u4,
    pub gpo0_inactive_state_high_en: bool,
    pub gpo1_inactive_state_high_en: bool,
    pub gpo2_inactive_state_high_en: bool,
    pub gpo3_inactive_state_high_en: bool,

    pub gpo0_rx_delay_us: u8,
    pub gpo0_tx_delay_us: u8,
    pub gpo1_rx_delay_us: u8,
    pub gpo1_tx_delay_us: u8,
    pub gpo2_rx_delay_us: u8,
    pub gpo2_tx_delay_us: u8,
    pub gpo3_rx_delay_us: u8,
    pub gpo3_tx_delay_us: u8,
}

pub struct AuxDacConfig {
    /// Default value for DAC 1 (mV)
    pub dac1_default_value: u16,
    /// Default value for DAC 2 (mV)
    pub dac2_default_value: u16,
    /// Enable DAC2 TX Auto Bar
    pub dac2_in_tx_en: bool,
    /// Enable DAC1 TX Auto Bar
    pub dac1_in_tx_en: bool,
    /// Enable DAC2 RX Auto Bar
    pub dac2_in_rx_en: bool,
    /// Enable DAC1 RX Auto Bar
    pub dac1_in_rx_en: bool,
    pub dac2_in_alert_en: bool,
    pub dac1_in_alert_en: bool,
    pub auxdac_manual_mode_en: bool,
    pub dac1_rx_delay_us: u8,
    pub dac2_rx_delay_us: u8,
    pub dac1_tx_delay_us: u8,
    pub dac2_tx_delay_us: u8,
}

impl Default for AuxDacConfig {
    /// no-OS defaults: both DACs 0 mV, manual mode
    fn default() -> Self {
        Self {
            dac1_default_value: 0,
            dac2_default_value: 0,
            dac2_in_tx_en: false,
            dac1_in_tx_en: false,
            dac2_in_rx_en: false,
            dac1_in_rx_en: false,
            dac2_in_alert_en: false,
            dac1_in_alert_en: false,
            auxdac_manual_mode_en: true,
            dac1_rx_delay_us: 0,
            dac2_rx_delay_us: 0,
            dac1_tx_delay_us: 0,
            dac2_tx_delay_us: 0,
        }
    }
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    /// Sets an AuxDAC in mV.
    pub fn auxdac_set(&mut self, dac: Dac, val_mv: u16) -> Result<(), S::Error> {
        // the manual bar bits are active low, set = DAC off
        let disable = val_mv == 0;
        self.modify_reg::<AuxDacEnableControl>(|reg| {
            let bit = 1u8 << dac.index();
            let bars = (reg.auxdac_manual_bar().value() & !bit) | if disable { bit } else { 0 };
            reg.with_auxdac_manual_bar(u2::new(bars))
        })?;

        let val_mv = val_mv.max(306);

        let (vref, val) = if val_mv < 1888 {
            // Vref = 1V, Step = 2
            let val = (((val_mv as u32 - 306) * 1000) / 1469) as u16;
            (u2::new(0), val)
        } else {
            // Vref = 2.5V, Step = 2
            let val = (((val_mv as u32 - 1761) * 1000) / 1512) as u16;
            (u2::new(3), val)
        };

        // 10 bits: top 8 in the word register, low 2 in the config register
        let val = val.clamp(0, 1023);
        let msb = (val >> 2) as u8;
        let lsb = u2::extract_u16(val, 0);
        match dac {
            Dac::Dac1 => {
                self.write_reg(AuxDac1Word(msb))?;
                self.write_reg(
                    AuxDac1Config::default()
                        .with_auxdac_1_word_lsb(lsb)
                        .with_auxdac_1_vref(vref),
                )?;
            }
            Dac::Dac2 => {
                self.write_reg(AuxDac2Word(msb))?;
                self.write_reg(
                    AuxDac2Config::default()
                        .with_auxdac_2_word_lsb(lsb)
                        .with_auxdac_2_vref(vref),
                )?;
            }
        }

        Ok(())
    }

    pub fn auxdac_setup(&mut self, config: &AuxDacConfig) -> Result<(), S::Error> {
        self.auxdac_set(Dac::Dac1, config.dac1_default_value)?;
        self.auxdac_set(Dac::Dac2, config.dac2_default_value)?;

        // bar bits again, active low. DAC1 is bit 0
        let bars = |dac1: bool, dac2: bool| u2::new(!(dac1 as u8 | (dac2 as u8) << 1) & 0b11);
        self.modify_reg::<AuxDacEnableControl>(|reg| {
            reg.with_auxdac_auto_tx_bar(bars(config.dac1_in_tx_en, config.dac2_in_tx_en))
                .with_auxdac_auto_rx_bar(bars(config.dac1_in_rx_en, config.dac2_in_rx_en))
                .with_auxdac_init_bar(bars(config.dac1_in_alert_en, config.dac2_in_alert_en))
        })?;

        self.modify_reg::<ExternalLnaControl>(|reg| {
            reg.with_auxdac_manual_select(config.auxdac_manual_mode_en)
        })?;

        self.write_reg(AuxDac1RxDelay(config.dac1_rx_delay_us))?;
        self.write_reg(AuxDac1TxDelay(config.dac1_tx_delay_us))?;
        self.write_reg(AuxDac2RxDelay(config.dac2_rx_delay_us))?;
        self.write_reg(AuxDac2TxDelay(config.dac2_tx_delay_us))?;

        Ok(())
    }

    pub fn gpo_setup(&mut self, config: &GpoConfig) -> Result<(), S::Error> {
        self.write_reg(
            AutoGpo::default()
                .with_gpo_enable_auto_rx(u4::from_u8(
                    ((config.gpo3_slave_rx_en as u8) << 3)
                        | ((config.gpo2_slave_rx_en as u8) << 2)
                        | ((config.gpo1_slave_rx_en as u8) << 1)
                        | (config.gpo0_slave_rx_en as u8),
                ))
                .with_gpo_enable_auto_tx(u4::from_u8(
                    ((config.gpo3_slave_tx_en as u8) << 3)
                        | ((config.gpo2_slave_tx_en as u8) << 2)
                        | ((config.gpo1_slave_tx_en as u8) << 1)
                        | (config.gpo0_slave_tx_en as u8),
                )),
        )?;

        self.write_reg(
            GpoForceAndInit::default()
                .with_gpo_manual_ctrl(config.gpo_manual_mode_enable_mask)
                .with_gpo_init_state(u4::from_u8(
                    ((config.gpo3_inactive_state_high_en as u8) << 3)
                        | ((config.gpo2_inactive_state_high_en as u8) << 2)
                        | ((config.gpo1_inactive_state_high_en as u8) << 1)
                        | (config.gpo0_inactive_state_high_en as u8),
                )),
        )?;

        self.write_reg(Gpo0RxDelay(config.gpo0_rx_delay_us))?;
        self.write_reg(Gpo0TxDelay(config.gpo0_tx_delay_us))?;
        self.write_reg(Gpo1RxDelay(config.gpo1_rx_delay_us))?;
        self.write_reg(Gpo1TxDelay(config.gpo1_tx_delay_us))?;
        self.write_reg(Gpo2RxDelay(config.gpo2_rx_delay_us))?;
        self.write_reg(Gpo2TxDelay(config.gpo2_tx_delay_us))?;
        self.write_reg(Gpo3RxDelay(config.gpo3_rx_delay_us))?;
        self.write_reg(Gpo3TxDelay(config.gpo3_tx_delay_us))?;

        // from ad9361.c: GPO manual mode clashes with ENSM slave and eLNA auto mode
        self.modify_reg::<ExternalLnaControl>(|reg| {
            reg.with_gpo_manual_select(config.gpo_manual_mode_en)
        })?;

        Ok(())
    }

    /// `ad9361_en_dis_tx()`.
    pub fn set_tx_channels(&mut self, tx1: bool, tx2: bool) -> Result<(), S::Error> {
        let field = (tx1 as u8) | ((tx2 as u8) << 1);
        self.modify_reg::<TxEnableFilterControl>(|reg| {
            reg.with_tx_channel_enable(u2::new(field))
        })
    }

    /// `ad9361_en_dis_rx()`.
    pub fn set_rx_channels(&mut self, rx1: bool, rx2: bool) -> Result<(), S::Error> {
        let field = (rx1 as u8) | ((rx2 as u8) << 1);
        self.modify_reg::<RxEnableFilterControl>(|reg| {
            reg.with_rx_channel_enable(u2::new(field))
        })
    }

    /// `ad9361_rf_port_setup()` with `is_out = true`. The TX monitor inputs from the C driver
    /// aren't supported.
    pub fn rf_port_setup(
        &mut self,
        rx_input: RxInput,
        tx_output_b: bool,
    ) -> Result<(), S::Error> {
        self.write_reg(
            InputSelect::default()
                .with_rx_input(u6::new(rx_input.select_bits()))
                .with_tx_output(tx_output_b),
        )
    }

    /// `ad9361_pp_port_setup()` with `restore_c3 = false`.
    pub fn pp_port_setup(&mut self, port: &PortConfig) -> Result<(), S::Error> {
        // already corrected in the state
        let conf3 = self.mode.pp_conf3;

        self.write_reg(port.conf1)?;
        self.write_reg(port.conf2)?;
        self.write_reg(conf3)?;
        self.write_reg(port.rx_clk_data_delay)?;
        self.write_reg(port.tx_clk_data_delay)?;
        self.write_reg(port.lvds_bias)?;
        self.write_reg(LvdsInvertCtrl1(port.lvds_invert1))?;
        self.write_reg(LvdsInvertCtrl2(port.lvds_invert2))?;

        if port.rx1rx2_phase_inversion || port.conf2.invert_rx2() {
            self.modify_reg::<ParallelPortConf2>(|reg| reg.with_invert_rx2(true))?;
            self.modify_reg::<InvertBits>(|reg| reg.with_invert_rx2_rf_dc_cgout_word(false))?;
        }

        Ok(())
    }

    /// `ad9361_pp_port_setup()` with `restore_c3 = true`.
    pub fn pp_port_restore_conf3(&mut self) -> Result<(), S::Error> {
        self.write_reg(self.mode.pp_conf3)
    }

    /// `ad9361_auxadc_setup()`, off the cached BBPLL rate. `InvalidRate` if that can't be
    /// divided down to the AuxADC clock. The config already checks the initial rate.
    pub fn auxadc_setup(&mut self, config: &AuxAdcConfig) -> Result<(), Ad9361Error<S::Error>> {
        let bbpll = self.clk.rates.bbpll.to_raw();
        let temp_decimation = config.temp_sensor_decimation.field();
        let aux_decimation = config.auxadc_decimation.field();
        let clock_divider = bbpll
            .checked_div(config.auxadc_clock_rate.to_raw())
            .and_then(|div| u8::try_from(div).ok())
            .filter(|div| *div < 64)
            .ok_or(Ad9361Error::InvalidRate)?;
        // interval is in units of 2^29 BBPLL cycles, rounded
        let interval = (config.temp_time_interval_ms as u64 * (bbpll / 1000) as u64
            + (1 << 28))
            >> 29;

        self.write_reg(TempOffset(config.offset as u8))?;
        self.write_reg(StartTempReading::default())?;
        self.write_reg(
            TempSense2::default()
                .with_measurement_time_interval(u7::new((interval & 0x7F) as u8))
                .with_temp_sense_periodic_enable(config.periodic_temp_measurement),
        )?;
        self.write_reg(TempSensorConfig::default().with_temp_sensor_decimation(temp_decimation))?;
        self.write_reg(
            AuxadcClockDivider::default().with_auxadc_clock_divider(u6::new(clock_divider)),
        )?;
        self.write_reg(AuxadcConfig::default().with_aux_adc_decimation(aux_decimation))?;
        Ok(())
    }

    /// `ad9361_ctrl_outs_setup()`.
    pub fn ctrl_outs_setup(&mut self, config: &CtrlOutsConfig) -> Result<(), S::Error> {
        self.write_reg(ControlOutputPointer(config.index))?;
        self.write_reg(ControlOutputEnable::from_raw(config.en_mask))
    }

    /// `ad9361_set_ref_clk_cycles()`.
    pub fn set_ref_clk_cycles(&mut self, ref_clk: ReferenceClock) -> Result<(), S::Error> {
        // 1-128 MHz, ReferenceClock guarantees it
        let mhz = ref_clk.get().to_raw() / 1_000_000;
        self.write_reg(
            ReferenceClockCycles::default()
                .with_reference_clock_cycles_per_us(u7::new((mhz - 1) as u8)),
        )
    }

    /// `ad9361_setup_ext_lna()`.
    pub fn setup_ext_lna(&mut self, config: &ElnaConfig) -> Result<(), S::Error> {
        self.modify_reg::<ExternalLnaControl>(|reg| {
            reg.with_external_lna1_ctrl(config.elna_1_control_en)
                .with_external_lna2_ctrl(config.elna_2_control_en)
        })?;
        self.write_reg(ExtLnaHighGain::default().with_ext_lna_high_gain(config.gain.field()))?;
        self.write_reg(ExtLnaLowGain::default().with_ext_lna_low_gain(config.bypass_loss.field()))?;
        Ok(())
    }
}
