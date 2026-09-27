//! Ports and the small blocks: RF/data ports, channel enables, AuxADC/AuxDAC, GPO, control
//! outputs, external LNA.

use super::*;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Dac {
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

/// AuxADC and temperature sensor settings.
///
/// The defaults leave the temperature sensor measuring once a second. `offset` is the sensor
/// calibration value (default 0xCE).
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

/// Control output pins (`CTRL_OUT`). `index` picks which set of internal signals appears on the
/// eight pins, and `en_mask` says which pins are driven. The signals are listed in the AD9361
/// reference manual under "Control Output". The default index 0 shows calibration busy/done, and
/// all pins are on.
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

/// External LNA control through GPO0 and GPO1. Leave it at the default unless an external LNA is
/// fitted.
///
/// `gain` and `bypass_loss` are the LNA's gain and its loss when bypassed, so the RX gain
/// readings and gain table can account for it. `settling_delay_ns` is the time the LNA needs
/// after a gain change.
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

/// RX or TX channel 1 or 2.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    Ch1 = 0,
    Ch2 = 1,
}

/// One channel or both, for settings that can go to either.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channels {
    Ch1,
    Ch2,
    Both,
}

impl Channels {
    /// Bit 0 for channel 1, bit 1 for channel 2, how the chip's select fields want it.
    pub(crate) const fn mask(self) -> u8 {
        match self {
            Channels::Ch1 => 0b01,
            Channels::Ch2 => 0b10,
            Channels::Both => 0b11,
        }
    }

    pub(crate) const fn contains(self, channel: Channel) -> bool {
        self.mask() & (1 << channel as u8) != 0
    }
}

impl From<Channel> for Channels {
    fn from(channel: Channel) -> Self {
        match channel {
            Channel::Ch1 => Channels::Ch1,
            Channel::Ch2 => Channels::Ch2,
        }
    }
}

/// Parallel data port settings, the LVDS or CMOS interface to the FPGA. Has to match the FPGA
/// design. The default is LVDS with 150 mV bias and on-chip RX termination, I and Q swapped in
/// both directions and a pulsed RX frame.
///
/// The delays are where the port starts. With
/// [`DigInterfaceTune`](crate::settings::DigInterfaceTune) other than `UseConfigured`, the
/// driver tunes them during init.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PortConfig {
    /// LVDS or CMOS, with the settings that only apply to that one
    pub mode: PortMode,
    pub rx_delays: PortDelays,
    pub tx_delays: PortDelays,
    /// Swap I and Q on the RX data
    pub rx_swap_iq: bool,
    /// Swap I and Q on the TX data
    pub tx_swap_iq: bool,
    /// Swap RX1 and RX2 on the port
    pub rx_swap_channels: bool,
    /// Swap TX1 and TX2 on the port
    pub tx_swap_channels: bool,
    /// RX_FRAME is a pulse at the start of each burst, otherwise a 50% duty cycle clock
    pub rx_frame_pulse_mode: bool,
    /// 1R1T uses the 2R2T data timing, with the second channel's slots unused
    pub two_by_two_timing: bool,
    /// In FDD, RX runs at twice the TX rate
    pub fdd_rx_rate_2x_tx_rate: bool,
    /// Alternate RX and TX words in FDD
    pub fdd_alt_word_order: bool,
    /// Inverts the data bus bits
    pub invert_data_bus: bool,
    /// Inverts DATA_CLK
    pub invert_data_clk: bool,
    /// Inverts RX_FRAME
    pub invert_rx_frame: bool,
    /// Inverts the sign of each channel's data. Inverting RX2 lines its phase up with RX1 on
    /// boards where the RX2 input is wired the other way round
    pub invert: ChannelInversion,
    /// Extra RX data delay, in DATA_CLK cycles
    pub rx_data_extra_delay: u2,
    /// CLK_OUT slew rate, 0 is the fastest
    pub clk_out_slew: u2,
}

/// Which channels have their data sign inverted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ChannelInversion {
    pub rx1: bool,
    pub rx2: bool,
    pub tx1: bool,
    pub tx2: bool,
}

/// LVDS or CMOS data port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortMode {
    /// Always dual port, full duplex, double data rate
    Lvds(LvdsConfig),
    Cmos(CmosConfig),
}

/// LVDS electrical settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LvdsConfig {
    pub bias: LvdsBias,
    /// 100 ohm termination on the RX side of the chip (TX data from the FPGA)
    pub rx_on_chip_termination: bool,
    /// Bypass the bias resistor
    pub bypass_bias_resistor: bool,
    /// Lower common mode voltage on the LVDS outputs
    pub low_common_mode: bool,
    /// Swaps P and N of single LVDS pairs to match the board routing. The bits of LVDS Invert
    /// Control 1 and 2 (0x03D, 0x03E) in the register map
    pub pair_inversion: [u8; 2],
}

impl Default for LvdsConfig {
    fn default() -> Self {
        Self {
            bias: LvdsBias::MV_150,
            rx_on_chip_termination: true,
            bypass_bias_resistor: false,
            low_common_mode: false,
            pair_inversion: [0xFF, 0x0F],
        }
    }
}

/// LVDS output swing, 75 to 450 mV in 75 mV steps.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LvdsBias(u3);

impl LvdsBias {
    pub const MV_150: Self = Self(u3::new(1));

    /// Rounded down to a 75 mV step.
    pub const fn from_mv(mv: u16) -> Result<Self, OutOfRange> {
        if mv < 75 || mv > 450 {
            return Err(OutOfRange);
        }
        Ok(Self(u3::new((mv / 75 - 1) as u8)))
    }

    pub const fn mv(self) -> u16 {
        (self.0.value() as u16 + 1) * 75
    }
}

/// CMOS port layout and timing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CmosConfig {
    pub ports: CmosPorts,
    /// One word per DATA_CLK cycle instead of two
    pub single_data_rate: bool,
    /// Swap P0 and P1
    pub swap_ports: bool,
    /// Swap the bits in full duplex
    pub full_duplex_swap_bits: bool,
}

/// How the two 12 bit CMOS ports are used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CmosPorts {
    /// One port, RX and TX take turns
    SinglePortHalfDuplex,
    /// One port, RX and TX interleaved
    SinglePortFullDuplex,
    /// Both ports, RX and TX take turns
    DualPortHalfDuplex,
    /// Both ports, RX and TX interleaved
    DualPortFullDuplex,
    /// P0 for RX, P1 for TX
    FullPort,
}

/// Clock and data delay of one direction of the port, 0..=15 steps each.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PortDelays {
    /// DATA_CLK delay for RX, FB_CLK delay for TX
    pub clock: u4,
    pub data: u4,
}

impl PortConfig {
    pub(crate) fn is_lvds(&self) -> bool {
        matches!(self.mode, PortMode::Lvds(_))
    }

    pub(crate) fn conf1(&self) -> ParallelPortConf1 {
        ParallelPortConf1::default()
            .with_pp_tx_swap_iq(self.tx_swap_iq)
            .with_pp_rx_swap_iq(self.rx_swap_iq)
            .with_tx_channel_swap(self.tx_swap_channels)
            .with_rx_channel_swap(self.rx_swap_channels)
            .with_rx_frame_pulse_mode(self.rx_frame_pulse_mode)
            .with_r2t2_timing(self.two_by_two_timing)
            .with_invert_data_bus(self.invert_data_bus)
            .with_invert_data_clk(self.invert_data_clk)
    }

    pub(crate) fn conf2(&self) -> ParallelPortConf2 {
        ParallelPortConf2::default()
            .with_fdd_alt_word_order(self.fdd_alt_word_order)
            .with_invert_rx1(self.invert.rx1)
            .with_invert_rx2(self.invert.rx2)
            .with_invert_tx1(self.invert.tx1)
            .with_invert_tx2(self.invert.tx2)
            .with_invert_rx_frame(self.invert_rx_frame)
            .with_delay_rx_data(self.rx_data_extra_delay)
    }

    pub(crate) fn conf3(&self) -> ParallelPortConf3 {
        let conf3 =
            ParallelPortConf3::default().with_fdd_rx_rate_2tx_rate(self.fdd_rx_rate_2x_tx_rate);
        match self.mode {
            PortMode::Lvds(_) => conf3.with_lvds_mode(true),
            PortMode::Cmos(cmos) => {
                let (single_port, half_duplex, full_port) = match cmos.ports {
                    CmosPorts::SinglePortHalfDuplex => (true, true, false),
                    CmosPorts::SinglePortFullDuplex => (true, false, false),
                    CmosPorts::DualPortHalfDuplex => (false, true, false),
                    CmosPorts::DualPortFullDuplex => (false, false, false),
                    CmosPorts::FullPort => (false, false, true),
                };
                conf3
                    .with_single_port_mode(single_port)
                    .with_half_duplex_mode(half_duplex)
                    .with_full_port(full_port)
                    .with_single_data_rate(cmos.single_data_rate)
                    .with_swap_ports(cmos.swap_ports)
                    .with_full_duplex_swap_bits(cmos.full_duplex_swap_bits)
            }
        }
    }

    pub(crate) fn rx_clock_data_delay(&self) -> RxClockDataDelay {
        RxClockDataDelay::default()
            .with_data_clk_delay(self.rx_delays.clock)
            .with_rx_data_delay(self.rx_delays.data)
    }

    pub(crate) fn tx_clock_data_delay(&self) -> TxClockDataDelay {
        TxClockDataDelay::default()
            .with_fb_clk_delay(self.tx_delays.clock)
            .with_tx_data_delay(self.tx_delays.data)
    }

    /// LVDS bias and CLK_OUT slew, and the pair inversion masks.
    pub(crate) fn lvds_registers(&self) -> (LvdsBiasControl, [u8; 2]) {
        let bias = LvdsBiasControl::default().with_clk_out_slew(self.clk_out_slew);
        match self.mode {
            PortMode::Lvds(lvds) => (
                bias.with_lvds_bias(lvds.bias.0)
                    .with_rx_on_chip_term(lvds.rx_on_chip_termination)
                    .with_lvds_bypass_bias_r(lvds.bypass_bias_resistor)
                    .with_lvds_tx_lo_vcm(lvds.low_common_mode),
                lvds.pair_inversion,
            ),
            PortMode::Cmos(_) => (bias, [0, 0]),
        }
    }
}

impl Default for PortConfig {
    fn default() -> Self {
        Self {
            mode: PortMode::Lvds(LvdsConfig::default()),
            rx_delays: PortDelays {
                clock: u4::new(0),
                data: u4::new(4),
            },
            tx_delays: PortDelays {
                clock: u4::new(7),
                data: u4::new(0),
            },
            rx_swap_iq: true,
            tx_swap_iq: true,
            rx_swap_channels: false,
            tx_swap_channels: false,
            rx_frame_pulse_mode: true,
            two_by_two_timing: false,
            fdd_rx_rate_2x_tx_rate: false,
            fdd_alt_word_order: false,
            invert_data_bus: false,
            invert_data_clk: false,
            invert_rx_frame: false,
            invert: ChannelInversion::default(),
            rx_data_extra_delay: u2::new(0),
            clk_out_slew: u2::new(0),
        }
    }
}

/// General purpose output pins. The chip can drive them from the ENSM state (`slaveX` flags and
/// delays) or by hand (`gpo_manual_mode_en`). Everything is off by default.
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

/// The two auxiliary DACs. Each DAC can be set by hand, or switched on for RX, TX or ALERT with a
/// delay after the state change. The default has both at 0 mV in manual mode.
pub struct AuxDacConfig {
    /// DAC 1 output in mV
    pub dac1_default_mv: u16,
    /// DAC 2 output in mV
    pub dac2_default_mv: u16,
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
    fn default() -> Self {
        Self {
            dac1_default_mv: 0,
            dac2_default_mv: 0,
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

impl<S, I> Engine<S, I>
where
    S: SpiDevice<u8>,
    I: DataInterface,
{
    /// Sets an AuxDAC in mV.
    pub(crate) async fn auxdac_set(&mut self, dac: Dac, val_mv: u16) -> Result<(), S::Error> {
        // the manual bar bits are active low, set = DAC off
        let disable = val_mv == 0;
        self.modify_reg::<AuxDacEnableControl>(|reg| {
            let bit = 1u8 << dac.index();
            let bars = (reg.auxdac_manual_bar().value() & !bit) | if disable { bit } else { 0 };
            reg.with_auxdac_manual_bar(u2::new(bars))
        })
        .await?;

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
                self.write_reg(AuxDac1Word(msb)).await?;
                self.write_reg(
                    AuxDac1Config::default()
                        .with_auxdac_1_word_lsb(lsb)
                        .with_auxdac_1_vref(vref),
                )
                .await?;
            }
            Dac::Dac2 => {
                self.write_reg(AuxDac2Word(msb)).await?;
                self.write_reg(
                    AuxDac2Config::default()
                        .with_auxdac_2_word_lsb(lsb)
                        .with_auxdac_2_vref(vref),
                )
                .await?;
            }
        }

        Ok(())
    }

    pub(crate) async fn auxdac_setup(&mut self, config: &AuxDacConfig) -> Result<(), S::Error> {
        self.auxdac_set(Dac::Dac1, config.dac1_default_mv).await?;
        self.auxdac_set(Dac::Dac2, config.dac2_default_mv).await?;

        // bar bits again, active low. DAC1 is bit 0
        let bars = |dac1: bool, dac2: bool| u2::new(!(dac1 as u8 | (dac2 as u8) << 1) & 0b11);
        self.modify_reg::<AuxDacEnableControl>(|reg| {
            reg.with_auxdac_auto_tx_bar(bars(config.dac1_in_tx_en, config.dac2_in_tx_en))
                .with_auxdac_auto_rx_bar(bars(config.dac1_in_rx_en, config.dac2_in_rx_en))
                .with_auxdac_init_bar(bars(config.dac1_in_alert_en, config.dac2_in_alert_en))
        })
        .await?;

        self.modify_reg::<ExternalLnaControl>(|reg| {
            reg.with_auxdac_manual_select(config.auxdac_manual_mode_en)
        })
        .await?;

        self.write_reg(AuxDac1RxDelay(config.dac1_rx_delay_us))
            .await?;
        self.write_reg(AuxDac1TxDelay(config.dac1_tx_delay_us))
            .await?;
        self.write_reg(AuxDac2RxDelay(config.dac2_rx_delay_us))
            .await?;
        self.write_reg(AuxDac2TxDelay(config.dac2_tx_delay_us))
            .await?;

        Ok(())
    }

    pub(crate) async fn gpo_setup(&mut self, config: &GpoConfig) -> Result<(), S::Error> {
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
        )
        .await?;

        self.write_reg(
            GpoForceAndInit::default()
                .with_gpo_manual_ctrl(config.gpo_manual_mode_enable_mask)
                .with_gpo_init_state(u4::from_u8(
                    ((config.gpo3_inactive_state_high_en as u8) << 3)
                        | ((config.gpo2_inactive_state_high_en as u8) << 2)
                        | ((config.gpo1_inactive_state_high_en as u8) << 1)
                        | (config.gpo0_inactive_state_high_en as u8),
                )),
        )
        .await?;

        self.write_reg(Gpo0RxDelay(config.gpo0_rx_delay_us)).await?;
        self.write_reg(Gpo0TxDelay(config.gpo0_tx_delay_us)).await?;
        self.write_reg(Gpo1RxDelay(config.gpo1_rx_delay_us)).await?;
        self.write_reg(Gpo1TxDelay(config.gpo1_tx_delay_us)).await?;
        self.write_reg(Gpo2RxDelay(config.gpo2_rx_delay_us)).await?;
        self.write_reg(Gpo2TxDelay(config.gpo2_tx_delay_us)).await?;
        self.write_reg(Gpo3RxDelay(config.gpo3_rx_delay_us)).await?;
        self.write_reg(Gpo3TxDelay(config.gpo3_tx_delay_us)).await?;

        // from ad9361.c: GPO manual mode clashes with ENSM slave and eLNA auto mode
        self.modify_reg::<ExternalLnaControl>(|reg| {
            reg.with_gpo_manual_select(config.gpo_manual_mode_en)
        })
        .await?;

        Ok(())
    }

    /// `ad9361_en_dis_tx()`.
    pub(crate) async fn set_tx_channels(&mut self, tx1: bool, tx2: bool) -> Result<(), S::Error> {
        let field = (tx1 as u8) | ((tx2 as u8) << 1);
        self.modify_reg::<TxEnableFilterControl>(|reg| reg.with_tx_channel_enable(u2::new(field)))
            .await
    }

    /// `ad9361_en_dis_rx()`.
    pub(crate) async fn set_rx_channels(&mut self, rx1: bool, rx2: bool) -> Result<(), S::Error> {
        let field = (rx1 as u8) | ((rx2 as u8) << 1);
        self.modify_reg::<RxEnableFilterControl>(|reg| reg.with_rx_channel_enable(u2::new(field)))
            .await
    }

    /// `ad9361_rf_port_setup()` with `is_out = true`. The TX monitor inputs from the C driver
    /// aren't supported.
    pub(crate) async fn rf_port_setup(
        &mut self,
        rx_input: RxInput,
        tx_output: TxOutput,
    ) -> Result<(), S::Error> {
        self.write_reg(
            InputSelect::default()
                .with_rx_input(u6::new(rx_input.select_bits()))
                .with_tx_output(tx_output == TxOutput::B),
        )
        .await
    }

    /// `ad9361_pp_port_setup()` with `restore_c3 = false`.
    pub(crate) async fn pp_port_setup(&mut self, port: &PortConfig) -> Result<(), S::Error> {
        let (lvds_bias, [invert1, invert2]) = port.lvds_registers();
        self.write_reg(port.conf1()).await?;
        self.write_reg(port.conf2()).await?;
        self.write_reg(self.mode.pp_conf3).await?;
        self.write_reg(port.rx_clock_data_delay()).await?;
        self.write_reg(port.tx_clock_data_delay()).await?;
        self.write_reg(lvds_bias).await?;
        self.write_reg(LvdsInvertCtrl1(invert1)).await?;
        self.write_reg(LvdsInvertCtrl2(invert2)).await?;

        if port.invert.rx2 {
            self.modify_reg::<InvertBits>(|reg| reg.with_invert_rx2_rf_dc_cgout_word(false))
                .await?;
        }

        Ok(())
    }

    /// `ad9361_pp_port_setup()` with `restore_c3 = true`.
    pub(crate) async fn pp_port_restore_conf3(&mut self) -> Result<(), S::Error> {
        self.write_reg(self.mode.pp_conf3).await
    }

    /// `ad9361_auxadc_setup()`, off the cached BBPLL rate. `InvalidRate` if that can't be
    /// divided down to the AuxADC clock. The config already checks the initial rate.
    pub(crate) async fn auxadc_setup(
        &mut self,
        config: &AuxAdcConfig,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let bbpll = self.clk.rates.bbpll.to_raw();
        let temp_decimation = config.temp_sensor_decimation.field();
        let aux_decimation = config.auxadc_decimation.field();
        let clock_divider = bbpll
            .checked_div(config.auxadc_clock_rate.to_raw())
            .and_then(|div| u8::try_from(div).ok())
            .filter(|div| *div < 64)
            .ok_or(Ad9361Error::InvalidRate)?;
        // interval is in units of 2^29 BBPLL cycles, rounded
        let interval =
            (config.temp_time_interval_ms as u64 * (bbpll / 1000) as u64 + (1 << 28)) >> 29;

        self.write_reg(TempOffset(config.offset as u8)).await?;
        self.write_reg(StartTempReading::default()).await?;
        self.write_reg(
            TempSense2::default()
                .with_measurement_time_interval(u7::new((interval & 0x7F) as u8))
                .with_temp_sense_periodic_enable(config.periodic_temp_measurement),
        )
        .await?;
        self.write_reg(TempSensorConfig::default().with_temp_sensor_decimation(temp_decimation))
            .await?;
        self.write_reg(
            AuxadcClockDivider::default().with_auxadc_clock_divider(u6::new(clock_divider)),
        )
        .await?;
        self.write_reg(AuxadcConfig::default().with_aux_adc_decimation(aux_decimation))
            .await?;
        Ok(())
    }

    /// `ad9361_ctrl_outs_setup()`.
    pub(crate) async fn ctrl_outs_setup(
        &mut self,
        config: &CtrlOutsConfig,
    ) -> Result<(), S::Error> {
        self.write_reg(ControlOutputPointer(config.index)).await?;
        self.write_reg(ControlOutputEnable::from_raw(config.en_mask))
            .await
    }

    /// `ad9361_set_ref_clk_cycles()`.
    pub(crate) async fn set_ref_clk_cycles(
        &mut self,
        ref_clk: ReferenceClock,
    ) -> Result<(), S::Error> {
        // 1-128 MHz, ReferenceClock guarantees it
        let mhz = ref_clk.get().to_raw() / 1_000_000;
        self.write_reg(
            ReferenceClockCycles::default()
                .with_reference_clock_cycles_per_us(u7::new((mhz - 1) as u8)),
        )
        .await
    }

    /// `ad9361_setup_ext_lna()`.
    pub(crate) async fn setup_ext_lna(&mut self, config: &ElnaConfig) -> Result<(), S::Error> {
        self.modify_reg::<ExternalLnaControl>(|reg| {
            reg.with_external_lna1_ctrl(config.elna_1_control_en)
                .with_external_lna2_ctrl(config.elna_2_control_en)
        })
        .await?;
        self.write_reg(ExtLnaHighGain::default().with_ext_lna_high_gain(config.gain.field()))
            .await?;
        self.write_reg(ExtLnaLowGain::default().with_ext_lna_low_gain(config.bypass_loss.field()))
            .await?;
        Ok(())
    }
}
