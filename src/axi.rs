//! [`DataInterface`] for ADI's AXI AD9361 IP core, through the `axi-ad9361` crate. Follows
//! `axi_adc_init()`, `axi_dac_init()` and the AXI parts of `ad9361_post_setup()` and
//! `ad9361_dig_tune_tx()` in no-OS.

use arbitrary_int::u4;
use axi_ad9361::{
    adc::Adc,
    dac::Dac,
    dds::{DdsScale, IqPair},
    regs::{
        adc::fields::{ChannelDataPathControl, ChannelStatus, PnSel},
        dac::regs::{ChannelDataSource, ChannelLegacyControl, DataSource},
        fields::R1Mode,
    },
};
use embassy_time::Timer;
use fugit::HertzU32;
use log::info;

use super::interface::{DataInterface, InterfaceError, PortLayout, TxSource};

/// Tone [`AxiCores::start_tx`] leaves on the DAC, relative to the TX LO.
const DEFAULT_TONE_FREQUENCY: HertzU32 = HertzU32::Hz(3_000_000);
const DEFAULT_TONE_SCALE: DdsScale = DdsScale::from_fraction(0.1);

/// I and Q for each of the two channels.
const MAX_LANES: usize = 4;

/// The ADC and DAC halves of the AXI AD9361 IP core, as handed out by `axi-ad9361`.
///
/// Take them with `take_adc_no_init()` and `take_dac_no_init()`. The core has no clock until the
/// AD9361 runs, so [`Ad9361::init`](crate::Ad9361::init) brings them up. After that the DAC
/// plays a DDS tone 3 MHz above the TX LO at 10% of full scale; change it through
/// [`Ad9361::interface_mut`](crate::Ad9361::interface_mut) with `dac.set_tone()`, and pass
/// [`Ad9361::sample_rate`](crate::Ad9361::sample_rate) along.
pub struct AxiCores {
    pub adc: Adc,
    pub dac: Dac,
    /// What [`DataInterface::start_tx_pn`] changed
    saved: [SavedLane; MAX_LANES],
}

#[derive(Clone, Copy)]
struct SavedLane {
    adc: ChannelDataPathControl,
    dac_source: ChannelDataSource,
    dac_legacy: ChannelLegacyControl,
}

impl AxiCores {
    pub fn new(adc: Adc, dac: Dac) -> Self {
        let lane = SavedLane {
            adc: ChannelDataPathControl::ZERO,
            dac_source: ChannelDataSource::default(),
            dac_legacy: ChannelLegacyControl::default(),
        };
        Self {
            adc,
            dac,
            saved: [lane; MAX_LANES],
        }
    }
}

/// Clock rate from the core's frequency and ratio registers.
fn core_clock_hz(freq: u32, ratio: u32) -> u64 {
    // wraps on purpose, no-OS does too
    (freq.wrapping_mul(ratio) as u64 * 390_625) >> 8
}

fn r1_mode(layout: PortLayout) -> R1Mode {
    if layout.two_channels {
        R1Mode::TwoChannels
    } else {
        R1Mode::OneChannel
    }
}

impl DataInterface for AxiCores {
    async fn init(&mut self, layout: PortLayout) -> Result<(), InterfaceError> {
        // axi_adc_init()
        self.adc.enable();
        for lane in 0..layout.lanes() {
            self.adc.channel_mut(u4::new(lane)).write_data_path_control(
                ChannelDataPathControl::ZERO
                    .with_format_signext(true)
                    .with_format_enable(true)
                    .with_enable(true),
            );
        }
        Timer::after_millis(100).await;
        if self.adc.read_status().raw_value() == 0 {
            return Err(InterfaceError);
        }
        let regs = self.adc.regs_mut();
        let clock_hz = core_clock_hz(regs.read_adc_clock_freq(), regs.read_adc_clock_ratio());
        info!("axi-ad9361 ADC: Successfully initialized ({clock_hz} Hz)");

        // the AXI part of ad9361_post_setup()
        self.adc
            .regs_mut()
            .modify_adc_control1(|reg| reg.with_r1_mode(r1_mode(layout)));
        self.dac
            .regs()
            .modify_control2(|reg| reg.with_r1_mode(r1_mode(layout)));
        let rate = match (layout.two_channels, layout.half_tx_rate, layout.lvds) {
            (false, false, lvds) => lvds as u8,
            (false, true, _) => 0,
            (true, false, true) => 3,
            (true, false, false) | (true, true, _) => 1,
        };
        self.dac
            .set_rate_div(core::num::NonZero::new(rate + 1).expect("rate divider is nonzero"));
        for lane in 0..layout.lanes() {
            // no DC filter offset. enabling the channel also sets the IQ correction coefficients
            self.adc.channel_mut(u4::new(lane)).write_control1(0);
            self.adc.enable_channel(u4::new(lane));
        }
        Ok(())
    }

    async fn start_tx(
        &mut self,
        layout: PortLayout,
        sample_rate: HertzU32,
    ) -> Result<(), InterfaceError> {
        // axi_dac_init()
        let rate_div = if layout.two_channels { 4 } else { 2 };
        self.dac.enable();
        self.dac
            .set_rate_div(core::num::NonZero::new(rate_div).expect("rate divider is nonzero"));
        Timer::after_millis(100).await;
        if self.dac.read_interface_status() == 0 {
            return Err(InterfaceError);
        }
        let regs = self.dac.regs();
        let clock_hz = core_clock_hz(regs.read_status1(), regs.read_interface_clock_ratio());
        info!("axi-ad9361 DAC: Successfully initialized ({clock_hz} Hz)");

        self.set_tx_source(layout, TxSource::Dds);
        let pairs: &[IqPair] = if layout.two_channels {
            &[IqPair::First, IqPair::Second]
        } else {
            &[IqPair::First]
        };
        for &pair in pairs {
            // fails below 6 MSPS, and then there's just no tone
            let _ = self.dac.set_tone(
                pair,
                DEFAULT_TONE_FREQUENCY,
                DEFAULT_TONE_SCALE,
                sample_rate,
            );
        }
        Ok(())
    }

    fn set_tx_source(&mut self, layout: PortLayout, source: TxSource) {
        let source = match source {
            TxSource::Dds => DataSource::InternalTone,
            TxSource::Dma => DataSource::InputData,
        };
        self.dac
            .set_data_source_all_channels_up_to(u4::new(layout.lanes()), source);
    }

    fn clear_pn_errors(&mut self, layout: PortLayout) {
        let clear = ChannelStatus::default()
            .with_pn_error(true)
            .with_pn_out_of_sync(true);
        for lane in 0..layout.lanes() {
            self.adc.channel_mut(u4::new(lane)).write_status(clear);
        }
    }

    fn pn_errors(&mut self, layout: PortLayout, require_rx_lock: bool) -> bool {
        if require_rx_lock && !self.adc.read_status().locked() {
            return true;
        }
        (0..layout.lanes()).any(|lane| {
            self.adc
                .channel_mut(u4::new(lane))
                .read_status()
                .raw_value()
                != 0
        })
    }

    fn relatch_rx(&mut self) {
        self.adc.reset_pulse();
    }

    fn start_tx_pn(&mut self, layout: PortLayout) {
        self.dac.release_reset();
        for lane in 0..layout.lanes() {
            let saved = &mut self.saved[lane as usize];
            let mut adc = self.adc.channel_mut(u4::new(lane));
            saved.adc = adc.read_data_path_control();
            adc.write_data_path_control(
                ChannelDataPathControl::ZERO
                    .with_format_signext(true)
                    .with_format_enable(true)
                    .with_enable(true)
                    .with_iqcor_enable(true),
            );
            adc.modify_pn_select(|reg| reg.with_pn_sel(PnSel::PnCustom));

            let mut dac = self.dac.channel_mut(u4::new(lane));
            saved.dac_legacy = dac.read_legacy_control();
            saved.dac_source = dac.read_data_source();
            dac.write_data_source(
                ChannelDataSource::builder()
                    .with_data_source(DataSource::PnX)
                    .build(),
            );
            // IQ correction off
            dac.write_legacy_control(ChannelLegacyControl::default());
            self.dac.synchronize();
        }
    }

    fn stop_tx_pn(&mut self, layout: PortLayout) {
        for lane in 0..layout.lanes() {
            let saved = self.saved[lane as usize];
            let mut adc = self.adc.channel_mut(u4::new(lane));
            adc.write_data_path_control(saved.adc);
            adc.modify_pn_select(|reg| reg.with_pn_sel(PnSel::Pn9));

            self.dac
                .channel_mut(u4::new(lane))
                .write_data_source(saved.dac_source);
            self.dac.synchronize();
            self.dac
                .channel_mut(u4::new(lane))
                .write_legacy_control(saved.dac_legacy);
        }
    }
}
