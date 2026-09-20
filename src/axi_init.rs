//! AXI ADC/DAC core init plus the `ad9361_init()` sequence. Follows `axi_adc_init()`,
//! `axi_dac_init()` and the DDS setup in no-OS' `main.c`.

use arbitrary_int::u4;
use axi_ad9361::regs::{
    adc::fields::ChannelDataPathControl,
    dac::regs::{Control1, DataSource},
};
use embassy_time::Timer;
use embedded_hal::spi::SpiDevice;
use log::info;

use super::{Ad9361Config, Ad9361Error, AxiCores, Engine};

/// DDS frequency no-OS uses for every tone.
const DEFAULT_DDS_FREQUENCY_HZ: u32 = 3_000_000;
/// DDS scale no-OS uses, in millionths of full scale.
const DEFAULT_DDS_SCALE_MICRO: i32 = 50_000;

/// Clock rate from the core's frequency and ratio registers.
fn core_clock_hz(freq: u32, ratio: u32) -> u64 {
    // wraps on purpose, no-OS does too
    (freq.wrapping_mul(ratio) as u64 * 390_625) >> 8
}

impl AxiCores {
    /// `axi_adc_init()`: reset, enable channels, check status.
    pub async fn adc_init(&mut self, num_channels: u8) -> Result<(), AxiInitError> {
        self.adc.enable();
        for chan in 0..num_channels {
            self.adc.channel_mut(u4::new(chan)).write_data_path_control(
                ChannelDataPathControl::ZERO
                    .with_format_signext(true)
                    .with_format_enable(true)
                    .with_enable(true),
            );
        }
        Timer::after_millis(100).await;

        if self.adc.read_status().raw_value() == 0 {
            return Err(AxiInitError::Status);
        }
        let regs = self.adc.regs_mut();
        let clock_hz = core_clock_hz(regs.read_adc_clock_freq(), regs.read_adc_clock_ratio());
        info!("axi-ad9361 ADC: Successfully initialized ({clock_hz} Hz)");
        Ok(())
    }

    /// `axi_dac_init()` plus `axi_dac_set_datasel(dac, -1, DDS)`. `rate` is the rate register
    /// value, divider minus one.
    pub async fn dac_init(&mut self, num_channels: u8, rate: u8) -> Result<(), AxiInitError> {
        self.dac.enable();
        self.dac
            .set_rate_div(core::num::NonZero::new(rate.saturating_add(1)).expect("nonzero"));
        Timer::after_millis(100).await;

        if self.dac.read_interface_status() == 0 {
            return Err(AxiInitError::Status);
        }
        let regs = self.dac.regs();
        let clock_hz = core_clock_hz(regs.read_status1(), regs.read_interface_clock_ratio());
        info!("axi-ad9361 DAC: Successfully initialized ({clock_hz} Hz)");

        // like axi_dac_data_setup(): two tones per channel, I and Q 90 degrees apart
        for chan in 0..num_channels {
            let phase_mdeg = if chan % 2 == 1 { 0 } else { 90_000 };
            for tone in [chan * 2, chan * 2 + 1] {
                self.dds_set_frequency(tone, DEFAULT_DDS_FREQUENCY_HZ, clock_hz);
                self.dds_set_phase(tone, phase_mdeg);
                self.dds_set_scale(tone, DEFAULT_DDS_SCALE_MICRO);
            }
        }
        self.dac
            .set_data_source_all_channels_up_to(u4::new(num_channels), DataSource::InternalTone);
        Ok(())
    }

    /// Data source for the first `num_channels` DAC channels.
    pub fn dac_set_data_source(&mut self, num_channels: u8, source: DataSource) {
        self.dac
            .set_data_source_all_channels_up_to(u4::new(num_channels), source);
    }

    /// Runs `f` on a DDS tone (two per channel), DAC sync pulsed around it.
    fn with_tone<T>(&mut self, tone: u8, f: impl FnOnce(&mut u32, &mut u32) -> T) -> T {
        self.dac.regs().write_control1(Control1::ZERO);
        let mut channel = self.dac.channel_mut(u4::new(tone >> 1));
        let second = tone & 1 == 1;
        let (mut scale, mut incr) = if second {
            (channel.read_control3(), channel.read_control4())
        } else {
            (channel.read_control1(), channel.read_control2())
        };
        let result = f(&mut scale, &mut incr);
        if second {
            channel.write_control3(scale);
            channel.write_control4(incr);
        } else {
            channel.write_control1(scale);
            channel.write_control2(incr);
        }
        self.dac.synchronize();
        result
    }

    /// `axi_dac_dds_set_frequency()`.
    fn dds_set_frequency(&mut self, tone: u8, freq_hz: u32, dac_clock_hz: u64) {
        let increment = (freq_hz as u64 * 0xFFFF / dac_clock_hz.max(1)) as u32 & 0xFFFF;
        self.with_tone(tone, |_, incr| *incr = (*incr & !0xFFFF) | increment | 1);
    }

    /// `axi_dac_dds_set_phase()`, phase in millidegrees.
    fn dds_set_phase(&mut self, tone: u8, phase_mdeg: u32) {
        let init = ((phase_mdeg as u64 * 0x10000 + 360_000 / 2) / 360_000) as u32 & 0xFFFF;
        self.with_tone(tone, |_, incr| *incr = (*incr & 0xFFFF) | (init << 16));
    }

    /// `axi_dac_dds_set_scale()`. Millionths of full scale, negative inverts.
    fn dds_set_scale(&mut self, tone: u8, scale_micro: i32) {
        let magnitude = scale_micro.unsigned_abs().min(1_999_000);
        let mut value = (magnitude as u64 * 0x4000 / 1_000_000) as u32;
        if scale_micro < 0 {
            value |= 0x8000;
        }
        self.with_tone(tone, |scale, _| *scale = value & 0xFFFF);
    }
}

/// AXI core init errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxiInitError {
    /// Core reports errors, usually no clock or no lock on the interface
    Status,
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    /// 4 for 2R2T, 2 for 1R1T.
    fn axi_num_channels(&self) -> u8 {
        if self.mode.rx2tx2 { 4 } else { 2 }
    }

    /// The part of `ad9361_init()` after reset and the ID check.
    pub(super) async fn init(&mut self, config: &Ad9361Config) -> Result<(), Ad9361Error<S::Error>> {
        self.setup(config).await?;

        let num_channels = self.axi_num_channels();
        let axi = &mut self.axi;
        axi.adc_init(num_channels)
            .await
            .map_err(|_| Ad9361Error::AxiStatus)?;
        self.post_setup().await
    }

    /// DAC core init, DDS as source. What `main.c` in no-OS does.
    ///
    pub async fn dac_init(&mut self) -> Result<(), Ad9361Error<S::Error>> {
        let (num_channels, rate) = if self.mode.rx2tx2 { (4, 3) } else { (2, 1) };
        let axi = &mut self.axi;
        axi.dac_init(num_channels, rate)
            .await
            .map_err(|_| Ad9361Error::AxiStatus)?;
        Ok(())
    }

    /// DAC takes data from the DMA.
    pub fn dac_use_dma_data(&mut self) {
        let num_channels = self.axi_num_channels();
        let axi = &mut self.axi;
        axi.dac_set_data_source(num_channels, DataSource::InputData);
    }

    /// Back to the DDS tones.
    pub fn dac_use_dds(&mut self) {
        let num_channels = self.axi_num_channels();
        let axi = &mut self.axi;
        axi.dac_set_data_source(num_channels, DataSource::InternalTone);
    }

    /// TX channels the DMA data is interleaved for.
    pub fn dac_num_tx_channels(&self) -> usize {
        self.axi_num_channels() as usize / 2
    }
}
