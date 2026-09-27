//! The public API: [`Ad9361`] and its handles [`ManualRxGain`], [`RxFir`] and [`TxFir`].

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiDevice;
use fugit::{HertzU32, HertzU64};

use super::interface::{DataInterface, TxSource};
use super::{
    Ad9361ClockRates, Ad9361Config, Ad9361Error, Channel, Channels, DcxoTrim, Engine, GainMode,
    InitError, InitFailure, ProductId, RfBandwidth, RxFirConfig, RxLoFrequency, SampleRate, State,
    TxAttenuation, TxFirConfig, TxLoFrequency, spi,
};

/// A running AD9361, made by [`Self::init`].
///
/// All methods take `&mut self`. Use a mutex to share it between tasks.
///
/// Change chip settings only through this type. It caches the clock rates and some other state,
/// and registers written behind its back leave the cache wrong.
///
/// Dropping it leaves the chip running as it is.
///
/// ```no_run
/// # #[cfg(feature = "axi")] mod example {
/// # use ad9361::{Ad9361, AxiCores, Channel, Channels, TxAttenuation, interface::TxSource};
/// # use axi_ad9361::dds::{DdsScale, IqPair};
/// # use embedded_hal::digital::OutputPin;
/// # use embedded_hal_async::spi::SpiDevice;
/// # use fugit::HertzU32;
/// # async fn example<S: SpiDevice<u8>, R: OutputPin>(mut ad9361: Ad9361<S, R, AxiCores>) {
/// // less TX power on both channels
/// ad9361.set_tx_attenuation(Channels::Both, TxAttenuation::from_db(30)).await.unwrap();
///
/// // a tone 1 MHz above the TX LO on TX1, at half of full scale
/// let sample_rate = ad9361.sample_rate();
/// let dac = &mut ad9361.interface_mut().dac;
/// dac.set_tone(IqPair::First, HertzU32::MHz(1), DdsScale::from_fraction(0.5), sample_rate).unwrap();
///
/// // fixed RX1 gain instead of AGC
/// ad9361.manual_gain(Channel::Ch1).await.unwrap().set_gain(40).await.unwrap();
///
/// // let the DMA feed the DAC instead of the DDS
/// ad9361.set_tx_source(TxSource::Dma);
/// # }
/// # }
/// ```
pub struct Ad9361<S: SpiDevice<u8>, R: OutputPin, I: DataInterface> {
    engine: Engine<S, I>,
    /// only held so the pin stays high
    _resetb: R,
    revision: u8,
}

impl<S, R, I> Ad9361<S, R, I>
where
    S: SpiDevice<u8>,
    R: OutputPin,
    I: DataInterface,
{
    /// Resets the chip and sets it up (`ad9361_init()` in no-OS):
    ///
    /// 1. pulse the reset pin and check the product ID
    /// 2. read the clock rates and run the setup (clock chain, synths, ports, gain control,
    ///    baseband filter and DC offset calibrations, TX quadrature calibration, tracking, RSSI)
    /// 3. bring up the RX side of the data `interface` and tune the data port
    /// 4. load the FIRs from the settings, still bypassed
    /// 5. bring up the TX side of the data `interface`, which picks the TX data source. With
    ///    [`AxiCores`](crate::AxiCores) that is a DDS tone 3 MHz above the TX LO, at 10% of
    ///    full scale.
    ///
    /// Needs:
    ///
    /// - an SPI device for the chip, mode 1 and 10 MHz at most, chip select handled by the
    ///   device
    /// - the reset pin, held low until now. The driver keeps it high afterwards
    /// - the FPGA side of the data port, like [`AxiCores`](crate::AxiCores)
    /// - the validated settings. `config` isn't consumed, so a failed init can be retried
    ///
    /// This takes a while, the calibrations and the tuning wait on the chip.
    ///
    /// # Errors
    ///
    /// The [`InitFailure`] hands back the SPI device, reset pin and interface. The chip is then
    /// in an undefined state, which the reset at the start of the next `init` fixes.
    /// [`InitError::UnsupportedDevice`] means the product ID is wrong, usually the SPI setup
    /// (mode, clock, wiring, chip select). A calibration or lock timeout usually means the
    /// reference clock or the data clock is missing.
    pub async fn init(
        mut spi: S,
        mut resetb: R,
        interface: I,
        config: &Ad9361Config,
    ) -> Result<Self, InitFailure<S, R, I>> {
        let early = async |spi: &mut S, resetb: &mut R| {
            spi::reset(resetb).await.map_err(InitError::ResetPin)?;
            let id = spi::read_reg::<S, ProductId>(spi).await.map_err(Ad9361Error::Spi)?;
            if id.product_id().value() != 1 {
                return Err(InitError::UnsupportedDevice { product_id: id.product_id().value() });
            }
            let ensm_state = spi::read_reg::<S, State>(spi).await.map_err(Ad9361Error::Spi)?;
            Ok((id.revision().value(), ensm_state.ensm_state().value()))
        };
        let (revision, ensm_state) = match early(&mut spi, &mut resetb).await {
            Ok(result) => result,
            Err(error) => return Err(InitFailure { error, spi, resetb, interface }),
        };

        let mut engine = Engine::new(spi, interface, config, ensm_state);
        let result = async {
            // no-OS calls this clks_resync
            engine.init_clocks().await?;
            engine.init(config).await
        }
        .await;
        match result {
            Ok(()) => Ok(Ad9361 { engine, _resetb: resetb, revision }),
            Err(error) => {
                let (spi, interface) = engine.into_parts();
                Err(InitFailure { error: InitError::Chip(error), spi, resetb, interface })
            }
        }
    }

    /// Silicon revision, read during `init`.
    pub fn revision(&self) -> u8 {
        self.revision
    }

    /// Every clock rate of the chip, as it really runs. Cached, so no SPI access.
    pub fn clock_rates(&self) -> &Ad9361ClockRates {
        self.engine.clock_rates()
    }

    /// RX sample rate, which is also the TX one. The rate the dividers make, can be a little off
    /// the one asked for.
    pub fn sample_rate(&self) -> HertzU32 {
        self.engine.clock_rates().rx_sample
    }

    /// The RX LO the synth really runs at. The fractional-N synth gets within a few Hz of the
    /// requested frequency, relative to the reference clock.
    pub fn rx_lo_frequency(&self) -> HertzU64 {
        self.engine.clock_rates().rx_rfpll
    }

    /// The TX LO the synth really runs at, see [`Self::rx_lo_frequency`].
    pub fn tx_lo_frequency(&self) -> HertzU64 {
        self.engine.clock_rates().tx_rfpll
    }

    /// Trims the crystal oscillator (DCXO), to pull the reference clock and with it every
    /// frequency the chip makes, including the LOs and the sample rate. Does nothing with [`ReferenceSource::External`](crate::ReferenceSource::External).
    /// Takes effect immediately.
    pub async fn set_dcxo_trim(&mut self, trim: DcxoTrim) -> Result<(), Ad9361Error<S::Error>> {
        Ok(self.engine.set_dcxo_trim(trim).await?)
    }

    /// Sets the RX and TX sample rate and retunes the digital interface
    /// (`ad9361_set_trx_clock_chain_freq()` in no-OS).
    ///
    /// Can fail with [`Ad9361Error::InvalidRate`] even for a [`SampleRate`] in range, when the dividers
    /// can't make it with the current FIR settings. Low rates usually need a FIR with
    /// interpolation or decimation.
    pub async fn set_sample_rate(&mut self, rate: SampleRate) -> Result<(), Ad9361Error<S::Error>> {
        self.engine.set_trx_clock_chain_freq(rate.get()).await
    }

    /// Sets the RX LO and loads the gain table for the new band. Waits for the synth to lock and can
    /// time out.
    pub async fn set_rx_lo_frequency(
        &mut self,
        frequency: RxLoFrequency,
    ) -> Result<(), Ad9361Error<S::Error>> {
        self.engine.set_rfpll_rate(false, frequency.get()).await
    }

    /// Sets the TX LO. A move of more than 100 MHz reruns the TX quadrature calibration, which takes
    /// a while. The calibration corrects TX DC offset, gain and phase error, and is also worth
    /// rerunning if the chip temperature changes a lot.
    pub async fn set_tx_lo_frequency(
        &mut self,
        frequency: TxLoFrequency,
    ) -> Result<(), Ad9361Error<S::Error>> {
        self.engine.set_rfpll_rate(true, frequency.get()).await
    }

    /// Sets the RF bandwidths. Slow, since the baseband filters, TIA and ADC are calibrated again,
    /// followed by the TX quadrature calibration.
    ///
    /// The filter corners come from dividing the BBPLL, so the bandwidth actually set can be a bit
    /// off the requested one, more so for narrow filters.
    pub async fn set_bandwidths(
        &mut self,
        rx: RfBandwidth,
        tx: RfBandwidth,
    ) -> Result<(), Ad9361Error<S::Error>> {
        self.engine.update_rf_bandwidth(rx.get(), tx.get()).await
    }

    /// Gain mode of the channel.
    pub fn gain_mode(&self, channel: Channel) -> GainMode {
        self.engine.gain_mode(channel)
    }

    /// Loads RX FIR coefficients and returns the [`RxFir`]. The filter is bypassed until enabled.
    pub async fn load_rx_fir(
        &mut self,
        config: &RxFirConfig,
    ) -> Result<RxFir<'_, S, R, I>, Ad9361Error<S::Error>> {
        self.engine.set_rx_fir_config(config).await?;
        Ok(RxFir { driver: self })
    }

    /// The RX FIR, or `None` if no coefficients are loaded. The default settings load some.
    pub fn rx_fir(&mut self) -> Option<RxFir<'_, S, R, I>> {
        self.engine.fir.rx.is_some().then_some(RxFir { driver: self })
    }

    /// Loads TX FIR coefficients and returns the [`TxFir`]. The filter is bypassed until enabled.
    pub async fn load_tx_fir(
        &mut self,
        config: &TxFirConfig,
    ) -> Result<TxFir<'_, S, R, I>, Ad9361Error<S::Error>> {
        self.engine.set_tx_fir_config(config).await?;
        Ok(TxFir { driver: self })
    }

    /// The TX FIR, or `None` if no coefficients are loaded. The default settings load some.
    pub fn tx_fir(&mut self) -> Option<TxFir<'_, S, R, I>> {
        self.engine.fir.tx.is_some().then_some(TxFir { driver: self })
    }

    /// Picks where the TX data comes from. [`TxSource::Dds`] is the state after `init`.
    ///
    /// With [`AxiCores`](crate::AxiCores) and [`TxSource::Dma`], the data is one 32 bit word per
    /// TX channel and sample, channels interleaved ([`Self::dac_num_tx_channels`]), I in the
    /// lower and Q in the upper half. The DMA has to be running for anything to come out.
    pub fn set_tx_source(&mut self, source: TxSource) {
        self.engine.set_tx_source(source);
    }

    /// The FPGA side of the data port.
    pub fn interface(&self) -> &I {
        &self.engine.interface
    }

    /// The FPGA side of the data port, for things the driver doesn't cover, like the DDS
    /// tones of [`AxiCores`](crate::AxiCores). Leave the data path setup alone, the driver
    /// tuned the port for it.
    pub fn interface_mut(&mut self) -> &mut I {
        &mut self.engine.interface
    }

    /// Number of TX channels the DMA data is interleaved for: 2 in 2R2T, 1 in 1R1T.
    pub fn dac_num_tx_channels(&self) -> usize {
        self.engine.dac_num_tx_channels()
    }

    /// TX attenuation of `channel`, read from the chip.
    pub async fn tx_attenuation(&mut self, channel: Channel) -> Result<TxAttenuation, Ad9361Error<S::Error>> {
        Ok(self.engine.tx_atten(channel == Channel::Ch2).await?)
    }

    /// Sets the TX attenuation, 0 to 89.75 dB. Takes effect immediately, so the output steps.
    pub async fn set_tx_attenuation(
        &mut self,
        channels: Channels,
        atten: TxAttenuation,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let (tx1, tx2) = (channels.contains(Channel::Ch1), channels.contains(Channel::Ch2));
        Ok(self.engine.set_tx_atten(atten, tx1, tx2, true).await?)
    }

    /// Sets the gain mode of the channel. The channel is switched off briefly during the change.
    ///
    /// Use [`Self::manual_gain`] to go to manual mode.
    pub async fn set_gain_mode(
        &mut self,
        channel: Channel,
        mode: GainMode,
    ) -> Result<(), Ad9361Error<S::Error>> {
        self.engine.set_gain_mode(channel, mode).await
    }

    /// Switches the channel to manual gain and returns a [`ManualRxGain`] to set the gain.
    ///
    /// Drop the handle and call [`Self::set_gain_mode`] to go back to AGC.
    ///
    /// # Errors
    ///
    /// [`Ad9361Error::UnsupportedGainTable`] with the split gain table.
    pub async fn manual_gain(
        &mut self,
        channel: Channel,
    ) -> Result<ManualRxGain<'_, S, R, I>, Ad9361Error<S::Error>> {
        if self.engine.gain.split_gt {
            return Err(Ad9361Error::UnsupportedGainTable);
        }
        self.engine.set_gain_mode(channel, GainMode::Manual).await?;
        Ok(ManualRxGain { driver: self, channel })
    }
}

/// An RX channel in manual gain mode, from [`Ad9361::manual_gain`].
///
/// Holds the driver mutably, so the gain mode can't change while it exists.
pub struct ManualRxGain<'a, S: SpiDevice<u8>, R: OutputPin, I: DataInterface> {
    driver: &'a mut Ad9361<S, R, I>,
    channel: Channel,
}

impl<S, R, I> ManualRxGain<'_, S, R, I>
where
    S: SpiDevice<u8>,
    R: OutputPin,
    I: DataInterface,
{
    /// Sets the gain in dB and returns the gain that was set.
    ///
    /// The gain table has fixed steps, so the closest entry is used. Values above the table give the
    /// last entry. The table depends on the RX LO, so the same value can give another gain after
    /// [`Ad9361::set_rx_lo_frequency`] changes the band.
    pub async fn set_gain(&mut self, gain_db: i8) -> Result<i8, Ad9361Error<S::Error>> {
        Ok(self.driver.engine.set_manual_rx_gain(self.channel, gain_db).await?)
    }
}

/// The RX FIR with coefficients loaded, from [`Ad9361::rx_fir`] or [`Ad9361::load_rx_fir`].
pub struct RxFir<'a, S: SpiDevice<u8>, R: OutputPin, I: DataInterface> {
    driver: &'a mut Ad9361<S, R, I>,
}

impl<S, R, I> RxFir<'_, S, R, I>
where
    S: SpiDevice<u8>,
    R: OutputPin,
    I: DataInterface,
{
    /// Enables or bypasses the filter. The clocks and bandwidths are redone, which fails if the
    /// filter doesn't fit the sample rate. The filter stays bypassed then.
    pub async fn set_enabled(&mut self, enable: bool) -> Result<(), Ad9361Error<S::Error>> {
        self.driver.engine.set_rx_fir_en_dis(enable).await
    }
}

/// The TX FIR with coefficients loaded, from [`Ad9361::tx_fir`] or [`Ad9361::load_tx_fir`].
pub struct TxFir<'a, S: SpiDevice<u8>, R: OutputPin, I: DataInterface> {
    driver: &'a mut Ad9361<S, R, I>,
}

impl<S, R, I> TxFir<'_, S, R, I>
where
    S: SpiDevice<u8>,
    R: OutputPin,
    I: DataInterface,
{
    /// Enables or bypasses the filter. The clocks and bandwidths are redone, which fails if the
    /// filter doesn't fit the sample rate. The filter stays bypassed then.
    pub async fn set_enabled(&mut self, enable: bool) -> Result<(), Ad9361Error<S::Error>> {
        self.driver.engine.set_tx_fir_en_dis(enable).await
    }
}
