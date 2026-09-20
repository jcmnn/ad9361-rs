//! The public API: [`Uninitialized`], [`Configured`] and [`Ad9361`], plus the handles
//! [`ManualRxGain`], [`RxFir`] and [`TxFir`].

use embedded_hal::spi::SpiDevice;

use super::{
    Ad9361ClockRates, Ad9361Config, Ad9361Error, InitError, Ad9361Settings, AxiCores, Channel,
    ConfigError, Engine, GainMode, ProductId, ReferenceClock, RfBandwidth, RxFirConfig, RxLoFrequency, SampleRate, State, TxLoFrequency,
    TxAttenuation, TxFirConfig, spi,
};

/// AD936X that has not been configured.
///
/// Needs:
///
/// - a blocking `SpiDevice` for the chip (mode 1, 10 MHz at most, chip select handled by the
///   device)
/// - the reset pin, held low until [`Configured::init`]
/// - the [`ReferenceClock`] frequency the chip runs on
/// - the [`AxiCores`] of the AXI AD9361 core, taken with `take_adc_no_init()` and
///   `take_dac_no_init()`
///
/// Next is [`Self::configure`].
pub struct Uninitialized<S: SpiDevice<u8>, R: embedded_hal::digital::OutputPin> {
    spi: S,
    resetb: R,
    ref_clk: ReferenceClock,
    axi: AxiCores,
}

impl<S, R> Uninitialized<S, R>
where
    S: SpiDevice<u8>,
    R: embedded_hal::digital::OutputPin,
{
    /// Stores the parts. Nothing is sent to the chip.
    pub fn new(spi: S, resetb: R, ref_clk: ReferenceClock, axi: AxiCores) -> Self {
        Self { spi, resetb, ref_clk, axi }
    }

    /// Checks the settings and returns a [`Configured`] if they are valid. Nothing is sent to the
    /// chip.
    ///
    /// Errors are reported here instead of half way through the setup: path clocks over the chip's
    /// limits, a reference clock that can't reach the synth reference window, settings that don't fit
    /// their register field, and combinations the chip can't do. See [`ConfigError`].
    pub fn configure(self, settings: Ad9361Settings) -> Result<Configured<S, R>, ConfigError> {
        Ok(Configured {
            config: Ad9361Config::new(settings, self.ref_clk)?,
            spi: self.spi,
            resetb: self.resetb,
            axi: self.axi,
        })
    }
}

/// Checked settings, chip not touched yet.
///
/// Call [`Self::init`] to set up the chip.
pub struct Configured<S: SpiDevice<u8>, R: embedded_hal::digital::OutputPin> {
    spi: S,
    resetb: R,
    axi: AxiCores,
    config: Ad9361Config,
}

impl<S, R> Configured<S, R>
where
    S: SpiDevice<u8>,
    R: embedded_hal::digital::OutputPin,
{
    /// Resets and sets up the chip. This is `ad9361_init()` from no-OS plus what its `main.c` does
    /// after it:
    ///
    /// 1. pulse the reset pin and check the product ID
    /// 2. read the clock rates and run the setup (clock chain, synths, ports, gain control,
    ///    baseband filter and DC offset calibrations, TX quadrature calibration, tracking, RSSI)
    /// 3. bring up the ADC core and tune the digital interface
    /// 4. load the FIRs from the settings, still bypassed
    /// 5. bring up the DAC core with the DDS tones as data source
    ///
    /// This takes a while, the calibrations and the tuning wait on the chip.
    ///
    /// # Errors
    ///
    /// [`InitError::UnsupportedDevice`] means the product ID is wrong. That is usually the SPI
    /// setup (mode, clock, wiring, chip select). A calibration or lock timeout usually means the
    /// reference clock or the data interface clock is missing.
    ///
    /// The SPI device and reset pin are consumed. After an error the chip is in an undefined state,
    /// which the reset at the start of the next `init` fixes.
    pub async fn init(mut self) -> Result<Ad9361<S, R>, InitError<S::Error, R::Error>> {
        spi::reset(&mut self.resetb)
            .await
            .map_err(InitError::ResetPin)?;

        let id = spi::read_reg::<S, ProductId>(&mut self.spi).map_err(Ad9361Error::Spi)?;
        if id.product_id().value() != 1 {
            return Err(InitError::UnsupportedDevice { product_id: id.product_id().value() });
        }
        let revision = id.revision().value();
        let ensm_state = spi::read_reg::<S, State>(&mut self.spi)
            .map_err(Ad9361Error::Spi)?
            .ensm_state()
            .value();

        let mut engine = Engine::new(self.spi, self.axi, &self.config, ensm_state).map_err(Ad9361Error::Spi)?;
        engine.init(&self.config).await?;
        if let Some(tx_fir) = &self.config.settings.tx_fir {
            engine.set_tx_fir_config(tx_fir).await.map_err(Ad9361Error::Spi)?;
        }
        if let Some(rx_fir) = &self.config.settings.rx_fir {
            engine.set_rx_fir_config(rx_fir).await.map_err(Ad9361Error::Spi)?;
        }
        engine.dac_init().await?;
        Ok(Ad9361 {
            engine,
            _resetb: self.resetb,
            revision,
        })
    }
}

/// A running chip, returned by [`Configured::init`].
///
/// All methods take `&mut self`. Use a mutex to share it between tasks.
///
/// Change chip settings only through this type. It caches the clock rates and some other state,
/// and registers written behind its back leave the cache wrong.
///
/// Dropping it leaves the chip running as it is.
///
/// ```no_run
/// # use ad9361::{Ad9361, Channel, TxAttenuation};
/// # use embedded_hal::{digital::OutputPin, spi::SpiDevice};
/// # async fn example<S: SpiDevice<u8>, R: OutputPin>(mut ad9361: Ad9361<S, R>) {
/// // less TX power
/// ad9361.set_tx_attenuation(TxAttenuation::from_db(30)).unwrap();
///
/// // fixed RX1 gain instead of AGC
/// ad9361.manual_gain(Channel::Ch1).unwrap().set_gain(40).unwrap();
///
/// // let the DMA feed the DAC instead of the DDS tones
/// ad9361.dac_use_dma_data();
/// # }
/// ```
pub struct Ad9361<S: SpiDevice<u8>, R: embedded_hal::digital::OutputPin> {
    engine: Engine<S>,
    /// only held so the pin stays high
    _resetb: R,
    revision: u8,
}

impl<S, R> Ad9361<S, R>
where
    S: SpiDevice<u8>,
    R: embedded_hal::digital::OutputPin,
{
    /// Silicon revision, read during `init`.
    pub fn revision(&self) -> u8 {
        self.revision
    }

    /// Current clock rates (`rx_sampl` is the RX sample rate, and so on). Cached, so no SPI access.
    pub fn clock_rates(&self) -> &Ad9361ClockRates {
        self.engine.clock_rates()
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

    /// TX1 attenuation, read from the chip. [`Self::set_tx_attenuation`] sets both channels to the
    /// same value.
    pub fn tx_attenuation(&mut self) -> Result<TxAttenuation, Ad9361Error<S::Error>> {
        Ok(self.engine.tx_atten(false)?)
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
    /// a while.
    pub async fn set_tx_lo_frequency(
        &mut self,
        frequency: TxLoFrequency,
    ) -> Result<(), Ad9361Error<S::Error>> {
        self.engine.set_rfpll_rate(true, frequency.get()).await
    }

    /// Sets the RF bandwidths. Slow, since the baseband filters, TIA and ADC are calibrated again,
    /// followed by the TX quadrature calibration.
    pub async fn set_bandwidths(
        &mut self,
        rx: RfBandwidth,
        tx: RfBandwidth,
    ) -> Result<(), Ad9361Error<S::Error>> {
        self.engine.update_rf_bandwidth(rx.get(), tx.get()).await
    }

    /// Sets the TX attenuation on both channels. Takes effect immediately, the output steps.
    pub fn set_tx_attenuation(&mut self, atten: TxAttenuation) -> Result<(), Ad9361Error<S::Error>> {
        Ok(self.engine.set_tx_atten(atten, true, true, true)?)
    }

    /// Gain mode of the channel.
    pub fn gain_mode(&self, channel: Channel) -> GainMode {
        self.engine.gain_mode(channel)
    }

    /// Sets the gain mode of the channel. The channel is switched off briefly during the change.
    ///
    /// Use [`Self::manual_gain`] to go to manual mode.
    pub fn set_gain_mode(
        &mut self,
        channel: Channel,
        mode: GainMode,
    ) -> Result<(), Ad9361Error<S::Error>> {
        self.engine.set_gain_mode(channel, mode)
    }

    /// Switches the channel to manual gain and returns a [`ManualRxGain`] to set the gain.
    ///
    /// Drop the handle and call [`Self::set_gain_mode`] to go back to AGC.
    ///
    /// # Errors
    ///
    /// [`Ad9361Error::UnsupportedGainTable`] with the split gain table.
    pub fn manual_gain(
        &mut self,
        channel: Channel,
    ) -> Result<ManualRxGain<'_, S, R>, Ad9361Error<S::Error>> {
        if self.engine.gain.split_gt {
            return Err(Ad9361Error::UnsupportedGainTable);
        }
        self.engine.set_gain_mode(channel, GainMode::Manual)?;
        Ok(ManualRxGain { driver: self, channel })
    }

    /// Loads RX FIR coefficients and returns the [`RxFir`]. The filter is bypassed until enabled,
    /// like in no-OS.
    pub async fn load_rx_fir(
        &mut self,
        config: &RxFirConfig,
    ) -> Result<RxFir<'_, S, R>, Ad9361Error<S::Error>> {
        self.engine.set_rx_fir_config(config).await?;
        Ok(RxFir { driver: self })
    }

    /// The RX FIR, or `None` if no coefficients are loaded. The default settings load some.
    pub fn rx_fir(&mut self) -> Option<RxFir<'_, S, R>> {
        self.engine.fir.rx.is_some().then_some(RxFir { driver: self })
    }

    /// Loads TX FIR coefficients and returns the [`TxFir`]. The filter is bypassed until enabled,
    /// like in no-OS.
    pub async fn load_tx_fir(
        &mut self,
        config: &TxFirConfig,
    ) -> Result<TxFir<'_, S, R>, Ad9361Error<S::Error>> {
        self.engine.set_tx_fir_config(config).await?;
        Ok(TxFir { driver: self })
    }

    /// The TX FIR, or `None` if no coefficients are loaded. The default settings load some.
    pub fn tx_fir(&mut self) -> Option<TxFir<'_, S, R>> {
        self.engine.fir.tx.is_some().then_some(TxFir { driver: self })
    }

    /// Makes the DAC play the data from its DMA. The data is one 32 bit word per TX channel and
    /// sample, channels interleaved ([`Self::dac_num_tx_channels`]), I in the lower and Q in the
    /// upper half. The DMA has to be running for anything to come out.
    pub fn dac_use_dma_data(&mut self) {
        self.engine.dac_use_dma_data();
    }

    /// Makes the DAC play its DDS tones. This is the state after `init`.
    pub fn dac_use_dds(&mut self) {
        self.engine.dac_use_dds();
    }

    /// Number of TX channels the DMA data is interleaved for: 2 in 2R2T, 1 in 1R1T.
    pub fn dac_num_tx_channels(&self) -> usize {
        self.engine.dac_num_tx_channels()
    }
}

/// An RX channel in manual gain mode, from [`Ad9361::manual_gain`].
///
/// Holds the driver mutably, so the gain mode can't change while it exists.
pub struct ManualRxGain<'a, S: SpiDevice<u8>, R: embedded_hal::digital::OutputPin> {
    driver: &'a mut Ad9361<S, R>,
    channel: Channel,
}

impl<S, R> ManualRxGain<'_, S, R>
where
    S: SpiDevice<u8>,
    R: embedded_hal::digital::OutputPin,
{
    /// Sets the gain in dB and returns the gain that was set.
    ///
    /// The gain table has fixed steps, so the closest entry is used. Values above the table give the
    /// last entry. The table depends on the RX LO, so the same value can give another gain after
    /// [`Ad9361::set_rx_lo_frequency`] changes the band.
    pub fn set_gain(&mut self, gain_db: i8) -> Result<i8, Ad9361Error<S::Error>> {
        Ok(self.driver.engine.set_manual_rx_gain(self.channel, gain_db)?)
    }
}

/// The RX FIR with coefficients loaded, from [`Ad9361::rx_fir`] or [`Ad9361::load_rx_fir`].
pub struct RxFir<'a, S: SpiDevice<u8>, R: embedded_hal::digital::OutputPin> {
    driver: &'a mut Ad9361<S, R>,
}

impl<S, R> RxFir<'_, S, R>
where
    S: SpiDevice<u8>,
    R: embedded_hal::digital::OutputPin,
{
    /// Enables or bypasses the filter. The clocks and bandwidths are redone, which fails if the
    /// filter doesn't fit the sample rate. The filter stays bypassed then.
    pub async fn set_enabled(&mut self, enable: bool) -> Result<(), Ad9361Error<S::Error>> {
        self.driver.engine.set_rx_fir_en_dis(enable).await
    }
}

/// The TX FIR with coefficients loaded, from [`Ad9361::tx_fir`] or [`Ad9361::load_tx_fir`].
pub struct TxFir<'a, S: SpiDevice<u8>, R: embedded_hal::digital::OutputPin> {
    driver: &'a mut Ad9361<S, R>,
}

impl<S, R> TxFir<'_, S, R>
where
    S: SpiDevice<u8>,
    R: embedded_hal::digital::OutputPin,
{
    /// Enables or bypasses the filter. The clocks and bandwidths are redone, which fails if the
    /// filter doesn't fit the sample rate. The filter stays bypassed then.
    pub async fn set_enabled(&mut self, enable: bool) -> Result<(), Ad9361Error<S::Error>> {
        self.driver.engine.set_tx_fir_en_dis(enable).await
    }
}
