//! RX/TX FIR loading and enabling (`ad9361_set_rx_fir_config()` and friends).
//!
//! no-OS verifies the coefficients by reading them back, but only in debug builds. Not done here.

use arbitrary_int::{u2, u3, u10};
use embedded_hal::spi::SpiDevice;
use fugit::HertzU32;

use super::{
    state::LoadedFir,
    OutOfRange, ForcedEnsmState, Engine, Ad9361Error, DigTuneFlags, Register, RxEnableFilterControl, RxFilterGain,
    TxEnableFilterControl, TxFilterCoefAddr, TxFilterCoefReadData2, TxFilterCoefWriteData1,
    TxFilterCoefWriteData2, TxFilterConf,
};

/// RX FIR registers sit this far above the TX ones.
const RX_FIR_REG_OFFSET: u16 = 0x90;

/// Band pass from the no-OS example, 3/20 fs to 1/4 fs, 64 taps.
pub const DEFAULT_FIR_COEFFICIENTS: [i16; 64] = [
    -4, -6, -37, 35, 186, 86, -284, -315,
    107, 219, -4, 271, 558, -307, -1182, -356,
    658, 157, 207, 1648, 790, -2525, -2553, 748,
    865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
    14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
    748, -2553, -2525, 790, 1648, 207, 157, 658,
    -356, -1182, -307, 558, 271, -4, 219, 107,
    -315, -284, 86, 186, 35, -37, -6, -4,
];

/// FIR gain, RX or TX.
#[derive(Clone, Copy, PartialEq, Eq)]
enum FirGain {
    Rx(RxFirGain),
    Tx(TxFirGain),
}

/// Channels a FIR is loaded for. Both channels share the coefficients.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FirChannels {
    Ch1 = 1,
    Ch2 = 2,
    Both = 3,
}

/// RX FIR decimation or TX FIR interpolation. `X1` is no rate change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FirFactor {
    X1 = 1,
    X2 = 2,
    X4 = 4,
}

/// FIR taps, 16 to 128 of them in steps of 16. `new` rejects any other length. The taps are
/// 16-bit signed, and the DC gain is set separately with `RxFirGain` or `TxFirGain`.
#[derive(Clone, Copy, Debug)]
pub struct FirCoefficients(&'static [i16]);

impl FirCoefficients {
    /// The no-OS example filter.
    pub const DEFAULT: Self = Self(&DEFAULT_FIR_COEFFICIENTS);

    pub const fn new(taps: &'static [i16]) -> Result<Self, OutOfRange> {
        if taps.is_empty() || taps.len() > 128 || !taps.len().is_multiple_of(16) {
            return Err(OutOfRange);
        }
        Ok(Self(taps))
    }

    pub const fn taps(&self) -> &'static [i16] {
        self.0
    }
}

/// RX FIR gain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RxFirGain {
    Minus12Db,
    Minus6Db,
    ZeroDb,
    Plus6Db,
}

impl RxFirGain {
    /// Register field value.
    fn field(self) -> u8 {
        match self {
            RxFirGain::Minus12Db => 3,
            RxFirGain::Minus6Db => 2,
            RxFirGain::ZeroDb => 1,
            RxFirGain::Plus6Db => 0,
        }
    }
}

/// TX FIR gain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxFirGain {
    Minus6Db,
    ZeroDb,
}

/// RX FIR settings. Load with `Ad9361::load_rx_fir`. The filter stays bypassed after loading
/// until it is enabled.
#[derive(Clone, Copy, Debug)]
pub struct RxFirConfig {
    pub channels: FirChannels,
    pub gain: RxFirGain,
    pub decimation: FirFactor,
    pub coefficients: FirCoefficients,
}

impl Default for RxFirConfig {
    /// The no-OS example filter.
    fn default() -> Self {
        Self {
            channels: FirChannels::Both,
            gain: RxFirGain::ZeroDb,
            decimation: FirFactor::X1,
            coefficients: FirCoefficients::DEFAULT,
        }
    }
}

/// TX FIR settings, loaded with `Ad9361::load_tx_fir`. Use `new` because the tap count is
/// limited by the interpolation. With no interpolation the chip has room for 64 taps.
#[derive(Clone, Copy, Debug)]
pub struct TxFirConfig {
    channels: FirChannels,
    gain: TxFirGain,
    interpolation: FirFactor,
    coefficients: FirCoefficients,
}

impl TxFirConfig {
    /// No interpolation means 64 taps at most.
    pub const fn new(
        channels: FirChannels,
        gain: TxFirGain,
        interpolation: FirFactor,
        coefficients: FirCoefficients,
    ) -> Result<Self, OutOfRange> {
        if matches!(interpolation, FirFactor::X1) && coefficients.taps().len() > 64 {
            return Err(OutOfRange);
        }
        Ok(Self { channels, gain, interpolation, coefficients })
    }
}

impl Default for TxFirConfig {
    /// The no-OS example filter.
    fn default() -> Self {
        Self {
            channels: FirChannels::Both,
            gain: TxFirGain::Minus6Db,
            interpolation: FirFactor::X1,
            coefficients: FirCoefficients::DEFAULT,
        }
    }
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    fn fir_addr<Reg: Register>(rx: bool) -> u10 {
        u10::new(Reg::ADDRESS.value() + if rx { RX_FIR_REG_OFFSET } else { 0 })
    }

    fn write_fir_reg<Reg: Register>(&mut self, rx: bool, reg: Reg) -> Result<(), S::Error> {
        self.write_bytes(&[reg.to_raw()], Self::fir_addr::<Reg>(rx))
    }

    /// `ad9361_load_fir_filter_coef()`. The filter clock runs at `factor` while the
    /// coefficients get written.
    async fn load_fir_filter_coef(
        &mut self,
        rx: bool,
        channels: FirChannels,
        gain: FirGain,
        factor: FirFactor,
        coefficients: FirCoefficients,
    ) -> Result<(), S::Error> {
        let saved_ensm = self.ensm_force_state(ForcedEnsmState::Alert).await?;
        let result = self.write_fir_coefficients(rx, channels, gain, factor, coefficients.taps());
        self.ensm_restore_state(saved_ensm)?;
        result
    }

    fn write_fir_coefficients(
        &mut self,
        rx: bool,
        channels: FirChannels,
        gain: FirGain,
        factor: FirFactor,
        coef: &[i16],
    ) -> Result<(), S::Error> {
        let ntaps = coef.len();
        let mut conf = TxFilterConf::default();
        // field is the factor, with 4 encoded as 3. Clock has to run while loading
        let clock_field = u2::new(if factor == FirFactor::X4 { 3 } else { factor as u8 & 0x3 });
        let fir_enable;
        if rx {
            let FirGain::Rx(gain) = gain else { unreachable!("RX FIRs have RX gains") };
            self.write_reg(RxFilterGain::default().with_filter_gain(u2::new(gain.field())))?;
            fir_enable = self
                .read_reg::<RxEnableFilterControl>()?
                .rx_fir_enable_decimation();
            let field = clock_field;
            self.modify_reg::<RxEnableFilterControl>(|reg| {
                reg.with_rx_fir_enable_decimation(field)
            })?;
        } else {
            conf = conf.with_tx_fir_gain_6db(gain == FirGain::Tx(TxFirGain::Minus6Db));
            fir_enable = self
                .read_reg::<TxEnableFilterControl>()?
                .tx_fir_enable_interpolation();
            let field = clock_field;
            self.modify_reg::<TxEnableFilterControl>(|reg| {
                reg.with_tx_fir_enable_interpolation(field)
            })?;
        }

        conf = conf
            .with_fir_num_taps(u3::new((ntaps / 16 - 1) as u8))
            .with_fir_select(u2::new(channels as u8))
            .with_fir_start_clk(true);
        self.write_fir_reg(rx, conf)?;

        for (i, tap) in coef.iter().enumerate() {
            self.write_fir_reg(rx, TxFilterCoefAddr(i as u8))?;
            self.write_fir_reg(rx, TxFilterCoefWriteData1(*tap as u8))?;
            self.write_fir_reg(rx, TxFilterCoefWriteData2((*tap >> 8) as u8))?;
            self.write_fir_reg(rx, conf.with_fir_write(true))?;
            // dummy writes, the write needs time
            self.write_fir_reg(rx, TxFilterCoefReadData2(0))?;
            self.write_fir_reg(rx, TxFilterCoefReadData2(0))?;
        }

        self.write_fir_reg(rx, conf)?;
        self.write_fir_reg(rx, conf.with_fir_start_clk(false))?;

        if rx {
            self.modify_reg::<RxEnableFilterControl>(|reg| {
                reg.with_rx_fir_enable_decimation(fir_enable)
            })
        } else {
            self.modify_reg::<TxEnableFilterControl>(|reg| {
                reg.with_tx_fir_enable_interpolation(fir_enable)
            })
        }
    }

    /// `ad9361_set_rx_fir_config()`.
    pub async fn set_rx_fir_config(
        &mut self,
        config: &RxFirConfig,
    ) -> Result<(), S::Error> {
        // still bypassed after loading, until someone enables it
        let bypassed = self.fir.rx_bypassed();
        self.fir.rx = Some(LoadedFir {
            factor: config.decimation,
            ntaps: config.coefficients.taps().len() as u32,
            bypassed,
        });
        self.load_fir_filter_coef(
            true,
            config.channels,
            FirGain::Rx(config.gain),
            config.decimation,
            config.coefficients,
        )
        .await
    }

    /// `ad9361_set_tx_fir_config()`.
    pub async fn set_tx_fir_config(
        &mut self,
        config: &TxFirConfig,
    ) -> Result<(), S::Error> {
        let bypassed = self.fir.tx_bypassed();
        self.fir.tx = Some(LoadedFir {
            factor: config.interpolation,
            ntaps: config.coefficients.taps().len() as u32,
            bypassed,
        });
        self.load_fir_filter_coef(
            false,
            config.channels,
            FirGain::Tx(config.gain),
            config.interpolation,
            config.coefficients,
        )
        .await
    }

    /// `ad9361_set_rx_fir_en_dis()`. Redoes clocks and bandwidths, and the filter stays off if
    /// that fails.
    pub async fn set_rx_fir_en_dis(&mut self, enable: bool) -> Result<(), Ad9361Error<S::Error>> {
        let fir = self.fir.rx.as_mut().ok_or(Ad9361Error::FirNotLoaded)?;
        if fir.bypassed == !enable {
            return Ok(());
        }
        fir.bypassed = !enable;
        let result = self.validate_enable_fir().await;
        if let (Err(_), Some(fir)) = (&result, self.fir.rx.as_mut()) {
            fir.bypassed = true;
        }
        result
    }

    /// `ad9361_set_tx_fir_en_dis()`. Redoes clocks and bandwidths, and the filter stays off if
    /// that fails.
    pub async fn set_tx_fir_en_dis(&mut self, enable: bool) -> Result<(), Ad9361Error<S::Error>> {
        let fir = self.fir.tx.as_mut().ok_or(Ad9361Error::FirNotLoaded)?;
        if fir.bypassed == !enable {
            return Ok(());
        }
        fir.bypassed = !enable;
        let result = self.validate_enable_fir().await;
        if let (Err(_), Some(fir)) = (&result, self.fir.tx.as_mut()) {
            fir.bypassed = true;
        }
        result
    }

    /// Checks the FIRs against the clock chain, recomputes the chain for the current TX rate
    /// and redoes the bandwidths (`ad9361_validate_enable_fir()`).
    pub async fn validate_enable_fir(&mut self) -> Result<(), Ad9361Error<S::Error>> {
        // chain comes from the current sample rate. no valid chain for it? use the lowest rate
        let tx_sample_rate = self.clk.rates.tx_sampl;
        let (rx, tx) = match self.calculate_rf_clock_chain(tx_sample_rate, self.clk.rate_governor) {
            Ok(chains) => chains,
            Err(_) => {
                let min = if self.clk.rate_governor != 0 { 1_500_000 } else { 1_000_000 };
                self.calculate_rf_clock_chain(HertzU32::Hz(min), self.clk.rate_governor)?
            }
        };

        if let Some(fir) = self.fir.tx.filter(|fir| !fir.bypassed) {
            let max = (tx.converter.to_raw() / tx.sample.to_raw().max(1)) * 16;
            if fir.ntaps > max {
                return Err(Ad9361Error::FirTooLong);
            }
        }
        if let Some(fir) = self.fir.rx.filter(|fir| !fir.bypassed) {
            let half = if rx.converter == rx.hb3 { 1 } else { 2 };
            let max = ((rx.converter.to_raw() / half) / rx.sample.to_raw().max(1)) * 16;
            if fir.ntaps > max {
                return Err(Ad9361Error::FirTooLong);
            }
        }

        self.set_trx_clock_chain(&rx, &tx).await?;

        // same as in set_trx_clock_chain()
        if !self.tune.dig_interface_tune_fir_disable && self.fir.tx_bypassed() && self.fir.rx_bypassed() {
            let flags = DigTuneFlags {
                restore_default: true,
                ..Default::default()
            };
            let _ = self.dig_tune(HertzU32::from_raw(0), flags).await;
        }

        let (rx_bw, tx_bw) = (self.cal.current_rx_bw, self.cal.current_tx_bw);
        self.update_rf_bandwidth(rx_bw, tx_bw).await
    }
}
