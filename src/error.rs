//! Error types. They implement `Display` and `core::error::Error`.

use core::fmt;

/// Errors of a running chip. `E` is the error type of the `SpiDevice`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ad9361Error<E> {
    /// SPI bus error
    Spi(E),
    /// Can't make that clock rate out of the parent, or the current rates don't allow it
    InvalidRate,
    /// Calibration or PLL lock didn't finish in time
    Timeout,
    /// Found no digital interface delay that works
    TuningFailed,
    /// AXI core flagged errors
    AxiStatus,
    /// Too many FIR taps for this sample rate
    FirTooLong,
    /// Tried to enable a FIR that has no coefficients
    FirNotLoaded,
    /// Manual gain isn't done for the split gain table
    UnsupportedGainTable,
    /// ENSM can't get to that state from where it is
    InvalidEnsmTransition,
    /// Calibration measured something it can't compute a setting from
    CalibrationResult,
    /// Gain control timing doesn't fit the current clock rates
    GainControlTiming,
    /// RSSI duration comes out as zero samples at this sample rate
    RssiDurationTooShort,
}

impl<E> From<E> for Ad9361Error<E> {
    fn from(e: E) -> Self {
        Ad9361Error::Spi(e)
    }
}

impl<E: fmt::Debug> fmt::Display for Ad9361Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Ad9361Error::Spi(e) => write!(f, "SPI error: {e:?}"),
            Ad9361Error::InvalidRate => f.write_str("the clock rate can't be produced"),
            Ad9361Error::Timeout => f.write_str("a calibration or lock timed out"),
            Ad9361Error::TuningFailed => f.write_str("no working digital interface delay found"),
            Ad9361Error::AxiStatus => f.write_str("an AXI core reported status errors"),
            Ad9361Error::FirTooLong => f.write_str("the FIR filter has too many taps"),
            Ad9361Error::FirNotLoaded => f.write_str("no FIR filter is loaded"),
            Ad9361Error::UnsupportedGainTable => {
                f.write_str("manual gain isn't supported with the split gain table")
            }
            Ad9361Error::InvalidEnsmTransition => {
                f.write_str("the enable state machine can't make this transition")
            }
            Ad9361Error::CalibrationResult => f.write_str("a calibration result is unusable"),
            Ad9361Error::GainControlTiming => {
                f.write_str("the gain control timing doesn't fit the clock rates")
            }
            Ad9361Error::RssiDurationTooShort => {
                f.write_str("the RSSI duration is zero samples at this rate")
            }
        }
    }
}

impl<E: fmt::Debug> core::error::Error for Ad9361Error<E> {}

/// Errors of [`Configured::init`](crate::Configured::init). `E` is the error type of the
/// `SpiDevice`, `P` the one of the reset pin.
///
/// A wrong product ID usually means the SPI setup is wrong (mode, clock, wiring).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitError<E, P> {
    /// Reset pin failed
    ResetPin(P),
    /// Wrong product ID, probably not an AD9361
    UnsupportedDevice { product_id: u8 },
    /// Chip or AXI core failed during setup
    Chip(Ad9361Error<E>),
}

impl<E, P> From<Ad9361Error<E>> for InitError<E, P> {
    fn from(e: Ad9361Error<E>) -> Self {
        InitError::Chip(e)
    }
}

impl<E: fmt::Debug, P: fmt::Debug> fmt::Display for InitError<E, P> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            InitError::ResetPin(e) => write!(f, "the reset pin failed: {e:?}"),
            InitError::UnsupportedDevice { product_id } => {
                write!(f, "unsupported device, product ID {product_id}")
            }
            InitError::Chip(e) => write!(f, "{e}"),
        }
    }
}

impl<E: fmt::Debug, P: fmt::Debug> core::error::Error for InitError<E, P> {}
