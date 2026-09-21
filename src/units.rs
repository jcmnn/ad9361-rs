//! Types for values with a limited range. The constructors return [`OutOfRange`] for anything
//! outside it. The `const fn` ones fail at compile time when used in a constant.

use arbitrary_int::{u3, u6};
use fugit::{HertzU32, HertzU64};

/// Value out of range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutOfRange;

/// TX attenuation, 0 to 89.75 dB in 0.25 dB steps.
///
/// ```
/// use ad9361::TxAttenuation;
///
/// const QUIET: TxAttenuation = TxAttenuation::from_db(40);
/// assert_eq!(QUIET.mdb(), 40_000);
///
/// // millidB rounds down to a 0.25 dB step
/// assert_eq!(TxAttenuation::from_mdb(10_249).unwrap().quarter_db(), 40);
/// assert!(TxAttenuation::from_mdb(90_000).is_err());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct TxAttenuation(u16);

impl TxAttenuation {
    /// Steps of 0.25 dB
    const MAX_QUARTER_DB: u16 = 359;
    pub const MIN: Self = Self(0);
    pub const MAX: Self = Self(Self::MAX_QUARTER_DB);

    /// Whole dB, 0..=89. Out of range in a `const` is a compile error.
    pub const fn from_db(db: u8) -> Self {
        assert!(db as u16 * 4 <= Self::MAX_QUARTER_DB, "TX attenuation above 89.75 dB");
        Self(db as u16 * 4)
    }

    /// Steps of 0.25 dB, 0..=359.
    pub const fn from_quarter_db(quarter_db: u16) -> Result<Self, OutOfRange> {
        if quarter_db <= Self::MAX_QUARTER_DB {
            Ok(Self(quarter_db))
        } else {
            Err(OutOfRange)
        }
    }

    /// Like [`Self::from_quarter_db`], but anything above the range becomes the max. The
    /// register can hold bigger values.
    pub(super) const fn saturating_from_quarter_db(quarter_db: u16) -> Self {
        if quarter_db <= Self::MAX_QUARTER_DB {
            Self(quarter_db)
        } else {
            Self::MAX
        }
    }

    /// In millidB. Rounded down to the 0.25 dB steps.
    pub const fn from_mdb(mdb: u32) -> Result<Self, OutOfRange> {
        let quarter_db = mdb / 250;
        if quarter_db > Self::MAX_QUARTER_DB as u32 {
            return Err(OutOfRange);
        }
        Ok(Self(quarter_db as u16))
    }

    pub const fn quarter_db(self) -> u16 {
        self.0
    }

    pub const fn mdb(self) -> u32 {
        self.0 as u32 * 250
    }
}

/// RF bandwidth, 200 kHz to 56 MHz. The baseband filters work on half of it, since the signal
/// is complex.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RfBandwidth(HertzU32);

impl RfBandwidth {
    pub const MIN: HertzU32 = HertzU32::Hz(200_000);
    pub const MAX: HertzU32 = HertzU32::Hz(56_000_000);

    /// Clamps into range (`ad9361_validate_rf_bw()`).
    pub fn new_clamped(bandwidth: HertzU32) -> Self {
        Self(bandwidth.clamp(Self::MIN, Self::MAX))
    }

    pub const fn new(bandwidth: HertzU32) -> Result<Self, OutOfRange> {
        if bandwidth.to_raw() >= Self::MIN.to_raw() && bandwidth.to_raw() <= Self::MAX.to_raw() {
            Ok(Self(bandwidth))
        } else {
            Err(OutOfRange)
        }
    }

    pub const fn get(self) -> HertzU32 {
        self.0
    }
}

/// Baseband sample rate, up to 61.44 MSPS.
///
/// Only checks the range. [`Ad9361::set_sample_rate`](crate::Ad9361::set_sample_rate) can still
/// fail with [`Ad9361Error::InvalidRate`](crate::Ad9361Error::InvalidRate) when the dividers can't
/// make the rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SampleRate(HertzU32);

impl SampleRate {
    pub const MAX: HertzU32 = HertzU32::Hz(61_440_000);

    pub const fn new(rate: HertzU32) -> Result<Self, OutOfRange> {
        if rate.to_raw() > 0 && rate.to_raw() <= Self::MAX.to_raw() {
            Ok(Self(rate))
        } else {
            Err(OutOfRange)
        }
    }

    pub const fn get(self) -> HertzU32 {
        self.0
    }
}

macro_rules! lo_frequency {
    ($(#[$doc:meta])* $name:ident, $min:expr) => {
        $(#[$doc])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct $name(HertzU64);

        impl $name {
            pub const MIN: HertzU64 = HertzU64::Hz($min);
            pub const MAX: HertzU64 = HertzU64::Hz(6_000_000_000);

            pub const fn new(frequency: HertzU64) -> Result<Self, OutOfRange> {
                if frequency.to_raw() >= Self::MIN.to_raw()
                    && frequency.to_raw() <= Self::MAX.to_raw()
                {
                    Ok(Self(frequency))
                } else {
                    Err(OutOfRange)
                }
            }

            /// In Hz, 70 MHz (46.875 MHz for TX) to 6 GHz. Out of range in a `const` is a
            /// compile error.
            pub const fn from_hz(hz: u64) -> Self {
                match Self::new(HertzU64::Hz(hz)) {
                    Ok(frequency) => frequency,
                    Err(_) => panic!("LO frequency out of range"),
                }
            }

            pub const fn get(self) -> HertzU64 {
                self.0
            }
        }
    };
}

lo_frequency!(
    /// RX LO frequency, 70 MHz to 6 GHz.
    RxLoFrequency,
    70_000_000
);
lo_frequency!(
    /// TX LO frequency, 46.875001 MHz to 6 GHz.
    TxLoFrequency,
    46_875_001
);

/// RX input pins. RX1 and RX2 switch together.
///
/// `A`, `B` and `C` are the three differential inputs. The single ended variants use one pin of a
/// set. The TX monitor inputs from the C driver are not supported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxInput {
    /// RX1A and RX2A, differential
    DifferentialA,
    /// RX1B and RX2B, differential
    DifferentialB,
    /// RX1C and RX2C, differential
    DifferentialC,
    /// The `_N`/`_P` pins of A, B and C, single ended
    SingleEndedAN,
    SingleEndedAP,
    SingleEndedBN,
    SingleEndedBP,
    SingleEndedCN,
    SingleEndedCP,
}

impl RxInput {
    /// Value for the RX input select field.
    pub(super) const fn select_bits(self) -> u8 {
        match self {
            RxInput::DifferentialA => 0b000011,
            RxInput::DifferentialB => 0b001100,
            RxInput::DifferentialC => 0b110000,
            RxInput::SingleEndedAN => 0b000001,
            RxInput::SingleEndedAP => 0b000010,
            RxInput::SingleEndedBN => 0b000100,
            RxInput::SingleEndedBP => 0b001000,
            RxInput::SingleEndedCN => 0b010000,
            RxInput::SingleEndedCP => 0b100000,
        }
    }
}

/// AuxADC / temperature sensor decimation, 256 to 32768. The register holds
/// log2(decimation) - 8.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuxAdcDecimation(u3);

impl AuxAdcDecimation {
    pub const D256: Self = Self(u3::new(0));

    /// Rounds down to a power of two, same as no-OS.
    pub const fn new(decimation: u32) -> Result<Self, OutOfRange> {
        match decimation.checked_ilog2() {
            Some(log) if log >= 8 && log < 16 => Ok(Self(u3::new((log - 8) as u8))),
            _ => Err(OutOfRange),
        }
    }

    pub(super) const fn field(self) -> u3 {
        self.0
    }
}

/// External LNA gain, 0.5 dB steps up to 31.5 dB.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ElnaGain(u6);

impl ElnaGain {
    pub const ZERO: Self = Self(u6::new(0));

    /// In millidB, rounded down to 0.5 dB.
    pub const fn from_mdb(mdb: u16) -> Result<Self, OutOfRange> {
        let steps = mdb / 500;
        if steps < 64 {
            Ok(Self(u6::new(steps as u8)))
        } else {
            Err(OutOfRange)
        }
    }

    pub(super) const fn field(self) -> u6 {
        self.0
    }
}
