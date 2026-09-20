//! Clock tree: cached rates and the divider/multiplier scalers.

use super::synth::{BBPLL_FREQ_WORD_ADDR, RX_SYNTH_WORD_ADDR, TX_SYNTH_WORD_ADDR};
use super::*;

pub(super) fn ref_div_sel(ref_in: HertzU32, max: HertzU32) -> HertzU32 {
    if ref_in <= (max / 2) {
        return 2 * ref_in;
    }

    if ref_in <= max {
        return ref_in;
    }

    if ref_in <= (max * 2) {
        return ref_in / 2;
    }

    if ref_in <= (max * 4) {
        return ref_in / 4;
    }

    HertzU32::from_raw(0)
}

pub(super) struct PllScaler {
    /// fractional part
    pub(super) fract: u32,
    /// integer part
    pub(super) integer: u32,
}

impl PllScaler {
    pub fn rate_from_parent(&self, parent: HertzU32) -> HertzU32 {
        let mut rate = ((parent.to_raw() as u64) * (self.fract as u64)) / BBPLL_MODULUS as u64;
        rate += parent.to_raw() as u64 * self.integer as u64;

        HertzU32::Hz(rate as u32)
    }
}

/// RX or TX RF synth: fractional-N PLL plus VCO post-divider, from the FRACT/INTEGER byte
/// registers and `REG_RFPLL_DIVIDERS`. Like `ad9361_calc_rfpll_int_freq()`. The VCO runs at
/// several GHz, way past `u32`, so everything is `u64` until `vco_div` brings it back down.
pub(super) struct RfPllScaler {
    /// fractional part, 23 bits
    pub(super) fract: u32,
    /// integer part, 11 bits
    pub(super) integer: u32,
    /// VCO post-divider exponent, the VCO runs at `2^(vco_div + 1)` times the output
    pub(super) vco_div: u32,
}

impl RfPllScaler {
    pub fn rate_from_parent(&self, parent: HertzU32) -> HertzU64 {
        let parent = parent.to_raw() as u64;
        let mut vco_freq = (parent * self.fract as u64) / RFPLL_MODULUS as u64;
        vco_freq += parent * self.integer as u64;

        HertzU64::Hz(vco_freq >> (self.vco_div + 1))
    }
}

/// Multiplier and divider for a clock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockMulDiv {
    pub mult: u32,
    pub div: u32,
}

impl ClockMulDiv {
    /// Closest multiplier/divider (one of them is 1) to get `rate` from `parent`. `None` if
    /// either rate is zero.
    pub fn closest(rate: HertzU32, parent: HertzU32) -> Option<Self> {
        let (rate, parent) = (rate.to_raw() as u64, parent.to_raw() as u64);
        if rate == 0 || parent == 0 {
            return None;
        }
        let (mult, div) = if rate >= parent {
            ((rate + parent / 2) / parent, 1)
        } else {
            (1, (parent + rate / 2) / rate)
        };
        Some(Self {
            mult: u32::try_from(mult).ok()?,
            div: u32::try_from(div).ok()?,
        })
    }

    /// `parent * mult / div`
    pub fn rate_from_parent(&self, parent: HertzU32) -> HertzU32 {
        HertzU32::from_raw(
            ((parent.to_raw() as u64) * (self.mult as u64) / (self.div as u64)) as u32,
        )
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Ad9361Clock {
    BbRef,
    RxRef,
    TxRef,
    Adc,
    R2,
    R1,
    ClkRf,
    RxSampl,
    Dac,
    T2,
    T1,
    ClkTf,
    TxSampl,
}

/// All clock rates of the chip in Hz.
///
/// A reference goes through the BB reference into the BBPLL, which feeds the ADC. The ADC feeds
/// the RX decimators (R2, R1, CLKRF) down to the RX sample rate. On the TX side the DAC feeds T2,
/// T1, CLKTF and the TX sample rate. The RF synths run from `rx_ref` and `tx_ref`.
///
/// no-OS has an `_int`/`_dummy` split for boards with an external LO. That isn't supported here,
/// so `rx_rfpll` and `tx_rfpll` are the internal synth.
pub struct Ad9361ClockRates {
    pub ext_ref: HertzU32,
    pub tx_ref: HertzU32,
    pub rx_ref: HertzU32,
    pub bb_ref: HertzU32,
    /// baseband PLL
    pub bbpll: HertzU32,
    pub adc: HertzU32,
    pub r2: HertzU32,
    pub r1: HertzU32,
    pub clkrf: HertzU32,
    pub rx_sampl: HertzU32,
    pub dac: HertzU32,
    pub t2: HertzU32,
    pub t1: HertzU32,
    pub clktf: HertzU32,
    pub tx_sampl: HertzU32,
    // 64 bit, the LO goes up to 6 GHz
    pub rx_rfpll: HertzU64,
    pub tx_rfpll: HertzU64,
}

impl Ad9361ClockRates {
    /// Only the reference is known, the rest gets read from the chip.
    pub(super) const fn uninitialized(ext_ref: HertzU32) -> Self {
        let zero = HertzU32::from_raw(0);
        Self {
            ext_ref,
            tx_ref: zero,
            rx_ref: zero,
            bb_ref: zero,
            bbpll: zero,
            adc: zero,
            r2: zero,
            r1: zero,
            clkrf: zero,
            rx_sampl: zero,
            dac: zero,
            t2: zero,
            t1: zero,
            clktf: zero,
            tx_sampl: zero,
            rx_rfpll: HertzU64::from_raw(0),
            tx_rfpll: HertzU64::from_raw(0),
        }
    }
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    pub fn read_clock_scaler(&mut self, clock: Ad9361Clock) -> Result<ClockMulDiv, S::Error> {
        fn scaler_to_mul_div(scaler: u2) -> ClockMulDiv {
            match scaler.value() {
                0 => ClockMulDiv { mult: 1, div: 1 },
                1 => ClockMulDiv { mult: 1, div: 2 },
                2 => ClockMulDiv { mult: 1, div: 4 },
                3 => ClockMulDiv { mult: 2, div: 1 },
                _ => unreachable!(),
            }
        }

        Ok(match clock {
            Ad9361Clock::BbRef => {
                scaler_to_mul_div(self.read_reg::<ClockControl>()?.ref_freq_scaler())
            }
            Ad9361Clock::RxRef => {
                let msb = self.read_reg::<RefDivideConfig1>()?.rx_ref_divider_msb();
                let lsb = self.read_reg::<RefDivideConfig2>()?.rx_ref_divider_lsb();

                scaler_to_mul_div(u2::new(((msb as u8) << 1) | (lsb as u8)))
            }
            Ad9361Clock::TxRef => {
                scaler_to_mul_div(self.read_reg::<RefDivideConfig2>()?.tx_ref_divider())
            }
            Ad9361Clock::Adc => {
                let div = self.read_reg::<BbPll>()?.bbpll_divider();
                ClockMulDiv {
                    mult: 1,
                    div: 1 << div.value(),
                }
            }
            Ad9361Clock::R2 => {
                let dec = self
                    .read_reg::<RxEnableFilterControl>()?
                    .dec3_enable_decimation();
                ClockMulDiv {
                    mult: 1,
                    div: dec.as_u32() + 1,
                }
            }
            Ad9361Clock::R1 => {
                let dec = self.read_reg::<RxEnableFilterControl>()?.rhb2_en();
                ClockMulDiv {
                    mult: 1,
                    div: (dec as u32) + 1,
                }
            }
            Ad9361Clock::ClkRf => {
                let dec = self.read_reg::<RxEnableFilterControl>()?.rhb1_en();
                ClockMulDiv {
                    mult: 1,
                    div: (dec as u32) + 1,
                }
            }
            Ad9361Clock::RxSampl => {
                let tmp = self
                    .read_reg::<RxEnableFilterControl>()?
                    .rx_fir_enable_decimation();

                let div = if tmp.value() == 0 {
                    1 // Bypass filter
                } else {
                    1 << (tmp.value() - 1)
                };

                ClockMulDiv { mult: 1, div }
            }
            Ad9361Clock::Dac => {
                let tmp = self.read_reg::<BbPll>()?.dac_clk_div2();
                ClockMulDiv {
                    mult: 1,
                    div: (tmp as u32) + 1,
                }
            }
            Ad9361Clock::T2 => {
                let tmp = self
                    .read_reg::<TxEnableFilterControl>()?
                    .thb3_enable_interp();
                ClockMulDiv {
                    mult: 1,
                    div: (tmp.as_u32()) + 1,
                }
            }
            Ad9361Clock::T1 => {
                let tmp = self.read_reg::<TxEnableFilterControl>()?.thb2_en();
                ClockMulDiv {
                    mult: 1,
                    div: (tmp as u32) + 1,
                }
            }
            Ad9361Clock::ClkTf => {
                let tmp = self.read_reg::<TxEnableFilterControl>()?.thb1_en();
                ClockMulDiv {
                    mult: 1,
                    div: (tmp as u32) + 1,
                }
            }
            Ad9361Clock::TxSampl => {
                let tmp = self
                    .read_reg::<TxEnableFilterControl>()?
                    .tx_fir_enable_interpolation();

                let div = if tmp.value() == 0 {
                    1 // Bypass filter
                } else {
                    1 << (tmp.value() - 1)
                };

                ClockMulDiv { mult: 1, div }
            }
        })
    }

    pub(super) fn read_bbpll_clock_scaler(&mut self) -> Result<PllScaler, S::Error> {
        let mut buf = [0u8; 4];
        self.read_bytes(&mut buf, BBPLL_FREQ_WORD_ADDR)?;
        let fract = ((buf[3] as u32) << 16) | ((buf[2] as u32) << 8) | (buf[1] as u32);
        let integer = buf[0] as u32;

        Ok(PllScaler { fract, integer })
    }

    /// Reads the fractional word, integer word and VCO post-divider of the RX or TX synth
    /// (`ad9361_rfpll_int_recalc_rate()` / `ad9361_calc_rfpll_int_freq()`).
    ///
    /// Always reads the live registers. no-OS reads back from fastlock memory instead when a
    /// fastlock profile is active, because the live registers can be wrong then. We have no
    /// fastlock, so that never happens. If fastlock ever gets added, this needs the same branch
    /// or it'll read stale words.
    pub(super) fn read_rfpll_scaler(&mut self, is_tx: bool) -> Result<RfPllScaler, S::Error> {
        // burst reads count addresses down: FRACT_BYTE_2, _1, _0, INTEGER_BYTE_1, _0
        let start = if is_tx {
            TX_SYNTH_WORD_ADDR
        } else {
            RX_SYNTH_WORD_ADDR
        };
        let mut buf = [0u8; 5];
        self.read_bytes(&mut buf, start)?;

        // fractional word <22:16> and integer word <10:8>
        let fract = (((buf[0] & 0x7F) as u32) << 16) | ((buf[1] as u32) << 8) | (buf[2] as u32);
        let integer = (((buf[3] & 0x7) as u32) << 8) | (buf[4] as u32);

        let dividers = self.read_reg::<RfPllDividers>()?;
        let vco_div = if is_tx {
            dividers.tx_vco_divider()
        } else {
            dividers.rx_vco_divider()
        };

        Ok(RfPllScaler {
            fract,
            integer,
            vco_div: vco_div.value() as u32,
        })
    }

    pub fn read_clock_rates(&mut self) -> Result<Ad9361ClockRates, S::Error> {
        let tx_ref = self
            .read_clock_scaler(Ad9361Clock::TxRef)?
            .rate_from_parent(self.ref_clk_in);
        let rx_ref = self
            .read_clock_scaler(Ad9361Clock::RxRef)?
            .rate_from_parent(self.ref_clk_in);
        let bb_ref = self
            .read_clock_scaler(Ad9361Clock::BbRef)?
            .rate_from_parent(self.ref_clk_in);

        let bbpll = self.read_bbpll_clock_scaler()?.rate_from_parent(bb_ref);

        let adc = self
            .read_clock_scaler(Ad9361Clock::Adc)?
            .rate_from_parent(bbpll);
        let r2 = self
            .read_clock_scaler(Ad9361Clock::R2)?
            .rate_from_parent(adc);
        let r1 = self
            .read_clock_scaler(Ad9361Clock::R1)?
            .rate_from_parent(r2);
        let clkrf = self
            .read_clock_scaler(Ad9361Clock::ClkRf)?
            .rate_from_parent(r1);
        let rx_sampl = self
            .read_clock_scaler(Ad9361Clock::RxSampl)?
            .rate_from_parent(clkrf);

        let dac = self
            .read_clock_scaler(Ad9361Clock::Dac)?
            .rate_from_parent(adc);
        let t2 = self
            .read_clock_scaler(Ad9361Clock::T2)?
            .rate_from_parent(dac);
        let t1 = self
            .read_clock_scaler(Ad9361Clock::T1)?
            .rate_from_parent(t2);
        let clktf = self
            .read_clock_scaler(Ad9361Clock::ClkTf)?
            .rate_from_parent(t1);
        let tx_sampl = self
            .read_clock_scaler(Ad9361Clock::TxSampl)?
            .rate_from_parent(clktf);

        // the RFPLLs hang off RX_REFCLK/TX_REFCLK, not the ADC/DAC datapath
        let rx_rfpll = self.read_rfpll_scaler(false)?.rate_from_parent(rx_ref);
        let tx_rfpll = self.read_rfpll_scaler(true)?.rate_from_parent(tx_ref);

        Ok(Ad9361ClockRates {
            ext_ref: self.ref_clk_in,
            tx_ref,
            rx_ref,
            bb_ref,
            bbpll,
            adc,
            r2,
            r1,
            clkrf,
            rx_sampl,
            dac,
            t2,
            t1,
            clktf,
            tx_sampl,
            rx_rfpll,
            tx_rfpll,
        })
    }

    pub fn set_dcxo_tune(&mut self, coarse: u6, fine: u13) -> Result<(), S::Error> {
        self.write_reg(DcxoCoarseTune::default().with_dcxo_tune_coarse(coarse))?;

        self.write_reg(
            DcxoFineTuneLow::default().with_dcxo_tune_fine_low(u5::extract_u16(fine.value(), 0)),
        )?;
        self.write_reg(DcxoFineTuneHigh((fine.value() >> 5) as u8))?;

        Ok(())
    }

    /// Reads every clock rate from the chip into the cache (`ad9361_register_clocks()` /
    /// `clks_resync` in no-OS). Needs to run before [`Self::set_clock_rate`], which works off the
    /// cached parent rate.
    pub fn init_clocks(&mut self) -> Result<(), S::Error> {
        self.clk.rates = self.read_clock_rates()?;
        Ok(())
    }

    /// All cached rates.
    pub fn clock_rates(&self) -> &Ad9361ClockRates {
        &self.clk.rates
    }

    /// Cached rate, no SPI.
    pub fn clock_rate(&self, clock: Ad9361Clock) -> HertzU32 {
        let r = &self.clk.rates;
        match clock {
            Ad9361Clock::BbRef => r.bb_ref,
            Ad9361Clock::RxRef => r.rx_ref,
            Ad9361Clock::TxRef => r.tx_ref,
            Ad9361Clock::Adc => r.adc,
            Ad9361Clock::R2 => r.r2,
            Ad9361Clock::R1 => r.r1,
            Ad9361Clock::ClkRf => r.clkrf,
            Ad9361Clock::RxSampl => r.rx_sampl,
            Ad9361Clock::Dac => r.dac,
            Ad9361Clock::T2 => r.t2,
            Ad9361Clock::T1 => r.t1,
            Ad9361Clock::ClkTf => r.clktf,
            Ad9361Clock::TxSampl => r.tx_sampl,
        }
    }

    pub(super) fn clock_rate_mut(&mut self, clock: Ad9361Clock) -> &mut HertzU32 {
        let r = &mut self.clk.rates;
        match clock {
            Ad9361Clock::BbRef => &mut r.bb_ref,
            Ad9361Clock::RxRef => &mut r.rx_ref,
            Ad9361Clock::TxRef => &mut r.tx_ref,
            Ad9361Clock::Adc => &mut r.adc,
            Ad9361Clock::R2 => &mut r.r2,
            Ad9361Clock::R1 => &mut r.r1,
            Ad9361Clock::ClkRf => &mut r.clkrf,
            Ad9361Clock::RxSampl => &mut r.rx_sampl,
            Ad9361Clock::Dac => &mut r.dac,
            Ad9361Clock::T2 => &mut r.t2,
            Ad9361Clock::T1 => &mut r.t1,
            Ad9361Clock::ClkTf => &mut r.clktf,
            Ad9361Clock::TxSampl => &mut r.tx_sampl,
        }
    }

    /// Cached rate of the parent. reference -> BBPLL -> ADC -> RX decimators, ADC -> DAC -> TX
    /// interpolators.
    pub(super) fn parent_rate(&self, clock: Ad9361Clock) -> HertzU32 {
        match clock {
            Ad9361Clock::BbRef | Ad9361Clock::RxRef | Ad9361Clock::TxRef => self.ref_clk_in,
            Ad9361Clock::Adc => self.clk.rates.bbpll,
            Ad9361Clock::R2 | Ad9361Clock::Dac => self.clk.rates.adc,
            Ad9361Clock::R1 => self.clk.rates.r2,
            Ad9361Clock::ClkRf => self.clk.rates.r1,
            Ad9361Clock::RxSampl => self.clk.rates.clkrf,
            Ad9361Clock::T2 => self.clk.rates.dac,
            Ad9361Clock::T1 => self.clk.rates.t2,
            Ad9361Clock::ClkTf => self.clk.rates.t1,
            Ad9361Clock::TxSampl => self.clk.rates.clktf,
        }
    }

    /// `ad9361_set_clk_scaler()`. `InvalidRate` if the chip can't do `ratio`. Unlike no-OS the
    /// ADC divider has to be an exact power of two (2..=64), no silent rounding down.
    pub fn set_clock_scaler(
        &mut self,
        clock: Ad9361Clock,
        ratio: &ClockMulDiv,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let ClockMulDiv { mult, div } = *ratio;
        // REFCLK scaler: 0 = x1, 1 = x1/2, 2 = x1/4, 3 = x2
        let ref_scaler = || match (mult, div) {
            (1, 1) => Ok(0u8),
            (1, 2) => Ok(1),
            (1, 4) => Ok(2),
            (2, 1) => Ok(3),
            _ => Err(Ad9361Error::InvalidRate),
        };
        let divider = |max: u32| {
            if mult == 1 && (1..=max).contains(&div) {
                Ok(())
            } else {
                Err(Ad9361Error::InvalidRate)
            }
        };
        // FIR sample clocks: /1, /2, /4 or FIR bypassed
        let fir_field = |bypass: bool| {
            if mult != 1 || !matches!(div, 1 | 2 | 4) {
                return Err(Ad9361Error::InvalidRate);
            }
            Ok(if bypass { 0 } else { div.ilog2() as u8 + 1 })
        };

        match clock {
            Ad9361Clock::BbRef => {
                let scaler = ref_scaler()?;
                self.modify_reg::<ClockControl>(|reg| reg.with_ref_freq_scaler(u2::new(scaler)))?;
            }
            Ad9361Clock::RxRef => {
                let scaler = ref_scaler()?;
                self.modify_reg::<RefDivideConfig1>(|reg| {
                    reg.with_rx_ref_divider_msb(scaler & 0b10 != 0)
                })?;
                self.modify_reg::<RefDivideConfig2>(|reg| {
                    reg.with_rx_ref_divider_lsb(scaler & 0b01 != 0)
                })?;
            }
            Ad9361Clock::TxRef => {
                let scaler = ref_scaler()?;
                self.modify_reg::<RefDivideConfig2>(|reg| {
                    reg.with_tx_ref_divider(u2::new(scaler))
                })?;
            }
            Ad9361Clock::Adc => {
                if mult != 1 || !div.is_power_of_two() || !(2..=64).contains(&div) {
                    return Err(Ad9361Error::InvalidRate);
                }
                self.modify_reg::<BbPll>(|reg| reg.with_bbpll_divider(u3::new(div.ilog2() as u8)))?;
            }
            Ad9361Clock::R2 => {
                divider(3)?;
                self.modify_reg::<RxEnableFilterControl>(|reg| {
                    reg.with_dec3_enable_decimation(u2::new(div as u8 - 1))
                })?;
            }
            Ad9361Clock::R1 => {
                divider(2)?;
                self.modify_reg::<RxEnableFilterControl>(|reg| reg.with_rhb2_en(div == 2))?;
            }
            Ad9361Clock::ClkRf => {
                divider(2)?;
                self.modify_reg::<RxEnableFilterControl>(|reg| reg.with_rhb1_en(div == 2))?;
            }
            Ad9361Clock::RxSampl => {
                let field = fir_field(self.fir.rx_bypassed())?;
                self.modify_reg::<RxEnableFilterControl>(|reg| {
                    reg.with_rx_fir_enable_decimation(u2::new(field))
                })?;
            }
            Ad9361Clock::Dac => {
                divider(2)?;
                self.modify_reg::<BbPll>(|reg| reg.with_dac_clk_div2(div == 2))?;
            }
            Ad9361Clock::T2 => {
                divider(3)?;
                self.modify_reg::<TxEnableFilterControl>(|reg| {
                    reg.with_thb3_enable_interp(u2::new(div as u8 - 1))
                })?;
            }
            Ad9361Clock::T1 => {
                divider(2)?;
                self.modify_reg::<TxEnableFilterControl>(|reg| reg.with_thb2_en(div == 2))?;
            }
            Ad9361Clock::ClkTf => {
                divider(2)?;
                self.modify_reg::<TxEnableFilterControl>(|reg| reg.with_thb1_en(div == 2))?;
            }
            Ad9361Clock::TxSampl => {
                let field = fir_field(self.fir.tx_bypassed())?;
                self.modify_reg::<TxEnableFilterControl>(|reg| {
                    reg.with_tx_fir_enable_interpolation(u2::new(field))
                })?;
            }
        }

        Ok(())
    }

    /// Sets `clock` to the closest rate its multiplier/divider can make from the cached parent
    /// rate, then refreshes that clock's cache from the chip.
    ///
    /// Children aren't updated, same as no-OS. Go parent first, or call [`Self::init_clocks`]
    /// afterwards.
    ///
    /// Reference, ADC/DAC and the decimators/interpolators only. BBPLL and the RF synths have
    /// their own frequency words and live elsewhere.
    pub fn set_clock_rate(
        &mut self,
        clock: Ad9361Clock,
        rate: HertzU32,
    ) -> Result<(), Ad9361Error<S::Error>> {
        if self.clock_rate(clock) == rate {
            return Ok(());
        }
        let parent = self.parent_rate(clock);
        let ratio = ClockMulDiv::closest(rate, parent).ok_or(Ad9361Error::InvalidRate)?;
        self.set_clock_scaler(clock, &ratio)?;
        *self.clock_rate_mut(clock) = self.read_clock_scaler(clock)?.rate_from_parent(parent);
        Ok(())
    }
}
