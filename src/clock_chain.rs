//! Works out the RX/TX clock chain for a sample rate (`ad9361_calculate_rf_clock_chain()`).

use super::*;

const MIN_ADC_CLK: u64 = 25_000_000;
const MAX_BBPLL_DIV: u64 = 64;
const MIN_BBPLL_DIV: u64 = 2;

/// R2/T2, R1/T1 and CLKRF/CLKTF dividers for each total divider off the ADC clock.
const CLK_DIVIDERS: [[i32; 4]; 7] = [
    [12, 3, 2, 2],
    [8, 2, 2, 2],
    [6, 3, 1, 2],
    [4, 2, 2, 1],
    [3, 3, 1, 1],
    [2, 2, 1, 1],
    [1, 1, 1, 1],
];

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    /// RX and TX path clocks for a TX sample rate. `rate_gov` 0 = highest oversampling, 1 = nominal.
    pub fn calculate_rf_clock_chain(
        &self,
        tx_sample_rate: HertzU32,
        rate_gov: u32,
    ) -> Result<(PathClocks, PathClocks), Ad9361Error<S::Error>> {
        let rx_intdec = self.fir.rx_decimation() as u64;
        let tx_intdec = self.fir.tx_interpolation() as u64;
        let rate = tx_sample_rate.to_raw() as u64;
        if rate > MAX_BASEBAND_RATE.to_raw() as u64 || rate == 0 {
            return Err(Ad9361Error::InvalidRate);
        }

        let clktf = rate * tx_intdec;
        let clkrf = rate * rx_intdec * if self.rx_eq_2tx() { 2 } else { 1 };

        // no-OS bumps the governor until it finds dividers that fit
        let mut gov = rate_gov as usize;
        let (index_rx, index_tx, adc_rate, dac_rate) = loop {
            let mut recursion = true;
            if gov == 1 && rx_intdec * rate * 8 < MIN_ADC_CLK {
                recursion = false;
                gov = 0;
            }

            let (mut index_rx, mut index_tx) = (-1i32, -1i32);
            let (mut adc_rate, mut dac_rate) = (0u64, 0u64);
            for (i, divider) in CLK_DIVIDERS.iter().enumerate().skip(gov) {
                adc_rate = clkrf * divider[0] as u64;
                dac_rate = clktf * divider[0] as u64;
                if !(MIN_ADC_CLK..=MAX_ADC_CLK.to_raw() as u64).contains(&adc_rate) {
                    continue;
                }
                if dac_rate == 0 {
                    return Err(Ad9361Error::InvalidRate);
                }

                let ratio = if dac_rate > adc_rate {
                    -((dac_rate / adc_rate) as i32)
                } else {
                    (adc_rate / dac_rate) as i32
                };
                let i = i as i32;
                index_rx = i;
                if adc_rate <= MAX_DAC_CLK.to_raw() as u64 {
                    index_tx = i - if ratio == 1 { 0 } else { ratio };
                    dac_rate = adc_rate; // ADC_CLK
                } else {
                    dac_rate = adc_rate / 2; // ADC_CLK/2
                    index_tx = if i == 4 && ratio >= 0 {
                        7 // STOP: 3/2 != 1
                    } else {
                        i + if i == 5 && ratio >= 0 { 1 } else { 2 }
                            - if ratio == 1 { 0 } else { ratio }
                    };
                }
                break;
            }

            let valid = (0..=6).contains(&index_tx) && (0..=6).contains(&index_rx);
            if valid {
                break (index_rx as usize, index_tx as usize, adc_rate, dac_rate);
            }
            if gov < 7 && recursion {
                gov += 1;
                continue;
            }
            // ADC clock too low or BBPLL too high
            return Err(Ad9361Error::InvalidRate);
        };

        let mut div = MAX_BBPLL_DIV;
        let bbpll = loop {
            let bbpll = adc_rate * div;
            div >>= 1;
            if !(bbpll > MAX_BBPLL_FREQ.to_raw() as u64 && div >= MIN_BBPLL_DIV) {
                break bbpll;
            }
        };

        let chain = |converter: u64, dividers: &[i32; 4], intdec: u64| {
            let hb3 = converter / dividers[1] as u64;
            let hb2 = hb3 / dividers[2] as u64;
            let hb1 = hb2 / dividers[3] as u64;
            PathClocks {
                bbpll: HertzU32::from_raw(bbpll as u32),
                converter: HertzU32::from_raw(converter as u32),
                hb3: HertzU32::from_raw(hb3 as u32),
                hb2: HertzU32::from_raw(hb2 as u32),
                hb1: HertzU32::from_raw(hb1 as u32),
                sample: HertzU32::from_raw((hb1 / intdec.max(1)) as u32),
            }
        };
        Ok((
            chain(adc_rate, &CLK_DIVIDERS[index_rx], rx_intdec),
            chain(dac_rate, &CLK_DIVIDERS[index_tx], tx_intdec),
        ))
    }

    /// `ad9361_set_trx_clock_chain_freq()`.
    pub async fn set_trx_clock_chain_freq(
        &mut self,
        freq: HertzU32,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let (rx, tx) = self.calculate_rf_clock_chain(freq, self.clk.rate_governor)?;
        self.set_trx_clock_chain(&rx, &tx).await
    }

    /// Same but skips the digital interface tuning. The tuning calls this one itself.
    pub(super) async fn set_trx_clock_chain_freq_no_tune(
        &mut self,
        freq: HertzU32,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let (rx, tx) = self.calculate_rf_clock_chain(freq, self.clk.rate_governor)?;
        self.apply_trx_clock_chain(&rx, &tx).await?;
        self.bb_clk_update()
    }
}

/// Path clocks within limits, and one of them matches DATA_CLK (`ad9361_validate_trx_clock_chain()`).
pub(super) fn validate_trx_clock_chain<E>(
    rx: &PathClocks,
    tx: &PathClocks,
    rx2tx2: bool,
    lvds_mode: bool,
) -> Result<(), Ad9361Error<E>> {
    let channels: u64 = if rx2tx2 { 4 } else { 2 };
    let interface_div: u64 = if lvds_mode { 1 } else { 2 };
    let data_clk = channels / interface_div * rx.sample.to_raw() as u64;

    if !lvds_mode && data_clk > MAX_BASEBAND_RATE.to_raw() as u64 {
        return Err(Ad9361Error::InvalidRate);
    }

    let max_rx = PathClocks {
        bbpll: MAX_BBPLL_FREQ,
        converter: MAX_ADC_CLK,
        hb3: MAX_RX_HB3,
        hb2: MAX_RX_HB2,
        hb1: MAX_RX_HB1,
        sample: MAX_BASEBAND_RATE,
    };
    let max_tx = PathClocks {
        bbpll: MAX_BBPLL_FREQ,
        converter: MAX_DAC_CLK,
        hb3: MAX_TX_HB3,
        hb2: MAX_TX_HB2,
        hb1: MAX_TX_HB1,
        sample: MAX_BASEBAND_RATE,
    };
    if rx.as_array().iter().zip(max_rx.as_array()).any(|(r, m)| *r > m)
        || tx.as_array().iter().zip(max_tx.as_array()).any(|(r, m)| *r > m)
    {
        return Err(Ad9361Error::InvalidRate);
    }

    let near = |rate: u64| rate.abs_diff(data_clk) < 4;
    let adc = rx.converter.to_raw() as u64;
    let hb3 = rx.hb3.to_raw() as u64;
    if (1..=3).any(|i| near(adc / i)) || (1..=4).any(|i| near(hb3 >> i)) {
        Ok(())
    } else {
        Err(Ad9361Error::InvalidRate)
    }
}

/// BBPLL rate and the rates along the RX or TX datapath.
///
/// RX: `bbpll -> converter (ADC) -> hb3 (R2) -> hb2 (R1) -> hb1 (CLKRF) -> sample`.
/// TX: `bbpll -> converter (DAC) -> hb3 (T2) -> hb2 (T1) -> hb1 (CLKTF) -> sample`.
///
/// The chip only has certain dividers, so not every combination works. [`Ad9361Config::new`]
/// checks the limits. [`Self::DEFAULT_RX`] and [`Self::DEFAULT_TX`] are for 30.72 MSPS.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PathClocks {
    pub bbpll: HertzU32,
    /// ADC (RX) / DAC (TX)
    pub converter: HertzU32,
    /// R2 (RX) / T2 (TX)
    pub hb3: HertzU32,
    /// R1 (RX) / T1 (TX)
    pub hb2: HertzU32,
    /// CLKRF (RX) / CLKTF (TX)
    pub hb1: HertzU32,
    /// sample rate
    pub sample: HertzU32,
}

impl PathClocks {
    /// no-OS default RX path, 30.72 MSPS.
    pub const DEFAULT_RX: Self = Self {
        bbpll: HertzU32::Hz(983_040_000),
        converter: HertzU32::Hz(245_760_000),
        hb3: HertzU32::Hz(122_880_000),
        hb2: HertzU32::Hz(61_440_000),
        hb1: HertzU32::Hz(30_720_000),
        sample: HertzU32::Hz(30_720_000),
    };
    /// no-OS default TX path, 30.72 MSPS.
    pub const DEFAULT_TX: Self = Self {
        bbpll: HertzU32::Hz(983_040_000),
        converter: HertzU32::Hz(122_880_000),
        hb3: HertzU32::Hz(122_880_000),
        hb2: HertzU32::Hz(61_440_000),
        hb1: HertzU32::Hz(30_720_000),
        sample: HertzU32::Hz(30_720_000),
    };

    const fn as_array(&self) -> [HertzU32; 6] {
        [self.bbpll, self.converter, self.hb3, self.hb2, self.hb1, self.sample]
    }
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    /// Sets BBPLL and datapath clocks, no digital interface tuning.
    pub(super) async fn apply_trx_clock_chain(
        &mut self,
        rx: &PathClocks,
        tx: &PathClocks,
    ) -> Result<(), Ad9361Error<S::Error>> {
        validate_trx_clock_chain(rx, tx, self.mode.rx2tx2, self.mode.lvds_mode)?;

        self.set_bbpll_rate(rx.bbpll).await?;

        let stages = [
            (Ad9361Clock::Adc, Ad9361Clock::Dac, rx.converter, tx.converter),
            (Ad9361Clock::R2, Ad9361Clock::T2, rx.hb3, tx.hb3),
            (Ad9361Clock::R1, Ad9361Clock::T1, rx.hb2, tx.hb2),
            (Ad9361Clock::ClkRf, Ad9361Clock::ClkTf, rx.hb1, tx.hb1),
            (Ad9361Clock::RxSampl, Ad9361Clock::TxSampl, rx.sample, tx.sample),
        ];
        for (rx_clk, tx_clk, rx_rate, tx_rate) in stages {
            self.set_clock_rate(rx_clk, rx_rate)?;
            self.set_clock_rate(tx_clk, tx_rate)?;
        }

        // the rates don't change when a FIR gets enabled or bypassed, so nothing would flip it
        // for us. do it by hand
        if self.fir.rx_decimation() == 1 {
            let enable = !self.fir.rx_bypassed();
            self.modify_reg::<RxEnableFilterControl>(|reg| {
                reg.with_rx_fir_enable_decimation(u2::new(enable as u8))
            })?;
        }
        if self.fir.tx_interpolation() == 1 {
            let enable = !self.fir.tx_bypassed();
            self.modify_reg::<TxEnableFilterControl>(|reg| {
                reg.with_tx_fir_enable_interpolation(u2::new(enable as u8))
            })?;
        }
        Ok(())
    }

    /// `ad9361_set_trx_clock_chain()`.
    pub async fn set_trx_clock_chain(
        &mut self,
        rx: &PathClocks,
        tx: &PathClocks,
    ) -> Result<(), Ad9361Error<S::Error>> {
        self.apply_trx_clock_chain(rx, tx).await?;

        // An enabled FIR shifts the interface timing. Usually harmless, but at 61.44 MSPS it
        // breaks some setups, so tune whenever a FIR is on. With FIRs off, put the original
        // delays back. Result is ignored, same as no-OS.
        if !self.tune.dig_interface_tune_fir_disable && !(self.fir.tx_bypassed() && self.fir.rx_bypassed()) {
            let flags = DigTuneFlags {
                skip_store_result: true,
                ..Default::default()
            };
            let _ = self.dig_tune(HertzU32::from_raw(0), flags).await;
        }
        self.bb_clk_change_handler().await
    }

    /// Refreshes everything that depends on the baseband rates. No retuning.
    pub(super) fn bb_clk_update(&mut self) -> Result<(), Ad9361Error<S::Error>> {
        self.gc_update()?;
        let rssi = self.cal.rssi_ctrl;
        self.rssi_setup(&rssi, true)?;
        let auxadc = self.cal.auxadc_config;
        self.auxadc_setup(&auxadc)
    }

    /// `ad9361_bb_clk_change_handler()`. Call after any BBPLL or datapath clock change.
    pub async fn bb_clk_change_handler(&mut self) -> Result<(), Ad9361Error<S::Error>> {
        self.bb_clk_update()?;
        // retune so DATA_CLK timing survives sample rate switches
        if self.tune.bb_clk_change_dig_tune_en {
            let flags = DigTuneFlags {
                restore_default: true,
                ..Default::default()
            };
            self.dig_tune(HertzU32::from_raw(0), flags).await?;
        }
        Ok(())
    }
}
