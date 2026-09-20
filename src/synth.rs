//! BBPLL and the RX/TX RF synths.

use super::*;

/// BBPLL word burst start. INTEGER_BB_FREQ_WORD, then FRACT_BB_FREQ_WORD_3, _2, _1 (addresses
/// count down).
pub(super) const BBPLL_FREQ_WORD_ADDR: u10 = u10::new(0x44);
/// RX synth word burst start, RX_FRACT_BYTE_2 down to RX_INTEGER_BYTE_0.
pub(super) const RX_SYNTH_WORD_ADDR: u10 = u10::new(0x235);
/// TX synth word burst start, TX_FRACT_BYTE_2 down to TX_INTEGER_BYTE_0.
pub(super) const TX_SYNTH_WORD_ADDR: u10 = u10::new(0x275);

/// BBPLL integer and fractional words for `rate` off `parent`, both in Hz.
pub(super) fn bbpll_words(rate: u64, parent: u64) -> (u32, u32) {
    let integer = rate / parent;
    let fract = ((rate % parent) * BBPLL_MODULUS as u64 + (parent >> 1)) / parent;
    (integer as u32, fract as u32)
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    /// RX synth register address, or the TX one, which sits 0x40 higher.
    pub(super) fn synth_addr<Reg: Register>(tx: bool) -> u10 {
        u10::new(Reg::ADDRESS.value() + if tx { 0x40 } else { 0 })
    }

    /// Writes the RX synth register, or the TX twin with `tx`.
    pub(super) fn write_synth_reg<Reg: Register>(&mut self, reg: Reg, tx: bool) -> Result<(), S::Error> {
        self.write_bytes(&[reg.to_raw()], Self::synth_addr::<Reg>(tx))
    }

    /// Charge pump cal for the RX or TX synth (`ad9361_txrx_synth_cp_calib()`).
    pub async fn txrx_synth_cp_calib(
        &mut self,
        ref_clk: HertzU32,
        tx: bool,
    ) -> Result<(), Ad9361Error<S::Error>> {
        // copied from no-OS, where they're marked "REVIST"
        self.write_synth_reg(RxCpLevelDetect::from_raw(0x17), tx)?;
        self.write_synth_reg(RxDsmSetup1::from_raw(0x00), tx)?;
        self.write_synth_reg(RxLoGenPowerMode::from_raw(0x00), tx)?;
        self.write_synth_reg(RxVcoLdo::from_raw(0x0B), tx)?;
        self.write_synth_reg(RxVcoPdOverrides::from_raw(0x02), tx)?;
        self.write_synth_reg(RxCpCurrent::from_raw(0x80), tx)?;
        self.write_synth_reg(RxCpConfig::default().with_cp_offset_off(true), tx)?;

        // See Table 70 "Example Calibration Times for RF VCO Cal"
        let count = if self.mode.fdd {
            3
        } else if ref_clk > HertzU32::Hz(40_000_000) {
            1
        } else {
            0
        };
        self.write_synth_reg(
            RxVcoCal::default()
                .with_vco_cal_en(true)
                .with_vco_cal_count(u2::new(count))
                .with_fb_clock_adv(u2::new(2)),
            tx,
        )?;

        // needs FDD during cal. stored conf3 stays as is, so `pp_port_restore_conf3()` puts
        // half duplex back
        if !self.mode.fdd {
            self.modify_reg::<ParallelPortConf3>(|reg| reg.with_half_duplex_mode(false))?;
        }

        self.write_reg(EnsmConfig2::default().with_dual_synth_mode(true))?;
        self.write_reg(
            EnsmConfig1::default()
                .with_force_alert_state(true)
                .with_to_alert(true),
        )?;
        self.write_reg(EnsmMode::default().with_fdd_mode(true))?;

        self.write_synth_reg(
            RxCpConfig::default()
                .with_cp_offset_off(true)
                .with_cp_cal_enable(true),
            tx,
        )?;

        self.wait_until::<RxCalStatus>(
            Self::synth_addr::<RxCalStatus>(tx),
            |reg| reg.cp_cal_valid(),
            120,
        )
        .await
    }

    pub(super) fn read_synth_reg<Reg: Register>(&mut self, tx: bool) -> Result<Reg, S::Error> {
        let mut raw = [0u8; 1];
        self.read_bytes(&mut raw, Self::synth_addr::<Reg>(tx))?;
        Ok(Reg::from_raw(raw[0]))
    }

    pub(super) fn modify_synth_reg<Reg: Register>(
        &mut self,
        tx: bool,
        f: impl FnOnce(Reg) -> Reg,
    ) -> Result<(), S::Error> {
        let reg = self.read_synth_reg::<Reg>(tx)?;
        self.write_synth_reg(f(reg), tx)
    }

    /// VCO and loop filter settings for `vco_freq` from the tables (`ad9361_rfpll_vco_init()`).
    pub(super) fn rfpll_vco_init(&mut self, tx: bool, vco_freq: u64, ref_clk: HertzU32) -> Result<(), S::Error> {
        let range = match ref_clk.to_raw() {
            0..50_000_000 => 0,
            50_000_000..=70_000_000 => 1,
            _ => 2,
        };
        let vco_mhz = vco_freq / 1_000_000;

        // FDD with different RX/TX LOs uses the FDD tables
        let fdd_table = self.mode.fdd
            && !self.mode.fdd_independent_mode
            && self.clk.current_tx_lo_freq != self.clk.current_rx_lo_freq;
        let table = if fdd_table {
            &SYNTH_LUT_FDD[range]
        } else {
            &SYNTH_LUT_TDD[range]
        };
        if tx {
            self.clk.current_tx_use_tdd_table = !fdd_table;
        } else {
            self.clk.current_rx_use_tdd_table = !fdd_table;
        }

        let idx = table
            .iter()
            .position(|entry| entry.vco_mhz as u64 <= vco_mhz)
            .unwrap_or(SYNTH_LUT_SIZE - 1);
        let e = table[idx];

        self.write_synth_reg(
            RxVcoOutput::default()
                .with_vco_output_level(u4::new(e.vco_output_level))
                .with_porb_vco_logic(true),
            tx,
        )?;
        self.modify_synth_reg::<RxAlcVaractor>(tx, |reg| {
            reg.with_vco_varactor(u4::new(e.vco_varactor))
        })?;
        self.write_synth_reg(
            RxVcoBias1::default()
                .with_vco_bias_ref(u3::new(e.vco_bias_ref))
                .with_vco_bias_tcf(u2::new(e.vco_bias_tcf)),
            tx,
        )?;
        self.write_synth_reg(
            RxForceVcoTune1::default().with_vco_cal_offset(u4::new(e.vco_cal_offset)),
            tx,
        )?;
        self.write_synth_reg(
            RxVcoVaractorControl1::default()
                .with_vco_varactor_reference(u4::new(e.vco_varactor_reference)),
            tx,
        )?;
        self.write_synth_reg(RxVcoCalRef::default().with_vco_cal_ref_tcf(u3::new(0)), tx)?;
        self.write_synth_reg(
            RxVcoVaractorControl0::default()
                .with_vco_varactor_offset(u4::new(0))
                .with_vco_varactor_reference_tcf(u3::new(7)),
            tx,
        )?;
        self.modify_synth_reg::<RxCpCurrent>(tx, |reg| {
            reg.with_charge_pump_current(u6::new(e.charge_pump_current))
        })?;
        self.write_synth_reg(
            RxLoopFilter1::default()
                .with_loop_filter_c2(u4::new(e.lf_c2))
                .with_loop_filter_c1(u4::new(e.lf_c1)),
            tx,
        )?;
        self.write_synth_reg(
            RxLoopFilter2::default()
                .with_loop_filter_r1(u4::new(e.lf_r1))
                .with_loop_filter_c3(u4::new(e.lf_c3)),
            tx,
        )?;
        self.write_synth_reg(RxLoopFilter3::default().with_loop_filter_r3(u4::new(e.lf_r3)), tx)?;
        Ok(())
    }

    /// Programs the synth to `freq` off `parent` and waits for lock (`ad9361_rfpll_int_set_rate()`).
    /// No fastlock un-prepare step like the C driver, we don't have fastlock.
    pub(super) async fn program_rfpll(
        &mut self,
        tx: bool,
        freq: u64,
        parent: HertzU32,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let min = if tx { MIN_TX_CARRIER_FREQ_HZ } else { MIN_RX_CARRIER_FREQ_HZ };
        if !(min..=MAX_CARRIER_FREQ_HZ).contains(&freq) || parent.to_raw() == 0 {
            return Err(Ad9361Error::InvalidRate);
        }
        let parent64 = parent.to_raw() as u64;
        let words = |lo: u64| {
            // double until the VCO is above its minimum
            let mut vco = lo;
            let mut div = -1i32;
            while vco <= MIN_VCO_FREQ_HZ {
                vco <<= 1;
                div += 1;
            }
            let integer = vco / parent64;
            let fract = ((vco % parent64) * RFPLL_MODULUS as u64 + (parent64 >> 1)) / parent64;
            (integer as u32, fract as u32, div as u8, vco)
        };
        let (mut integer, mut fract, mut vco_div, mut vco) = words(freq);
        if tx {
            self.clk.current_tx_lo_freq = Some(freq);
        } else {
            self.clk.current_rx_lo_freq = Some(freq);
        }

        // TDD can skip VCO cal on the way from TX/RX to alert
        if self.mode.tdd_skip_vco_cal {
            self.modify_synth_reg::<RxPfdConfig>(tx, |reg| reg.with_bypass_ld_synth(false))?;
        }

        let mut synth_tx = tx;
        let lock = loop {
            self.rfpll_vco_init(synth_tx, vco, parent)?;

            // upper integer bits share the register with other stuff
            let upper = self.read_synth_reg::<RxIntegerByte1>(synth_tx)?.raw_value() & !0x07;
            let buf = [
                (fract >> 16) as u8 & 0x7F,
                (fract >> 8) as u8,
                fract as u8,
                ((integer >> 8) as u8 & 0x07) | upper,
                integer as u8,
            ];
            let start = if synth_tx { TX_SYNTH_WORD_ADDR } else { RX_SYNTH_WORD_ADDR };
            self.write_bytes(&buf, start)?;
            self.modify_reg::<RfPllDividers>(|reg| {
                if synth_tx {
                    reg.with_tx_vco_divider(u4::new(vco_div))
                } else {
                    reg.with_rx_vco_divider(u4::new(vco_div))
                }
            })?;

            let lock = self
                .wait_until::<RxCpOverrangeVcoLock>(
                    Self::synth_addr::<RxCpOverrangeVcoLock>(synth_tx),
                    |reg| reg.vco_lock(),
                    120,
                )
                .await;

            // FDD with RX LO == TX LO uses the TDD tables (less VCO pulling), so the other synth
            // has to be reprogrammed to match
            let shared = self.mode.fdd && !self.mode.fdd_independent_mode;
            let same_lo = self.clk.current_tx_lo_freq == self.clk.current_rx_lo_freq;
            let tables_differ = self.clk.current_tx_use_tdd_table != self.clk.current_rx_use_tdd_table;
            let any_tdd = self.clk.current_tx_use_tdd_table || self.clk.current_rx_use_tdd_table;
            if !(shared && ((same_lo && tables_differ) || (!same_lo && any_tdd))) {
                break lock;
            }
            synth_tx = !synth_tx;
            if !same_lo {
                // never programmed counts as 0 Hz, like no-OS
                let other = if synth_tx { self.clk.current_tx_lo_freq } else { self.clk.current_rx_lo_freq };
                (integer, fract, vco_div, vco) = words(other.unwrap_or(0));
            }
        };

        if self.mode.tdd_skip_vco_cal {
            self.modify_synth_reg::<RxPfdConfig>(tx, |reg| reg.with_bypass_ld_synth(true))?;
        }

        lock
    }

    /// Sets the RX or TX LO and updates the cached rate (`clk_set_rate()` on the RFPLL clocks).
    ///
    /// RX: also loads the gain table for the new band. TX: runs the TX quad cal if the LO moved
    /// more than the threshold.
    ///
    /// TODO: the C driver triggers external band switching here too.
    pub async fn set_rfpll_rate(
        &mut self,
        tx: bool,
        freq: HertzU64,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let cached = if tx { self.clk.rates.tx_rfpll } else { self.clk.rates.rx_rfpll };
        if cached == freq {
            return Ok(());
        }
        let parent = if tx { self.clk.rates.tx_ref } else { self.clk.rates.rx_ref };
        self.program_rfpll(tx, freq.to_raw(), parent).await?;
        if !tx {
            self.load_gain_table(freq.to_raw(), GainTableDest::Both)?;
        }

        let rate = self.read_rfpll_scaler(tx)?.rate_from_parent(parent);
        if tx {
            self.clk.rates.tx_rfpll = rate;
        } else {
            self.clk.rates.rx_rfpll = rate;
        }

        // RX LO changes are covered by tracking cal, nothing to do
        if tx
            && self.cal.auto_cal_en
            && self.cal.last_tx_quad_cal_freq.abs_diff(freq.to_raw()) > self.cal.cal_threshold_freq
        {
            let result = self.tx_quad_calib_run().await;
            self.cal.last_tx_quad_cal_freq = freq.to_raw();
            return result;
        }
        Ok(())
    }

    /// Polls until `done` (`ad9361_check_cal_done()`).
    pub(super) async fn wait_until<Reg: Register>(
        &mut self,
        address: u10,
        done: impl Fn(Reg) -> bool,
        period_us: u64,
    ) -> Result<(), Ad9361Error<S::Error>> {
        for _ in 0..20_000 {
            let mut raw = [0u8; 1];
            self.read_bytes(&mut raw, address)?;
            if done(Reg::from_raw(raw[0])) {
                return Ok(());
            }
            Timer::after_micros(period_us).await;
        }
        Err(Ad9361Error::Timeout)
    }

    /// Programs the BBPLL and waits for lock (`ad9361_bbpll_set_rate()`). `rate` has to be
    /// validated already.
    pub(super) async fn program_bbpll(
        &mut self,
        rate: HertzU32,
        parent: HertzU32,
    ) -> Result<(), Ad9361Error<S::Error>> {
        let (rate, parent) = (rate.to_raw() as u64, parent.to_raw() as u64);
        // loop filter + charge pump current. 150 uA at 1280 MHz BBPLL / 40 MHz REFCLK
        if parent < 128 {
            return Err(Ad9361Error::InvalidRate);
        }
        let icp = (((rate >> 7) * 150) / ((parent >> 7) * 32)) as u32;
        // 25 uA/LSB, offset 25 uA
        let icp = ((icp + 12) / 25).saturating_sub(1).clamp(1, 63);
        let (integer, fract) = bbpll_words(rate, parent);
        let integer = u8::try_from(integer).map_err(|_| Ad9361Error::InvalidRate)?;

        self.write_reg(CpCurrent::default().with_charge_pump_current(u6::new(icp as u8)))?;
        // LOOP_FILTER_3, _2, _1 (burst counts down)
        self.write_bytes(&[0x35, 0x5B, 0xE8], LoopFilter3::ADDRESS)?;
        // allow cal, count 1024 for best accuracy
        self.write_reg(
            VcoControl::default()
                .with_freq_cal_enable(true)
                .with_freq_cal_count_length(u2::new(3)),
        )?;
        // cal clock REFCLK/4, more accurate
        self.write_reg(SdmControl::default().with_cal_clock_div_4(true))?;

        self.write_reg(IntegerBbFreqWord(integer))?;
        self.write_reg(FractBbFreqWord3(fract as u8))?;
        self.write_reg(FractBbFreqWord2((fract >> 8) as u8))?;
        self.write_reg(FractBbFreqWord1((fract >> 16) as u8))?;

        // start cal, then clear the bit
        self.write_reg(
            SdmControl1::default()
                .with_init_bb_fo_cal(true)
                .with_bbpll_reset_bar(true),
        )?;
        self.write_reg(SdmControl1::default().with_bbpll_reset_bar(true))?;

        // more BBPLL KV and phase margin
        self.write_reg(VcoProgram1(0x86))?;
        self.write_reg(VcoProgram2(0x01))?;
        self.write_reg(VcoProgram2(0x05))?;

        self.wait_until::<Ch1Overflow>(Ch1Overflow::ADDRESS, |reg| reg.bbpll_lock(), 120)
            .await
    }

    /// Sets the BBPLL to the closest rate the cached BB reference allows, within
    /// [`MIN_BBPLL_FREQ`]..=[`MAX_BBPLL_FREQ`]. Updates the cache.
    pub async fn set_bbpll_rate(&mut self, rate: HertzU32) -> Result<(), Ad9361Error<S::Error>> {
        if self.clk.rates.bbpll == rate {
            return Ok(());
        }
        let parent = self.clk.rates.bb_ref;
        if parent.to_raw() == 0 {
            return Err(Ad9361Error::InvalidRate);
        }
        let rate = rate.clamp(MIN_BBPLL_FREQ, MAX_BBPLL_FREQ);
        let (integer, fract) = bbpll_words(rate.to_raw() as u64, parent.to_raw() as u64);
        let rounded = PllScaler { fract, integer }.rate_from_parent(parent);

        self.program_bbpll(rounded, parent).await?;
        self.clk.rates.bbpll = self.read_bbpll_clock_scaler()?.rate_from_parent(parent);
        Ok(())
    }
}
