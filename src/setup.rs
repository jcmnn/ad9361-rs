//! `ad9361_setup()`.

use super::*;

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    pub(super) async fn setup(&mut self, config: &Ad9361Config) -> Result<(), Ad9361Error<S::Error>> {
        let settings = &config.settings;
        self.auxdac_setup(&settings.auxdac)?;
        self.gpo_setup(&settings.gpo)?;

        self.write_reg(Control::default().with_ctrl_enable(true))?;
        self.write_reg(BandgapConfig0::default().with_master_bias_trim(u5::new(0x0E)))?;
        self.write_reg(BandgapConfig1::default().with_bandgap_temp_trim(u5::new(0x0E)))?;

        self.set_dcxo_tune(settings.dcxo_coarse_tune, settings.dcxo_fine_tune)?;

        self.modify_reg::<RefDivideConfig1>(|reg| reg.with_rx_ref_reset_bar(true))?;
        self.modify_reg::<RefDivideConfig2>(|reg| {
            reg.with_tx_ref_reset_bar(true)
                .with_tx_ref_doubler_fb_delay(u2::new(3))
                .with_rx_ref_doubler_fb_delay(u2::new(3))
        })?;

        self.write_reg(
            ClockEnable::default()
                .with_digital_power_up(true)
                .with_clock_enable_dflt(true)
                .with_bbpll_enable(true)
                .with_xo_bypass(settings.use_external_clock),
        )?;

        self.set_clock_rate(Ad9361Clock::BbRef, config.bbpll_ref)?;

        self.write_reg(FractBbFreqWord2(0x12))?;
        self.write_reg(FractBbFreqWord3(0x34))?;

        self.set_trx_clock_chain(&settings.rx_path_clks, &settings.tx_path_clks)
            .await?;

        match settings.channels {
            ChannelMode::TwoByTwo => {
                self.set_tx_channels(true, true)?;
                self.set_rx_channels(true, true)?;
            }
            ChannelMode::OneByOne { rx, tx } => {
                self.set_tx_channels(tx == Channel::Ch1, tx == Channel::Ch2)?;
                self.set_rx_channels(rx == Channel::Ch1, rx == Channel::Ch2)?;
            }
        }

        self.rf_port_setup(settings.rf_rx_input_sel, settings.rf_tx_output_b)?;
        self.pp_port_setup(&settings.port)?;

        self.auxadc_setup(&settings.auxadc)?;
        self.ctrl_outs_setup(&settings.ctrl_outs)?;
        self.set_ref_clk_cycles(config.ref_clk)?;
        self.setup_ext_lna(&settings.elna)?;

        let synth_ref = config.synth_ref;
        self.set_clock_rate(Ad9361Clock::RxRef, synth_ref)?;
        self.set_clock_rate(Ad9361Clock::TxRef, synth_ref)?;
        self.txrx_synth_cp_calib(synth_ref, false).await?;
        self.txrx_synth_cp_calib(synth_ref, true).await?;

        // load the table up front, set_rfpll_rate then sees it's already there
        self.write_gain_table(self.gain.current_table, GainTableDest::Both)?;
        self.set_rfpll_rate(false, settings.rx_synth_freq.get()).await?;
        // TX quad cal comes later
        self.set_rfpll_rate(true, settings.tx_synth_freq.get()).await?;

        self.load_mixer_gm_subtable()?;
        self.gc_setup(&settings.gain_ctrl)?;

        // config is RF bandwidth, the baseband filters only see half
        let real_rx_bw = settings.rf_rx_bandwidth.get() / 2;
        let real_tx_bw = settings.rf_tx_bandwidth.get() / 2;
        let bbpll = self.clk.rates.bbpll;
        let rxbbf_div = self.rx_bb_analog_filter_calib(real_rx_bw, bbpll).await?;
        self.tx_bb_analog_filter_calib(real_tx_bw, bbpll).await?;
        self.rx_tia_calib(real_rx_bw)?;
        self.tx_bb_second_filter_calib(real_tx_bw)?;
        self.rx_adc_setup(bbpll, self.clk.rates.adc, rxbbf_div)?;

        self.bb_dc_offset_calib().await?;
        self.rf_dc_offset_calib(self.clk.rates.rx_rfpll.to_raw()).await?;

        self.tx_quad_calib(real_rx_bw.to_raw(), real_tx_bw.to_raw(), RxPhase::Auto)
            .await?;

        self.tracking_control(&settings.tracking)?;
        self.pp_port_restore_conf3()?;

        let ensm_pin_ctrl = settings.ensm_pin_ctrl;
        self.set_ensm_mode(self.mode.fdd, ensm_pin_ctrl)?;

        self.modify_reg::<TxAttenOffset>(|reg| reg.with_mask_clr_atten_update(false))?;
        let (tx1, tx2) = match settings.channels {
            ChannelMode::TwoByTwo => (true, true),
            ChannelMode::OneByOne { tx, .. } => (tx == Channel::Ch1, tx == Channel::Ch2),
        };
        self.set_tx_atten(settings.tx_attenuation, tx1, tx2, true)?;
        if !self.mode.rx2tx2 {
            // mute the unused one
            self.set_tx_atten(TxAttenuation::MAX, tx2, tx1, true)?;
        }

        self.rssi_setup(&settings.rssi, false)?;
        self.rssi_gain_step_calib().await?;
        self.clkout_control(settings.clkout_mode)?;
        self.txmon_setup(&settings.txmon)?;

        self.ensm.current = self.ensm_state()?;
        let running = if self.mode.fdd { RequestedEnsmState::Fdd } else { RequestedEnsmState::Rx };
        self.ensm_set_state(running, ensm_pin_ctrl).await?;

        self.cal.auto_cal_en = true;
        self.cal.cal_threshold_freq = 100_000_000;

        Ok(())
    }
}
