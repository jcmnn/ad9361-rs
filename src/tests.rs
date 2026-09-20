//! Host tests. Pure functions, config checks, and register behavior against a fake AD9361 on
//! SPI.

use core::convert::Infallible;
use std::vec::Vec;

use embedded_hal::spi::{ErrorType, Operation, SpiDevice};
use fugit::HertzU32;

use super::*;

/// Fake AD9361 register file. Bit 15 of the first word = write, 14:12 = byte count - 1, 9:0 =
/// address, counting down per byte.
struct MockSpi {
    regs: [u8; 1024],
    /// every write, (address, value)
    writes: Vec<(u16, u8)>,
}

impl MockSpi {
    fn new() -> Self {
        Self { regs: [0; 1024], writes: Vec::new() }
    }
}

impl ErrorType for MockSpi {
    type Error = Infallible;
}

impl SpiDevice<u8> for MockSpi {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Infallible> {
        let [Operation::Write(header), data] = operations else {
            panic!("expected a header and a data operation");
        };
        let write = header[0] & 0x80 != 0;
        let count = ((header[0] >> 4) & 0x7) as usize + 1;
        let addr = (((header[0] & 0x3) as u16) << 8) | header[1] as u16;
        match data {
            Operation::Write(bytes) => {
                assert!(write, "write data with a read header");
                assert_eq!(bytes.len(), count);
                for (i, byte) in bytes.iter().enumerate() {
                    let addr = addr - i as u16;
                    self.regs[addr as usize] = *byte;
                    self.writes.push((addr, *byte));
                }
            }
            Operation::Read(out) => {
                assert!(!write, "read data with a write header");
                assert_eq!(out.len(), count);
                for (i, byte) in out.iter_mut().enumerate() {
                    *byte = self.regs[(addr - i as u16) as usize];
                }
            }
            _ => panic!("unexpected operation"),
        }
        Ok(())
    }
}

const REF_CLK: HertzU32 = HertzU32::Hz(40_000_000);

fn default_config() -> Ad9361Config {
    Ad9361Config::new(Ad9361Settings::default(), ReferenceClock::new(REF_CLK).unwrap()).unwrap()
}

/// Engine on the fake chip. The AXI cores get zeroed memory, nothing here touches it.
fn engine() -> Engine<MockSpi> {
    let memory: &'static mut [u32] = std::boxed::Box::leak(std::vec![0u32; 0x4000].into_boxed_slice());
    let mut axi = axi_ad9361::AxiAd9361::new(memory.as_mut_ptr() as usize);
    let cores = AxiCores {
        adc: axi.take_adc_no_init().unwrap(),
        dac: axi.take_dac_no_init().unwrap(),
        core_id: 0,
    };
    Engine::new(MockSpi::new(), cores, &default_config(), 0).unwrap()
}

#[test]
fn spi_write_and_read_use_the_protocol_header() {
    let mut engine = engine();
    engine.write_bytes(&[0xAB, 0xCD], u10::new(0x274)).unwrap();
    // 0x274, then 0x273
    assert_eq!(engine.spi.writes, [(0x274, 0xAB), (0x273, 0xCD)]);

    let mut out = [0u8; 2];
    engine.read_bytes(&mut out, u10::new(0x274)).unwrap();
    assert_eq!(out, [0xAB, 0xCD]);
}

#[test]
fn tx_attenuation_range_and_steps() {
    assert_eq!(TxAttenuation::from_db(10).mdb(), 10_000);
    assert_eq!(TxAttenuation::from_db(10).quarter_db(), 40);
    assert_eq!(TxAttenuation::MAX.mdb(), 89_750);
    assert!(TxAttenuation::from_mdb(89_750).is_ok());
    assert!(TxAttenuation::from_mdb(90_000).is_err());
    // rounds down
    assert_eq!(TxAttenuation::from_mdb(10_249).unwrap().quarter_db(), 40);
    assert!(TxAttenuation::from_quarter_db(360).is_err());
    assert_eq!(TxAttenuation::saturating_from_quarter_db(1023), TxAttenuation::MAX);
}

#[test]
fn set_tx_atten_writes_both_bytes_msb_first() {
    let mut engine = engine();
    engine.set_tx_atten(TxAttenuation::from_quarter_db(300).unwrap(), true, false, true).unwrap();
    // 300 = 0x12C, 0x74 has bit 8 and 0x73 the low byte
    assert_eq!(engine.spi.regs[0x74], 0x01);
    assert_eq!(engine.spi.regs[0x73], 0x2C);
    // TX2 not touched
    assert_eq!(engine.spi.regs[0x76], 0x00);
    assert_eq!(engine.spi.regs[0x75], 0x00);
}

#[test]
fn reference_clock_range() {
    assert!(ReferenceClock::new(HertzU32::Hz(40_000_000)).is_ok());
    assert!(ReferenceClock::new(HertzU32::Hz(0)).is_err());
    assert!(ReferenceClock::new(HertzU32::Hz(500_000)).is_err());
    assert!(ReferenceClock::new(HertzU32::Hz(200_000_000)).is_err());
}

#[test]
fn default_settings_are_valid_for_the_pluto_reference() {
    default_config();
}

#[test]
fn ensm_pin_control_excludes_fdd_independent_mode() {
    let settings = Ad9361Settings {
        ensm_pin_ctrl: true,
        duplex: Duplex::Fdd { independent_mode: true },
        ..Ad9361Settings::default()
    };
    let result = Ad9361Config::new(settings, ReferenceClock::new(REF_CLK).unwrap());
    assert!(matches!(result, Err(ConfigError::EnsmPinControlInFddIndependentMode)));
}

#[test]
fn synth_reference_must_be_reachable() {
    // 40 MHz / 4 is just over the lowest window (9.999 MHz)
    let settings = Ad9361Settings {
        trx_synth_max_fref: MIN_SYNTH_FREF,
        ..Ad9361Settings::default()
    };
    let result = Ad9361Config::new(settings, ReferenceClock::new(REF_CLK).unwrap());
    assert!(matches!(result, Err(ConfigError::SynthReference)));
}

#[test]
fn clock_chain_beyond_the_limits_is_rejected() {
    let mut rx = PathClocks::DEFAULT_RX;
    rx.converter = HertzU32::Hz(700_000_000);
    let settings = Ad9361Settings { rx_path_clks: rx, ..Ad9361Settings::default() };
    let result = Ad9361Config::new(settings, ReferenceClock::new(REF_CLK).unwrap());
    assert!(matches!(result, Err(ConfigError::ClockChain)));
}

#[test]
fn clock_chain_for_30_72_msps_is_the_default_chain() {
    let engine = engine();
    let (rx, tx) = engine
        .calculate_rf_clock_chain(HertzU32::Hz(30_720_000), RateGovernor::Nominal as u32)
        .unwrap();
    assert_eq!(rx, PathClocks::DEFAULT_RX);
    // DAC runs at the ADC rate when it can, like no-OS. The static chain in main.c uses half
    assert_eq!(
        tx,
        PathClocks { converter: HertzU32::Hz(245_760_000), ..PathClocks::DEFAULT_TX }
    );
}

#[test]
fn clock_chain_rejects_impossible_rates() {
    let engine = engine();
    assert!(engine.calculate_rf_clock_chain(HertzU32::Hz(0), 1).is_err());
    assert!(engine.calculate_rf_clock_chain(HertzU32::Hz(70_000_000), 1).is_err());
}

#[test]
fn bbpll_words_reproduce_the_rate() {
    let (integer, fract) = synth::bbpll_words(983_040_000, 40_000_000);
    assert_eq!(integer, 24);
    let rate = 40_000_000u64 * integer as u64
        + (40_000_000u64 * fract as u64 + BBPLL_MODULUS as u64 / 2) / BBPLL_MODULUS as u64;
    // fractional word resolution is parent / modulus, about 19 Hz
    assert!(rate.abs_diff(983_040_000) <= 40_000_000 / BBPLL_MODULUS as u64 + 1, "{rate}");
}

#[test]
fn gain_table_lookup() {
    let full = gain_control::gain_table_index(false, 2_400_000_000);
    let split = gain_control::gain_table_index(true, 2_400_000_000);
    assert_ne!(full, split);
    // out of range falls back to the first table
    assert_eq!(gain_control::gain_table_index(false, 7_000_000_000), 0);
}

#[test]
fn manual_gain_selects_the_closest_table_entry() {
    let mut engine = engine();
    let table = engine.gain.current_table;
    let abs = gain_tables::GAIN_TABLES[table].abs_gain;

    let actual = engine.set_manual_rx_gain(Channel::Ch1, 30).unwrap();
    assert!((actual - 30).abs() <= 1, "closest entry to 30 dB is {actual} dB");
    let index = abs.iter().position(|g| *g == actual).unwrap();
    assert_eq!(engine.spi.regs[0x109] & 0x7F, index as u8);

    // RX2 has its own register
    engine.set_manual_rx_gain(Channel::Ch2, 10).unwrap();
    assert_ne!(engine.spi.regs[0x10C] & 0x7F, engine.spi.regs[0x109] & 0x7F);

    // above the table, last entry
    let max = engine.set_manual_rx_gain(Channel::Ch1, 120).unwrap();
    assert_eq!(max, *abs.last().unwrap());
    assert_eq!(engine.spi.regs[0x109] & 0x7F, (abs.len() - 1) as u8);
}

#[test]
fn ensm_state_is_saved_and_restored() {
    let mut engine = engine();
    engine.spi.regs[State::ADDRESS.value() as usize] = EnsmState::Fdd.raw();
    let saved = engine.save_ensm_state().unwrap();
    engine.spi.regs[State::ADDRESS.value() as usize] = EnsmState::Alert.raw();
    engine.ensm_restore_state(saved).unwrap();
    let config = engine.read_reg::<EnsmConfig1>().unwrap();
    assert!(config.force_tx_on());
    assert!(!config.force_rx_on());
}

#[test]
fn dcxo_tune_splits_the_fine_value() {
    let mut engine = engine();
    engine.set_dcxo_tune(u6::new(8), u13::new(5920)).unwrap();
    assert_eq!(engine.spi.regs[DcxoCoarseTune::ADDRESS.value() as usize] & 0x3F, 8);
    // 5920 = 0x1720, low 5 bits then the upper 8
    assert_eq!(engine.spi.regs[DcxoFineTuneLow::ADDRESS.value() as usize] & 0x1F, 0x00);
    assert_eq!(engine.spi.regs[DcxoFineTuneHigh::ADDRESS.value() as usize], 0xB9);
}

#[test]
fn fir_state_uses_unit_rates_until_a_filter_is_loaded_and_enabled() {
    use state::{FirState, LoadedFir};

    let mut fir = FirState::default();
    assert!(fir.rx_bypassed() && fir.tx_bypassed());
    assert_eq!((fir.rx_decimation(), fir.tx_interpolation()), (1, 1));

    // loaded, still bypassed
    fir.rx = Some(LoadedFir { factor: FirFactor::X4, ntaps: 64, bypassed: true });
    assert!(fir.rx_bypassed());
    assert_eq!(fir.rx_decimation(), 1);

    fir.rx.as_mut().unwrap().bypassed = false;
    assert!(!fir.rx_bypassed());
    assert_eq!(fir.rx_decimation(), 4);
    // TX doesn't care
    assert_eq!(fir.tx_interpolation(), 1);
}

#[test]
fn engine_state_starts_from_the_configuration() {
    let engine = engine();
    assert!(engine.mode.rx2tx2 && engine.mode.fdd);
    assert_eq!(engine.mode.rx1tx1_use_rx, Channel::Ch1);
    // table for the RX LO (2.4 GHz) is the loaded one
    assert_eq!(engine.gain.current_table, gain_control::gain_table_index(false, 2_400_000_000));
    assert!(engine.clk.current_rx_lo_freq.is_none() && engine.clk.current_tx_lo_freq.is_none());
    assert!(engine.cal.last_tx_quad_cal_phase.is_none());
    assert!(engine.fir.rx.is_none() && engine.fir.tx.is_none());
}

#[test]
fn lvds_port_configuration_is_corrected_to_full_rate_dual_port() {
    let mut port = PortConfig::default();
    port.conf3 = port.conf3.with_lvds_mode(true).with_half_duplex_mode(true).with_single_port_mode(true);
    let conf3 = port.sanitized_conf3();
    assert!(conf3.lvds_mode());
    assert!(!conf3.half_duplex_mode() && !conf3.single_port_mode() && !conf3.single_data_rate());
}

#[test]
fn settings_that_would_be_truncated_are_rejected() {
    let mut settings = Ad9361Settings::default();
    settings.gain_ctrl.mgc_inc_gain_step = 9;
    let result = Ad9361Config::new(settings, ReferenceClock::new(REF_CLK).unwrap());
    assert!(matches!(result, Err(ConfigError::OutOfRange("gain_ctrl.mgc_inc_gain_step"))));

    let mut settings = Ad9361Settings::default();
    settings.rssi.duration = 0;
    let result = Ad9361Config::new(settings, ReferenceClock::new(REF_CLK).unwrap());
    assert!(matches!(result, Err(ConfigError::OutOfRange("rssi.duration"))));
}

#[test]
fn errors_display() {
    use std::string::ToString;
    let error: Ad9361Error<()> = Ad9361Error::FirNotLoaded;
    assert_eq!(error.to_string(), "no FIR filter is loaded");
    let init: InitError<(), ()> = InitError::UnsupportedDevice { product_id: 3 };
    assert!(init.to_string().contains("product ID 3"));
}
