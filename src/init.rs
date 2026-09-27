//! The `ad9361_init()` sequence after reset and the ID check, and the data interface glue.

use embedded_hal_async::spi::SpiDevice;

use super::interface::{DataInterface, PortLayout, TxSource};
use super::{Ad9361Config, Ad9361Error, Engine};

impl<S, I> Engine<S, I>
where
    S: SpiDevice<u8>,
    I: DataInterface,
{
    /// What the FPGA side needs to know about the data port.
    pub(crate) fn port_layout(&self) -> PortLayout {
        PortLayout {
            two_channels: self.mode.rx2tx2,
            lvds: self.mode.lvds_mode,
            half_tx_rate: self.tune.axi_half_dac_rate_en,
        }
    }

    /// Chip setup, then the data interface: RX side up, port tuned, TX side up.
    pub(super) async fn init(&mut self, config: &Ad9361Config) -> Result<(), Ad9361Error<S::Error>> {
        self.setup(config).await?;
        let layout = self.port_layout();
        self.interface.init(layout).await.map_err(Ad9361Error::Interface)?;
        self.post_setup().await?;
        if let Some(tx_fir) = &config.settings.tx_fir {
            self.set_tx_fir_config(tx_fir).await?;
        }
        if let Some(rx_fir) = &config.settings.rx_fir {
            self.set_rx_fir_config(rx_fir).await?;
        }
        let sample_rate = self.clk.rates.tx_sample;
        self.interface
            .start_tx(layout, sample_rate)
            .await
            .map_err(Ad9361Error::Interface)?;
        Ok(())
    }

    /// TX channels the DMA data is interleaved for.
    pub(crate) fn dac_num_tx_channels(&self) -> usize {
        self.port_layout().lanes() as usize / 2
    }

    pub(crate) fn set_tx_source(&mut self, source: TxSource) {
        let layout = self.port_layout();
        self.interface.set_tx_source(layout, source);
    }
}
