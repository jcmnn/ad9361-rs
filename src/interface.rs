//! The FPGA side of the data port: [`DataInterface`], what the driver needs from it.
//!
//! The driver brings the interface up during init, and uses its PN checkers and PN generators
//! to tune the data port delays. [`AxiCores`](crate::AxiCores) implements it for ADI's
//! AXI AD9361 IP core (`axi` feature, on by default).

/// How the chip's data port is set up, so the FPGA side can match it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PortLayout {
    /// Both RX and both TX channels (2R2T), otherwise one of each (1R1T)
    pub two_channels: bool,
    /// LVDS, otherwise CMOS
    pub lvds: bool,
    /// [`Ad9361Settings::axi_half_dac_rate_en`](crate::Ad9361Settings::axi_half_dac_rate_en)
    pub half_tx_rate: bool,
}

impl PortLayout {
    /// Data lanes per direction, an I and a Q per channel.
    pub const fn lanes(self) -> u8 {
        if self.two_channels { 4 } else { 2 }
    }
}

/// Where the TX data comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxSource {
    /// Tone generators in the FPGA
    Dds,
    /// Sample data, usually from a DMA
    Dma,
}

/// The FPGA side didn't come up, usually because the data clock from the AD9361 is missing or
/// the interface doesn't lock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterfaceError;

impl core::fmt::Display for InterfaceError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("the FPGA data interface reported errors")
    }
}

impl core::error::Error for InterfaceError {}

/// The FPGA side of the AD9361 data port.
///
/// The driver calls these in this order: [`Self::init`] once the AD9361 drives the data clock,
/// the PN methods while tuning the port delays, then [`Self::start_tx`]. The PN methods run
/// again whenever the port is retuned, for example after a sample rate change.
#[allow(async_fn_in_trait, reason = "the driver itself isn't `Send` either")]
pub trait DataInterface {
    /// Brings up the RX side and sets both sides up for `layout`. Can wait for the clock to
    /// settle.
    async fn init(&mut self, layout: PortLayout) -> Result<(), InterfaceError>;

    /// Brings up the TX side after tuning. `sample_rate` is the TX sample rate, for anything
    /// the interface wants to play before real data arrives.
    async fn start_tx(
        &mut self,
        layout: PortLayout,
        sample_rate: fugit::HertzU32,
    ) -> Result<(), InterfaceError>;

    /// Picks the TX data source.
    fn set_tx_source(&mut self, layout: PortLayout, source: TxSource);

    /// Clears the PN checker errors on every RX lane.
    fn clear_pn_errors(&mut self, layout: PortLayout);

    /// Whether a PN checker saw errors since [`Self::clear_pn_errors`]. With
    /// `require_rx_lock`, a PN checker that isn't locked to the sequence counts as an error.
    fn pn_errors(&mut self, layout: PortLayout, require_rx_lock: bool) -> bool;

    /// Makes the RX side latch the data again, after the port delays changed.
    fn relatch_rx(&mut self);

    /// Sends a PN sequence on every TX lane for the TX tuning, which the AD9361 loops back to
    /// the PN checkers. Save whatever [`Self::stop_tx_pn`] needs to undo it.
    fn start_tx_pn(&mut self, layout: PortLayout);

    /// Back to what the TX and RX lanes did before [`Self::start_tx_pn`].
    fn stop_tx_pn(&mut self, layout: PortLayout);
}
