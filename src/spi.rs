//! SPI access.

use super::*;

/// Pulse reset.
pub(super) async fn reset<R: embedded_hal::digital::OutputPin>(resetb: &mut R) -> Result<(), R::Error> {
    resetb.set_low()?;
    Timer::after_millis(1).await;
    resetb.set_high()?;
    Timer::after_millis(1).await;
    Ok(())
}

/// Writes 1-8 bytes, addresses count down from `addr`.
pub(super) fn write_bytes<S: SpiDevice<u8>>(
    spi: &mut S,
    data: &[u8],
    addr: u10,
) -> Result<(), S::Error> {
    assert!(!data.is_empty() && data.len() <= 8);

    // bit 15 = write, 14:12 = byte count - 1, 9:0 = address
    let ctrl_field = [
        (1u8 << 7) | ((data.len() as u8 - 1) << 4) | ((addr.value() >> 8) & 0b11) as u8,
        (addr.value() & 0xFF) as u8,
    ];

    spi.transaction(&mut [
        embedded_hal::spi::Operation::Write(&ctrl_field),
        embedded_hal::spi::Operation::Write(data),
    ])
}

/// Reads 1-8 bytes, addresses count down from `addr`.
pub(super) fn read_bytes<S: SpiDevice<u8>>(
    spi: &mut S,
    out: &mut [u8],
    addr: u10,
) -> Result<(), S::Error> {
    assert!(!out.is_empty() && out.len() <= 8);

    // bit 15 clear = read, 11:10 unused
    let ctrl_field = [
        ((out.len() as u8 - 1) << 4) | ((addr.value() >> 8) & 0b11) as u8,
        (addr.value() & 0xFF) as u8,
    ];

    spi.transaction(&mut [
        embedded_hal::spi::Operation::Write(&ctrl_field),
        embedded_hal::spi::Operation::Read(out),
    ])
}

pub(super) fn read_reg<S: SpiDevice<u8>, Reg: Register>(spi: &mut S) -> Result<Reg, S::Error> {
    let mut raw = [0u8; 1];
    read_bytes(spi, &mut raw, Reg::ADDRESS)?;
    Ok(Reg::from_raw(raw[0]))
}

impl<S> Engine<S>
where
    S: SpiDevice<u8>,
{
    pub fn write_bytes(&mut self, data: &[u8], addr: u10) -> Result<(), S::Error> {
        write_bytes(&mut self.spi, data, addr)
    }

    pub fn write_reg<Reg: Register>(&mut self, reg: Reg) -> Result<(), S::Error> {
        self.write_bytes(&[reg.to_raw()], Reg::ADDRESS)
    }

    pub fn read_bytes(&mut self, out: &mut [u8], addr: u10) -> Result<(), S::Error> {
        read_bytes(&mut self.spi, out, addr)
    }

    pub fn read_reg<Reg: Register>(&mut self) -> Result<Reg, S::Error> {
        read_reg(&mut self.spi)
    }

    /// Read, change, write back.
    pub fn modify_reg<Reg: Register>(
        &mut self,
        f: impl FnOnce(Reg) -> Reg,
    ) -> Result<(), S::Error> {
        let reg = self.read_reg::<Reg>()?;
        self.write_reg(f(reg))
    }
}
