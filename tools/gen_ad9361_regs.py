#!/usr/bin/env python3
"""Generates src/registers.rs from the no-OS ad9361.h register definitions.

Usage: tools/gen_ad9361_regs.py [path/to/ad9361.h] > src/registers.rs
Skipped/adjusted definitions are reported on stderr.
"""
import re
import sys

from adi_notice import rust_notice

HEADER = sys.argv[1] if len(sys.argv) > 1 else \
    "../no-OS/drivers/rf-transceiver/ad9361/ad9361.h"

TOKEN_MAP = {"CTRL": "Control", "RFPLL": "RfPll", "BBPLL": "BbPll"}
# Registers whose fields are defined by hand (header only has whole-byte masks).
SPECIAL = {"REG_PRODUCT_ID"}
# Registers the header gives bogus sub-byte fields (the AuxDAC 2 word MSB is a full byte,
# as the no-OS driver's `val >> 2` write shows).
BYTE_ONLY = {"REG_AUXDAC_2_WORD"}
# Fields the header defines only for one of two mirrored registers (the C driver relies on the
# macro being visible for both).
EXTRA_FIELDS = {"REG_RX_VCO_CAL": [("FB_CLOCK_ADV", 0, 1, "FB Clock Adv<1:0>")]}
# Header typos: (register, field) -> corrected mask (before shifting).
MASK_FIXES = {("REG_AUXDAC_2_CONFIG", "AUXDAC_2_VREF"): 0x3}
KEYWORDS = {"type", "match", "ref", "loop", "in", "as", "mod", "fn", "if", "else",
            "for", "while", "use", "let", "mut", "move", "self", "super", "crate"}


def camel(reg):
    name = reg[len("REG_"):]
    out = []
    for tok in name.split("_"):
        m = re.match(r"^AUXDAC(\d*)$", tok)
        if m:
            out.append("AuxDac" + m.group(1))
        elif tok in TOKEN_MAP:
            out.append(TOKEN_MAP[tok])
        else:
            out.append(tok.capitalize())
    return "".join(out)


lines = open(HEADER, encoding="utf-8", errors="replace").read().splitlines()

# --- register address table
regs = {}  # REG_NAME -> (addr, doc)
order = []
for l in lines:
    m = re.match(r"#define\s+(REG_\w+)\s+(0x[0-9A-Fa-f]+)\s*(?:/\*\s*(.*?)\s*\*/)?", l)
    if m:
        regs[m.group(1)] = (int(m.group(2), 16), m.group(3) or "")
        order.append(m.group(1))

# --- field sections
sections = {}  # REG_NAME -> [(field, lo, hi, doc)]
cur = []
in_sections = False
end_line = next(i for i, l in enumerate(lines) if "SPI Comm Helpers" in l)
i = 0
while i < end_line:
    l = lines[i]
    if re.match(r"^\*\s*REG_", l):
        names = []
        j = i
        while j < end_line and re.match(r"^\*\s*(REG_|\s)", lines[j]) and "*/" not in lines[j]:
            names += re.findall(r"REG_\w+", lines[j])
            # "REG_GAIN_RX1,2" -> also REG_GAIN_RX2
            m = re.search(r"(REG_\w*?)(\d+),(\d+)\s*$", lines[j])
            if m:
                names.append(m.group(1) + m.group(3))
            j += 1
        cur = names
        for n in cur:
            sections.setdefault(n, [])
        i = j
        continue
    if l.startswith("/*") and not l.startswith("/*\n") and cur and l.strip() == "/*":
        cur = []
    if cur:
        m = re.match(r"#define\s+(\w+)\s*(\(x\))?\s+(.*?)\s*(?:/\*\s*(.*?)\s*\*/)?$", l)
        if m:
            fname, has_x, expr, doc = m.group(1), m.group(2), m.group(3), m.group(4) or ""
            lo = hi = None
            if not has_x:
                b = re.fullmatch(r"\(1\s*<<\s*(\d+)\)", expr)
                c = re.fullmatch(r"\((0x[0-9A-Fa-f]+|\d+)\s*<<\s*(\d+)\)", expr)
                if b:
                    lo = hi = int(b.group(1))
                elif c:
                    mask, sh = int(c.group(1), 0), int(c.group(2))
                    lo, hi = sh, sh + mask.bit_length() - 1
            else:
                e = re.fullmatch(r"\(\(\(x\)\s*&\s*(0x[0-9A-Fa-f]+|\d+)\)\s*<<\s*(\d+)\)", expr)
                f = re.fullmatch(r"\(\(\(x\)\s*>>\s*(\d+)\)\s*&\s*(0x[0-9A-Fa-f]+|\d+)\)", expr)
                if e or f:
                    if e:
                        mask, sh = int(e.group(1), 0), int(e.group(2))
                    else:
                        mask, sh = int(f.group(2), 0), int(f.group(1))
                    for n in cur:
                        if (n, fname) in MASK_FIXES:
                            mask = MASK_FIXES[(n, fname)]
                    if mask & (mask + 1):
                        print(f"skip non-contiguous mask {fname} {mask:#x}", file=sys.stderr)
                    else:
                        lo, hi = sh, sh + mask.bit_length() - 1
            if lo is not None:
                for n in cur:
                    sections[n].append((fname, lo, hi, doc))
    i += 1


for reg, extra in EXTRA_FIELDS.items():
    have = {f[0] for f in sections.get(reg, [])}
    sections.setdefault(reg, []).extend(f for f in extra if f[0] not in have)


def field_ident(name, used):
    ident = name.lstrip("_").lower()
    if name.startswith("_"):
        ident += "_mirror"
    if ident[0].isdigit():
        ident = "f_" + ident
    if ident in KEYWORDS:
        ident += "_"
    base, k = ident, 2
    while ident in used:
        ident, k = f"{base}_{k}", k + 1
    used.add(ident)
    return ident


def out(s=""):
    print(s)


out(rust_notice("Copyright 2014(c) Analog Devices, Inc. (drivers/rf-transceiver/ad9361/ad9361.h)"))
out("""//! AD9361 register map.
//!
//! GENERATED by `tools/gen_ad9361_regs.py` from the no-OS `ad9361.h` register definitions --
//! regenerate instead of editing by hand (hand-written overrides live in the generator).
//!
//! Every register is a `bitbybit` bitfield (or a plain-byte newtype if the header defines no
//! bit fields for it) implementing [`Register`], so it can be used with `Ad9361::read_reg`,
//! `write_reg` and `modify_reg`. Field names follow the header's macro names. Fields that
//! overlap an earlier field of the same register are omitted.
#![allow(dead_code)]

use arbitrary_int::{u2, u3, u4, u5, u6, u7, u10};

/// A single-byte AD9361 register with a fixed address.
pub trait Register: Copy {
    const ADDRESS: u10;

    fn from_raw(raw: u8) -> Self;

    fn to_raw(self) -> u8;
}

/// Implements [`Register`] for a `bitbybit` bitfield register at the given address.
macro_rules! impl_reg {
    ($t:ty, $addr:expr) => {
        impl Register for $t {
            const ADDRESS: u10 = u10::new($addr);

            fn from_raw(raw: u8) -> Self {
                Self::new_with_raw_value(raw)
            }

            fn to_raw(self) -> u8 {
                self.raw_value()
            }
        }
    };
}

/// Defines a register that is a plain byte value (no bit fields) and implements [`Register`].
macro_rules! byte_reg {
    ($(#[$meta:meta])* $t:ident, $addr:expr) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        pub struct $t(pub u8);

        impl Register for $t {
            const ADDRESS: u10 = u10::new($addr);

            fn from_raw(raw: u8) -> Self {
                Self(raw)
            }

            fn to_raw(self) -> u8 {
                self.0
            }
        }
    };
}
""")

seen = set()
n_fields = n_byte = 0
for reg in order:
    addr, doc = regs[reg]
    name = camel(reg)
    assert name not in seen, name
    seen.add(name)
    doc = doc.strip() or name
    if reg in SPECIAL:
        out(f"""/// {doc}
#[bitbybit::bitfield(u8, default = 0)]
pub struct {name} {{
    /// Product ID (`0x01` for the AD9361)
    #[bits(3..=7, rw)]
    product_id: u5,
    /// Silicon revision
    #[bits(0..=2, rw)]
    revision: u3,
}}
impl_reg!({name}, {addr:#x});
""")
        n_fields += 1
        continue
    fields = []
    used = set()
    taken = 0
    for fname, lo, hi, fdoc in ([] if reg in BYTE_ONLY else sections.get(reg, [])):
        mask = ((1 << (hi - lo + 1)) - 1) << lo
        if hi > 7:
            print(f"skip {reg}.{fname}: bits {lo}..{hi} exceed a byte", file=sys.stderr)
            continue
        if mask & taken:
            print(f"skip {reg}.{fname}: overlaps an earlier field", file=sys.stderr)
            continue
        taken |= mask
        fields.append((fname, lo, hi, fdoc))
    if not fields:
        out(f"byte_reg!(\n    /// {doc}\n    {name}, {addr:#x}\n);\n")
        n_byte += 1
        continue
    out(f"/// {doc}")
    out("#[bitbybit::bitfield(u8, default = 0)]")
    out(f"pub struct {name} {{")
    for fname, lo, hi, fdoc in sorted(fields, key=lambda f: -f[1]):
        ident = field_ident(fname, used)
        w = hi - lo + 1
        if fdoc:
            out(f"    /// {fdoc}")
        if w == 1:
            out(f"    #[bit({lo}, rw)]\n    {ident}: bool,")
        else:
            ty = "u8" if w == 8 else f"u{w}"
            out(f"    #[bits({lo}..={hi}, rw)]\n    {ident}: {ty},")
    out("}")
    out(f"impl_reg!({name}, {addr:#x});\n")
    n_fields += 1
print(f"{n_fields} bitfield registers, {n_byte} byte registers", file=sys.stderr)
