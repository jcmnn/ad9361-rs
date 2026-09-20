"""The Analog Devices notice for the files generated from the no-OS sources.

The generated files contain register definitions and tables taken from the no-OS driver, so they
carry its copyright notice and license text (3-clause BSD). The text is read from LICENSE-ADI-BSD.
"""
import os

_LICENSE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "LICENSE-ADI-BSD")


def rust_notice(copyright_line):
    """The notice as a Rust line comment block, followed by an empty line."""
    with open(_LICENSE, encoding="utf-8") as f:
        text = f.read()
    body = text[text.index("Redistribution and use"):].rstrip("\n")
    lines = ["// SPDX-License-Identifier: BSD-3-Clause", "//", "// " + copyright_line, "//"]
    lines += [("// " + line).rstrip() for line in body.split("\n")]
    return "\n".join(lines) + "\n"
