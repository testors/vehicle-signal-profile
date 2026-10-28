#!/usr/bin/env python3
"""
Read and write AiM Race Studio 3 XC1 files, plus the XML helpers vsp2xc1 uses.

XC1 layout:

    u32le plaintext_length
    AES-128-ECB encrypted XML payload, zero-padded to 16 bytes

A file without a password uses Race Studio's built-in key, the ASCII marker
`CrIP/xml/xC1` zero-padded to 16 bytes; it is kept here for interoperability.
The AES step runs through the `openssl` command.

    python3 tools/aim_xc1.py unpack driver.xc1 -o driver.xml   # decrypt for inspection
    python3 tools/aim_xc1.py pack driver.xml -o driver.xc1     # encrypt again
"""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


DEFAULT_KEY = (b"CrIP/xml/xC1" + (b"\x00" * 16))[:16]
BLOCK_SIZE = 16


def crypt_op(data: bytes, *, decrypt: bool, key: bytes = DEFAULT_KEY) -> bytes:
    command = ["openssl", "enc", "-d" if decrypt else "-e", "-aes-128-ecb", "-nopad", "-K", key.hex()]
    try:
        proc = subprocess.run(command, input=data, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        raise SystemExit("openssl not found: XC1 files need the openssl command") from None
    if proc.returncode != 0:
        raise SystemExit(proc.stderr.decode(errors="replace").strip() or "openssl failed")
    return proc.stdout


def decrypt_xc1(data: bytes, name: str = "XC1") -> bytes:
    """The XML inside an XC1 file's bytes."""
    if len(data) < 4:
        raise SystemExit(f"{name}: too small for XC1")
    plain_len = int.from_bytes(data[:4], "little")
    encrypted = data[4:]
    if len(encrypted) % BLOCK_SIZE != 0:
        raise SystemExit(f"{name}: encrypted payload is not 16-byte aligned")
    if plain_len > len(encrypted):
        raise SystemExit(f"{name}: plaintext length exceeds encrypted payload length")
    return crypt_op(encrypted, decrypt=True)[:plain_len]


def read_xc1(path: Path) -> bytes:
    return decrypt_xc1(path.read_bytes(), str(path))


def write_xc1(xml_bytes: bytes, output_path: Path) -> None:
    pad_len = (-len(xml_bytes)) % BLOCK_SIZE
    padded = xml_bytes + (b"\x00" * pad_len)
    encrypted = crypt_op(padded, decrypt=False)
    output_path.write_bytes(len(xml_bytes).to_bytes(4, "little") + encrypted)


def load_template(path: Path) -> ET.Element:
    """A Race Studio CAN driver as XML: a plain XML file, or an XC1 decrypted."""
    data = path.read_bytes()
    if data.lstrip(b"\xef\xbb\xbf \t\r\n").startswith(b"<"):
        try:
            return ET.fromstring(data)
        except ET.ParseError:
            pass   # an XC1 whose length happens to start with "<"
    return ET.fromstring(decrypt_xc1(data, str(path)))


def xml_bytes(tree: ET.ElementTree) -> bytes:
    body = ET.tostring(tree.getroot(), encoding="unicode", short_empty_elements=True)
    return ('<?xml version="1.0"?>\n' + body + "\n").encode()


@dataclass(frozen=True)
class DbcSignal:
    """A signal as a DBC writes it (the DBCInfo text of a Race Studio channel)."""
    message_id: int
    message_name: str
    name: str
    start_bit: int
    bit_length: int
    byte_order: int     # 1 Intel, 0 Motorola
    signed: bool
    factor: float
    offset: float
    minimum: float
    maximum: float
    unit: str
    canonical: str | None


def fmt_number(value: float) -> str:
    if value == 0:
        return "0"
    return f"{value:.12g}"


def safe_stem(value: str) -> str:
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    stem = stem.strip("._-")
    return stem or "custom_can"


def set_prop(parent: ET.Element, name: str, value: object) -> None:
    node = parent.find(f"./p[@n='{name}']")
    if node is None:
        node = ET.SubElement(parent, "p", {"n": name})
    node.text = str(value)


def find_first(parent: ET.Element, path: str, context: str) -> ET.Element:
    node = parent.find(path)
    if node is None:
        raise SystemExit(f"Template is missing {context}")
    return node


def motorola_lsb_start(start_bit: int, bit_length: int) -> int:
    """Convert a DBC Motorola MSB start bit to the XC1/CANP extraction bit."""
    bit = start_bit
    for _ in range(bit_length - 1):
        if bit % 8 == 0:
            bit += 15
        else:
            bit -= 1
    return bit


def dbc_info_text(signal: DbcSignal) -> str:
    endian = "Little Endian" if signal.byte_order == 1 else "Big Endian"
    signed = "True" if signal.signed else "False"
    return (
        f"Name:\t{signal.name}\r\n"
        "\r\n"
        f"First Bit:\t\t{signal.start_bit}\n"
        f"Length:\t\t{signal.bit_length}\n"
        f"Endianness:\t{endian}\n"
        f"Is Signed:\t{signed}\n"
        f"Factor:\t\t{fmt_number(signal.factor)}\n"
        f"Offset:\t\t{fmt_number(signal.offset)}\n"
        f"Min Value:\t{fmt_number(signal.minimum)}\n"
        f"Max Value:\t{fmt_number(signal.maximum)}\n"
        f"Unit:\t\t{signal.unit}\n"
    )


def rewrite_i_attributes(node: ET.Element, index: int) -> None:
    """Renumber a channel's element ids (the 4th field of every "i") to its index."""
    replacement = rf"\g<1>{index}"
    for element in node.iter():
        ident = element.attrib.get("i")
        if ident:
            element.attrib["i"] = re.sub(r"(\.1\.1\.2\.)\d+(?=\.|$)", replacement, ident, count=1)


def display_decimals(factor: float) -> int:
    if factor == 0 or abs(factor) >= 1:
        return 0
    return max(0, min(3, math.ceil(-math.log10(abs(factor)) - 1e-9)))


def update_header(
    root: ET.Element,
    *,
    dbc_path: Path,
    manufacturer: str,
    model: str,
    uid: str,
    bitrate: int,
    bus_mode: int,
    channel_count: int,
) -> None:
    conf = find_first(root, ".//e[@c='GConfParam']", "GConfParam")
    set_prop(conf, "CANOrigFile", f"{uid}.xml")
    set_prop(conf, "CustCANUnID", uid)
    set_prop(conf, "DATE_TIME", datetime.now().strftime("%Y/%m/%d %H:%M:%S"))
    set_prop(conf, "FileNameExtDBCOrig", dbc_path.name)
    set_prop(conf, "ImportedFromDBC", "1")
    set_prop(conf, "MaxMetaCanali", channel_count)
    set_prop(conf, "manlabel", manufacturer)
    set_prop(conf, "manname", manufacturer)
    set_prop(conf, "modlabel", model)
    set_prop(conf, "modname", model)
    set_prop(conf, "tipocan", "6")
    set_prop(conf, "drvtype", "CAN")

    bus = find_first(root, ".//e[@c='BusConfiguration']", "BusConfiguration")
    set_prop(bus, "bitrate", bitrate)
    set_prop(bus, "mode", bus_mode)


def write_index_fragments(
    output_dir: Path,
    *,
    manufacturer: str,
    model: str,
    uid: str,
    bitrate: int,
    bus_mode: int,
    dbc_path: Path,
) -> None:
    """The two Race Studio list entries of a custom CAN driver: one for
    user/resources/can/custom/indiceCANCustom.xml, one for user/xmldb_unicode32_custecus.xml."""
    indice_entry = ET.Element("e", {"c": "mod", "i": "1000"})
    for name, value in (
        ("busbitrate", bitrate),
        ("busmode", bus_mode),
        ("cust_ecu_note", ""),
        ("cust_uniid", uid),
        ("dev_type", "1"),
        ("disclaimer", ""),
        ("drvtype", "CAN"),
        ("file", f"{uid}.xml"),
        ("filebin", ""),
        ("filepdf", ""),
        ("folderbin", ""),
        ("label", model),
        ("name", model),
        ("verapi", "00.00.00"),
        ("verdate", "0"),
        ("verdrv", "00.00.00"),
    ):
        child = ET.SubElement(indice_entry, "p", {"n": name})
        child.text = str(value)
    ET.indent(indice_entry, space="  ")
    output_dir.joinpath(f"{uid}.indiceCANCustom.entry.xml").write_text(
        ET.tostring(indice_entry, encoding="unicode") + "\n"
    )

    xmldb_entry = ET.Element(
        "e",
        {
            "t": "CAN",
            "x": "0",
            "h": "0",
            "n": f"{uid}.xc1",
            "p": "c:/aim_sport/racestudio3/user/resources/can/custom/",
            "dt": datetime.now().strftime("%Y/%m/%d %H:%M:%S"),
        },
    )
    for tag, value in (
        ("dc", "1"),
        ("dv", "0"),
        ("n", f"{manufacturer}_{model}"),
        ("pk", ""),
        ("dt", datetime.now().strftime("%Y/%m/%d %H:%M:%S")),
        ("mf", manufacturer),
        ("md", model),
        ("fl", f"{uid}.xc1"),
        ("br", bitrate),
        ("bt", "6"),
        ("cd", "1"),
        ("pv", ""),
        ("nt", ""),
        ("fd", dbc_path.name),
    ):
        child = ET.SubElement(xmldb_entry, "p", {"t": tag})
        child.text = str(value)
    ET.indent(xmldb_entry, space="  ")
    output_dir.joinpath(f"{uid}.xmldb_unicode32_custecus.entry.xml").write_text(
        ET.tostring(xmldb_entry, encoding="unicode") + "\n"
    )


def command_unpack(args: argparse.Namespace) -> int:
    output = Path(args.output) if args.output else Path(args.input).with_suffix(".xml")
    output.write_bytes(read_xc1(Path(args.input)))
    print(f"written: {output}")
    return 0


def command_pack(args: argparse.Namespace) -> int:
    output = Path(args.output) if args.output else Path(args.input).with_suffix(".xc1")
    write_xc1(Path(args.input).read_bytes(), output)
    print(f"written: {output}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(dest="command", required=True)
    unpack = subparsers.add_parser("unpack", help="decrypt an XC1 to XML")
    unpack.add_argument("input")
    unpack.add_argument("-o", "--output", help="default: the input with .xml")
    unpack.set_defaults(func=command_unpack)
    pack = subparsers.add_parser("pack", help="encrypt XML to an XC1")
    pack.add_argument("input")
    pack.add_argument("-o", "--output", help="default: the input with .xc1")
    pack.set_defaults(func=command_pack)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
