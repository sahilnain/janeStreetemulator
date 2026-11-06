# SPDX-License-Identifier: Apache-2.0
"""
sigrok: runs the third-party sigrok protocol decoders on a recorded trace.

What this file does
-------------------
Writes the samples (one byte per cycle, bit n = pin n) to a file, runs sigrok-cli on it and
returns the annotations as a list of Ann(start, end, cls, text), with `cls` the decoder's
annotation class. sigrok-cli prints no class, so it is run once per class of interest.
Nothing here interprets the protocol: that is the decoder's job.
"""

import os
import re
import subprocess
import tempfile
from collections import namedtuple

Ann = namedtuple("Ann", "start end cls text")
LINE = re.compile(r"^(\d+)-(\d+) [^:]+: (.*)$")
RATE = 48_000_000


def run(samples, decoders, classes):
    """decoders: the -P string. classes: {decoder id: [annotation class, ...]}."""
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        f.write(bytes(samples))
        path = f.name
    out = []
    try:
        for pd, cl in classes.items():
            for c in cl:
                r = subprocess.run(
                    ["sigrok-cli", "-I", f"binary:numchannels=8:samplerate={RATE}", "-i", path,
                     "-P", decoders, "-A", f"{pd}={c}", "--protocol-decoder-samplenum"],
                    capture_output=True, text=True, env={**os.environ, "PYTHONWARNINGS": "ignore"})
                if r.returncode:
                    raise RuntimeError(f"sigrok-cli failed: {r.stderr[-500:]}")
                for ln in r.stdout.splitlines():
                    m = LINE.match(ln)
                    if m:
                        out.append(Ann(int(m[1]), int(m[2]), c, m[3]))
    finally:
        os.unlink(path)
    out.sort(key=lambda a: (a.start, a.end))
    return out


def text(anns, cls):
    return [a.text for a in anns if a.cls == cls]


def uart(samples, pin, baud, bits=8, parity="none", stop=1.0, msb=False, inv=False):
    """Returns dict(data=[ints], warn=[...], perr=[...], brk=[...])."""
    d = (f"uart:rx={pin}:baudrate={baud}:data_bits={bits}:parity={parity}:stop_bits={stop}"
         f":bit_order={'msb-first' if msb else 'lsb-first'}:format=hex:invert_rx={'yes' if inv else 'no'}")
    a = run(samples, d, {"uart": ["rx-data", "rx-warnings", "rx-parity-err", "rx-break"]})
    return dict(data=[int(x, 16) for x in text(a, "rx-data")], warn=text(a, "rx-warnings"),
                perr=text(a, "rx-parity-err"), brk=text(a, "rx-break"), ann=a)


def spi(samples, sck, mosi, miso, cs, cpol=0, cpha=0, msb=True, bits=8):
    d = f"spi:clk={sck}:cs={cs}:cpol={cpol}:cpha={cpha}:bitorder={'msb-first' if msb else 'lsb-first'}:wordsize={bits}"
    if mosi is not None:
        d += f":mosi={mosi}"
    if miso is not None:
        d += f":miso={miso}"
    cl = ["warnings"]
    if mosi is not None:
        cl.append("mosi-data")
    if miso is not None:
        cl.append("miso-data")
    a = run(samples, d, {"spi": cl})
    return dict(mosi=[int(x, 16) for x in text(a, "mosi-data")], miso=[int(x, 16) for x in text(a, "miso-data")],
                warn=text(a, "warnings"), ann=a)


def i2c(samples, scl, sda):
    """Returns the bus events in order, as ('S'|'Sr'|'P'|'A'|'N') or ('AW'|'AR'|'DW'|'DR', value)."""
    a = run(samples, f"i2c:scl={scl}:sda={sda}:address_format=unshifted",
            {"i2c": ["start", "repeat-start", "stop", "ack", "nack", "address-read", "address-write",
                     "data-read", "data-write", "warnings"]})
    ev, warn = [], []
    names = {"address-read": "AR", "address-write": "AW", "data-read": "DR", "data-write": "DW"}
    for x in a:
        if x.cls == "start":
            ev.append("S")
        elif x.cls == "repeat-start":
            ev.append("Sr")
        elif x.cls == "stop":
            ev.append("P")
        elif x.cls == "ack":
            ev.append("A")
        elif x.cls == "nack":
            ev.append("N")
        elif x.cls == "warnings":
            warn.append(x.text)
        else:
            ev.append((names[x.cls], int(x.text.split()[-1], 16)))
    return dict(ev=ev, warn=warn, ann=a)


def usb(samples, dp, dm, speed):
    """Stacked signalling + packet decoders. Returns packets and every error they report."""
    sp = {"FS": "full-speed", "LS": "low-speed"}[speed]
    d = f"usb_signalling:dp={dp}:dm={dm}:signalling={sp},usb_packet:signalling={sp}"
    a = run(samples, d, {"usb_packet": ["sync-err", "crc5-err", "crc16-err", "crc5-ok", "crc16-ok", "packet-invalid",
                                        "packet-out", "packet-in", "packet-sof", "packet-setup", "packet-data0",
                                        "packet-data1", "packet-ack", "packet-nak", "packet-stall"],
                         "usb_signalling": ["error", "reset", "keep-alive"]})
    return dict(packets=[(x.start, x.cls[7:].upper(), x.text) for x in a if x.cls.startswith("packet-")],
                errors=[(x.start, x.cls, x.text) for x in a
                        if x.cls in ("sync-err", "crc5-err", "crc16-err", "packet-invalid", "error")],
                crc_ok=[x.cls for x in a if x.cls in ("crc5-ok", "crc16-ok")],
                reset=[x for x in a if x.cls == "reset"], keep=[x for x in a if x.cls == "keep-alive"], ann=a)
