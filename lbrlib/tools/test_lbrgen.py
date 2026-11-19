#!/usr/bin/env python3
"""Self test of lbrgen.py: a minimal catalog generates, and each broken one is
refused with a message naming the problem. Standard library only.

    python3 lbrlib/tools/test_lbrgen.py
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lbrgen  # noqa: E402

GOOD = """
can_max = 2

[node.sensor]
id = 1
slots = [{ name = "fast", input = "io", hint = "pulse inputs" }, { name = "slow", signals = ["B"] }]

[node.hub]
id = 2
role = "central"
slots = []

[[frame]]
id = 0x100
name = "FAST"
sender = "sensor"
slot = "fast"
rate_hz = 100
counter = true
signals = [
  { name = "A", id = 0x0001, unit = "rpm", enc = "u16", desc = "a" },
  { name = "C", id = 0x0003, unit = "percent", enc = "i16", scale = 0.5, desc = "c" },
]

[[frame]]
id = 0x101
name = "SLOW"
sender = "sensor"
slot = "slow"
rate_hz = 1
signals = [{ name = "B", id = 0x0002, type = "u32", unit = "count", enc = "u8", desc = "b" }]

[[signal]]
name = "EVENT"
id = 0x0700
type = "status"
unit = "none"
desc = "e"

[[signal]]
name = "FAULT"
id = 0x0701
type = "status"
unit = "none"
desc = "f"

[cockpit]
signals = ["A"]
"""


def load_text(text: str) -> lbrgen.Catalog:
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "catalog.toml"
        p.write_text(text, encoding="ascii")
        return lbrgen.load(p)


class Good(unittest.TestCase):
    def test_layout_and_slots(self) -> None:
        cat = load_text(GOOD)
        fast = cat.frames[0]
        self.assertEqual([f.offset for f in fast.fields], [0, 2])
        self.assertEqual(fast.dlc, 8)  # counter frames are 8 bytes, counter at 6
        self.assertEqual(cat.frames[1].dlc, 1)
        sensor = cat.nodes[0]
        self.assertEqual([s.name for s in sensor.slots], ["fast", "slow"])
        self.assertEqual([s.name for s in sensor.slots[0].signals], ["A", "C"])
        self.assertEqual(sensor.slots[1].rate_hz, 1)
        self.assertEqual([s.id for s in cat.signals], sorted(s.id for s in cat.signals))

    def test_outputs_are_ascii_and_stable(self) -> None:
        cat = load_text(GOOD)
        with tempfile.TemporaryDirectory() as d:
            car = Path(d) / "car"
            a = lbrgen.outputs(cat, car, Path(d))
            b = lbrgen.outputs(cat, car, Path(d))
            self.assertEqual(a, b)
            text = a[car / "catalog" / "include" / "lbr" / "catalog_frames.def"]
            self.assertIn('LBR_FRAME(0x100, "FAST", SENSOR, 100, 8, 6, 2)', text)
            self.assertIn("LBR_FIELD(C, 2, I16, 0.5f)", text)
            plan = a[car / "nodes" / "sensor" / "main" / "sensor_plan.c"]
            self.assertIn(".input     = LBR_INPUT_IO,", plan)
            self.assertIn(".io_spec   = CONFIG_LBR_IO_FAST,", plan)
            self.assertIn('#define CONFIG_LBR_IO_FAST ""', plan)  # host builds: empty spec
            kconfig = a[car / "nodes" / "sensor" / "main" / "Kconfig"]
            self.assertIn("config LBR_IO_FAST", kconfig)
            self.assertIn("Channels, in order: A, C (frame 0x100). Typically pulse", kconfig)
            self.assertNotIn("LBR_IO_SLOW", kconfig)  # no input: bound in main.c
            self.assertNotIn(car / "nodes" / "hub" / "main" / "Kconfig", a)  # no io slots
            nodes = a[car / "catalog" / "include" / "lbr" / "catalog_nodes.def"]
            self.assertIn("LBR_NODE_ROLE(CENTRAL, HUB)", nodes)
            self.assertIn("LBR_NODE_ROLE(COCKPIT, RESERVED)", nodes)


class Bad(unittest.TestCase):
    def refused(self, old: str, new: str, message: str) -> None:
        self.assertIn(old, GOOD)
        with self.assertRaises(lbrgen.CatalogError) as e:
            load_text(GOOD.replace(old, new, 1))
        self.assertIn(message, str(e.exception))

    def test_duplicate_signal_id(self) -> None:
        self.refused("id = 0x0003", "id = 0x0001", "duplicate signal id 0x1")

    def test_fields_overflow_counter(self) -> None:
        text = GOOD.replace('enc = "u16", desc = "a"', 'enc = "u32", desc = "a"')
        text = text.replace('enc = "i16", scale = 0.5', 'enc = "i32", scale = 0.5')
        with self.assertRaises(lbrgen.CatalogError) as e:
            load_text(text)
        self.assertIn("8 bytes of fields do not fit in 6", str(e.exception))

    def test_unknown_sender(self) -> None:
        self.refused('sender = "sensor"\nslot = "fast"', 'sender = "nobody"\nslot = "fast"',
                     "unknown sender 'nobody'")

    def test_missing_slot(self) -> None:
        self.refused('slot = "fast"', 'slot = "fastt"', "has no slot 'fastt'")

    def test_integer_signal_scaled(self) -> None:
        self.refused('enc = "u8", desc', 'enc = "u8", scale = 2.0, desc', "travel unscaled")

    def test_bad_rate(self) -> None:
        self.refused("rate_hz = 1\n", "rate_hz = 3\n", "must divide 1000000")

    def test_frames_out_of_order(self) -> None:
        self.refused("id = 0x101", "id = 0x0FF", "outside the data range")

    def test_bad_unit(self) -> None:
        self.refused('unit = "rpm"', 'unit = "rpms"', "unit 'rpms'")

    def test_no_central(self) -> None:
        self.refused('role = "central"\n', "", "role = \"central\"")

    def test_explicit_slot_mismatch(self) -> None:
        self.refused('signals = ["B"]', 'signals = ["A"]', "exactly those of its frames")

    def test_unknown_cockpit_signal(self) -> None:
        self.refused('signals = ["A"]\n"""'[:-4], 'signals = ["Z"]', "unknown signal 'Z'")

    def test_unknown_input(self) -> None:
        self.refused('input = "io"', 'input = "adc"', "input 'adc' is not one of")

    def test_gps_only_on_central(self) -> None:
        self.refused('input = "io"', 'input = "gps"', "only on the central node")

    def test_missing_standard_signal(self) -> None:
        self.refused('name = "FAULT"', 'name = "FAULTS"', "standard signal FAULT")


if __name__ == "__main__":
    unittest.main(verbosity=1)
