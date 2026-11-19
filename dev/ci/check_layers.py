#!/usr/bin/env python3
"""Check that LBRLib's components stay layered (no dependency web).

    python3 dev/ci/check_layers.py

LBRLib is a stack of components; each may include headers only from its own
layer or below:

    catalog       the car's generated catalog (car/catalog)
    lbr_base      foundations (pure C)
    lbr_core      the logic of every board (pure C)
    lbr_hw        ESP32 glue every board uses
    lbr_links     ESP32 glue for the radios and USB (may use lbr_hw)
    lbr_node, lbr_central, lbr_cockpit
                  board roles; none includes another role

The pure layers (lbr_base, lbr_core) also may not include anything but the C
standard library and the layers below them: no ESP-IDF, no FreeRTOS, no
sdkconfig. That is what lets the host tests and the simulator run exactly the
code that is flashed. One exception: the umbrella header lbr.h includes a
role's header when that role is in the build (behind __has_include). Exit
status 1 lists every violation.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
COMPONENTS = ROOT / "lbrlib" / "components"
CATALOG = ROOT / "car" / "catalog" / "include"

RANK = {"catalog": 0, "lbr_base": 1, "lbr_core": 2, "lbr_hw": 3, "lbr_links": 4}
ROLE_RANK = 5
PURE = {"lbr_base", "lbr_core"}
STD_C = {
    "assert.h", "ctype.h", "errno.h", "float.h", "inttypes.h", "limits.h", "math.h", "stdarg.h",
    "stdatomic.h", "stdbool.h", "stddef.h", "stdint.h", "stdio.h", "stdlib.h", "string.h",
}
UMBRELLA = "lbr.h"
INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]', re.M)


def rank(component: str) -> int:
    return RANK.get(component, ROLE_RANK)


def header_owners() -> dict[str, str]:
    """Map 'lbr/x.h' (and 'lbr.h') to the component that provides it."""
    owners: dict[str, str] = {}
    for inc in COMPONENTS.glob("*/include"):
        for h in inc.rglob("*.h"):
            owners[h.relative_to(inc).as_posix()] = inc.parent.name
    for h in CATALOG.rglob("*"):
        if h.is_file():
            owners[h.relative_to(CATALOG).as_posix()] = "catalog"
    return owners


def main() -> int:
    owners = header_owners()
    errors: list[str] = []
    components = sorted(p.name for p in COMPONENTS.iterdir() if p.is_dir())
    for comp in components:
        for f in sorted((COMPONENTS / comp).rglob("*")):
            if f.suffix not in (".c", ".h") or not f.is_file():
                continue
            where = f.relative_to(ROOT)
            local = {p.name for p in f.parent.iterdir()}
            for bracket, name in INCLUDE.findall(f.read_text(encoding="ascii", errors="replace")):
                owner = owners.get(name)
                if owner is None:
                    if comp in PURE and not (bracket == "<" and name in STD_C) and name not in local:
                        errors.append(f"{where}: pure layer {comp} includes <{name}> "
                                      "(only the C library and lower LBRLib layers)")
                    continue
                if owner == comp or (f.name == UMBRELLA and rank(owner) == ROLE_RANK):
                    continue
                if rank(owner) > rank(comp) or (rank(owner) == ROLE_RANK and rank(comp) == ROLE_RANK):
                    errors.append(f"{where}: {comp} includes {name} from {owner}, "
                                  "a layer above it (or another role)")
    for e in errors:
        print(e, file=sys.stderr)
    if errors:
        print(f"check_layers: {len(errors)} violation(s)", file=sys.stderr)
        return 1
    print(f"check_layers: {len(components)} components, layered")
    return 0


if __name__ == "__main__":
    sys.exit(main())
