# Protocol

The dashboard and firmware implement **one versioned protocol/log specification**. The
canonical spec is shared with [`lbr-dashboard`](https://github.com/longhorn-baja-racing/lbr-dashboard)
— do not let the two diverge, and keep every node on one protocol/CAN-ID version.

This document tracks the firmware-side view; the authoritative schema and golden fixtures
live in the shared spec.

## Principles

- **Schema-driven.** Signal identifiers, value types, units, device/node identity, topology,
  and sample rates are metadata (see `components/protocol/include/lbr/signals.h`), not
  assumptions baked into a fixed layout.
- **Stable identifiers.** Signal IDs are append-only and must stay synchronized with the
  dashboard. Never renumber an existing signal.
- **Explicit unavailability.** A missing measurement is encoded with `valid = false`, never
  substituted with zero.
- **Shared time base.** Every sample carries a monotonic `t_mono_us` referenced to the
  gateway-distributed time base (epoch: gateway boot; unit: microseconds).
- **Sequence + integrity.** Live packets carry monotonic sequence numbers; framing includes
  checksums and supports resync after corruption. Counter widths/rollover are specified here.

## To define (tracked here as the spec solidifies)

- [ ] wire/log byte layout for `lbr_sample_t` and the health record
- [ ] checksum/integrity scheme and framing for each transport
- [ ] sequence-number width and rollover behavior
- [ ] session/topology metadata block format
- [ ] golden fixtures shared with `lbr-dashboard`
- [ ] protocol version bump + compatibility rules
