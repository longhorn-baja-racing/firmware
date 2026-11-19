# Tools

## `lbrlib/tools/lbrgen.py`: the catalog generator

```sh
python3 lbrlib/tools/lbrgen.py car/catalog.toml           # regenerate everything from the catalog
python3 lbrlib/tools/lbrgen.py car/catalog.toml --check   # fail if a generated file is out of date
python3 lbrlib/tools/test_lbrgen.py                       # the generator's own tests
```

Python 3.11+, standard library only. See [the catalog](catalog.md). `ctest` runs both checks
(`catalog`, `lbrgen`).

## The host build and tests

```sh
cmake -S dev -B build-dev -G Ninja && cmake --build build-dev
ctest --test-dir build-dev --output-on-failure
ctest --test-dir build-dev -R logic                # one suite
ctest --test-dir build-dev -R sim_car_logic        # one simulator scenario
```

C23 (gcc 14+ or clang 19+), with AddressSanitizer and UndefinedBehaviorSanitizer by default
(`-DLBR_SANITIZE=OFF` to turn them off). Suites: `util protocol health can timebase logging
sensors node plans transport gateway wlink cores update value logic`, the golden `fixtures`,
the `decoder` self test, `catalog`, `lbrgen`, and every simulator scenario.

## `dev/ci/check_layers.py`: keep the layers

```sh
python3 dev/ci/check_layers.py
```

Fails, naming the file and include, if a component includes a header from a layer above it (or
another role), or if the pure layers include anything but the C library. In `ctest` (`layers`)
and CI.

## The simulator

```sh
python3 dev/sim/ui/server.py                            # browser UI at http://127.0.0.1:8765/
build-dev/sim/lbr_sim dev/sim/scenarios/car_logic.sim   # a scenario; exit status = failed checks
build-dev/sim/lbr_sim --seed 7 --duration 30            # a timed run
build-dev/sim/lbr_sim --trace powertrain dev/sim/scenarios/can_faults.sim   # a node's hardware calls
```

It runs every node's real firmware logic on a simulated car: CAN with arbitration and error
counters, drifting crystals, the SD card, WiFi loss, and a vehicle model feeding the sensors.
Scenarios are plain text:

```text
at 3.0  disconnect powertrain                  # do something at t = 3 s
at 7.0  expect central event:NODE_LOST >= 1    # check something
at 7.0  expect ENGINE_RPM age_ms > 1000        # a signal, as the Central received it
at 7.0  expect out:CVT/Ratio status == NODE_LOST   # a car logic output
end 10.0
```

Output checks: `out:<key>` with `valid`, `value`, or `status`. Every command and check is in
[`dev/sim/README.md`](../../dev/sim/README.md).

## `dev/tools/lbr_decode.py`: the reference decoder

An independent decoder (it shares no code with the firmware) driven by the exported catalog:

```sh
python3 dev/tools/lbr_decode.py log FILE.LBR [--json]
python3 dev/tools/lbr_decode.py csv FILE.LBR [--out FILE.csv]
python3 dev/tools/lbr_decode.py stream usb_stream.bin
python3 dev/tools/lbr_decode.py serial /dev/ttyUSB0 [--cmd STAT|LIST|"DUMP name"] [--out file]
python3 dev/tools/lbr_decode.py --selftest
```

## `dev/tools/lbr_deploy.py`: build and deploy

```sh
python3 dev/tools/lbr_deploy.py list
python3 dev/tools/lbr_deploy.py build <node>
python3 dev/tools/lbr_deploy.py usb <node> --port /dev/ttyUSB0
python3 dev/tools/lbr_deploy.py can --port /dev/ttyUSB0     # every CAN node, through the Central
```

The release checklist is [`docs/deploying.md`](../../docs/deploying.md).
