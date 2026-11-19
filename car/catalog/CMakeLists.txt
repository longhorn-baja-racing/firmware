# This car's catalog, generated from car/catalog.toml by lbrlib/tools/lbrgen.py:
# header-only X-macro data (signals, nodes, CAN frames, link selections) that
# LBRLib's protocol component expands. LBRLib requires a component named
# "catalog"; every node project adds this directory to EXTRA_COMPONENT_DIRS.
idf_component_register(INCLUDE_DIRS "include")
