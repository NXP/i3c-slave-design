# MIPI I3C Slave

A BSD licensed MIPI I3C Slave implemented in Verilog for use in FPGAs and Silicon parts.

# Read the README1ST.pdf or .txt file before proceeding.

Highly configurable using parameters, and contains:

- I3C SDR
- All required CCCs (builtin commands) plus some optional ones.
- IBI (in band interrupt) including optional IBI data byte.
- Support for I2C with a static address.
- Full APB memory mapped registers for processor based systems.
- Adjustable FIFO depth for each direction
- Autonomous mode for state machine ASICs.
- Documentation including programmer’s model, micro-architecture spec, basic I3C spec.
- Test vectors for verification.

Free version does not include HDR-DDR, HDR-Ternary, Time-control, nor Master support.

---
Copyright (c) 2015-2017 NXP Semiconductors.

