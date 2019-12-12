# MIPI I3C Slave

An NXP Free license MIPI I3C Slave implemented in Verilog for use in FPGAs and Silicon parts.

-- Version tag 1.1.11.a.1.0 (Early Access release)

# BSD Licensed RTL - see README for exact text

Highly configurable using parameters, and contains:

- Support for I3C Basic v1.0 as available on MIPI Website for download (member or not)
- I3C SDR protocol
- All required CCCs (builtin commands) plus some optional ones.
- IBI (in band interrupt) including optional IBI data byte.
- Support for I2C with a static address.
- Add SlaveReset
- Two different integrations depending on system
  - Full APB memory mapped registers for processor based systems.
    - Adjustable FIFO depth for each direction
  - Autonomous model for state machine ASICs.
    - Supports auto-register creation and system side control
- Documentation including programmer’s model, micro-architecture spec, basic I3C spec.

Free version does not include HDR-DDR, HDR-Ternary, Time-control, nor Master support.
Full version of Slave and Master available for licensing from Silvaco
Support contract of free Slave available from Silvaco

---
Copyright (c) 2015-2019 NXP Semiconductors.
