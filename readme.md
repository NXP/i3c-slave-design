# MIPI I3C Slave

An NXP Free license MIPI I3C Slave implemented in Verilog for use in FPGAs and Silicon parts.

# Not available as BSD (open source) due to MIPI confidentiality - see below for details

Highly configurable using parameters, and contains:

- I3C SDR
- All required CCCs (builtin commands) plus some optional ones.
- IBI (in band interrupt) including optional IBI data byte.
- Support for I2C with a static address.
- Full APB memory mapped registers for processor based systems.
- Adjustable FIFO depth for each direction
- Autonomous model for state machine ASICs.
- Documentation including programmer’s model, micro-architecture spec, basic I3C spec.
- Simple Test vectors for verification.

Free version does not include HDR-DDR, HDR-Ternary, Time-control, nor Master support.

# Requires a confidentiality agreement for non-MIPI-members
So, cannot be BSD licensed. License agreement text for both MIPI members and non-members in this directory.
Available from NXP for download if license is acceptable: http://www.nxp.com/webapp/software-center/library.jsp#/home/query/MIPI%20I3C%20Slave%20IP%20for%20MIPI/~filter~/popularity/0

---
Copyright (c) 2015-2017 NXP Semiconductors.

