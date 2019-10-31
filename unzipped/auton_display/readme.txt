The check_auton_params.v file is used to get the simulator to print 
out your registers in a formatted way to verify the parameters.
You paste in your params for autonomous mode and you will see how
it will be configured. You can also include auton_display.v
and invoke AutonomousDisplayRules() from a test harness.

An example display is:

Autonomous Rules decoded from params
Rules:
[0] Write allowed in middle of a Run
[1] Writes can go over Run boundaries
[2] oraw_reg_touch[] signal used on each register
[3] Emits oraw_reg_touch[] signal on both Write and Read
[4] Index is not reset on START, so Read following goes from last use
Maximum register index=31. This uses 5 bits to hold index.
Registers and virtual registers:
0: Virtual cluster map from 28
------------------------------Run-Start----
1: Normal R/W reg; local flops
2: Normal R/W reg; local flops
3: Normal R/W reg; local flops
4: Normal R/W reg; local flops
5: Normal R/W reg; local flops
6: Normal R/W reg; local flops
------------------------------Run-End------
7: none
------------------------------Run-Start----
8: Normal 2-level register; write local flops, read system flops
9: Normal 2-level register; write local flops, read system flops
10: Normal 2-level register; write local flops, read system flops
11: Normal 2-level register; write local flops, read system flops
12: Normal RO reg; system flops
13: Normal RO reg; system flops
14: Normal RO reg; system flops
15: Normal RO reg; system flops
------------------------------Run-Start----
16: Normal R/W reg; local flops
------------------------------Run-Start----
17: Normal R/W reg; local flops
------------------------------Run-Start----
18: Normal R/W reg; local flops
------------------------------Run-Start----
19: Normal R/W reg; local flops
20: Normal R/W reg; local flops
21: Normal R/W reg; local flops
22: Normal R/W reg; local flops
23: Normal R/W reg; local flops
24: Normal R/W reg; local flops
25: Normal R/W reg; local flops
26: Normal R/W reg; local flops
27: Normal R/W reg; local flops
------------------------------Run-Start----
28: Start of Cluster remapped to 0
28: Normal RO reg; system flops
29: Normal RO reg; system flops
30: Normal RO reg; system flops
31: Normal RO reg; system flops