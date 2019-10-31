module check_auton_params;
////////////////INCLUDE PARAMS HERE///////////////////////

  // Example below: Goals were (by reg index):
  //  0: reads out as 4 bytes mapped with 0x11, 0x22, 0x33, 0x44
  //  1-6: RW bytes as one run
  //  7: gap
  //  8,9,10,11: W and R back as ~ of write as 4 separate. Run flows into 12-15
  //  12-15: RO counter which updates on each read. Part of run from 8
  //  16: Write a byte to use as IBI byte and to start IBI request (on next START). With or without TC
  //  17–18: RW with no run
  //  19-27: RW run
  //  28-31 same RO regs as 0.
  // So, implemented as:
  // Rules: [0]=W only at start of run, [1]=W ign after end of run
  //        [2]=touch only start loc of run, [3]=touch on W not R
  //        [4]=Idx reset on START (affects read)
  parameter REG_RULES    = 16'b000000000_0000_000;
  parameter MAX_REG      = 8'h1F;       // 32 locations
  `ifdef SINGLE_PARAM_STYLE
  // note on below: right most digit is 0th reg, left most is 31st reg
  parameter REG_WRITABLE = 32'b0000111111111111_0000111101111110;
  parameter REG_READABLE = 32'b1111000000000000_1111111100000000;
  parameter REG_RUN      = 32'b1110111111110000_1111111001111100;
  parameter REG_MASK     = {((MAX_REG+1)*8){1'b1}};
  parameter REG_BLEND    = 32'b0000000000000000_00000000;
  parameter REG_REMAP    = 256'hFF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF__FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_FF_1C;
  `else // multi-param style
    // 9 groups as shown from goals (from last to 1st)
  parameter [31:0] REG_WRITABLE = {4'h0,9'h1FF,2'h3,1'h1,4'h0,4'hF,1'h0,6'h3F,1'h0};
  parameter [31:0] REG_READABLE = {4'hF,9'h000,2'h0,1'h0,4'hF,4'hF,1'h0,6'h00,1'h0};
  parameter [31:0] REG_RUN      = {4'hE,9'h1FE,2'h0,1'h0,4'hF,4'hE,1'b0,6'h3E,1'h0};
    // note: no masks and no blending (masked mix of R and W bits)
  parameter        REG_MASK     = {((MAX_REG+1)*8){1'b1}};
  parameter [31:0] REG_BLEND    = {4'h0,9'h000,2'h0,1'h0,4'h0,4'h0,1'h0,6'h00,1'h0};
  parameter [255:0]REG_REMAP    = {{4*8{1'b1}},{9*8{1'b1}},{2*8{1'b1}},{1*8{1'b1}},
                                   {4*8{1'b1}},{4*8{1'b1}},{1*8{1'b1}},{6*8{1'b1}},
                                   8'h1C}; // mapped to 0 from 28
  `endif

////////////////END PARAMS HERE///////////////////////
integer i;
`include "./auton_display.v"

initial begin
  AutonomousDisplayRules();
  $stop;
end

endmodule
