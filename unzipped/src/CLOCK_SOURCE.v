// This is a STUB only
// NOTE: you must replace this with clock-safe (glitch free) cells
//       for muxing and inversion (or XOR).
//       The PD tools can use constraints to treat the output as
//       clock for clock distribution and STA.


module CLOCK_SOURCE_MUX(
  input    use_scan_clock,
  input    scan_clock,
  input    pin_clock,
  output   clock);

  assign clock = use_scan_clock ? scan_clock : pin_clock;
endmodule

module CLOCK_SOURCE_INV(
  input    scan_no_inv,
  input    good_clock,
  output   clock);

  assign clock = ~scan_no_inv ^ good_clock;
endmodule
