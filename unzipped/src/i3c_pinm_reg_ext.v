/*--------------------------------------------------------------------
  Copyright (C) 2015-2019, NXP B.V.
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions 
  are met:
  
  1. Redistributions of source code must retain the above copyright 
     notice, this list of conditions and the following disclaimer.
  
  2. Redistributions in binary form must reproduce the above copyright 
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the 
     distribution.
  
  3. Neither the name of the copyright holder nor the names of its 
     contributors may be used to endorse or promote products derived 
     from this HDL software without specific prior written permission.
  
  THIS (HDL) SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  NOTE: No license under any third-party patent is granted or implied. 
        It is the responsibility of the licensee to obtain any required 
        third-party patent licenses.

  Note on terms used above:
  1. Software and HDL software are used interchangeably.
  2. Binary includes FPGA, simulation, and physical forms such 
     as Silicon chips.
  3. Clause 2 allows for such notice on a Web page or other 
     electronic form not part of a distribution.
  The original BSD source is available from:
    https://github.com/NXP/i3c-slave-design
  -------------------------------------------------------------------- */

//
//  ----------------------------------------------------------------------------
//                    Design Information
//  ----------------------------------------------------------------------------
//  File            : i3c_pinm_reg_ext.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Wed Jun 12 23:47:03 2019 $
//  Revision        : $Revision: 1.61 $
//
//  IP Name         : i3c_pinm_reg_ext - Detached pad SDA output for path timing
//  Description     : MIPI I3C pad controls when not to be integrated in block
//    This module is for when PIN_MODEL==`PINM_EXT_REG, which means the
//    pad output controls (out enable and out) are detached from the
//    slave peripheral and moved close to the pads.
//    This mechanism is used when the path timing is a problem. This block
//    is fed the combinatorial D inputs, which are setup before the SCL
//    edge, and so this closes the loop for clock-in-to-data out, by
//    moving the pin_SCL_in from the nearby pad physically close. So,
//    SCL_in to SDA_out is a short path, with the D feeding the choice
//    of SDA_out coming from the peripheral logic, farther away.
//
//  ----------------------------------------------------------------------------
//                    Revision History
//  ----------------------------------------------------------------------------
//
//
//  ----------------------------------------------------------------------------
//                    Implementation details
//  ----------------------------------------------------------------------------
//  See the micro-arch spec, with specific details of this mechanism.
//
//    The SCL is converted into a clock locally. This can be handled 
//    outside this module, but should NOT be coming from the peripheral,
//    but only from the local SCL pad. 
//  ----------------------------------------------------------------------------

module i3c_pinm_reg_ext #(
  parameter  ENA_DDR   = 0)             // enable if HDR-DDR
  (
  // 1st reset which is system wide or from block
  input               RSTn,             // master Reset
  // now the SCL as a pin, which we make into a clock and
  // inverted clocl. You can change falling edge to negedge
  // logic if desired.
  input               i_pad_SCL,        // direct from pad
  // now the signals from the i3c slave peripheral
  input               pin_SDA_out,      // out from peripheral
  input               pin_SDA_oena,     // falling edge out-en from peripheral
  input               pin_SDA_oena_rise0,// rising edge out-ena from peripheral
  input               pin_SDA_oena_rise1,// force start
  input         [1:0] pin_SDA_oena_rise32,// rising edge out if HDR-DDR only (else not used)
  // the output to SDA pads
  output              o_pad_SDA_oe,     // output enable for SDA
  output              o_pad_SDA_out,    // out to SDA when oe=1
  // now scan if used
  input               scan_single_clock,// use scan_CLK uninverted if SCAN
  input               scan_CLK
  );
  wire clk_SCL, clk_SCL_n;

  // we generate the clocks using the same clock safe block. This
  // could be reduce to a simpler one since only need falling
  CLOCK_SOURCE_MUX source_scl  (.use_scan_clock(scan_single_clock), .scan_clock(scan_CLK), 
                                .pin_clock(i_pad_SCL), .clock(clk_SCL));
  CLOCK_SOURCE_INV source_scl_n(.scan_no_inv(scan_single_clock), .good_clock(clk_SCL), 
                                .clock(clk_SCL_n));

  // We pick up the intended pad state. Note that test before change
  // is just to allow enable flops to save toggle power.
  // These next two are XORed to form the pad output-enable
  reg   SDA_oena_r;
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      SDA_oena_r <= 1'b0;
    else if (pin_SDA_oena != SDA_oena_r) // enable check not needed
      SDA_oena_r <= ~SDA_oena_r;

  // now just the output itself (if OE=1). Note that it only
  // changes on falling and is often parked at 0 (for OD)
  reg SDA_out_r;
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      SDA_out_r <= 1'b0;
    else if (pin_SDA_out != SDA_out_r)  // enabled check not needed
      SDA_out_r <= ~SDA_out_r;

  // note: OE uses one XOR and one AND as: XOR(SDA_oena_r, AND(rise_chg, SCL)); 
  assign o_pad_SDA_oe   = pin_SDA_oena_rise1 |
                           (SDA_oena_r ^ (pin_SDA_oena_rise0 & i_pad_SCL));
  // next XOR will be removed if not HDR-DDR, else is used for DDR only
  wire hdr_xor = (pin_SDA_oena_rise1 & SDA_out_r) |
                 (ENA_DDR & pin_SDA_oena_rise32[1] & i_pad_SCL &
                  (pin_SDA_oena_rise32[0] != SDA_out_r));
  assign o_pad_SDA_out  = SDA_out_r ^ hdr_xor;
endmodule
