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
//  File            : i3c_exit_detectors.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Wed Jun 12 23:47:03 2019 $
//  Revision        : $Revision: 1.61 $
//
//  IP Name         : i3c_exit_detectors 
//  Description     : MIPI I3C Combined detector for HDR Exit and Restart
//    This contains an Exit detector needed for all I3C Slaves and Masters
//    to be able to ignore HDR mode until cleanly out, whether the
//    HDR mode is supported or not.
//    The Restart is only checked for if HDR is supported.
//
//  ----------------------------------------------------------------------------
//                    Revision History
//  ----------------------------------------------------------------------------
//
//
//  ----------------------------------------------------------------------------
//                    Implementation details
//  ----------------------------------------------------------------------------
//  See the micro-arch spec and MIPI I3C spec
//
//    The HDR Exit is detected whether in HDR or not; this is a 
//    precaution, and only toggles on flop from SDR SDA changes
//    since SCL high resets each time.
//  ----------------------------------------------------------------------------

module i3c_exit_detector #(
    parameter ENA_HDR = 3'b000 // default is no HDR
  )
  (
  // there are 3 clock domains in this tiny block (!)
  // clock source from pins and inversion must be done
  // safely in the upper layer
  input         clk_SDA_n,              // SDA inverted as clock
  input         clk_SDA,                // SDA not inverted as clock
  input         clk_SCL,                // SCL as clock
  input         RSTn,                   // global reset
  // now normal nets. Note the layered reset controls
  input         pin_SCL_in,             // SCL as a pin (not clock)
  input         in_HDR_mode,            // SDR detected CCC ENTHDRn
  input         HDR_restart_ack,        // signalled by HDR block 
  output        oHDR_exit,              // 1 if exit detected (clears on SCL)
  output        oHDR_restart,           // 1 if restart (clears on ack)
  input         scan_no_rst             // prevents layered reset
  );
`include "i3c_params.v"                 // local parameters/constants

  wire          scl_rst_n;
  reg [3:0]     stp_cnt;                // HDR STOP counter (shift chain)

  // SCL must be 0 for exit/restart, so SCL=1 resets 
  assign scl_rst_n     = RSTn & (~pin_SCL_in | scan_no_rst);

  // counter for 3 and 4 rising SDA when SCL is High 
  // (else SCL resets). 
  // Note: we allow this to be active all of the time,
  // even if not in HDR. This is to catch cases of missed
  // HDR entry as a safety measure. It means one flop 
  // toggling when SDR SDA changes.
  // Note: SDA_n (falling SDA) driven
  always @ (posedge clk_SDA_n or negedge scl_rst_n)
    if (~scl_rst_n)
      stp_cnt <= 4'd0;                  // SCL High or not HDR
    else 
      stp_cnt <= {stp_cnt[2:0], 1'b1};  // shift chain counter
  assign oHDR_exit = stp_cnt[3];        // detects Exit (STOP)

  // 
  // HDR Restart is only used if HDR support is enabled in block
  //
  generate if (ENA_HDR != 0) begin : HDR_restart 
    wire        restart_rst_n;
    reg         poss_restart;           // possible restart
    reg         is_restart;

    // restart not possible until most of way through exit, but
    // restart is reset by not being in HDR and restart ack
    assign restart_rst_n = RSTn & 
             ((~HDR_restart_ack & in_HDR_mode)| scan_no_rst);
    `Observe(observe_restart_rst, clk_SCL, ~HDR_restart_ack & in_HDR_mode) // optional DFT observer

    // Possible Restart means exactly 2 falling edges
    // of SDA and then rising edge of SDA. Actual
    // Restart is from SCL then rising 
    // Note SDA driven (SDA rising)
    always @ (posedge clk_SDA or negedge restart_rst_n)
      if (~restart_rst_n)
        poss_restart <= 1'b0;     // Restart ACK or not HDR
      else if (stp_cnt[1] & ~stp_cnt[2]) // after 2nd fall only
        poss_restart <= 1'b1;     // SDA rise after 2 falls
      else 
        poss_restart <= 1'b0;     // else not possible restart

    // Note SCL driven (SCL rising)
    always @ (posedge clk_SCL or negedge restart_rst_n)
      if (~restart_rst_n)
        is_restart <= 1'b0;       // Restart ACK or not HDR
      else if (poss_restart)
        is_restart <= 1'b1;       // SCL rises after SDA does
      else 
        is_restart <= 1'b0;       // else not restart
    assign oHDR_restart = is_restart; // signal restart, clear on ACK
  end else begin : no_HDR
    // if no HDR support, we never check for restart
    assign oHDR_restart = 1'b0;   
  end endgenerate


endmodule
