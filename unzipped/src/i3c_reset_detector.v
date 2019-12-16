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
//  File            : i3c_reset_detector.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Thu Sep 12 16:44:05 2019 $
//  Revision        : $Revision: 1.14 $
//
//  IP Name         : i3c_reset_detector
//  Description     : MIPI I3C SlaveReset block detector (can be in GO2 domain)
//    This contains the detector for Slave Reset (SRST). It is a
//    separate file so it can go in an always-on domain, whether GO2 or just
//    kept powered and active even in deepest sleep.
//    It can be used to wake device from deepest sleep (e.g. i3c peripheral is 
//    unpowered) as well as reset when the block or chip is in trouble.
//    NOTE: because this can be used to reset the whole device, the system
//          both needs to allow that, but also needs to only allow the 
//          feed of SDA and SCL if i3c is in use (so cannot happen if pattern
//          happens to exist on those GPIOs when used for some other purpose). 
//          Since this block is supposed to reset the chip or peripheral when 
//          it is hung or otherwise misbehaving, it must be controlled in a way 
//          that ensures the same failure does then prevent it working to recover
//          device. See implementation notes.
//
//  ----------------------------------------------------------------------------
//                    Revision History
//  ----------------------------------------------------------------------------
//
//
//  ----------------------------------------------------------------------------
//                    Implementation details
//  ----------------------------------------------------------------------------
//  See the MIPI I3C spec for details. The reset pattern is an extended form of
//  the HDR Exit pattern. RSTACT is handled in the I3C peripheral.
//
//  How block should be connected into the system depends on use model:
//  1. If will also be used to wake from deepest sleep, must be in an always-on 
//     domain, and the wake and sleep signals should be connected; else sleep=0
//  2. Reset of i3c peripheral (default) is done via oRstBlock and may be
//     effected by actual reset of block or by IRQ to processor to make it
//     do so. The I3C peripheral will register the cause using an input
//     port connected to oRstBlock (if only interrupting processor) or 
//     indirectly from system-controlled reset.
//  3. If will be used to reset main system/chip, then full hookup is needed, 
//     and protection is from the unlock rules (i3c active and in slave mode).
//     This uses the oRstAll signal.
//  4. If this block is in always-on domain, and I3C peripheral may be unpowered,
//     isolation clamps should be used for nets crossing between. 
//
//  The use of layered resets in this block is somewhat complex. This is to
//  ensure no false triggering. The final stage of the SlaveReset is the STOP
//  which is detected using an SDA rising net fed by the chain. That flop,
//  srst_act2, is itself reset (cleared) in one of 3 ways:
//  1. iRstAction[3]=1 from i3c peipheral on RSTACT, GESTATUS, or clear of cause
//  2. SCL going low, which would be a START
//  3. The side effect of the chip being reset and so RSTn=0
//
//  An additional factor is the use of an escalation flop to handle the case of
//  default moving from peripheral reset to full reset if the peripheral reset
//  did not work. The iRstAction[3] clears the escalation.
//  ----------------------------------------------------------------------------

module i3c_reset_detector
  #(parameter USE_RST_FLOP=1)           // avoid tight timing on Reset
  (
  // there are 3 clock domains in this tiny block 
  // clock from pins (including invert of SDA) is done
  // outside this block and will need to consider scan
  input         clk_SDA,                // SDA rising edge as clock for reset
  input         clk_SDA_n,              // SDA falling edge as clock
  input         clk_SCL,                // SCL rising edge as clock
  input         RSTn,                   // global reset - including from oRstAll
  // now above pins as pins and also global enable
  input         i3c_slave_active,       // 1 if i3c active and we are Slave (vs. master)
  input         pin_SCL_in,             // SCL as a pin (not clock) for reset
  input         pin_SDA_in,             // SDA as a pin (not clock) for reset
  // now reset/wake outputs based on RstAction
  input         iDeepestSleep,          // optional: in deepest sleep now
  output        oWake,                  // optional: wake from sleep if in deepest sleep
  output        oRstBlock,              // only reset the I3C peripheral
  output        oRstAll,                // reset of i3c block and system
  output        oRstCustom,             // Customer trigger if enabled
  // now interaction with block. iRstAction has [2:0]=action, 
  // [3]=1 on RSTACT or GETSTATUS or clear of RST Cause (so ACK of block reset)
  input   [3:0] iRstAction,             // [2:0]=RSTACT req: 0=def/peripheral, 1=full, 2=none
  output        oRstRstAction,          // reset action back to normal
  // next is scan for layered reset
  input         scan_no_rst             // prevents layered reset
  );
`include "i3c_params.v"

  //
  // Table of contents:
  // 1. Define nets and basic connections
  // 2. Expanded Exit detector -> Reset detector. 7 falling edges
  // 3. Then handler for SCL rising, Sr, then engagement of P (STOP)
  // 4. Logic to handle special cases: reset-all hold and escalation
  //


  //
  // 1. Degine nets and basic connections
  //
  wire          enable_RSTn;            // normal global reset if active
  wire          scl_rst_n;
  reg     [6:0] stp_cnt;                // HDR-Exit counter (shift chain)
  reg           srst_act0, srst_act1, srst_act2;
  wire    [2:0] rst_action;             // only action part
  wire          rst_clear;              // from [3]
  reg           rst_all;
  reg           srst_escalate;
  wire          rst_escalate_n;
  wire          rst0_act_n, rst1_act_n, rst2_act_n;

  // enable_RST_n is global reset only when active, else held in reset
  assign enable_RSTn = RSTn & (i3c_slave_active | scan_no_rst);

  // We have one of 4 actions:
  // 1. If was in deepest sleep, we wake
  // 2. If requested no action, we do nothing
  // 3. If unspecified and not cleared, we reset peripheral and escalate
  //    -- Escalate clears if new RSTACT or GETSTATUS seen
  // 4. If requested or previous reset peripheral did not work, then chip reset

  // Note: deepest sleep means we do not look at RstAction (since unpowered)
  assign rst_action   = iDeepestSleep ? `RACT_NONE : iRstAction[2:0];
  assign rst_clear    = iDeepestSleep ? 1'b0       : iRstAction[3];
  assign oWake        = srst_act2 & iDeepestSleep;
  // block Reset is used if selected by action
  assign oRstBlock    = srst_act2 & (rst_action==`RACT_DEF) & ~oRstAll;
  // full reset is controlled by its own flop, else oRstRstAction would stop it early
  assign oRstAll      = rst_all;
  // reset iRstAction - held until next SCL edge, but no CCC possible without SCL
  assign oRstRstAction= srst_act2;
  // next is for custom ones, if enabled
  assign oRstCustom   = srst_act2 & (rst_action==`RACT_CUST);


  //
  // 2. Expanded Exit detector for Reset detection
  //    -- Note that Sr then P is isolated
  //

  // counter for 7 falling SDA (rising then Sr is handled sep)
  // If SCL goes high, we reset since can only be valid while
  // SCL Low
  // Note: SDA_n (falling SDA) driven
  always @ (posedge clk_SDA_n or negedge scl_rst_n)
    if (~scl_rst_n)
      stp_cnt <= 7'd0;                  // SCL High or disabled
    else 
      stp_cnt <= {stp_cnt[5:0], 1'b1};  // shift chain counter

  generate if (USE_RST_FLOP) begin : rst_flop
    // reset flop if concerned with race on SCL rise
    // this is a bit complex looking, but here is how it works:
    // 1. If stp_cnt[]==0, then mechanism is off (no impact on stp_cnt)
    // 2. If stp_cnt[]!=0, this mechanism engages, waiting for SCL rise
    // 3. SCL rise causes it to reset stp_cnt, but after stp_cnt[6]
    //    registered below.
    // 4. The reset of stp_cnt causes this flop to reset per 1 above.
    reg rst_r;
    wire rst_reset_r = enable_RSTn & (|stp_cnt | scan_no_rst);
    always @ (posedge clk_SCL or negedge rst_reset_r)
      if (~rst_reset_r)
        rst_r <= 1'b1;
      else if (|stp_cnt)
        rst_r <= 1'b0;
    assign scl_rst_n = enable_RSTn & (rst_r | scan_no_rst);
    `Observe(observe_srstp_rst, clk_SDA_n, i3c_slave_active & rst_r) // optional DFT observer
  end else begin
    // direct reset (combo)
    // SCL must be 0 for SlaveReset, so SCL=1 resets 
    assign scl_rst_n = enable_RSTn & (~pin_SCL_in | scan_no_rst);
    `Observe(observe_srstp_rst, clk_SDA_n, i3c_slave_active & ~pin_SCL_in) // optional DFT observer
  end endgenerate

  //
  // 3. Then handler for SCL rising, Sr, then engagement of P (STOP)
  //    -- Once stp_cnt[7] is 1, we need to register on rising SCL
  //    -- Chain is: 
  //       a) register stp_cnt[6]=1 when SDA=1 on SCL rising before it resets
  //          Note: this reg resets when it is 1, SCL is Low or c.Q=1
  //       b) register SDA falling while SCL is High (so Sr) and a.Q=1
  //          Note: this reg resets when SCL=Low or SDA=High
  //       c) register STOP on rising SDA when SCL=1 and b.Q=1
  //          Note: Reset on SCL=0 or feedback from Peripheral or RSTn due to chip reset
  //


  // a) register stp_cnt[6]=1 when SDA=1 on SCL rising
  //    Resets when it is 1 and SCL=0 or c.Q=1
  assign rst0_act_n = enable_RSTn & (~(srst_act0 & (~pin_SCL_in | srst_act2)) | scan_no_rst);
  `Observe(observe_sract0_rst, clk_SCL, i3c_slave_active & (srst_act0 & (~pin_SCL_in | srst_act2))) // optional DFT observer
  always @ (posedge clk_SCL or negedge rst0_act_n)
    if (~rst0_act_n)
      srst_act0    <= 1'b0;
    else if (stp_cnt[6] & pin_SDA_in)   // SCL rises after SDA
      srst_act0    <= 1'b1;

  // b) register START (SDA falling when SCL=1) if a.Q=1
  //    Resets when it is 1 and SCL=0 or SDA=1
  assign rst1_act_n = enable_RSTn & (~(srst_act1 & (~pin_SCL_in | pin_SDA_in)) | scan_no_rst);
  `Observe(observe_sract1_rst, clk_SDA_n, i3c_slave_active & (srst_act1 & (~pin_SCL_in | pin_SDA_in))) // optional DFT observer
  always @ (posedge clk_SDA_n or negedge rst1_act_n)
    if (~rst1_act_n)
      srst_act1     <= 1'b0;
    else if (srst_act0 & pin_SCL_in)
      srst_act1     <= 1'b1;

  // c) register STOP on rising SDA when SCL=1 and b.Q=1
  //    Reset on SCL=0 but or feedback from Peripheral or RSTn due to chip reset
  assign  rst2_act_n = enable_RSTn & (~(srst_act2 & (~pin_SCL_in | rst_clear)) | scan_no_rst);
  `Observe(observe_sract2_rst, clk_SDA, i3c_slave_active & (srst_act2 & (~pin_SCL_in | rst_clear))) // optional DFT observer
  wire    final_match = srst_act1 & pin_SCL_in; // STOP
  always @ (posedge clk_SDA or negedge rst2_act_n)
    if (~rst2_act_n)
      srst_act2    <= 1'b0;
    else if (final_match)
      srst_act2    <= 1'b1;             // reset action

  // next one is latched so we reset chip (if wanted) and that is not cleared
  // too soon by oRstRstAction. This is ONLY reset by the chip being reset, or
  // by is_slave pulsing
  `Observe(observe_srall_rst, clk_SDA, i3c_slave_active & (srst_act2 & (~pin_SCL_in | rst_clear))) // optional DFT observer
  always @ (posedge clk_SDA or negedge enable_RSTn)
    if (~enable_RSTn)
      rst_all <= 1'b0;
    else if (final_match)
      rst_all <= (rst_action==`RACT_FULL) | srst_escalate;// full Reset only
      
  // now default escalation, which is cleared by iResetAction[3]
  assign rst_escalate_n = enable_RSTn & (~(srst_escalate & rst_clear) | scan_no_rst);
  `Observe(observe_srescal_rst, clk_SDA, i3c_slave_active & (srst_escalate & rst_clear)) // optional DFT observer
  always @ (posedge clk_SDA or negedge rst_escalate_n)
    if (~rst_escalate_n)
      srst_escalate <= 1'b0;
    else if (final_match & (rst_action==`RACT_DEF))
      srst_escalate <= 1'b1;            // escalate default until cleared 

endmodule

