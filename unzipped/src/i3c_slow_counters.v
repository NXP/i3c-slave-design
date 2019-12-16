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
//  File            : i3c_slow_counters.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Wed Nov 13 19:04:07 2019 $
//  Revision        : $Revision: 1.45 $
//
//  IP Name         : i3c_slow_counters 
//  Description     : MIPI I3C counters for timing for Slave for both 
//    uses like waiting on BusAvailable for IBI, as well as error detect
//    and breakout. It also can count to 200us for Hot Join tIDLE
//    This works off the slow clock. Not to be confused with the Time
//    control features
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
//  ----------------------------------------------------------------------------
// 
//  Count model is as follows:
//  1. We have a detector for bus idle (SCL not changing and
//     in_STOP for all but Read).
//  2. We have a 1us counter, which can be turned on from a
//     variety of causes. It will be stopped and/or restarted 
//     when the I3C SCL changes.
//     -- Base use is Bus Available
//  3. We have a 100us count model for 60us and 100us counts
//     -- Two match points: 60us for S0/S1 end, 100us for Read stall
//  4. We have a 200us count model (2 x 100us) for Hot Join
//     -- This stops being used after 1st, so we have an input to
//        say if new HJ (were powered off or super low power)
//  NOTE: because the 1us may be inaccurate, we use the 1ms value to
//        to determine 60us and 100us by approximation.
//        We mainly care that it is not less, but more is OK.
//        FUTURE: this could be reduced for 200us now that IDLE is not
//        1ms anymore. But, we would need an old and new way for 
//        backwards compatible uses.
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_slow_counters #(
    // params are driven from upper layer to control how this block
    // is built and operates
    parameter ENA_IBI_MR_HJ   = 0,      // 0 if no events, else events as mask
    parameter  CLK_SLOW_BITS  = 6,      // number of bits needed for count for Bus Avail
    parameter  CLK_SLOW_MATCH = 6'd47,  // count: e.g. 47 for 1us if 48MHz CLK (0 rel)
    parameter  CLK_SLOW_HJMUL = 10'd1000,// number of MATCHes (const or reg) for 1ms (1 rel)
    parameter  ENA_TIMEC      = 6'b000010,    // clock_reg, res, res, mode 1, mode 0, sync
    parameter  TIMEC_FREQ_ACC = {8'd24,8'd10},// freq=12MHz (12.0=24) with 1.0% accuracy
    parameter ENA_CCC_HANDLING= 6'd0    // passed down as to what to support
  )
  (
  // define clock and reset
  input               RSTn,             // reset from system
  input               CLK_SLOW,         // clock to use for IBI forced
  output              slow_gate,        // 1 if may gate CLK_SLOW
  input               clk_SCL,          // SCL clock 
  input               clk_SCL_n,        // SCL clock inverted
  // match clock counts
  input               cf_SlvEna,        // only process bus if 1
  input         [7:0] cf_BAMatch,       // Bus Available count match (if IBI+BAM)
  // requests to run the counters
  input         [2:0] event_pending,    // IBI, MR, or HJ
  input               run_60,           // S0/S1 stall
  input               run_100,          // Read stuck check
  // state detection 
  input               int_in_STOP,      // sync: in stop mode
  input               pin_SCL_in,       // used for state detect
  input               pin_SDA_in,       // used for state detect
  // now outputs to trigger actions
  output              force_sda,        // force SDA for IBI, MR, or HJ
  output              done_60,          // done with S0/S1
  output              done_100,         // done with read abort
  output              hold_engine       // hold off engine until HJ emits start
  );

  //
  // Table of contents
  //
  // 1. Request for counting controls for each reason
  // 2. Counter for 1us
  // 3. Detection of SCL state to detect clocks across clock domain
  // 4. Mid rate counter for 100us and 60us
  // 5. Hot-Join counter of 100us periods


  wire                    run_cnt, run_hotjoin;
  wire                    is_1us;
  wire                    is_hj;
  reg [CLK_SLOW_BITS-1:0] microsec_cnt;
  reg               [6:0] mid_cnt;      // max 127 reload
  reg               [1:0] hj_cnt;       // max of 2
  reg                     hj_done;      // only do HJ once after power up
  reg                     request_start, check_idle_n, check_idle;
  wire                    remote_check_idle, remote_check_idle_n;
  reg                     scl_check_idle, scl_check_idle_n;
  wire                    sclsda_state;
  wire                    safe_ibi_hj;

  assign run_cnt     = |event_pending[1:0] | run_60 | run_100 | run_hotjoin;
  assign run_hotjoin = &event_pending[1:0] & ENA_IBI_MR_HJ[`EV_HJ_b];
    // next makes sure SCL and SDA are high for IBI and  HJ. Note state vs. 
    // in_STOP since in_STOP does not clear until complete START
    // note: test below allows run_60/100 to override event_pending since 
    //       pending does not mean we are in STOP yet
  wire   is_stop     = int_in_STOP &    // HJ inits to in_STOP
                       (sclsda_state | force_sda); 
  assign safe_ibi_hj = (~|event_pending[1:0] | run_60 | run_100) | is_stop;
  // we gate the clock unless need it or finishing up counters
  assign slow_gate   = ~(run_cnt | (|microsec_cnt) | request_start | (|mid_cnt));
  // we notify when ready to pull SDA low (IBI, MR, HJ)
  assign force_sda   = request_start &
                       ((|event_pending[1:0] & ~&event_pending[1:0] & is_1us) |
                        is_hj);

  // we define the 1us (Bus Available) match as const or MMR
  generate if (ENA_IBI_MR_HJ[`EV_BAMATCH_b]) begin : ba_math
    assign is_1us = microsec_cnt == cf_BAMatch[CLK_SLOW_BITS-1:0];
  end else begin
    assign is_1us = microsec_cnt == CLK_SLOW_MATCH[CLK_SLOW_BITS-1:0];
  end endgenerate

    
  always @ (posedge CLK_SLOW or negedge RSTn)
    if (~RSTn) 
      microsec_cnt   <= {CLK_SLOW_BITS{1'b0}};
    else if (~request_start & |microsec_cnt)
      microsec_cnt   <= {CLK_SLOW_BITS{1'b0}};
    else if (request_start)             // if bus stall, 1 shot or free running
      if (~|is_1us)
        microsec_cnt <= microsec_cnt + {{CLK_SLOW_BITS-1{1'b0}},1'b1};
      else if (run_60 | run_100 | 
               (run_hotjoin & ~hj_done)) 
        microsec_cnt <= {CLK_SLOW_BITS{1'b0}}; // free running

  always @ (posedge CLK_SLOW or negedge RSTn)
    if (~RSTn) begin
      check_idle      <= 1'b0;
      check_idle_n    <= 1'b0;
      request_start   <= 1'b0;
    end else if (run_cnt & cf_SlvEna & safe_ibi_hj) begin
      // We use STOP & SCL=1/SDA=1 for Bus Avail and HJ, but just requesting state 
      // for the read abort and S0/S1 error; no risk since they wait for long enough
      // with no clock change to be sure not in SDR and HDR-DDR respectievly
      // We are here for IBI, MR, HJ, stall detect, or s0/s1 condition
      if ((check_idle   == remote_check_idle) | 
          (check_idle_n == remote_check_idle_n)) begin
        // SCL must have changed, so we start over
        check_idle    <= ~remote_check_idle;   // (!=) so we can tell if change happened
        check_idle_n  <= ~remote_check_idle_n; // "
        request_start <= 1'b0;          // can start over below
      end else if (~request_start) begin
        // we set our flag 1st
        request_start <= 1'b1;          // kick off timer
      end 
    end else begin
      check_idle      <= remote_check_idle; // match - ready for next time
      check_idle_n    <= remote_check_idle_n;
      request_start   <= 1'b0;
    end

  // bus inactive check is used to make sure no clocks go by. This provides
  // safety for measuring time the bus is idle using a slow clock. 
  // The app sets the iCheckIdle to ~oCheckIdleNot. If on the next 
  // tick (or some time later) it is still the opposite, there were 
  // no SCL clocks. If it is matching, then SCL has changed.
  // we handle both clock edges. Note that we do not disable the below
  // since it will stabilize
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn)
      scl_check_idle <= 1'b0;
    else if (scl_check_idle ^ check_idle)  // CDC
      scl_check_idle <= ~scl_check_idle;   // match
  always @ (posedge clk_SCL_n or negedge RSTn)
    if (!RSTn)
      scl_check_idle_n <= 1'b0;
    else if (scl_check_idle_n ^ check_idle_n)// CDC
      scl_check_idle_n <= ~scl_check_idle_n; // match
  SYNC_S2C #(.WIDTH(1)) sync_idle_check(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(scl_check_idle), 
                                .out_clk(remote_check_idle));
  SYNC_S2C #(.WIDTH(1)) sync_idle_check_n(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(scl_check_idle_n), 
                                  .out_clk(remote_check_idle_n));
    // next two track state
  SYNC_S2C #(.WIDTH(1)) sync_sclsda_state(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(pin_SCL_in&pin_SDA_in), 
                                  .out_clk(sclsda_state));

  // now we build the 100us and 60us counters. 
  // we build counts by rounding up from 1ms info
  localparam LOAD_100 = (CLK_SLOW_HJMUL+9)/10;  // from 1ms to ~100us
  localparam LOAD_60  = (CLK_SLOW_HJMUL+15)/16; // from 1ms to ~60us
  wire [0:0] bad_load;
  assign     bad_load[LOAD_100>127] = 1'b0; // catches overflow

  // counter is one-shot or free run depending on use
  always @ (posedge CLK_SLOW or negedge RSTn)
    if (~RSTn) 
      mid_cnt       <= 7'd0;
    else if (~run_60 & ~run_100 & ~run_hotjoin) begin
      if (|mid_cnt)
        mid_cnt     <= 7'd0;
    end else begin
      if (~request_start)
        mid_cnt     <= 7'd0;            // condition changed, reset cnt
      else if (is_1us) begin
        if (run_60) begin
          // 60us is one shot
          if (mid_cnt != LOAD_60)
            mid_cnt <= mid_cnt + 7'd1;  // count untol 60us
        end else if (mid_cnt != LOAD_100)
          mid_cnt   <= mid_cnt + 7'd1;  // count untol 60us
        else if (run_hotjoin)
          mid_cnt   <= 7'd0;            // hot join is free run until 10ms
      end
    end
  // we notify when 60us or 100us timing done
  assign done_100  = (run_100 & (mid_cnt==LOAD_100));
  assign done_60   = (run_60  & (mid_cnt==LOAD_60));

  // now Hot Join, if enabled. This uses 2 of the 100us counts (200us)
  always @ (posedge CLK_SLOW or negedge RSTn)
    if (~RSTn) begin 
      hj_cnt       <= 2'd0;
      hj_done      <= 1'b0;             // only once after POR
    end else if (~run_hotjoin) begin
      if (|hj_cnt)
        hj_cnt     <= 2'd0;
    end else if (~hj_done) begin
      if (~request_start)
        hj_cnt     <= 2'd0;             // condition changed, reset cnt
      else if (is_1us & (mid_cnt==LOAD_100)) begin
        // we only count 200us once; after that we use hj_done so like IBI
        if (hj_cnt != 2'd2) begin
          hj_cnt   <= hj_cnt + 2'd1;
          if (hj_cnt[0]) begin          // Done once we have started
            hj_done  <= 1'b1;           // one and done
            hj_cnt   <= 2'd0;           // not needed, done does it all
          end
        end
      end
    end
  assign is_hj = run_hotjoin & (hj_done & is_1us);

  // hold engine is because HotJoin means bus state unknown
  assign hold_engine = run_hotjoin & ~hj_done;

endmodule
