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
//  File            : sync_support.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Wed Jun 12 23:47:03 2019 $
//  Revision        : $Revision: 1.61 $
//
//  IP Name         : SYNC_ modules for general use
//  Description     : Clock crossing support for Slave
//    This contains support for cross-clock-domain (CDC) synchronization
//    of type 2-phase and other, covering pulses and levels. Used for 
//    SCL to (system) CLK and CLK to SCL. Used for single nets, handshakes,
//    FIFOs, etc. 
//    Note use of 2-phase handshakes due to SCL stopping suddenly.
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
//    This suppports the Clock Domain Crossing using named blocks
//    so Spyglass (or equiv) can be told that the sync is here.
//    For most modern process, the single flop model is fine, although
//    the flop type may need to be changed by constraint for older
//    process (e.g. a sync flop, which has a "gravity" to settle
//    faster). 
//    1 flop is fine in normal cases because the Q out metastable
//    noise will be short enough in time for the paths used at the 
//    lower speeds. That is, there is ~40ns minimum to both settle
//    and arrive settled at the other end. 
//  ----------------------------------------------------------------------------

// Note naming: SYNC_= synchronizing, 2PH_=2-phase, S2P_=SCL to CLK, STATE=state with clr in
//              LVL_=level vs. pulse input, LVLH_=level high
// Other naming: ASet=async set, local clear, AClr=local set, async clear
//               Seq2=2 flop sequencer to ensure we get 1 pulse in local domain
module SYNC_2PH_S2C_STATE( 
  input             rst_n,
  input             scl,
  input             clk,
  input             trig_scl,   // trigger input from SCL domain
  output            out_clk,    // output state in CLK domain
  input             clear_clk   // input clear from CLK domain
  );

  reg               scl_hshake;
  reg               clk_state;
  reg               clk_ack;

  // NOTE: this can be a bit confusing. The scl_  to clk_
  //       is 2-phase. The scl_hshake changes on the trigger.
  //       The clk_state changes on hshake^ack==1 for the 
  //       purpose of the use in the CLK domain. The clk_ack
  //       only changes when the clk_state has, so that we
  //       do not miss it and yet we do not expose metastable
  //       logic (hshake^ack) outside.
  //       We use 2-phase because SCL is not free running and
  //       may stop. So, CLK does all of the work.
  //       Note that state based ones are only cleared by
  //       explit request; so more changes may happen before
  //       cleared and that is OK.

  always @ (posedge scl or negedge rst_n)
    if (!rst_n)
      scl_hshake <= 1'b0;
    else if (trig_scl)                  // trigger pulse (1 clock in scl)
      scl_hshake <= ~scl_hshake;        // flips state

  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_state  <= 1'b0;
    else if (scl_hshake ^ clk_ack)      // CDC
      clk_state  <= 1'b1;               // sets on diff
    else if (clear_clk)
      clk_state  <= 1'b0;               // clears on explicit clear
  assign out_clk = clk_state;

  // ACK only chnages after state does and if still diff
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_ack    <= 1'b0;
    else if (clk_state & (scl_hshake^clk_ack)) // change on state - CDC
      clk_ack    <= ~clk_ack; 

endmodule 

module SYNC_2PH_LVL_S2C_STATE(
  input             rst_n,
  input             scl,
  input             clk,
  input             trig_scl,   // trigger level from SCL domain
  output            out_clk,    // output state in CLK domain
  input             clear_clk   // input clear from CLK domain
  );

  reg               clk_state;
  reg               clk_ack;

  // see SYNC_2PH_S2C_STATE for details. Only difference is
  // level held does not need scl_hshake since level (and 
  // if level too short for clk, then not registered). If
  // we want to make sure seen, then need independent bit
  // FUTURE: look at that for cases of stopped CLK

  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_state   <= 1'b0;
    else if (trig_scl ^ clk_ack) // level change - CDC
      clk_state   <= 1'b1; 
    else if (clear_clk)
      clk_state   <= 1'b0; 
  assign out_clk = clk_state;

  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_ack     <= 1'b0;
    else if (clk_state &
            (trig_scl ^ clk_ack)) // still diff - CDC
      clk_ack     <= ~clk_ack; 

endmodule

module SYNC_2PH_LVLH_S2C_STATE(
  input             rst_n,
  input             scl,
  input             clk,
  input             trig_scl,   // trigger level from SCL domain
  output            out_clk,    // output state in CLK domain
  input             clear_clk   // input clear from CLK domain
  );

  reg               clk_state;
  reg         [1:0] clk_ack;

  // see SYNC_2PH_S2C_STATE for details. 
  // Level High is a bit tricky. Clk_state only
  // goes high if level becomes High (edge if you
  // will). So, we need to remember what state it
  // was in so we only catch the edge. But, because
  // this across the clock domain, we cannot just
  // compare with the SCL level, since we may see 
  // it sooner or later and so miss the edge.
  // So we use an extra flop
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_ack     <= 2'b00;
    else if (clk_ack[0] ^ trig_scl)
      clk_ack[0]  <= ~clk_ack[0];       // CDC
    else if (^clk_ack)
      clk_ack[1]  <= ~clk_ack[1];       // 1 cycle behind

  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_state   <= 1'b0;
    else if (^clk_ack & ~clk_ack[1])
      clk_state   <= 1'b1; 
    else if (clear_clk)
      clk_state   <= 1'b0; 
  assign out_clk = clk_state;

endmodule

// Next is set async and clear local
module SYNC_Pulse_S2C(
  input SCL,
  input CLK,
  input RSTn,
  input local_set,
  output o_pulse);

  // 4 phase handshake, but pulse out
  reg  svalue, cvalue, cpulse;
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      svalue <= 1'b0;
    else if (local_set)
      svalue <= 1'b1;
    else if (cvalue) // CDC
      svalue <= 1'b0;

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      cvalue <= 1'b0;
    else if (svalue) // CDC
      cvalue <= 1'b1;
    else 
      cvalue <= 1'b0;
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      cpulse <= 1'b0;
    else if (cvalue)
      cpulse <= 1'b1;
    else
      cpulse <= 1'b0;
  assign o_pulse = cvalue & ~cpulse;

endmodule


// Next is set async and clear local, with sequence for 1 pulse out
module SYNC_ASet_Seq2(
  input CLK,
  input RSTn,
  input async_set,
  input local_clear, 
  output o_pulse);

  reg  value, seq;
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (async_set) // CDC
      value <= 1'b1;
    else if (local_clear) 
      value <= 1'b0;
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (seq ^ value)
      seq <= ~seq;
  // below pulses for 1 clock high, else is low
  assign o_pulse = value & ~seq;

endmodule

// Next is set local and clear async,  with sequence for 1 pulse out
module SYNC_AClr_Seq2(
  input CLK,
  input RSTn,
  input local_set,
  input async_clear, 
  output o_pulse);

  reg  value, seq;
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (async_clear) // CDC
      value <= 1'b0;
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (seq ^ value)
      seq <= ~seq;
  // below pulses for 1 clock high, else is low
  assign o_pulse = value & ~seq;

endmodule

// next is straight sync from one domain to the other
// for local use - 1 flop
module SYNC_S2C #(parameter WIDTH=1) ( 
  input             rst_n,
  input             clk,
  input  [WIDTH-1:0]scl_data,
  output [WIDTH-1:0]out_clk     // output copy in CLK domain
  );

  reg  [WIDTH-1:0]  clk_copy;

  assign out_clk = clk_copy;

  // note: could use clk_copy^scl_data as test to allow ICG
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_copy <= {WIDTH{1'b0}};
    else 
      clk_copy <= scl_data;
endmodule

// next is straight sync from one domain to the other
// for local use - 1 flop
module SYNC_C2S #(parameter WIDTH=1) ( 
  input             rst_n,
  input             scl,        // could be SCL_n as well
  input  [WIDTH-1:0]clk_data,
  output [WIDTH-1:0]out_scl     // output copy in CLK domain
  );

  reg   [WIDTH-1:0] scl_copy;

  assign out_scl = scl_copy;

  // note: could use clk_copy^scl_data as test to allow ICG
  always @ (posedge scl or negedge rst_n)
    if (!rst_n)
      scl_copy <= {WIDTH{1'b0}};
    else 
      scl_copy <= clk_data;
endmodule

// Next is set local and clear async from CLK to SCL
module SYNC_AClr_C2S(
  input CLK, // system domain
  input RSTn,
  input local_set,
  input async_clear,
  output o_value);

  reg  value;                   // CLK domain
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (async_clear) // CDC
      value <= 1'b0;
  assign o_value = value;

endmodule

