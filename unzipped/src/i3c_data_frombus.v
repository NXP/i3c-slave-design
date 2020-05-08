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
//  File            : i3c_data_frombus.v
//  Organisation    : MCO
//  Tag             : 1.1.11
//  Date            : $Date: Thu Nov 14 12:28:26 2019 $
//  Revision        : $Revision: 1.62 $
//
//  IP Name         : i3c_data_frombus 
//  Description     : MIPI I3C Master inbound Data buffered/fifoed (M->S)
//    This contains support for the data buffering model and the mapping
//    between the SCL clock domain and the system (e.g. PCLK) domain.
//    It is for 'write' messages and for commands not handled by the block
//    This supports FIFO like mapping, including the base of a ping
//    pong buffer (2 entry FIFO). 
//    Use of internal deeper FIFO is optional. External FIFO support
//    is implicit from the shallow ping pong version
//    This also handles sync of errors from-bus.
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
//    This suppports the Clock Domain Crossing aspect for data for 
//    From-bus (from Master to us, the Slave) including errors
//    (e.g. overrun).
//    This also creates an optional 1 byte holding buffer for from-bus 
//    when no FIFO, so that the system has a bit more time.
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_data_frombus #(
    parameter FIFO_TYPE       = 0,      // [1:0]==b01 if internal FIFO
    parameter ENA_FROMBUS_FIFO= 0,      // depth of from-bus as power of 2 from 2 up
    parameter ANY_HDR         = 0       // HDR of any kind
  )
  (
  // define clocks and reset
  input               RSTn,             // global reset
  input               CLK,              // system clock (bus or other)
  input               SCL,              // passed up from SDR - not SCL_n
  input               SCL_n,            // passed up from SDR - same as engine uses
  // now nets in CLK domain to memory mapped registers or nets
  output              notify_fb_ready,  // from-bus byte is ready     
  output        [7:0] notify_fb_data,   // byte
  input               notify_fb_ack,    // byte taken
  input               fb_flush,         // want us to wipe out the RX buffers
  output        [3:0] set_fb_err,       // parity error, CRC, S0/S1 status/int
  output              set_fb_orun,      // overrun fb status/int
  input         [3:0] clear_fb_err,     // cleared by app
  input               clear_fb_orun,    // "
  // now the FIFO related info (whether FIFO or not)
  output        [4:0] avail_byte_cnt,   // count of bytes in FIFO
  output              avail_fb_empty,   // truly empty and not trig
  input         [1:0] rx_trig,          // trigger for FIFO
  output              int_rx,           // RX not empty or trigger 
  // now the SCL_n domain controls and buffers (they own bytes)
  output              fb_data_use,      // b1 if OK, b0 if full
  input         [7:0] fb_datab,         // output byte to system if done=1 and use=1
  input               fb_datab_done,    // done pulsed for one clk_SCL_n when written
  input               fb_datab_err,     // set with done if i3c parity error
  input               fb_orun,          // pulse for one clk_SCL_n if over-run (full when data in)
  input         [2:0] fb_ddr_errs,      // HDR framing, parity, CRC
  input               fb_s0s1_err,      // held 1 if in S0 or S1 error hold
  output              brk_s0s1_err,     // output 1 in scl domain to break S0/S1 error hold
  input               scan_no_rst       // prevents layered reset
  );

  // Table of Contents
  // 1. Overview of the buffer/FIFO model
  // 2. registers and wires for support, including a byte buffer
  // 3. Buffer/FIFO read side in CLK domain
  // 4. Buffer/FIFO write control in SCL domain (not SCL_n)
  // 5. Sync handshakes from SCL domain for errors

 //
 // 2-entry default ping-pong scheme
 //
 generate if (FIFO_TYPE[`FIFO_INT_b]==0 || ENA_FROMBUS_FIFO==0) begin : fb_pingpong

  // This block provides a 2 entry FIFO (ping pong buffer) in
  // the SCL domain, with data from the Master. Note that this
  // is distinct from the engine's byte buffer since that (a)
  // has to access by bits (and so would create mux depth with 
  // a FIFO), and (b) the SCL can stop any time, so we want 
  // control from the CLK domain side.
  // The 2 entry FIFO allows for a very simple CDC boundary
  // since it is reduced from the normal binary vs. gray code
  // issues. 
  // The model for writer (SCL side) and reader (this CLK
  // side) is as follows:
  // 1. SCL scl_widx (write index) is 2 bits, so one more 
  //    bit than is needed to index entries [0] & [1] of 
  //    the FIFO.
  // 2. The system side fb_ridx is synchronized over to 
  //    scl_ridx as a straight copy (since a gray code)
  // 3. The scl_widx (and by extension scl_ridx) increment
  //    using gray code: 00,01,11,10
  //    The index into the FIFO then is gray[1]^gray[0], 
  //    which gives 0,1,0,1 as expected.
  // 4. Empty is fb_widx==fb_ridx, where fb_ridx is the
  //    synchronized version of the scl_ridx. 
  //    Likewise empty is scl_widx==scl_ridx, where 
  //    scl_ridx is the synchronized version of fb_ridx
  // 5. The SCL side can push when empty or when only
  //    one away, otherwise known as not full. This is 
  //    determined by:
  //      scl_widx==scl_ridx and ^(scl_widx^scl_ridx)==1
  //    Note that widx cannot be 3 away, only 2 away is
  //    full. The SCL side synchronizes fb_ridx using SCL
  //    vs. SCL_n, so it waits less.
  // 6. The system side can pull when not empty. The only 
  //    way to get to empty is by the system side reading, 
  //    so simple. The system side synchronizes scl_widx.
  reg           [1:0] scl_widx;         // reg: our write ptr in SCL
  wire          [1:0] scl_ridx;         // sync/reg: sys read ptr in SCL
  wire          [1:0] fb_widx;          // sync/reg: SCL write ptr in sys domain
  reg           [1:0] fb_ridx;          // reg: sys read index
  wire                scl_not_full;     // set when not full, so can push
  wire                scl_empty;        // set when empty
  wire                fb_empty;         // empty detect in sys side
  wire                fb_1in;           // special case to give depth back
  reg           [1:0] next_widx, next_ridx; // case to expand gray code
  reg           [7:0] fbdata[0:1];      // actual FIFO
  wire          [8:0] opt_holding;
  wire                holding_data;

  // now sync SCL write index and SCL write index.
  // gray codes, so independent
  SYNC_S2C #(.WIDTH(2)) sync_scl_ridx(.rst_n(RSTn), .clk(CLK), .scl_data(scl_widx), 
                              .out_clk(fb_widx));
  SYNC_C2S #(.WIDTH(2)) sync_fb_widx(.rst_n(RSTn), .scl(SCL), .clk_data(fb_ridx), 
                             .out_scl(scl_ridx));

  //
  // system CLK domain
  //

  // manage the index
  always @ (posedge CLK or negedge RSTn)
    if (~RSTn)  
      fb_ridx      <= 2'b00;
    else if ((~opt_holding[8] | notify_fb_ack) & ~fb_empty) 
      fb_ridx      <= next_ridx;        // advance when they take
    else if (fb_flush) 
      fb_ridx      <= fb_widx;          // match write buffer to empty

  // now optional holding
  if (FIFO_TYPE[`FIFO_FBHOLD_b]) begin : fb_holding
    reg         [8:0] holding;          // holding buffer
    // holding buffer is used if they want. Note that the
    // logic below matches fb_ridx 
    always @ (posedge CLK or negedge RSTn)
      if (~RSTn)
        holding      <= 9'd0;
      else if ((~holding[8] | notify_fb_ack) & ~fb_empty) begin
        holding[7:0] <= fbdata[^fb_ridx];
        holding[8]   <= 1'b1;
      end else if (notify_fb_ack) 
        holding[8]   <= 1'b0;
      else if (fb_flush) 
        holding[8]   <= 1'b0;

    assign opt_holding = holding;
    assign holding_data    = holding[8];
    assign avail_fb_empty  = fb_empty & ~holding[8];
  end else begin
    assign opt_holding = {~fb_empty, fbdata[^fb_ridx]};
    assign avail_fb_empty  = fb_empty;
    assign holding_data    = 1'b0;
  end


  assign notify_fb_ready = opt_holding[8];
  assign int_rx          = &rx_trig ? (~fb_empty & ~fb_1in) : notify_fb_ready;
  assign fb_empty        = fb_widx==fb_ridx;
  assign fb_1in          = ^(fb_widx ^ fb_ridx); // not full, but 1 in
  assign avail_byte_cnt  = fb_empty ? (holding_data ? 5'd1 : 5'd0) :
                           fb_1in   ? (holding_data ? 5'd2 : 5'd1) : // 1 in the FIFO
                                       holding_data ? 5'd3 : 5'd2;   // 2 in the FIFO
  assign notify_fb_data  = opt_holding[7:0];

  // gray code
  always @ ( * ) 
    case(fb_ridx)
    2'b00: next_ridx = 2'b01;
    2'b01: next_ridx = 2'b11;
    2'b11: next_ridx = 2'b10;
    2'b10: next_ridx = 2'b00;
    // no default since complete
    endcase

  //
  // SCL clock domain (not SCL_n) for CDC
  //

  assign scl_empty   = scl_widx==scl_ridx;
  assign scl_not_full= scl_empty | ^(scl_widx ^ scl_ridx);

  // write pointer
  // note in SCL and not SCL_n - available sooner
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn) 
      scl_widx         <= 2'b00;
    else if (fb_datab_done & scl_not_full)
      // can safely push next value. Note that they sync 
      // scl_widx, so no risk on bits changing (they see 
      // widx after its holding stable)
      scl_widx         <= next_widx;

  // FIFO reset used for cleanliness
  always @ (posedge SCL or negedge RSTn)
    if (~RSTn) begin
      fbdata[0]         <= 8'd0;
      fbdata[1]         <= 8'd0;
    end else if (fb_datab_done & scl_not_full)
      fbdata[^scl_widx] <= fb_datab;

  // we tell them if we have consumed their buffer, so free
  assign fb_data_use = scl_not_full;

  // gray code
  always @ ( * ) 
    case(scl_widx)
    2'b00: next_widx = 2'b01;
    2'b01: next_widx = 2'b11;
    2'b11: next_widx = 2'b10;
    2'b10: next_widx = 2'b00;
    // no default since complete
    endcase

 //
 // internal FIFO
 //
 end else begin : fb_fifo_inst
  // instantiate FIFO
  i3c_internal_fb_fifo #(.BITS(ENA_FROMBUS_FIFO)) 
    fb_fifo(.RSTn(RSTn), .CLK(CLK), .SCL(SCL), .SCL_n(SCL_n), 
            .notify_fb_ready(notify_fb_ready), .notify_fb_data(notify_fb_data), 
            .notify_fb_ack(notify_fb_ack), .fb_flush(fb_flush), 
            .avail_fb_empty(avail_fb_empty), .avail_byte_cnt(avail_byte_cnt), 
            .rx_trig(rx_trig), .int_rx(int_rx), .fb_data_use(fb_data_use), 
            .fb_datab(fb_datab), .fb_datab_done(fb_datab_done), 
            .scan_no_rst(scan_no_rst));
 end endgenerate

  //
  // Synchronized errors using SCL
  //

  // over run on from-bus - we did not drain fast enough
  SYNC_2PH_S2C_STATE synch_fb_orun(.rst_n(RSTn), .scl(SCL), .clk(CLK), 
                                   .trig_scl(fb_orun), 
                                   .out_clk(set_fb_orun), .clear_clk(clear_fb_orun));

  //
  // Synchronized errors using SCL_n
  //

  // parity error on from-bus - bad 9th bit. 
  // Has to use SCL_n since parity arrives on SCL pos
  SYNC_2PH_S2C_STATE synch_fb_err0(.rst_n(RSTn), .scl(SCL_n), .clk(CLK), 
                                   .trig_scl(fb_datab_done & fb_datab_err), 
                                   .out_clk(set_fb_err[0]), .clear_clk(clear_fb_err[0]));
  generate if (ANY_HDR) begin : hdr_err
    // DDR parity and framing error if DDR enabled
    SYNC_2PH_S2C_STATE synch_fb_err1(.rst_n(RSTn), .scl(SCL_n), .clk(CLK), 
                                     .trig_scl(|fb_ddr_errs[1:0]), // framing and parity 
                                     .out_clk(set_fb_err[1]), .clear_clk(clear_fb_err[1]));
    SYNC_2PH_S2C_STATE synch_fb_err2(.rst_n(RSTn), .scl(SCL), .clk(CLK), 
                                     .trig_scl(fb_ddr_errs[2]), // CRC
                                     .out_clk(set_fb_err[2]), .clear_clk(clear_fb_err[2]));
  end else begin
    assign set_fb_err[2:1] = 2'b00;
  end endgenerate

  // S0 and S1 are level held, so we only sync to PCLK domaim
    // The break is a reset control, so we do not have to sync
  assign brk_s0s1_err = clear_fb_err[3]; // reset, so no sync
    // just map across
  SYNC_S2C sync_s0s1(.rst_n(RSTn), .clk(CLK), .scl_data(fb_s0s1_err), 
                     .out_clk(set_fb_err[3]));
endmodule
