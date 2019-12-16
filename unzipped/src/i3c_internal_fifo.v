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
//  File            : i3c_internal_fifo.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Thu Nov 14 12:28:26 2019 $
//  Revision        : $Revision: 1.62 $
//
//  IP Name         : i3c_internal_fifo 
//  Description     : MIPI I3C optional FIFO when enabled
//    This contains support for Sunburst design style FIFOs across
//    the clock domains - System and SCL. 
//    This file contains 4 modules:
//    1. The To-bus FIFO support, including FIFO mem
//    2. The From-bus FIFO support, including FIFO mem
//    3. The Write side FIFO control
//    4. The Read side FIFO control
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
//    This implements two low level FIFO controls for clock crossing.
//    It then instantiates one for from-bus and one for to-bus
//    Note that forcing the fifo to empty is handled by using reset
//    in system clock domain, which is safe as long as no actions
//    are trying to take place on reset release (which may be on clock
//    boundary).
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

// To-bus uses CLK to write, and SCL to read
module i3c_internal_tb_fifo #(
    parameter BITS = 3,                 // size in bits (2^BITS)
    parameter USE_HOLDING = 0           // 1 if holding buff
  )
  (
  input               RSTn,             // global reset
  input               CLK,              // system clock (bus or other)
  input               SCL,              // passed up from SDR - not SCL_n
  input               SCL_n,            // passed up from SDR - SCL_n
  // now nets in CLK domain to memory mapped registers or nets
  input               avail_tb_ready,   // to-bus byte is ready
  input         [7:0] avail_tb_data,    // byte
  output              avail_tb_ack,     // we have used byte
  input               avail_tb_end,     // last one - with ready
  input               tb_flush,         // 1 cycle pulse to flush buff
  output              avail_tb_full,    // truly full (not trig)
  // next is FIFO count even if no FIFO
  output        [4:0] avail_byte_cnt,   // count in bytes in FIFO
  input         [1:0] tx_trig,          // trigger percent
  output              int_tb,           // set when TX not full/trig
  // now nets in SCL_n domain
  output              tb_data_valid,    // b1 if full, b0 if empty
  output        [7:0] tb_datab,         // data for them to use if valid
  output              tb_end,           // held until data consumed
  input               tb_datab_ack,     // ack for 1 SCL_n clock when consumed
  input               scan_no_rst       // prevents layered reset
  );

  wire       [BITS:0] rclk_wgray, wclk_rgray;
  wire                read_empty, write_full;
  wire       [BITS:0] export_rgray, export_wgray;
  wire     [BITS-1:0] read_idx, write_idx, local_ridx;
  reg                 wr_ack;
  wire                reset_flush_n;
  wire          [4:0] tmp;
  wire                wr_is_empty;
  wire          [1:0] match;
  reg                 trig;
  wire     [BITS-1:0] hold_next_idx;
  // actual FIFO
  reg           [8:0] tb_fifo[0:(1<<BITS)-1]; // a memory. Note 9 bits
  wire          [8:0] opt_holding;
  wire                write_one;

  // we sync between: write in CLK, read in SCL
  SYNC_S2C #(.WIDTH(BITS+1)) w2r (.rst_n(RSTn), .clk(SCL), .scl_data(export_wgray), 
                          .out_clk(rclk_wgray));
  SYNC_S2C #(.WIDTH(BITS+1)) r2w (.rst_n(RSTn), .clk(CLK), .scl_data(export_rgray), 
                          .out_clk(wclk_rgray));

  // flush clears everthing by reset, released on next clock (sync)
  assign reset_flush_n = RSTn & (~tb_flush | scan_no_rst);

  // tobus means we write from CLK and we read from SCL
  i3c_internal_fifo_write #(.BITS(BITS)) tb_wr_fifo(.RSTn(reset_flush_n), .WCLK(CLK), 
                                     .write_one(write_one), .wclk_rgray(wclk_rgray),
                                     .write_full(write_full), .export_wgray(export_wgray), 
                                     .write_idx(write_idx));
  i3c_internal_fifo_read #(.BITS(BITS)) tb_rd_fifo(.RSTn(reset_flush_n), .RCLK(SCL), 
                                     .read_one(tb_datab_ack), .rclk_wgray(rclk_wgray), 
                                     .read_empty(read_empty), .export_rgray(export_rgray), 
                                     .read_idx(read_idx), .next_read_idx(hold_next_idx));

  // ack is the 1 cycle push operation (write_one)
  assign write_one = avail_tb_ready & ~write_full;
  always @ (posedge CLK or negedge RSTn)
    if (~RSTn)
      wr_ack <= 1'b0;
    else if (write_one)
      wr_ack <= 1'b1;           // we can push over multiple cycles
    else 
      wr_ack <= 1'b0;
  assign avail_tb_ack   = wr_ack; // accepted byte
  assign tb_data_valid  = ~read_empty;
  assign tb_datab       = opt_holding[7:0];
  assign tb_end         = opt_holding[8] & tb_data_valid;

  // now build holding buffer if they want - isolates from FIFO
  generate if (USE_HOLDING) begin : to_bus_holding
    reg       [8:0] holding;
    reg  [BITS-1:0] last_idx;
    wire [BITS-1:0] use_idx = (tb_datab_ack & ~read_empty) ? hold_next_idx : read_idx;
    always @ (posedge SCL or negedge RSTn)
      if (!RSTn) begin
        holding   <= 9'd0;
        last_idx  <= {BITS{1'b0}};
      end else if ((last_idx != use_idx) |
                   (holding != tb_fifo[use_idx])) begin
        // note: I do not like this compare. Would rather check
        // if write has happened, but for now...
        holding   <= tb_fifo[use_idx];
        last_idx  <= use_idx;
      end
    assign opt_holding = holding;
  end else begin
    assign opt_holding = tb_fifo[read_idx];
  end endgenerate

  // FIFO memory needs no reset since we write 1st
  // But, we use here to make easier for DFT
  integer ini;
  always @ (posedge CLK or negedge RSTn)
    if (~RSTn) begin
      for (ini = 0; ini < (1<<BITS); ini = ini + 1)
        tb_fifo[ini]     <= 9'd0;
    end else if (avail_tb_ready & ~write_full)
      tb_fifo[write_idx] <= {avail_tb_end, avail_tb_data};
    
  // we need to produce the available bytes value which is hard as is, 
  // but we make read gray into normal number and then subtract
  assign wr_is_empty    = export_wgray == wclk_rgray;
  assign avail_tb_full  = write_full;
  assign tmp            = (write_idx-local_ridx);
  assign avail_byte_cnt = tmp[(BITS-1):0] ? tmp[(BITS-1):0] : wr_is_empty ? 5'd0 : (5'd1<<BITS);
  assign match          = tmp[(BITS-1) -: 2];   // top two bits
  assign int_tb         = write_full ? 1'b0 : // never on full
                          wr_is_empty ? 1'b1 :// always on empty
                          trig;               // else percentage
  always @ ( * )
    case (tx_trig)
    2'b00:   trig = wr_is_empty;        // only if empty
    2'b01:   trig = tmp[(BITS-1):0] <= (1<<BITS)/4;
    2'b10:   trig = tmp[(BITS-1):0] <= (1<<BITS)/2;
    2'b11:   trig = 1'b1;               // anything but full
    default: trig = 1'b0;
    endcase

  // Conversion is just XOR of upper to current
  genvar i;
  generate for (i = BITS-1; i >= 0; i = i -1) begin : tb_gen_num
    assign local_ridx[i] = ^wclk_rgray[BITS:i];
  end endgenerate

endmodule

// From-bus uses SCL to write and CLK to read
// Note that control is still on CLK side
module i3c_internal_fb_fifo #(
    parameter BITS = 3                  // size in bits (so 2^BITS)
  )
  (
  input               RSTn,             // global reset
  input               CLK,              // system clock (bus or other)
  input               SCL,              // passed up from SDR - not SCL_n
  input               SCL_n,            // passed up from SDR - SCL_n
  // now nets in CLK domain to memory mapped registers or nets
  output              notify_fb_ready,  // from-bus byte is ready     
  output        [7:0] notify_fb_data,   // byte
  input               notify_fb_ack,    // byte taken
  input               fb_flush,         // 1 cycle pulse to flush buff
  output              avail_fb_empty,   // truly empty not trig
  output        [4:0] avail_byte_cnt,   // count in bytes in FIFO
  input         [1:0] rx_trig,          // trigger for FIFO
  output              int_rx,           // RX not empty or trigger 
  // now nets in SCL_n domain
  output              fb_data_use,      // b01, b10, or b00 (full)
  input         [7:0] fb_datab,         // output byte to system if done=1 and use=01
  input               fb_datab_done,    // done pulsed for one clk_SCL_n when written
  input               scan_no_rst       // prevents layered reset
  );

  wire       [BITS:0] rclk_wgray, wclk_rgray;
  wire                read_empty, write_full;
  wire       [BITS:0] export_rgray, export_wgray;
  wire     [BITS-1:0] read_idx, write_idx, local_widx;
  wire                reset_flush_n;
  wire          [4:0] tmp;
  wire          [1:0] match;
  reg                 trig;
  wire     [BITS-1:0] unused2;
  // actual FIFO
  reg           [7:0] fb_fifo[0:(1<<BITS)-1]; // a memory. Note 8 bits

  // we sync between: write in SCL, read in CLK
  SYNC_S2C #(.WIDTH(BITS+1)) w2r (.rst_n(RSTn), .clk(CLK), .scl_data(export_wgray), 
                          .out_clk(rclk_wgray));
  SYNC_S2C #(.WIDTH(BITS+1)) r2w (.rst_n(RSTn), .clk(SCL), .scl_data(export_rgray), 
                          .out_clk(wclk_rgray));

  // flush clears everthing by reset, released on next clock (sync)
  assign reset_flush_n = RSTn & (~fb_flush | scan_no_rst);

  // tobus means we write from CLK and we read from SCL
  i3c_internal_fifo_write #(.BITS(BITS)) fb_wr_fifo(.RSTn(reset_flush_n), .WCLK(SCL),
                                     .write_one(fb_datab_done), .wclk_rgray(wclk_rgray),
                                     .write_full(write_full), .export_wgray(export_wgray), 
                                     .write_idx(write_idx));
  i3c_internal_fifo_read #(.BITS(BITS)) fb_rd_fifo(.RSTn(reset_flush_n), .RCLK(CLK), .read_one(notify_fb_ack), 
                                     .rclk_wgray(rclk_wgray), 
                                     .read_empty(read_empty), .export_rgray(export_rgray), 
                                     .read_idx(read_idx), .next_read_idx(unused2));

  assign notify_fb_ready= ~read_empty;  // data waiting
  assign notify_fb_data = fb_fifo[read_idx];
  assign fb_data_use    = write_full ? 1'b0 : 1'b1; 

  // FIFO memory has no reset since we write 1st
  always @ (posedge SCL)
    if (fb_datab_done & ~write_full)
      fb_fifo[write_idx] <= fb_datab; 
    
  // we need to produce the available bytes value which is hard as is, 
  // but we make read gray into normal number and then subtract
  assign tmp            = (local_widx-read_idx);
  assign avail_byte_cnt = tmp[(BITS-1):0] ? tmp[(BITS-1):0] : 
                             read_empty ? 5'd0 : (5'd1<<BITS);
  assign match          = tmp[(BITS-1) -: 2];// top two bits
  assign int_rx         = read_empty ? 1'b0 :// never on empty
                          ~|tmp[(BITS-1):0] ? 1'b1 : // full
                          trig;              // else percentage
  assign avail_fb_empty = read_empty;

  always @ ( * )
    case (rx_trig)
    2'b00:   trig = 1'b1;               // anything but empty
    2'b01:   trig = tmp[(BITS-1):0] >= (1<<BITS)/4;
    2'b10:   trig = tmp[(BITS-1):0] >= (1<<BITS)/2;
    2'b11:   trig = tmp[(BITS-1):0] >= ((1<<BITS)/4)*3;
    default: trig = 1'b0;
    endcase

  // Conversion is just XOR of upper to current
  genvar i;
  generate  for (i = BITS-1; i >= 0; i = i -1) begin : fb_gen_num
    assign local_widx[i] = ^rclk_wgray[BITS:i];
  end endgenerate

endmodule


// FIFO for write side. Based on Sunburst design FIFO
// -- Note that we do not register full since short paths
module i3c_internal_fifo_write #(parameter BITS=3) // bit size, so 2^BITS
  (
  input               RSTn,             // global reset
  input               WCLK,             // clock of write side (CLK or SCL)
  input               write_one,        // want to push new value (if not full)
  input      [BITS:0] wclk_rgray,       // read gray count sync to our domain
  output              write_full,       // is full, else can write
  output     [BITS:0] export_wgray,     // our exported gray count
  output   [BITS-1:0] write_idx         // index into FIFO
  );
 
  // notes:
  // 1. Write side only cares about full or not full since we
  //    can write if not full, else we cannot.
  // 2. Can only get full by us writing (pushing)
  //    -- So detected as it happens in this (wclk) domain
  //    -- This is safe since it cannot over write regardless
  //       of ratio of clocks
  // 3. Change to Not-full synchronized from read side, so delayed
  //    -- Which only menas it takes longer before we can write
  //       a new value, but no risk
  // Model of use is counters are 1 bit larger than needed to index
  // FIFO. This allows us to separate ridx==widx as emprt vs. as
  // 2. Can only get empty by us reading, so clocked to rclk
  // 3. Not empty synchronized from write side, so delayed
  // -- So, naturally safe
  // Model of use is counters are 1 bit larger than needed to index
  // FIFO. This allows us to separate ridx==widx is empty vs. full.
  // The extra bit means empty is r==w and full differs by MSB
  reg     [BITS:0] write_full_idx, write_gray;
  wire    [BITS:0] next_full_idx, next_gray;

  // normal counter 1st: write increments the counter, but only if 
  // not full. Error on write full handled higher up
  // We use this to form FIFO index as well (but 1 less bit)
  always @ (posedge WCLK or negedge RSTn)
    if (~RSTn)
      write_full_idx <= {BITS+1{1'b0}};
    else if (write_one & ~write_full)
      write_full_idx <= next_full_idx;
  assign next_full_idx = write_full_idx + {{BITS{1'b0}}, 1'b1};
  assign write_idx     = write_full_idx[BITS-1:0]; // index into FIFO
    // note below: write only cares about full, not empty. We use
    // combinatorial full because short paths for use
    // logic below tests for inner ==, but then MSB and next-MSB !=
    // to match gray wrap
  assign write_full    = write_gray == {~wclk_rgray[BITS -: 2],
                                       wclk_rgray[BITS-2:0]};

  // gray code counter for export over clock domain
  // write increments the gray code for sharing 
  always @ (posedge WCLK or negedge RSTn)
    if (~RSTn)
      write_gray <= {BITS+1{1'b0}};
    else if (write_one & ~write_full)
      write_gray <= next_gray;
  assign next_gray     = next_full_idx[BITS:1] ^ next_full_idx;
  assign export_wgray  = write_gray;

endmodule


// FIFO for read side. Based on Sunburst design FIFO
// -- Note that we do not register empty since short paths
module i3c_internal_fifo_read #(parameter BITS=3) // bit size, so 2^BITS
  (
  input               RSTn,             // global reset
  input               RCLK,             // clock of read side (CLK or SCL)
  input               read_one,         // want new value
  input      [BITS:0] rclk_wgray,       // write gray count sync to our domain
  output              read_empty,       // is empty (else can read)
  output     [BITS:0] export_rgray,     // our exported gray count
  output   [BITS-1:0] read_idx,         // index into FIFO
  // next is only used for holding buffer if enabled
  output   [BITS-1:0] next_read_idx
  );
 
  // notes:
  // 1. Read side only cares about empty or not empty since we
  //    can read if not empty, else we cannot
  // 2. Can only get empty by us reading (popping)
  //    -- So detected as it happens in this (rclk) domain
  //    -- Which is safe since cannot over read regardless of 
  //       ratio of clocks.
  // 3. Change to Not-empty synchronized from write side, so delayed
  //    -- Which only means it takes longer before we can read
  //       a new value, but no risk
  // Model of use is counters are 1 bit larger than needed to index
  // FIFO. This allows us to separate ridx==widx is empty vs. full.
  // The extra bit means empty is r==w and full differs by MSB
  reg     [BITS:0] read_full_idx, read_gray;
  wire    [BITS:0] next_full_idx, next_gray;

  // normal counter 1st: read increments the counter, but only if 
  // not empty. Error on read empty handled higher up
  // We use this to form FIFO index as well (but 1 less bit)
  always @ (posedge RCLK or negedge RSTn)
    if (~RSTn)
      read_full_idx <= {BITS+1{1'b0}};
    else if (read_one & ~read_empty)
      read_full_idx <= next_full_idx;
  assign next_full_idx = read_full_idx + {{BITS{1'b0}}, 1'b1};
  assign read_idx      = read_full_idx[BITS-1:0]; // index into FIFO
    // note below: read only cares about empty, not full. We use
    // combinatorial empty because short paths for use
  assign read_empty    = read_gray == rclk_wgray; // empty is matching

  // gray code counter for export over clock domain
  // read increments the gray code for sharing 
  always @ (posedge RCLK or negedge RSTn)
    if (~RSTn)
      read_gray <= {BITS+1{1'b0}};
    else if (read_one & ~read_empty)
      read_gray <= next_gray;
  assign next_gray     = next_full_idx[BITS:1] ^ next_full_idx;
  assign export_rgray  = read_gray;

  // special for holding
  assign next_read_idx   = next_full_idx[BITS-1:0]; // index into FIFO
endmodule


