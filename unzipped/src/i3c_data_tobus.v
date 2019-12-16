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
//  File            : i3c_data_tobus.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Thu Nov 14 12:28:26 2019 $
//  Revision        : $Revision: 1.62 $
//
//  IP Name         : i3c_data_tobus 
//  Description     : MIPI I3C Master-Read outbound Data buffered/fifoed S->M)
//    This contains support for the data buffering model and the mapping
//    between the SCL clock domain and the system (e.g. PCLK) domain.
//    This supports FIFO like mapping, including the base of a ping
//    pong buffer (2 entry FIFO). 
//    Use of internal deeper FIFO is optional. External FIFO support
//    is implicit from the shallow ping pong version
//    This also handles sync of errors during to-bus.
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
//    To-bus (from us to Master) including errors (e.g. underrun)
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_data_tobus #(
    parameter FIFO_TYPE       = 0,      // [1:0]==b01 if internal FIFO
    parameter EXT_FIFO        = 0,      // only if external fifo type
    parameter ENA_TOBUS_FIFO  = 0       // depth of to-bus as power of 2 from 2 up
  )
  (
  // define clocks and reset
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
  output              avail_tb_full,    // is 1 if full (FIFO trig or not)
  // next is FIFO count even if no FIFO
  output        [4:0] avail_byte_cnt,   // count in bytes in FIFO
  input         [1:0] tx_trig,          // FIFO trigger level if FIFO used
  output              int_tb,           // set when TX not full or <=trig
  // next ones are set and clear for sync of errors
  output              set_tb_urun_nack, // under run on header (so NACKed)
  output              set_tb_urun,      // under run on data
  output              set_tb_term,      // master rerminated read
  input               clear_tb_urun_nack,
  input               clear_tb_urun,
  input               clear_tb_term,
  // now nets in SCL_n domain
  output              tb_data_valid,    // b1 if full, b0 if empty
  output        [7:0] tb_datab,         // data for them to use if valid
  output              tb_end,           // held until data consumed
  input               tb_datab_ack,     // ack for 1 SCL_n clock when consumed
  input               tb_urun_nack,     // pulse for 1 SCL_n if NACKed due to no data
  input               tb_urun,          // pulse for 1 SCL_n under run and not end
  input               tb_term,          // pulse for 1 SCL_n when Master terminates read
  input               scan_no_rst       // prevents layered reset
  );

  // Table of Contents
  // 1. Overview of the buffer/FIFO model
  // 2. registers and wires for support
  // 3. Buffer/FIFO write control in CLK domain
  // 4. Buffer/FIFO read in SCL domain (not SCL_n)
  // 5. Sync handshakes from SCL domain for errors

 //
 // 2-entry default ping-pong scheme
 //
 generate if (FIFO_TYPE[`FIFO_INT_b]==0 || ENA_TOBUS_FIFO==0) begin : tb_pingpong

  //
  // This block provides a 2 entry FIFO (ping pong buffer) for
  // the SCL domain, with data to go to the Master.
  // The 2 entry FIFO allows for a very simple CDC boundary
  // since it is reduced from the normal binary vs. gray code
  // issues. Further, since SCL can stop suddenly, all of the
  // work is done on the system side.
  // The model for writer (this block) and reader (the SCL
  // side) is as follows:
  // 1. System tb_widx (write index) is 2 bits, so one 
  //    more bit than is needed to index entries [0] & [1] 
  //    of the FIFO.
  // 2. The SCL scl_ridx is synchronized over to tb_ridx
  //    as a straight copy (since a gray code)
  // 3. The tb_widx (and by extension tb_ridx) increment
  //    using gray code: 00,01,11,10
  //    The index into the FIFO then is gray[1]^gray[0], 
  //    which gives 0,1,0,1 as expected.
  // 4. Empty is tb_widx==tb_ridx, where tb_ridx is the
  //    synchronized version of the scl_ridx.
  //    Likewise empty is scl_widx==scl_ridx, where 
  //    scl_widx is the synchronized version of tb_widx
  // 5. The system side can push when empty or when only
  //    one away, otherwise known as not full. This is 
  //    determined by:
  //      tb_widx==tb_ridx and ^(tb_widx^tb_ridx)==1
  //    Note that widx cannot be 3 away, only 2 away is
  //    full.
  // 6. The SCL side can pull when not empty. The only way
  //    to get to empty is by the SCL side reading, so
  //    simple. The SCL side synchronizes tb_widx on SCL
  //    vs. SCL_n, so safe.
  reg           [1:0] tb_widx;          // reg: sys write ptr 
  wire          [1:0] tb_ridx;          // sync/reg: SCL read ptr
  wire          [1:0] scl_widx;         // sync/reg: sys write ptr in SCL domain
  reg           [1:0] scl_ridx;         // reg: SCL read index
  wire                tb_not_full;      // set when not full, so can push
  wire                tb_empty;         // set when empty
  wire                tb_wptr;          // point into FIFO from tb_widx
  wire                scl_rptr;         // point into FIFO from scl_ridx
  wire                scl_empty;        // empty detect in SCL side
  reg                 ack_push;         // 1 cycle pulse. Safest this way
  reg           [1:0] next_widx, next_ridx; // case to expand gray code
    // now fifo buffers - used to pull from ext FIFO or reg 
  reg           [8:0] tbfifo[0:1];     // extra bit for end mark

  // sync their read index and our write index. 
  // gray codes, so independent
  SYNC_S2C #(.WIDTH(2)) sync_scl_ridx(.rst_n(RSTn), .clk(CLK), .scl_data(scl_ridx), 
                              .out_clk(tb_ridx));
  SYNC_C2S #(.WIDTH(2)) sync_tb_widx(.rst_n(RSTn), .scl(SCL), .clk_data(tb_widx), 
                             .out_scl(scl_widx));

  //
  // system CLK domain
  //
  assign tb_empty    = tb_widx==tb_ridx;
  assign tb_not_full = tb_empty | ^(tb_widx ^ tb_ridx);
  assign tb_wptr     = ^tb_widx;        // gives 0,1,0,1

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn) begin
      tb_widx          <= 2'b00;
      ack_push         <= 1'b0;
      tbfifo[0]        <= 9'd0;
      tbfifo[1]        <= 9'd0;
    end else if (tb_flush) // note that flush overrides push
      tb_widx          <= tb_ridx; 
    else if (avail_tb_ready & tb_not_full) begin
      // can safely push next value. Note that they sync 
      // tb_widx, so no risk on bits changing (they see 
      // widx  after it is holding stable)
      tbfifo[tb_wptr]  <= {avail_tb_end, avail_tb_data};
      tb_widx          <= next_widx;
      ack_push         <= 1'b1;
    end else
      ack_push         <= 1'b0;

  assign avail_tb_ack   = (EXT_FIFO==`EXT_FIFO_REQ) ?
                          (avail_tb_ready & tb_not_full) : // combo
                          ack_push;     // registered
  assign int_tb         = |tx_trig ? tb_not_full : tb_empty; // trig_level 0 special
  assign avail_tb_full  = ~tb_not_full;
  assign avail_byte_cnt = tb_empty ? 5'd0 : tb_not_full ? 5'd1 : 5'd2;

  // gray code
  always @ ( * ) 
    case(tb_widx)
    2'b00: next_widx = 2'b01;
    2'b01: next_widx = 2'b11;
    2'b11: next_widx = 2'b10;
    2'b10: next_widx = 2'b00;
    // no default since complete
    endcase

  //
  // SCL clock domain (not SCL_n) for CDC
  //

  // read pointer
  always @ (posedge SCL or negedge RSTn)
    if (~RSTn) 
      scl_ridx <= 2'b00;
    else if (tb_datab_ack & ~scl_empty)
      scl_ridx <= next_ridx;

  assign scl_rptr      = ^scl_ridx;     // gives 0,1,0,1
  assign scl_empty     = scl_widx==scl_ridx;
  assign tb_data_valid = ~scl_empty; 
  assign tb_end        = tbfifo[scl_rptr][8];
    // note below will have uncertain values when not selected and not used yet
  assign tb_datab      = tbfifo[scl_rptr][7:0];

  // gray code
  always @ ( * ) 
    case(scl_ridx)
    2'b00: next_ridx = 2'b01;
    2'b01: next_ridx = 2'b11;
    2'b11: next_ridx = 2'b10;
    2'b10: next_ridx = 2'b00;
    // no default since complete
    endcase

 //
 // internal FIFO
 //
 end else begin : tb_fifo_inst
  // instantiate FIFO, with correct size and optional holding
  i3c_internal_tb_fifo #(.BITS(ENA_TOBUS_FIFO), 
                         .USE_HOLDING(FIFO_TYPE[`FIFO_TBHOLD_b]))
    tb_fifo(.RSTn(RSTn), .CLK(CLK), .SCL(SCL), .SCL_n(SCL_n), 
            .avail_tb_ready(avail_tb_ready),
            .avail_tb_data(avail_tb_data), .avail_tb_ack(avail_tb_ack),
            .avail_tb_end(avail_tb_end), .tb_flush(tb_flush), 
            .avail_tb_full(avail_tb_full), .avail_byte_cnt(avail_byte_cnt), 
            .tx_trig(tx_trig), .int_tb(int_tb), .tb_data_valid(tb_data_valid), 
            .tb_datab(tb_datab), .tb_end(tb_end), .tb_datab_ack(tb_datab_ack), 
            .scan_no_rst(scan_no_rst));
 end endgenerate

  //
  // Sync errors using SCL
  //

  // underun errors on to-bus (header and read byte)
  SYNC_2PH_S2C_STATE synch_tb_urun_nack(.rst_n(RSTn), .scl(SCL), .clk(CLK), 
                                        .trig_scl(tb_urun_nack), 
                                        .out_clk(set_tb_urun_nack), .clear_clk(clear_tb_urun_nack));
  SYNC_2PH_S2C_STATE synch_tb_urun     (.rst_n(RSTn), .scl(SCL), .clk(CLK), 
                                        .trig_scl(tb_urun), 
                                        .out_clk(set_tb_urun), .clear_clk(clear_tb_urun));

  // terminate is from the master killing a read before we are done
  SYNC_2PH_S2C_STATE synch_tb_term     (.rst_n(RSTn), .scl(SCL), .clk(CLK), 
                                        .trig_scl(tb_term), 
                                        .out_clk(set_tb_term), .clear_clk(clear_tb_term));


endmodule
