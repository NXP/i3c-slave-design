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
//  File            : i3c_dma_control.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Wed Jun 12 23:47:03 2019 $
//  Revision        : $Revision: 1.52 $
//
//  IP Name         : i3c_dma_control
//  Description     : MIPI I3C DMA managemenr for APB interface (slave and master)
//    This contains the DMA request/ack handling to support DMA load/drain of 
//    FIFOs. DMA is quite messy due to lack of standards. So, we have to separately
//    accomodate different types. See below under Imp details for more info.
//
//  ----------------------------------------------------------------------------
//                    Revision History
//  ----------------------------------------------------------------------------
//
//
//  ----------------------------------------------------------------------------
//                    Implementation details
//  ----------------------------------------------------------------------------
//  See the programmer's model specs for MMR control.
//  This block handles different kinds of DMAs known to exist. They differ in how
//  they handle request and ack and what they expect. The basic types are:
//  1. 4-phase handshake, ACK after read is done. So, current empty/full state is 
//     all that is needed. Most likely to get burned by MATCHSS for from-bus.
//  2. 4-phase handshake, ACK with read data (last phase of bus transaction), so 
//     can be prepared to read, 2 cycles after read.
//  3. 4-phase handshake, ACK at start of read (address phase), so can be prepared 
//     to read on cycle after (data phase drops request, so next cycle has request 
//     again). Since we never stall (PREADY is strapped 1), this looks to us like 
//     1 and 2 in  the sense of whether to request again.
//  4. Req/ACK normal. Only one-transaction will be outstanding. So, ACK tells us 
//     they saw the request, so if last value in from-bus FIFO or last empty space 
//     in to-bus FIFO, we will de-assert the request. Else, we hold it. 
//     a) If the ACK occurs with or after the MMR op, we can just skip the ACK and
//        only react to the read MMR op (read or wite data).
//     b) If the ACK precedes the MMR op, then we have an ARMing problem. So,
//        we need to deactivate req if the last FIFO entry, and re-activate req
//        on the MMR op.
//  5. Same as 4 but using 4-phase.
//  6. Burst trigger style.
//
//  Basic waveforms shown below. The changes for ACK are shown just before setup 
//  as with combinatorial or long path from Q:
//
//  4-phase fully registered looks like (cases 1, 2, 3, and 5)
//  Clk __^^^___^^^___^^^___^^^___^^^
//  Req ___/^^^^^^^^^^^\_____X======= (Req again if more)
//  Ack ____________/^^^^^^^^\_______
//
//  The request with ACK pulse looks like (caaes 4 a and b):
//  Clk __^^^___^^^___^^^___^^^
//  Req ___/^^^^^^^^^^^^^^^^^^^ Request held if not last on ACK
//  Req ___/^^^^^^^^^^^\_______ Request de-asserted if Last item on ACK
//  Ack __________/^^^^\_______
//  RdBef ................./^^^ // ACK before read
//  RdAft ......../^^^^\....... // ACK with or after read
//
//  We handle 1,2,3 using DMA_TYPE=3 (4PHASE)
//  We handle 4a using DMA_TYPE=2 (ACK1_AFT) meaning ACK with/after MMR access
//  We handle 4b using DMA_TYPE=1 (ACK1_BEF) meaning ACK begore MMR access
//  We do not handle 5 or 6 - IF SOMEONE NEEDS IT, WE CAN ADD EITHER
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_dma_control #(
  parameter  DMA_TYPE         = 2'd0)   // 0=req/ack, 1=trig, 2=req/ack with done, 3=req/ack with handshake
  (
  input               PRESETn,          // reset from system
  input               PCLK,             // system clock for regs and sync
  input         [5:0] dma_ctrl,         // settings from regs
  input               rx_trig,
  input               tx_trig,
  input         [1:0] rx_fullness,      // only care if 2 or more 
  input         [1:0] tx_avail,         // only care if space for 2 or more
  input               pread,            // read on APB happened
  output              dma_req_tb, 
  output              dma_req_fb, 
  input               dma_req_tb_ack, 
  input               dma_req_fb_ack
  );

  // trigger means one pulse in PCLK domain and DMA knows count.
  // else request/ack as long as possible (fullness)
  wire                rx_is_empty, tx_is_full;
  wire                rx_is_last, tx_is_last;
  //localparam DMA_TRIG   = 2'd1;

  generate
  if (DMA_TYPE==`DMA_4PHASE) begin : dma_type_ctrl
    // Covers case 1, 2, 3 but not 5 from Imp Notes
    // 1) if data to process, drive req flop. 2) On Ack, clear request.
    // Then go back to 1.
    // NOTE: we assume ACK->REQ=0 cycle happens on or after data phase, so request
    //       check on next cycle is safe.
    reg dma_req_fb_hshake;
    reg dma_req_tb_hshake;
    reg fb_auto_cancel;
    // NOTE: auto-cancel allows last RX byte to get to DMA for 1-frame case. But, 
    //       it cannot drain an RX FIFO if DMA is too slow to keep up.
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn) begin
        dma_req_fb_hshake <= 1'b0;
        fb_auto_cancel    <= 1'b0;
      end else if (dma_req_fb & ~|dma_ctrl[1:0] & ~fb_auto_cancel)
        dma_req_fb_hshake <= 1'b0;      // DMA canceled manually
      else if (dma_req_fb & dma_req_fb_ack)
        dma_req_fb_hshake <= 1'b0;      // done with a request
      else if (|dma_ctrl[1:0] & ~dma_req_fb & ~dma_req_fb_ack & 
               ~rx_is_empty) begin      // no trig since 4-phase
        dma_req_fb_hshake <= 1'b1;      // start request
        fb_auto_cancel    <= ~dma_ctrl[1];
      end
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn)
        dma_req_tb_hshake <= 1'b0;
      else if (dma_req_tb & ~|dma_ctrl[3:2])
        dma_req_tb_hshake <= 1'b0;      // DMA canceled 
      else if (dma_req_tb & dma_req_tb_ack)
        dma_req_tb_hshake <= 1'b0;      // done with a request
      else if (|dma_ctrl[3:2] & ~dma_req_tb & ~dma_req_tb_ack & 
               ~tx_is_full)             // no trig since 4-phase
        dma_req_tb_hshake <= 1'b1;      // start request

     assign dma_req_fb = dma_req_fb_hshake;
     assign dma_req_tb = dma_req_tb_hshake;
  end else if (DMA_TYPE==`DMA_ACK1AFT) begin
    // Covers case 4 but not 5 or 6 from Imp Notes
    // We set request as long as not empty/full, and each ack is
    // 1 cycle. We use ack with last to decide if we should 
    // de-assert so we can handle any pipelined look-ahead in
    // DMA (but not aggressive look ahead)
    reg fb_start;
    reg tb_start;
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn) begin
        fb_start <= 1'b0;
      end else if (dma_req_fb & ~|dma_ctrl[1:0])
        fb_start <= 1'b0;               // DMA canceled
      else if (fb_start & dma_req_fb_ack & (rx_is_last | rx_is_empty))
        fb_start <= 1'b0;               // done with a request
      else if (|dma_ctrl[1:0] & ~fb_start & ~dma_req_fb_ack & 
               (rx_trig | ~rx_is_empty))// should we include empty if trig used?
        fb_start <= 1'b1;               // start request
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn)
        tb_start <= 1'b0;
      else if (dma_req_tb & ~|dma_ctrl[3:2])
        tb_start <= 1'b0;               // DMA canceled 
      else if (tb_start & dma_req_tb_ack & (tx_is_last | tx_is_full))
        tb_start <= 1'b0;               // done with a request
      else if (|dma_ctrl[3:2] & ~tb_start & ~dma_req_tb_ack & 
               (tx_trig | ~tx_is_full)) // do we need is_full if trig used?
        tb_start <= 1'b1;               // start request

    // note combinatorial protections below as protections
    assign dma_req_fb = fb_start & ~rx_is_empty;
    assign dma_req_tb = tb_start & ~tx_is_full;
  end else if (DMA_TYPE==`DMA_ACK1BEF) begin
    // Covers case 4 but not 5 or 6 from Imp Notes
    // We set request as long as not empty/full, and each ack is
    // 1 cycle. We use ack with last to decide if we should 
    // de-assert so we can handle any pipelined look-ahead in
    // DMA (but not aggressive look ahead)
    reg fb_start;
    reg tb_start;
    reg fb_hold;
    reg tb_hold;
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn) begin
        fb_start <= 1'b0;
        fb_hold  <= 1'b0;
      end else if (dma_req_fb & ~|dma_ctrl[1:0]) begin
        fb_start <= 1'b0;               // DMA canceled
        fb_hold  <= 1'b0;
      end else if (~fb_start & fb_hold & 
                   (pread | (~rx_is_last & ~rx_is_empty))) begin // re-arm
        fb_start <= 1'b1;               // start request
        fb_hold  <= 1'b0;
      end else if (fb_start & dma_req_fb_ack & (rx_is_last | rx_is_empty)) begin
        fb_start <= 1'b0;               // done with a request
        fb_hold  <= 1'b1;               // hold until read or more than 1 waiting
      end else if (|dma_ctrl[1:0] & ~fb_start & ~dma_req_fb_ack & 
                   ~fb_hold & rx_trig) begin // arm again
        fb_start <= 1'b1;               // start request
        fb_hold  <= 1'b0;
      end
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn) begin
        tb_start <= 1'b0;
        tb_hold  <= 1'b0;
      end else if (dma_req_tb & ~|dma_ctrl[3:2]) begin
        tb_start <= 1'b0;               // DMA canceled 
        tb_hold  <= 1'b0;
      end else if (~tb_start & tb_hold & ~tx_is_last & 
                   ~tx_is_full) begin   // re-arm when room
        tb_start <= 1'b1;               // start request
        tb_hold  <= 1'b0;
      end else if (tb_start & dma_req_tb_ack & (tx_is_last | tx_is_full)) begin
        tb_start <= 1'b0;               // done with a request
        tb_hold  <= 1'b1;               // hold until room
      end else if (|dma_ctrl[3:2] & ~tb_start & ~dma_req_tb_ack & 
                   ~tb_hold & tx_trig) begin
        tb_start <= 1'b1;               // start request
        tb_hold  <= 1'b0;
      end

    // note combinatorial protections below as protections
    assign dma_req_fb = fb_start & ~rx_is_empty;
    assign dma_req_tb = tb_start & ~tx_is_full;
  end
  endgenerate

  // empty and last test needs to know if we move by 2 bytes at a time or 1
  // so dma_ctrl[5] checked (means by half words). Note that empty and full
  // therefore may have 1 byte left (waiting or avail)
  assign rx_is_empty = dma_ctrl[5] ? ~rx_fullness[1] : ~|rx_fullness;
  assign tx_is_full  = dma_ctrl[5] ? ~tx_avail[1]    : ~|tx_avail;
  assign rx_is_last  = dma_ctrl[5] ? (rx_fullness==2'd2) : (rx_fullness==2'd1);
  assign tx_is_last  = dma_ctrl[5] ? (tx_avail==2'd2)    : (tx_avail==2'd1);

endmodule


