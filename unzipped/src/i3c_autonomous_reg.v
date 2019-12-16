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
//  File            : i3c_autonomous_reg.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Thu Sep 12 16:42:33 2019 $
//  Revision        : $Revision: 1.62 $
//
//  IP Name         : i3c_autonomous_reg - generated registers
//  Description     : MIPI I3C message handling into/from registers
//    This contains auto-generated registers to support an autonomous
//    slave model which wanst to have the normal i2c/i3c "register" 
//    interface. That is, an i2c/i3c 1st data written is the index into
//    this bank of registers and then the Master can write 1 or more
//    register or flip to an i2c/i3c Read to read 1 or more registers.
//    Parameters completely control ths construction of these regs and
//    allow for continguous and discontiguous breaks to support all normal
//    models for this.
//    The FSM in the system is notified of these changes so it can 
//    act on them after the fact.
//
//  ----------------------------------------------------------------------------
//                    Revision History
//  ----------------------------------------------------------------------------
//
//
//  ----------------------------------------------------------------------------
//                    Implementation details
//  ----------------------------------------------------------------------------
//  See Autonmous spec and MIPI I3C spec
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_autonomous_reg #(
      // rules are: [0]=W only at RUN[idx]=0, [1]=W ignored at RUN[idx]=0,
      //            [2]=touch only at RUN[idx], [3]=touch on W not R
      //            [4]=reset idx on S (not Sr), [5]=Magic regs (bus)
      //            [7:6]=DDR rule: 0:CMD=index, 1:CMD=index_except_7F, 2:CMD_ignored
      //                  (see REG_DDRCMD_WRIDX if [7:6]=2. Sets Index only CMD val)
      //            [8]=NACK if read from invalid reg, [15:9]=reserved
    parameter         REG_RULES    = 16'd0,   // rules as explained above
    parameter         MAX_REG      = 8'h00,   // set this 1 to 255 as last index
      // next are bits to set
    parameter         REG_WRITABLE = {MAX_REG+1{1'b0}}, // which are W
    parameter         REG_READABLE = {MAX_REG+1{1'b0}}, // which are R
    parameter         REG_RUN      = {MAX_REG+1{1'b0}}, // discontig bounds
    parameter         REG_MASK     = {((MAX_REG+1)*8){1'b1}}, // full or partial
    parameter         REG_BLEND    = {MAX_REG+1{1'b0}}, // blending W/R
    parameter         REG_REMAP    = {((MAX_REG+1)*8){1'b1}}, // optional remapping
    parameter         REG_RESET    = {((MAX_REG+1)*8){1'b0}}, // reset value of W flop
    parameter         REG_DDRCMD_WRIDX=8'h00,// only used if REG_RULES[7:6]=2
    parameter         MAGIC_RULES  = 4'b0000,// bits for rules. [0]=RW vs. WO, [1]=CCCs, [2]=no idx
    parameter         MAGIC_MASK   = 8'h00,  // mask for match
    parameter         MAGIC_MATCH  = 8'h00,  // match via mask (index&mask)==match
      // next is always computed - not passed in. If CCC, we need a byte always
    parameter         IDXMX        = MAGIC_RULES[1]?7 :
                                     (MAX_REG<4)?1:(MAX_REG<8)?2:(MAX_REG<16)?3:
                                     (MAX_REG<32)?4:(MAX_REG<64)?5:(MAX_REG<128)?6:7
  )
  (
  // we only operate in the SCL clock domain
  input               clk_SCL_n,        // SCL falling edge: start and launching data
  input               clk_SCL,          // SCL rising edge: sample data and read T bit
  input               CLK,              // system clock for sync
  input               RSTn,             // master Reset
  // now the data to-bus and from-bus as bytes
  input               bus_new1,         // new transaction to us
  input               bus_read,         // is a read - with bus_new
  input               bus_ddr,          // new1 is DDR vs. SDR/i2c
  input               bus_in_ddr,       // mode is HDR-DDR
  input               bus_done1,        // transaction ended (STOP or repeated START)
  input               bus_in_STOP,      // held-state (no SCL clock) for bus_done case
  input               fb_new1,          // from-bus data arrived
  input         [7:0] fb_data,          // data if fb_new1
  input               fb_is_ccc,        // 1 if fb_new1 is CCC related (only if enabled)
  output              fb_err,           // on invalid write attempt
  input               tb_need1,         // need next byte for read
  output        [7:0] tb_data,          // data we return
  output              tb_end,           // last byte for read (write just ignores)
  output              tb_err,           // underun on read error
  output              tx_valid,         // used to NACK if read not OK
  // now the vectors that are ported to the system CLK domain (raw and sync)
  output              osync_started,    // pulse 1 when new trans started for us
  output              o_read,           // 1 if started trans is read (held) from us
  output              osync_done,       // pulse 1 if end of read or write (for us)
  output              osync_any_touch,  // pulse 1 if any reg read or written, when osync_done=1
  output  [MAX_REG:0] oraw_reg_touch,   // bits 1 if reg has been read/written (clr on OK)
  input   [MAX_REG:0] iraw_touch_OK,    // sys accept touch from CLK domain: RULE for pulse/held
  output [(8*MAX_REG)+7:0] wo_regs,     // regs written by Master - raw
  input  [(8*MAX_REG)+7:0] ro_regs,     // sys provided regs read by Master - raw
  // now Magic regs, which means mapped into system using SCL based bus. If not using
  // this, these are not used.
  // note that params must match between magic and normal (no regs where magic
  // ones if they do both, only readable  marked if magic is write-only.
  output        [7:0] mr_idx,           // current register index (auto inc)
  output        [7:0] mr_next_idx,      // next index after trans_done
  output              mr_dir,           // this is 0 if write, 1 if read (note timing)
  output              mr_trans_w_done,  // pulses for a cycle; at end of wdata (not idx)
  output              mr_trans_r_done,  // pulses for a cycle; at end of rdata
  output              mr_trans_ddrcmd,  // CMD from DDR
  output              mr_trans_ccc_done,// if enabled: pulses for a cycle same as _w_done
  output        [7:0] mr_wdata,         // data with mr_dir=0 and trans_w_done=1
  input         [7:0] mr_rdata,         // live data from system - change on trans_r_done
  input               mr_rdata_none,    // is 1 if no read data - will NACK if 1st
  input               mr_rdata_end,     // 1 on read if last byte
  input               scan_no_rst       // prevents layered reset
  );
  // we load the index on the write as 1st byte in
  reg       [IDXMX:0] fbus_index;
  wire                index_magic;      // 1 if magic match
  wire                ccc_magic;        // unhandled CCC over mr bus
  wire                bnew_magic;       // bus new carry over for DDR
  wire      [IDXMX:0] new_index;
  wire      [IDXMX:0] map_index, remap_index;
  reg                 fbus_wr;
  wire                use_ddr;          // special CMD suppress
  wire                index_only;
  wire      [IDXMX:0] next_index;
  reg                 sys_read;
  reg                 wr_none;
  wire          [7:0] tobus_data[0:MAX_REG]; // RO regs (sys owns) + W/R
  wire    [MAX_REG+1:0] is_valid;
  wire    [MAX_REG+1:0] is_end;
  wire    [MAX_REG:0] is_writable;
  wire    [MAX_REG:0] is_readable;
  wire                s_not_sr; // only used if reset index on S
  localparam ALL_MAGIC = REG_RULES[5] & ~|MAGIC_MASK & ~|MAGIC_MATCH; // all magic regs
  
  //
  // Build up regs that are handled in normal way, as well as empty slots
  //

  // we 1st build up the data pool
  // note that they are obligated to not have the MAGIC regs overlap
  // other regs except where it makes sense (e.g. readable back)
  genvar i;
    // `define below since one lint does not like localparam in generate
  `define UPPER (((i+1)*8)-1)
  generate 
    // we create regs as flops, but ones never used will be pruned out
    // note that we have 4 types of index-regs:
    // - RO is readable only and controlled by system
    // - WR is writable by master and so can read-back
    // - Blended, which has some bits WR and some RO using mask
    // - 2 stage, which has both RO and WO. So, M writes WO, reads RO
    if (ALL_MAGIC) begin : magic
      // strap all internal to 0
      assign wo_regs     = 0; 
      assign is_writable = 0;
      assign is_readable = 0;
      assign is_valid    = 0;
      assign is_end      = 0;
      for (i = 0; i <= MAX_REG; i = i + 1) begin : init_unused
        assign tobus_data[i] = 0;
      end
      // strap external to 0
      assign oraw_reg_touch  = 0;
      assign osync_any_touch = 0;
    end else begin
      for (i = 0; i <= MAX_REG; i = i + 1) begin : auto_regs
        if (REG_WRITABLE[i]) begin
          // writeable, so we create flops
          reg [7:0] wreg;                 // note if masked, flops will be removed
          always @ (posedge clk_SCL or negedge RSTn)
            if (~RSTn) 
              wreg <= REG_RESET[`UPPER -: 8];
            else if (fb_new1 & ~wr_none & fbus_wr & (fbus_index==i))
              wreg <= fb_data & REG_MASK[`UPPER -: 8]; // only where masked
          assign wo_regs[`UPPER -: 8] = wreg & REG_MASK[`UPPER -: 8];
          if (REG_READABLE[i])
            // WO and RO at same time - two regs or blended
            assign tobus_data[i] = ro_regs[`UPPER -: 8] &
                                   (REG_BLEND[i] ? ~REG_MASK[`UPPER -: 8] :
                                                   REG_MASK[`UPPER -: 8]);
          else
            // just Writable and so reads as last written
            assign tobus_data[i] = wreg & REG_MASK[`UPPER -: 8]; 
        end else if (REG_READABLE[i]) begin
          // read only if here - provided by system
          assign wo_regs[`UPPER -: 8] = 8'd0; // nothing Master can write
          assign tobus_data[i]       = ro_regs[`UPPER -: 8] & REG_MASK[`UPPER -: 8]; 
        end else begin
          // no register - not readable or writable
          assign wo_regs[`UPPER -: 8] = 8'd0; 
          assign tobus_data[i]       = 8'd0;
        end
        // now related to runs and end
        assign is_writable[i] = REG_WRITABLE[i] ? 1'b1 : 1'b0;
        assign is_readable[i] = REG_READABLE[i] ? 1'b1 : 1'b0;
        assign is_valid[i]    = is_writable[i] | is_readable[i];
        assign is_end[i]      = REG_RUN[i] ? 1'b0 : 1'b1;
        // we mark all that are written and read if they want
        if (~REG_RULES[2] | ~REG_RUN[i]) begin : touch_bit
          // reg rule[2] says only touch on RUN=0 points
          wire local_set = (fb_new1 & ~wr_none & fbus_wr & REG_WRITABLE[i]) |
                           (tb_need1 & REG_READABLE[i] & ~REG_RULES[3]);
                            // read touch if not disallowed
          reg touchbit;
          // we reset the touch bit async since from a different clock domain. No
          // risk on release since would not be happening when D input changing (by rule)
          wire async_rstn = RSTn & (~iraw_touch_OK[i] | scan_no_rst);

          always @ (posedge clk_SCL or negedge async_rstn)
            if (!async_rstn)
              touchbit <= 1'b0;
            else if (local_set & (fbus_index==i))
              touchbit <= 1'b1;   // clear by async reset from touch_OK
          assign oraw_reg_touch[i] = touchbit;
          /// Not needed: SYNC_AClr_S2C sync_touch(clk_SCL, async_rstn, local_set, 1'b0, oraw_reg_touch[i]);
        end else 
          assign oraw_reg_touch[i] = 1'b0;
      end // loop
      assign is_end[MAX_REG+1]   = 1'b1;    // since uses next_ check
      assign is_valid[MAX_REG+1] = 1'b0;  // since uses next_ check
      assign osync_any_touch     = |oraw_reg_touch & osync_done;// any are 1 right now
    end // not magic
  endgenerate

  // 
  // Index and optional CCC handling (magic)
  //

  // now we handle the index on write (1st value written) and
  // the read and write requests which advance index (the actual 
  // write is above)
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn) begin
      fbus_index        <= 0;
      fbus_wr           <= 1'b0;
    end else if (bus_new1) begin
      if (fb_new1 & use_ddr) begin
        // special case for DDR where new match and data in same cycle
        fbus_wr         <= 1'b1;
        fbus_index      <= remap_index[IDXMX:0]; // 1st is index on write
      end else
        fbus_wr         <= 1'b0;
    end else if (fb_new1) begin
      if (~fbus_wr) begin         // 1st write is always index
        fbus_wr         <= 1'b1;
        fbus_index      <= fb_is_ccc ? fb_data : remap_index[IDXMX:0]; // 1st is index on write
      end else if (~is_end[next_index] & is_writable[next_index] & ~fb_is_ccc & ~index_only)
        fbus_index      <= next_index;
      else if (index_magic & ~fb_is_ccc & ~index_only)
        fbus_index      <= next_index;
    end else if (tb_need1) begin
      // read will go from whatever index was last at
      if (~is_end[next_index] & is_valid[next_index] & ~fb_is_ccc)
        fbus_index      <= next_index;
      else if (index_magic & ~fb_is_ccc & MAGIC_RULES[0])
        fbus_index      <= next_index;
    end else if (bus_done1) begin
      if (REG_RULES[4] & s_not_sr)// is START and not repeated START
        fbus_index      <= 9'd0; // reset index on START for read that may follow
      fbus_wr           <= 1'b0; // start clears the write flag
    end
    
  // note: if no DDR support, the below logic will fall away
  wire [7:0] ddr_index  = {1'b0,fb_data[6:0]}; // note no reserved CMDs anymore
  assign use_ddr        = ~|REG_RULES[7:6] | ((REG_RULES[7:6]==2'b01) & ~&fb_data[6:0]);
  wire [IDXMX:0] fb_idx = (bus_ddr&use_ddr) ? ddr_index[IDXMX:0] : fb_data[IDXMX:0]; 
  assign new_index      = (fb_idx>MAX_REG) ? 0 : fb_idx;
  wire [7:0] rm_idx     = REG_REMAP[({new_index[IDXMX:0],3'd7}) -: 8];
  assign map_index      = rm_idx[IDXMX:0];// 1st is index on write
  assign remap_index    = &map_index ? new_index : map_index;
  assign tb_end         = (index_magic|ccc_magic) ? (mr_rdata_end|mr_rdata_none) : // folds away if no magic reg
                          is_end[next_index];// honored for read
  assign tb_err         = tb_need1 & 
                          ((index_magic|ccc_magic) ? mr_rdata_none : ~is_valid[fbus_index]);
  assign next_index     = fbus_index+{{IDXMX{1'b0}},1'b1};
  // tx_valid is up to "none" if MR, based on validity of reg if REG_RULES[8]=1
  assign tx_valid       = (index_magic|ccc_magic) ? ~mr_rdata_none : 
                          (~REG_RULES[8] | is_valid[fbus_index]); 
  // suppression should help with reliable results on to-bus 
  assign tb_data        = (index_magic|ccc_magic) ? mr_rdata : // folds away if no magic-reg
                          is_valid[fbus_index] ? tobus_data[fbus_index] : 8'hFF;
  wire   index_over;
  wire   [7:0] full_idx = bus_ddr ? ddr_index : fb_data; 
  generate if (IDXMX < 7) begin : index_check
    assign index_over   = |full_idx[7:(IDXMX+1)];
  end else begin
    assign index_over   = 1'b0;
  end endgenerate
  // wr_none remembers if not in allowed write range
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn)
      wr_none   <= 1'b0;
    else if (bus_new1 & ~fb_new1) 
      wr_none   <= 1'b0;
    else if (fb_new1 & ~fb_is_ccc) begin
      if (bus_new1 & ~use_ddr) 
        wr_none <= 1'b0;
      else if (REG_RULES[0] & ~fbus_wr & ~is_end[fb_data])
        wr_none <= 1'b1;                // invalid start (only on run start)
      else if (~fbus_wr & ((full_idx[IDXMX:0]>MAX_REG) | index_over))
        wr_none <= 1'b1;                // invalid index - beyond allowed
      else if (~REG_RULES[1] & fbus_wr & is_end[next_index] & ~index_magic)
        wr_none <= 1'b1;                // end of run
      else if ((REG_RULES[7:6]==2'b10) & index_only)
        wr_none <= 1'b1;                // only index on this write
    end
  assign fb_err     = fb_new1 & wr_none;// invalid write

  generate if (REG_RULES[7:6]==2'b10) begin : cmd_not_idx
    reg  idx_only_r;
    always @ (posedge clk_SCL or negedge RSTn)
      if (!RSTn)
        idx_only_r <= 1'b0;
      else if (bus_new1 & fb_new1)
        idx_only_r <= remap_index[IDXMX:0] == REG_DDRCMD_WRIDX;
      else if (bus_new1)
        idx_only_r <= 1'b0;
    assign index_only = idx_only_r;
  end else begin
    assign index_only = 1'b0;
  end endgenerate

  //
  // Magic regs - read side handled above
  //
  assign mr_idx           = fbus_index;
  assign mr_next_idx      = fb_is_ccc ? fbus_index : next_index;
  assign mr_dir           = bus_read; 
  assign mr_trans_w_done  = index_magic & fb_new1 & ~fb_is_ccc & ~(bnew_magic|bus_new1);
  assign mr_trans_r_done  = index_magic & tb_need1 & ~fb_is_ccc;
  assign mr_trans_ddrcmd  = index_magic & fb_new1 & ~fb_is_ccc & (bus_new1|bnew_magic);
  assign mr_trans_ccc_done= (ccc_magic|MAGIC_RULES[2]) & (fb_new1 | tb_need1) & fb_is_ccc;
  assign mr_wdata         = fb_data;
  generate if (REG_RULES[5]) begin : magic_matching
    if (~MAGIC_RULES[2]) begin : mr_no_idx
      reg is_magic1, is_magic2;
      always @ (posedge clk_SCL or negedge RSTn)
        if (!RSTn) 
          is_magic1 <= 1'b0;
        else if (fb_is_ccc & is_magic2)
          is_magic1 <= 1'b0;
        else if (bus_new1) begin
          // DDR uses both at once, else if not read, clear for next wr
          if (fb_new1 & use_ddr)
            is_magic1 <= 1'b1;
          else if (~bus_read)
            is_magic1 <= 1'b0;
        end else if (fb_new1) begin
          if (~fbus_wr) begin             // normal write index
            // note that remap is not considered - the index itself is looked at
            if (~bus_read | MAGIC_RULES[0])
              is_magic1 <= (full_idx & MAGIC_MASK) == MAGIC_MATCH;
          end
        end else if (bus_done1)
          if (REG_RULES[4] & s_not_sr & // is START and not repeated START
                 ((MAGIC_MASK&8'd0) != MAGIC_MATCH))   // will reset to 0 index
            is_magic1 <= 1'b0;

      // we want change on falling edge of SCL
      always @ (posedge clk_SCL or negedge RSTn)
        if (!RSTn) 
          is_magic2 <= 1'b0;
        else if (fb_is_ccc & is_magic2)
          is_magic2 <= 1'b0;              // CCC not using this
        else if (~fb_is_ccc & (is_magic1 ^ is_magic2))
          is_magic2 <= is_magic1;         // normal delay
        else if (~fb_is_ccc & fb_new1 & ~fbus_wr & bus_in_ddr)
          is_magic2 <= 1'b1;              // DDR using byte as index

      assign index_magic = is_magic2;
    end else begin
      assign index_magic = 1'b1;          // no index, so emit all writes
    end

    // CCC is special case to allow for direct as well as bcast
    // we only register so we skip the CCC code byte
    reg is_ccc2; // latter will fold if not used
    always @ (posedge clk_SCL or negedge RSTn)
      if (!RSTn) 
        is_ccc2   <= 1'b0;
      else if (MAGIC_RULES[1] & fb_is_ccc & fbus_wr)
        is_ccc2   <= 1'b1;
      else
        is_ccc2   <= 1'b0;
    
    // Bus new is registered so we can pulse one half cycle later
    reg is_bnew;
    always @ (posedge clk_SCL_n or negedge RSTn)
      if (!RSTn) 
        is_bnew <= 1'b0;
      else if (bus_new1)
        is_bnew <= 1'b1;
      else
        is_bnew <= 1'b0;
    assign ccc_magic   = is_ccc2;
    assign bnew_magic  = is_bnew;
  end else begin
    assign index_magic = 1'b0;          // no magic reg 
    assign ccc_magic   = 1'b0;
    assign bnew_magic  = 1'b0;
  end
  endgenerate 

  //
  // Now system notify - CDC normally
  //
  // we have to indicate start - 4-phase handshake
  SYNC_ASelfClr_S2C_Seq2 sync_start(.SCL(clk_SCL), .CLK(CLK), .RSTn(RSTn), 
                                    .local_set(bus_new1), .o_pulse(osync_started));
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn) 
      sys_read  <= 1'b0;
    else if (bus_new1) 
      sys_read  <= bus_read;
  assign o_read    = sys_read;

  reg in_trans;
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn) 
      in_trans  <= 1'b0;
    else if (bus_new1)
      in_trans  <= 1'b1;
    else if (bus_done1)
      in_trans  <= 1'b0;
  // we have to indicate done - 4-phase handshake
  wire sys_done;
  SYNC_ASelfClr_LVL_S2C_Seq2 sync_done(.SCL(clk_SCL), .CLK(CLK), .RSTn(RSTn), 
                                       .local_set(bus_done1), .local_hold(bus_in_STOP), 
                                       .o_pulse(sys_done));
  assign osync_done = sys_done & in_trans;

  generate if (REG_RULES[4]) begin : detect_s
    // STOP resets, so we ignore any other start
    wire    rst_s_n = RSTn & ~bus_in_STOP;
    reg     is_s;
    always @ (posedge clk_SCL or negedge rst_s_n)
      if (~rst_s_n)
        is_s <= 1'b1;                   // is 1st by default
      else if (bus_done1)
        is_s <= 1'b0;
    assign s_not_sr = is_s;
  end else
    assign s_not_sr = 1'b0;
  endgenerate

endmodule
