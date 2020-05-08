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
//  File            : i3c_auton_wrapper.v
//  Organisation    : MCO
//  Tag             : 1.1.11
//  Date            : $Date: Wed Jun 12 23:47:02 2019 $
//  Revision        : $Revision: 1.61 $
//
//  IP Name         : i3c_auton_wrapper
//  Description     : MIPI I3C Slave support with Autonomous registers
//    This contains the autonomous wrapper for I3C where the messages
//    all go into/out-of generated registers, to support a state 
//    machine based design. The i3c_autonomous_regs block provides
//    a generated set of registers and nets based on parameters. This
//    allows for indexed registers (using a byte index into pools
//    of 1 or more registers (by offset increment). This wrapper just
//    helps seal up the combination. It also handles notifications and
//    IBI (interrupt) requests from the system.
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
//
//  Note: this just instantiates the full wrapper for autonomous
//
//  Note that very little CDC is used or needed. The system should
//  follow the correct behavior to avoid bit tearing of read-data 
//  (changing while being read will give a mismatch of bits since
//  from separate clock domains) in the same way as used to avoid
//  pool tearing (partly old and partly new data being read).
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_auton_wrapper #(
    // params are driven from upper layer to control how this block
    // is built and operates
    // 1st set is related to autonomous regs
      // rules are: [0]=W only at RUN[idx]=0, [1]=W ignored at RUN[idx]=0,
      //            [2]=touch only at RUN[idx], [3]=touch on W not R
      //            [4]=reset idx on S (not Sr), [5]=reserved for Magic regs (bus)
      //            [7:6]=DDR rule: 0:CMD=index, 1:CMD=index_except_7F, 2:CMD_ignored
      //            [8]=NACK if read from invalid reg, [15:9]=reserved
    parameter REG_RULES       = 16'd0,  // rules as explained above
    parameter MAX_REG         = 8'h00,  // set this 1 to 255 as last index
    parameter REG_WRITABLE    = {MAX_REG+1{1'b0}}, // which are W
    parameter REG_READABLE    = {MAX_REG+1{1'b0}}, // which are R
    parameter REG_RUN         = {MAX_REG+1{1'b0}}, // discontig bounds
    parameter REG_MASK        = {((MAX_REG+1)*8){1'b1}}, // full or partial
    parameter REG_BLEND       = {MAX_REG+1{1'b0}}, // blending W/R
    parameter REG_REMAP       = {((MAX_REG+1)*8){1'b1}}, // optional remapping
    parameter REG_RESET       = {((MAX_REG+1)*8){1'b0}}, // reset value of W flop
    // now standard ones, even if do not directly apply
    parameter ENA_ID48B       = `ID48B_CONST, // const vs. reg
    parameter  ID_48B         = 48'h0,  // must be filled in from above with some/all of 48-bit ID
    parameter ID_AS_REGS      = 12'd0,  // [0]=IDINST, [1]=ISRAND, [2]=IDDCR, [3]=IDBCR
    // note that BCR is below so can use other params
    parameter  ID_DCR         = 8'd0,   // filled in from above with DCR if not from regs
    parameter ENA_SADDR       = `SADDR_NONE, // none, const, reg/net
    parameter  SADDR_P        = 0,      // 7 bits as 6:0
    parameter ENA_IBI_MR_HJ   = 0,      // 0 if no events, else events as mask
    parameter  CLK_SLOW_BITS  = 6,      // number of bits needed for count for Bus Avail
    parameter  CLK_SLOW_MATCH = 6'd47,  // count: e.g. 47 for 1us if 48MHz CLK (0 rel)
    parameter  CLK_SLOW_HJMUL = 10'd1000,// number of MATCHes (const or reg) for 1ms (1 rel)
      // next is error handling. Read Abort is default if IBI is enabled
    parameter  ERROR_HANDLING = 3'd0|((|ENA_IBI_MR_HJ)<<`ERR_RDABT_b), 
    parameter  ENA_TIMEC      = 6'b000010,    // if reg, res, res, mode 1, mode 0, sync
    parameter  TIMEC_FREQ_ACC = {8'd24,8'd10},// freq=12MHz (12.0=24) with 1.0% accuracy
    parameter ENA_CCC_HANDLING= 6'd0,   // passed down as to what to support
    parameter MAX_RDLEN       = 0,      // default S->M len max
    parameter MAX_WRLEN       = 0,      // default M->S len max
    parameter MAX_DS_WR       = 0,      // data speed limits for M->S
    parameter MAX_DS_RD       = 0,      // data speed limits for S->M
    parameter MAX_DS_RDTURN   = 0,      // latency needs for S->M read req
    parameter ENA_HDR         = 0,      // enables for HDR modes
    parameter PIN_MODEL       = `PINM_COMBO, // combinatorial pin use
      // BCR auto filled in from params. But, cannot do offline or bridge
    parameter  ID_BCR         = (|ENA_HDR<<5) | ((ENA_IBI_MR_HJ&8'h03)<<1) |
                                |(MAX_DS_RDTURN|MAX_DS_WR|MAX_DS_RD), // limits at [0]
    // next is never passed - computed here
    parameter priv_sz         = PIN_MODEL[1] ? (ENA_HDR[0] ? 3 : 1) : 0// wider if ext-pad+ddr
  )
  (
  // define clock and reset
  input               RSTn,             // reset from system
  input               CLK,              // system clock for regs and sync
  input               CLK_SLOW,         // clock to use for IBI forced
  output              slow_gate,        // 1 if may gate CLK_SLOW
  input               CLK_SLOW_TC,      // time-control clockl may be same as CLK_SLOW
  output              tc_slow_gate,     // 1 if CLK_SLOW_TC may be gated; AND with slow_gate if common
  // define pins. SCL is input only
  input               pin_SCL_in,       // SCL: normally clock in, data if ternary
  input               pin_SDA_in,       // SDA: M->S
    // Note: next 3 are used special when PIN_MODEL==`PINM_EXT_REG
    //       They feed 3 flops placed close to the pads
  output              pin_SDA_out,      // SDA: S->M on read
  output              pin_SDA_oena,     // SDA: S->M on read
  output  [priv_sz:0] pin_SDA_oena_rise,// special on SCL rising if EXT_REG, else 0
    // next one is only used if SDA & SCL pads have i2c 50ns Spike 
    // filters and they can be turned on/off via net.
  output              i2c_spike_ok,     // Is 1 to allow i2c spike filter
    // Autonomous use: the o_ nets of notification are synchronized to
    // the CLK domain from SCL. Note that start and done are messages
    // to/from this slave (but not CCC) and are 1 for at least one CLK.
    // The wo_regs and raw from SCL (so should be sampled only when safe using
    // notifications), and ro_regs are assumed to be raw from CLK (and so
    // should only change when safe using notifications).
    // The touch net is raw per bit. The system should handle local
    // sync using the provided sync module. This belongs in the system
    // so the path from Q to use is kept short to prevent metastable issues. 
    // Note that touch may be per register or per run (controlled by
    // REG_RULES).
    // The OK (completion vs. purely ACK) per bit is pulsed from the CLK
    // domain. The pulse must last as long as a flop reset is required to
    // (which is async and usually very short, so not normally an issue).
  output              osync_started,    // pulse 1 CLK when new trans to us started
  output              o_read,           // 1 if started a read from us, else write
  output              osync_done,       // pulse 1 CLK for end of read or write 
  output              osync_any_touch,  // pulse 1 if any reg read or written, when osync_done=1
  output  [MAX_REG:0] oraw_reg_touch,   // bits 1 if reg or run has been read/written 
  input   [MAX_REG:0] iraw_touch_OK,    // sys pulse accept touch[i] from CLK domain: as reset 
  output [(8*MAX_REG)+7:0] wo_regs,     // regs written by Master - raw
  input  [(8*MAX_REG)+7:0] ro_regs,     // sys provided regs read by Master - raw
    // Now related to Autonomous is interrupt request. The request is from CLK
    // domain and is synchronized locally. The output done is also synchronized
    // to the CLK domain. If o_ibi_done pulses, the IBI went through and was
    // accepted (including byte if used). If o_ibi_nacked pulses, the system
    // has to decide if it wants to try again.
  input               i_ibi_event,      // Pulse 1 CLK to signal event if 
  input               i_ibi_req,        // Hold 1 request an IBI, clear with done (ack) or cancel
  input               i_hj_req,         // Hold 1 to req HJ (if allowed); clear with ibi_done
  output              o_ibi_done,       // Pulse 1 CLK when IBI done with ACK
  output              o_ibi_nacked,     // Pulse 1 CLK when IBI done with NACK
  input         [7:0] i_ibi_byte,       // IBI byte if needed. Hold until done
  // now the configuration (stable) inputs. These should be tied off as 0 or 1
  // when not used (selected by param).
  input               cf_SlvEna,        // only process bus if 1
  input         [7:0] cf_SlvSA,         // optional slave i2c address ([0]=1 if enabled)
  input         [3:0] cf_IdInst,        // Instance of ID when selected (not if partno used)
  input               cf_IdRand,        // random partno of ID if used
  input        [31:0] cf_Partno,        // partno od ID if used
  input         [7:0] cf_IdBcr,         // BCR of DAA if not const
  input         [7:0] cf_IdDcr,         // DCR of DAA if not const
  input        [14:0] cf_IdVid,         // Vendor ID if not const
  // Now special states from SCL domain raw - these can be synchronized
  // on the fly for STATUS reads.
  output      [29:28] raw_ActState,     // activity state on bus from ENTASn
  output      [27:24] raw_EvState,      // event enabled states from ENEC/DISEC
  output        [6:0] raw_Request,      // bus request in process w/details
  output        [7:0] raw_DynAddr,      // dynamic address if set - is stable
  output       [13:0] raw_timec_sync,   // SYNC tc handled by system. [13]=clk
  // these next ones are to be safe-raw. That means either changed when
  // bus is STOPped or otherwise not doing a GETSTATUS or synchronized.
  // They can be strapped as 0 if not needed.
  input         [7:6] sraw_ActMode,     // System activity mode (or 0 if not used)
  input         [3:0] sraw_PendInt,     // Pending interrupt (or 0 if not used)
  input        [15:8] sraw_StatusRes,   // reserved bits of status (or 0 if not used) 
  // next are detected errors (e.g. pass run end)
  // they can be ignored. Simply here if needed
  // these are raw in SCL domain, so would need to
  // be sticky or handshaked if used
  output              raw_tb_reg_err,
  output              raw_fb_reg_err,
  // now scan related
  input               scan_single_clock,// single clock domain from scan_clk for pin-based clocks
  input               scan_clk,         // clock to use for pin clocks if in scan
  input               scan_no_rst,      // prevents layered reset
  input               scan_no_gates     // prevents arch clock gating
  );

  // we just instantiate full wrap and tie off unused nets from magic reg
  i3c_auton_wrap_full #(.ENA_ID48B(ENA_ID48B),.ID_48B(ID_48B),
                        .ID_AS_REGS(ID_AS_REGS),.ID_BCR(ID_BCR),.ID_DCR(ID_DCR),
                        .ENA_SADDR(ENA_SADDR),.SADDR_P(SADDR_P),
                        .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ),
                          .CLK_SLOW_BITS(CLK_SLOW_BITS),
                          .CLK_SLOW_MATCH(CLK_SLOW_MATCH),
                          .CLK_SLOW_HJMUL(CLK_SLOW_HJMUL),
                        .ERROR_HANDLING(ERROR_HANDLING),
                        .ENA_CCC_HANDLING(ENA_CCC_HANDLING), 
                         .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
                        .MAX_RDLEN(MAX_RDLEN),.MAX_WRLEN(MAX_WRLEN),
                         .MAX_DS_WR(MAX_DS_WR),.MAX_DS_RD(MAX_DS_RD),
                         .MAX_DS_RDTURN(MAX_DS_RDTURN),
                        .ENA_HDR(ENA_HDR),
                        .PIN_MODEL(PIN_MODEL),
                        .REG_RULES(REG_RULES), .MAX_REG(MAX_REG), 
                         .REG_WRITABLE(REG_WRITABLE), .REG_READABLE(REG_READABLE),
                         .REG_RUN(REG_RUN), .REG_MASK(REG_MASK), .REG_BLEND(REG_BLEND),
                         .REG_REMAP(REG_REMAP),.REG_RESET(REG_RESET),
                        .MAGIC_RULES(0),.MAGIC_MASK(0),.MAGIC_MATCH(0))
    auton_full 
    (
    .RSTn             (RSTn), 
    .CLK              (CLK), 
    .CLK_SLOW         (CLK_SLOW), 
    .slow_gate        (slow_gate), 
    .CLK_SLOW_TC      (CLK_SLOW), // same as clock slow
    .tc_slow_gate     (tc_slow_gate), // if use, would AND with slow_gate since common
    .pin_SCL_in       (pin_SCL_in), 
    .pin_SDA_in       (pin_SDA_in), 
    .pin_SDA_out      (pin_SDA_out), 
    .pin_SDA_oena     (pin_SDA_oena), 
    .pin_SDA_oena_rise(pin_SDA_oena_rise), 
    .i2c_spike_ok     (i2c_spike_ok), 
    .osync_started    (osync_started), 
    .o_read           (o_read), 
    .osync_done       (osync_done), 
    .osync_any_touch  (osync_any_touch), 
    .oraw_reg_touch   (oraw_reg_touch), 
    .iraw_touch_OK    (iraw_touch_OK), 
    .wo_regs          (wo_regs), 
    .ro_regs          (ro_regs), 
    .i_ibi_event      (i_ibi_req), // same for event and req
    .i_ibi_req        (i_ibi_req), 
    .i_hj_req         (i_hj_req), 
    .o_ibi_done       (o_ibi_done), 
    .o_ibi_nacked     (o_ibi_nacked), 
    .i_ibi_byte       (i_ibi_byte), 
    .cf_SlvEna        (cf_SlvEna), 
    .cf_SlvSA         (cf_SlvSA), 
    .cf_IdInst        (cf_IdInst), 
    .cf_IdRand        (cf_IdRand), 
    .cf_Partno        (cf_Partno), 
    .cf_IdVid         (cf_IdVid),
    .cf_SlvNack       (1'b0),
    .cf_BAMatch       (8'd0),
    .cf_IdBcr         (cf_IdBcr), 
    .cf_IdDcr         (cf_IdDcr), 
    .raw_ActState     (raw_ActState), 
    .raw_EvState      (raw_EvState), 
    .raw_Request      (raw_Request), 
    .raw_DynAddr      (raw_DynAddr), 
    .raw_timec_sync   (raw_timec_sync),
    .raw_slvr_reset   (),
    .iraw_rst_slvr_reset(1'b0),
    .sraw_ActMode     (sraw_ActMode), 
    .sraw_PendInt     (sraw_PendInt), 
    .sraw_StatusRes   (sraw_StatusRes), 
    .raw_fb_reg_err   (raw_fb_reg_err),
    .raw_tb_reg_err   (raw_tb_reg_err),
    .mr_clk_SCL       (mr_clk_SCL),
    .mr_clk_SCL_n     (mr_clk_SCL_n),
    .mr_idx           (),
    .mr_next_idx      (),
    .mr_dir           (),
    .mr_trans_w_done  (),
    .mr_trans_r_done  (),
    .mr_trans_ccc_done(),
    .mr_trans_ddrcmd  (),
    .mr_wdata         (),
    .mr_rdata         (8'd0),
    .mr_rdata_none    (1'b0),
    .mr_rdata_end     (1'b0),
    .mr_bus_new       (),
    .mr_bus_done      (),
    .scan_single_clock(scan_single_clock), 
    .scan_clk         (scan_clk), 
    .scan_no_rst      (scan_no_rst), 
    .scan_no_gates    (scan_no_gates)
  );


endmodule
