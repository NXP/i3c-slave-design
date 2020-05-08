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
//  File            : i3c_auton_wrap_full.v
//  Organisation    : MCO
//  Tag             : 1.1.11
//  Date            : $Date: Thu Oct 24 08:54:29 2019 $
//  Revision        : $Revision: 1.45 $
//
//  IP Name         : i3c_auton_wrap_full
//  Description     : MIPI I3C Slave support with Autonomous registers
//    This contains the more extensive autonomous reg support, including
//    the magic register(s) which map to a bus in the system (using SCL
//    as clock).
//    This contains the autonomous wrapper for I3C where the messages
//    all go into/out-of generated registers, to support a state 
//    machine based design. The i3c_autonomous_regs block provides
//    a generated set of registers and nets based on parameters, as well
//    as exporting the bus for the magic registers (if any). This
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
//  Note that very little CDC is used or needed. The system should
//  follow the correct behavior to avoid bit tearing of read-data 
//  (changing while being read will give a mismatch of bits since
//  from separate clock domains) in the same way as used to avoid
//  pool tearing (partly old and partly new data being read).
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_auton_wrap_full #(
    // params are driven from upper layer to control how this block
    // is built and operates
    // 1st set is related to autonomous regs
      // rules are: [0]=W only at RUN[idx]=0, [1]=W ignored at RUN[idx]=0,
      //            [2]=touch only at RUN[idx], [3]=touch on W not R
      //            [4]=reset idx on S (not Sr), [5]=Magic regs (bus)
      //            [7:6]=DDR rule: 0:CMD=index, 1:CMD=index_except_7F, 2:CMD_ignored
      //                  (see REG_DDRCMD_WRIDX if [7:6]=2. Sets Index only CMD val)
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
    parameter REG_DDRCMD_WRIDX= 7'h00,  // only used if REG_RULES[7:6]=2
    parameter MAGIC_RULES     = 4'b0000,// bits for rules. [0]=RW vs. WO, [1]=CCCs, [2]=no idx
    parameter MAGIC_MASK      = 8'h00,  // mask for match
    parameter MAGIC_MATCH     = 8'h00,  // match via mask (index&mask)==match
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
    parameter RSTACT_CONFIG   = 26'd0,  // Slave Reset RSTACT CCC config (see RSTA_xxx_b fields)
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
    // next one should only be set/cleared when bus not busy unless done in SCL clock domain
    // strap 0 if not needed.
  input               cf_SlvNack,       // Set to 1 to cause NACK of our address (R or W)
    // below is only used if EV_BAMATCH_b set in param. May change dynamically 
  input         [7:0] cf_BAMatch,
  // Now special states from SCL domain raw - these can be synchronized
  // on the fly for STATUS reads.
  output      [29:28] raw_ActState,     // activity state on bus from ENTASn
  output      [27:24] raw_EvState,      // event enabled states from ENEC/DISEC
  output        [6:0] raw_Request,      // bus request in process w/details
  output        [7:0] raw_DynAddr,      // dynamic address if set - is stable
  output       [13:0] raw_timec_sync,   // SYNC tc handled by system. [13]=clk
  output        [3:0] raw_slvr_reset,   // SlaveRst controls
  input               iraw_rst_slvr_reset, // clear of SlaveRst controls in SCL domain
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
  // now Magic regs, which means mapped into system using SCL based bus. If not using
  // this, normally the register autonomous_wrap would be used
  output              mr_clk_SCL,       // SCL clock we use (see also SCL_n)
  output              mr_clk_SCL_n,     // inverted as valid clock (when not scan)
  output        [7:0] mr_idx,           // current register index (auto inc)
  output        [7:0] mr_next_idx,      // next index after trans_done
  output              mr_dir,           // this is 0 if write, 1 if read (note timing)
  output              mr_trans_w_done,  // pulses for a cycle; at end of wdata (not idx)
  output              mr_trans_r_done,  // pulses for a cycle; at end of rdata
  output              mr_trans_ddrcmd,  // pulses for a cycle; at end of DDR CMD byte
  output              mr_trans_ccc_done,// if enabled: pulses for a cycle same as _w_done
  output        [7:0] mr_wdata,         // data with mr_dir=0 and trans_w_done=1
  input         [7:0] mr_rdata,         // live data from system - change on trans_r_done
  input               mr_rdata_none,    // is 1 if no read data - will NACK if 1st
  input               mr_rdata_end,     // 1 on read if last byte
  // Note: below signals same as osync but not synchronized - in SCL domain
  output              mr_bus_new,       // pulsed on match to us (note delay)
  output              mr_bus_done,      // pulsed on end of message
  // now scan related
  input               scan_single_clock,// single clock domain from scan_clk for pin-based clocks
  input               scan_clk,         // clock to use for pin clocks if in scan
  input               scan_no_rst,      // prevents layered reset
  input               scan_no_gates     // prevents arch clock gating
  );


  //
  // Wires - We define wires which are to/from registers and handleed 
  //         here or by instance.
  //

  // the following are used from the CRTL register to start an event.
  wire          [2:0] event_pending;
  wire                force_sda;
  wire                ibi_has_byte;
  wire          [7:0] opt_ibi_byte;
  wire          [7:0] timec_ibi_byte;
  wire                ibi_timec;        // 1 if time control on
  wire          [1:0] ibi_timec_marks;  // SC1 and SC2 stop - pulses
  wire          [2:0] ibi_timec_sel;    // output selector
  wire                timec_oflow;
  wire                timec_oflow_clr;
  wire                hold_engine;      // hold during HJ
  // next two are exported up in case needed
  wire                clk_SCL;
  wire                clk_SCL_n;
  // these next ones are handled here for transactions
  wire          [7:0] tb_datab;
  wire                tb_datab_ack;
  wire                tb_end;
  wire                tb_urun_nack;
  wire                tb_urun;
  wire                tb_term;
  wire          [7:0] fb_datab;
  wire                fb_datab_done;
  wire                fb_datab_err;
  wire          [2:0] fb_ddr_errs;      // HDR framing, parity, CRC
  wire                fb_s0s1_err;      // S0/S1 error lockup (exit clears)
  wire                abt_s0s1_err;
  wire                read_abort;
  wire                fb_orun;
  wire                tx_valid;         // used for NACK of header for example
  wire                tb_err;
  // interrupts and states from SCL domain
  wire                int_start_seen;
  wire                int_start_err;
  wire                int_da_matched;
  wire                int_sa_matched;
  wire                int_7e_matched;
  wire                int_ddr_matched;
  wire                int_in_STOP;
  wire                int_event_sent;
  wire                int_event_ack;
  wire                int_in_evproc;
  wire                int_ccc_handled;
  wire                int_ccc;
  wire                state_in_ccc;
  wire                state_in_daa;
  wire                state_in_hdr;
  wire                state_dir;
  reg           [1:0] bus_start;
  reg                 bus_ddr;
  wire                willbe_ccc;
  wire                is_ccc = (willbe_ccc|state_in_ccc) & ~state_in_hdr;
  
  // next is enable from Time control, so we can annotate an IBI 
  wire          [2:0] raw_TimeC; // FUTURE: need to look at adding this

  //
  // The Slave instance handles all aspects of the slave in SCL domain
  //
  i3c_slave_wrapper #(.ENA_ID48B(ENA_ID48B),.ID_48B(ID_48B),
                      .ID_AS_REGS(ID_AS_REGS),.ID_BCR(ID_BCR),.ID_DCR(ID_DCR),
                      .ENA_SADDR(ENA_SADDR),.SADDR_P(SADDR_P),
                      .ENA_MAPPED(0), .MAP_CNT(1), .MAP_I2CID(24'd0),
                      .MAP_DA_AUTO(0), .MAP_DA_DAA(0),
                      .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
                      .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ),
                      .ENA_CCC_HANDLING(ENA_CCC_HANDLING),.RSTACT_CONFIG(RSTACT_CONFIG),
                      .MAX_RDLEN(MAX_RDLEN), .MAX_WRLEN(MAX_WRLEN), .MAX_DS_WR(MAX_DS_WR), 
                      .MAX_DS_RD(MAX_DS_RD), .MAX_DS_RDTURN(MAX_DS_RDTURN),
                      .ENA_HDR(ENA_HDR), .ENA_MASTER(0), 
                      .PIN_MODEL(PIN_MODEL)) 
    slave 
    (
    .RSTn             (RSTn), 
    .pin_SCL_in       (pin_SCL_in), 
    .pin_SCL_out      (), 
    .pin_SCL_oena     (), 
    .pin_SDA_in       (pin_SDA_in), 
    .pin_SDA_out      (pin_SDA_out), 
    .pin_SDA_oena     (pin_SDA_oena), 
    .pin_SDA_oena_rise(pin_SDA_oena_rise), 
    .i2c_spike_ok     (i2c_spike_ok),
    .i2c_hs_enabled   (),               // no mapped, so not supported; could though
    .slv_enable       (cf_SlvEna & ~hold_engine),
    .int_start_seen   (int_start_seen), 
    .int_start_err    (int_start_err), 
    .int_da_matched   (int_da_matched), 
    .int_sa_matched   (int_sa_matched), 
    .int_7e_matched   (int_7e_matched), 
    .int_ddr_matched  (int_ddr_matched),// if DDR message header to us
    .int_in_STOP      (int_in_STOP), 
    .int_event_sent   (int_event_sent), 
    .int_event_ack    (int_event_ack),  // combinatorial from pin, so only use registered
    .int_in_evproc    (int_in_evproc),  // signals if in IBI now so no cancel
    .int_ccc_handled  (int_ccc_handled),// held if handled by lower block
    .int_ccc          (int_ccc),        // pulse if not handled by lower block
    .state_req        (raw_Request),    // [6]=HDR, [5]=ENTDAA, [4]=Wr, [3]=Rd, [2]=CCChandle, [1]=W/R, [0]=busy
    .opt_state_AS     (raw_ActState),   // Act state if known
    .opt_ev_mask      (raw_EvState),    // Event mask if known
    .opt_TimeC        (raw_TimeC),      // one hot Enables for Time Control
    .opt_timec_sync   (raw_timec_sync[12:0]), // if SYNC time control used
    .opt_slvr_reset   (raw_slvr_reset),
    .opt_match_idx    (),
    .i_rst_slvr_reset (iraw_rst_slvr_reset),
    .clr_slvr_cause   (1'b0),           // reset of block handled without regs
    .state_in_ccc     (state_in_ccc), 
    .state_in_daa     (state_in_daa), 
    .state_in_hdr     (state_in_hdr),
    .state_dir        (state_dir),      // held 1 if in-read (from header)
    .dyn_addr         (raw_DynAddr), 
    .dyn_addr_chg     (),               // ignored
    .dyn_chg_cause    (),               // just says which CCC caused
    .opt_static_addr  ((ENA_SADDR==`SADDR_CONST) ? {SADDR_P[6:0],1'b1} :
                       (ENA_SADDR==`SADDR_CONFIG)? cf_SlvSA :
                       (ENA_SADDR==`SADDR_NET)   ? cf_SlvSA :
                       8'h0),           // from param or none 
    .event_pending    (event_pending), 
    .force_sda        (force_sda), 
    .ibi_has_byte     (ibi_has_byte), 
    .opt_ibi_byte     (opt_ibi_byte), 
    .ibi_timec        (ibi_timec),
    .ibi_timec_marks  (ibi_timec_marks),
    .ibi_timec_sel    (ibi_timec_sel),
    .ibi_extdata      (1'b0),
    .ibi_mapidx       (4'b0),           // not using mapping
    .ibi_in_extdata   (),               // dont care since no ext data
    .timec_oflow      (timec_oflow),
    .timec_oflow_clr  (timec_oflow_clr),
    .oclk_SCL         (clk_SCL),
    .oclk_SCL_n       (clk_SCL_n),
    .tb_data_valid    (tx_valid),  
    .tb_datab         (tb_datab), 
    .tb_end           (tb_end),
    .tb_datab_ack     (tb_datab_ack), 
    .tb_urun_nack     (tb_urun_nack), 
    .tb_urun          (tb_urun), 
    .tb_term          (tb_term),
    .fb_data_use      (1'b1), 
    .fb_datab         (fb_datab), 
    .fb_datab_done    (fb_datab_done), 
    .fb_datab_err     (fb_datab_err), 
    .fb_orun          (fb_orun), 
    .fb_ddr_errs      (fb_ddr_errs),
    .fb_s0s1_err      (fb_s0s1_err),
    .brk_s0s1_err     (abt_s0s1_err),
    .rd_abort         (read_abort),
    .fb_hdr_exit      (),
    .obit_cnt         (),               // not used
    .ccc_byte         (),               // not used
    .willbe_ccc       (willbe_ccc),     // special 1 cycle early for is_ccc
    .cf_IdInst        (cf_IdInst), 
    .cf_IdRand        (cf_IdRand), 
    .cf_Partno        (cf_Partno), 
    .cf_IdBcr         (cf_IdBcr), 
    .cf_IdDcr         (cf_IdDcr),
    .cf_IdVid         (cf_IdVid),
    .cf_MaxRd         (12'd0),
    .cf_MaxWr         (12'd0),
    .cf_RstActTim     ({RSTACT_CONFIG[`RSTA_CUS_b] ? 
                          RSTACT_CONFIG[`RSTA_TIM_PER_b] : 8'd0, 
                        RSTACT_CONFIG[`RSTA_TIM_SYS_b], 
                        RSTACT_CONFIG[`RSTA_TIM_PER_b]}),
    .cf_SlvNack       (cf_SlvNack),
    .cf_SdrOK         (1'b1),           // if 0, i2c only
    .cf_DdrOK         (1'b0), 
    .cf_TspOK         (1'b0), 
    .cf_TslOK         (1'b0),
    .cf_TCclk         (16'd0),          // always const
    .cf_s0ignore      (1'b0),
    .cf_SetDA         (8'd0),           // only used for Main Master and retention in SW
    .cf_SetMappedDASA (10'd0),          // mapped not used in auton. MAP_CNT=1, so 9-1:0
    .cf_SetSA10b      (3'd0),           // "
    .cf_HdrCmd        (2'b00),
    .cf_vgpio         (9'd0),
    .cf_CccMask       (7'b1111111),     // if 0s would mask off unhandled CCCs. Param???
    .cf_MasterAcc     (1'b0),           // only for M+S, slave OK to accept Mastership
    .opt_MasterAcc    (),
    .map_daa_use      (1'b0),    // if MAP has auto-DAA regs
    .map_daa_dcr      (8'd0),    // "
    .map_daa_pid      (2'd0),    // "
    .raw_vgpio_done   (),               // only used if ENA_MAPPED VGPIO, so not here
    .raw_vgpio_byte   (),
    .raw_hdr_newcmd   (),
    .raw_hdr_cmd      (),
    .raw_setaasa      (),    // sync if used
    .raw_rstdaa       (),     // sync if used
    .raw_daa_ena      (),    // pulse if DASA/DAA for MAP
    .map_sa_idx       (),     // index in map if daa_ena
    .map_daa_da       (),     // new DA if daa_ena for map
    .i2c_dev_rev      (3'd0),           // if extended i2c, so unused
    .i2c_sw_rst       (),               // "
    .sraw_ActMode     (sraw_ActMode),
    .sraw_PendInt     (sraw_PendInt),
    .sraw_StatusRes   (sraw_StatusRes),
    .opt_ChgMaxRd     (),
    .opt_ChgMaxWr     (),
    .opt_MaxRdWr      (),
    .slv_debug_observ (),
    .scan_single_clock(scan_single_clock), 
    .scan_clk         (scan_clk), 
    .scan_no_rst      (scan_no_rst), 
    .scan_no_gates    (scan_no_gates)
  );
  assign raw_timec_sync[13] = clk_SCL;

  // Autonomous reg is a built up set of regs based on params
  i3c_autonomous_reg #(.REG_RULES(REG_RULES), .MAX_REG(MAX_REG), 
                       .REG_WRITABLE(REG_WRITABLE), .REG_READABLE(REG_READABLE),
                       .REG_RUN(REG_RUN), .REG_MASK(REG_MASK), .REG_BLEND(REG_BLEND),
                       .REG_REMAP(REG_REMAP),.REG_RESET(REG_RESET),
                       .REG_DDRCMD_WRIDX(REG_DDRCMD_WRIDX),
                       .MAGIC_RULES(MAGIC_RULES),.MAGIC_MASK(MAGIC_MASK),
                       .MAGIC_MATCH(MAGIC_MATCH))
  autonomous_reg
  (
    .clk_SCL_n       (clk_SCL_n), 
    .clk_SCL         (clk_SCL), 
    .CLK             (CLK),
    .RSTn            (RSTn), 
    .bus_new1        (bus_ddr ? bus_start[0] : bus_start[1]), 
    .bus_read        (state_dir), 
    .bus_ddr         (bus_ddr),
    .bus_in_ddr      (raw_Request[6]), // mode of bus
    .bus_done1       (int_start_seen),
    .bus_in_STOP     (int_in_STOP), 
    .fb_new1         (fb_datab_done & (~is_ccc | MAGIC_RULES[1])), 
    .fb_data         (fb_datab), 
    .fb_is_ccc       (is_ccc & MAGIC_RULES[1]), // conditions fb_new1
    .fb_err          (raw_fb_reg_err), 
    .tb_need1        (tb_datab_ack), 
    .tb_data         (tb_datab), 
    .tb_end          (tb_end), 
    .tb_err          (tb_err), 
    .tx_valid        (tx_valid),
      // next map to system
    .osync_started   (osync_started), 
    .o_read          (o_read), 
    .osync_done      (osync_done), 
    .osync_any_touch (osync_any_touch), 
    .oraw_reg_touch  (oraw_reg_touch), 
    .iraw_touch_OK   (iraw_touch_OK), 
    .wo_regs         (wo_regs), 
    .ro_regs         (ro_regs),
        // below for magic bus use - bus to system using SCL
    .mr_idx          (mr_idx),
    .mr_next_idx     (mr_next_idx),
    .mr_dir          (mr_dir),
    .mr_trans_w_done (mr_trans_w_done),
    .mr_trans_r_done (mr_trans_r_done),
    .mr_trans_ddrcmd (mr_trans_ddrcmd),
    .mr_trans_ccc_done(mr_trans_ccc_done),
    .mr_wdata        (mr_wdata),
    .mr_rdata        (mr_rdata),
    .mr_rdata_none   (mr_rdata_none),
    .mr_rdata_end    (mr_rdata_end),
    .scan_no_rst     (scan_no_rst)
  );
  assign mr_clk_SCL  = clk_SCL;
  assign mr_clk_SCL_n= clk_SCL_n;
  assign mr_bus_new  = bus_ddr ? bus_start[0] : bus_start[1];
  assign mr_bus_done = int_start_seen;

  // next is a delay so read is known at same time as new
  // exception is DDR
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn) begin
      bus_start <= 2'b00;
      bus_ddr   <= 1'b0;
    end else if ((int_da_matched|int_sa_matched) & ~is_ccc & // sdr
                 (~state_dir | tx_valid)) // if read, has data 
      bus_start <= 2'b01;
    else if (int_ddr_matched) begin // ddr if enabled
      bus_ddr   <= 1'b1;
      bus_start <= 2'b01;
    end else if (bus_start == 2'b01)
      bus_start <= 2'b10;
    else begin
      bus_start <= 2'b00;
      bus_ddr   <= 1'b0;
    end
  // next signals an error also for under-run NACK on SDR/DDR read and CCC GET
  assign raw_tb_reg_err = tb_err | (state_dir&(int_da_matched|int_sa_matched)&~tx_valid);

  //
  // Event management within CDC
  //

  // This portion will convert an event request between CLK and SCL
  generate  
    if (ENA_IBI_MR_HJ != 0) begin : sync_events
      // we simply convert the i_ibi_req from CLK to CLK_SLOW domain.
      // The ibi_byte is stable/held so no worries. The IBI req
      // is only made from STOP into START. But, bus available stop
      // period can be used to force the start (hold SDA Low). They 
      // must clear the request when ACKed (done) or they give up.
      wire syn_ibi_req, syn_hj_req;
      SYNC_S2B sync_ibi_req(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(i_ibi_req), 
                            .out_clk(syn_ibi_req));
      if (ENA_IBI_MR_HJ[`EV_HJ_b]) begin : hj_syn
        SYNC_S2B sync_hj_req(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(i_hj_req), 
                             .out_clk(syn_hj_req));
      end else 
        assign syn_hj_req = 1'b0;
      // we can be done with ACK or NACK
      SYNC_ASelfClr_S2C_Seq2 sync_ibi_done(.SCL(clk_SCL_n), .CLK(CLK), .RSTn(RSTn), 
                                          .local_set(int_event_sent&int_event_ack), 
                                          .o_pulse(o_ibi_done));
      SYNC_ASelfClr_S2C_Seq2 sync_ibi_nack(.SCL(clk_SCL_n), .CLK(CLK), .RSTn(RSTn), 
                                          .local_set(int_event_sent&~int_event_ack), 
                                          .o_pulse(o_ibi_nacked));
      wire        allow_ev = (syn_hj_req & raw_EvState[27]) | 
                             (syn_ibi_req & raw_EvState[24]);
      // next is a mask so that if event completes, we stop requesting - allows
      // CLK_SLOW to catch up and turn off. We use reset based on event going
      // away to remove mask. 
      reg         ev_maskoff;
      wire        rst_ev_maskoff_n = RSTn & (allow_ev | scan_no_rst);
      always @ (posedge clk_SCL_n or negedge rst_ev_maskoff_n)
        if (!rst_ev_maskoff_n)
          ev_maskoff <= 1'b0;   // no mask unless enabled - clears when IBI req goes away
        else if (int_event_sent&int_event_ack)
          ev_maskoff <= 1'b1;   // mask off event until req clears

      assign event_pending = (~allow_ev | ev_maskoff) ? 3'b000 : // no event
                  (syn_hj_req) ? 3'b111 : // no state can be used for HJ
                  {(syn_ibi_req&int_in_STOP),(syn_ibi_req ? 2'b01 : 2'b00)}; // IBI 

      // now the counters for IBI, MR, S0/S1 breakout, Read stall detect, and HJ
      wire syn_s0s1;
      wire syn_inread;
      wire raw_rd_abort;
      wire sync_gate;
        // S0S1 error is synchronized to slow clock
      SYNC_S2C sync_s0s1_err(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(fb_s0s1_err), 
                             .out_clk(syn_s0s1));
        // in read and not i2c is synchronized to slow clock
      SYNC_S2C sync_in_read(.rst_n(RSTn), .clk(CLK_SLOW), 
                            .scl_data(raw_DynAddr[0]&raw_Request[3]), .out_clk(syn_inread));
        // read abort causes reset of state machine, so no sync
      assign read_abort = raw_rd_abort & raw_Request[3]; 
        // slow gate has to allow clock for sync above
        // So combination of synchronized and unsync
      wire raw_gate = ~(raw_DynAddr[0]&raw_Request[3]) & ~fb_s0s1_err &
                      ~i_ibi_req & ~(ENA_IBI_MR_HJ[`EV_HJ_b]&i_hj_req);
      wire local_syn_gate = ~syn_ibi_req & ~syn_hj_req & ~syn_s0s1; // ~syn_inread is handled by counter
      assign slow_gate = sync_gate & raw_gate & local_syn_gate;

      i3c_slow_counters #(.ENA_IBI_MR_HJ(ENA_IBI_MR_HJ),
                            .CLK_SLOW_BITS(CLK_SLOW_BITS),.CLK_SLOW_MATCH(CLK_SLOW_MATCH),
                            .CLK_SLOW_HJMUL(CLK_SLOW_HJMUL),
                            .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
                          .ENA_CCC_HANDLING(ENA_CCC_HANDLING))
        counters 
        (
          .RSTn             (RSTn), 
          .clk_SCL_n        (clk_SCL_n), 
          .clk_SCL          (clk_SCL),
          .CLK_SLOW         (CLK_SLOW),
          .slow_gate        (sync_gate),
          .cf_SlvEna        (cf_SlvEna), 
          .cf_BAMatch       (cf_BAMatch),
          .event_pending    (event_pending),
          .pin_SCL_in       (pin_SCL_in),   // for state check
          .pin_SDA_in       (pin_SDA_in),   // "
          .run_60           (syn_s0s1), 
          .run_100          (syn_inread),   // in read op but not i2c
          .int_in_STOP      (int_in_STOP), 
          .force_sda        (force_sda),
          .done_60          (abt_s0s1_err), // no sync needed since reset
          .done_100         (raw_rd_abort),
          .hold_engine      (hold_engine)   // hold engine off until HJ emits start
        );

      // next pulls in time control if enabled (only with IBI of course)
      if (|ENA_TIMEC) begin : timec_handling
        wire                    ev_pend;
        wire                    time_oflow;
        wire                    syn_tc_gate;
        // sync to SLOW clock from CLK - 1 bit as true or not true
        // is held signal
        SYNC_C2S #(1) sync_IBI_pend(.rst_n(RSTn), .scl(CLK_SLOW_TC), 
                                    .clk_data(event_pending[0]), .out_scl(ev_pend));
        SYNC_AClr_C2S sync_tcoflow(.CLK(CLK_SLOW_TC), .RSTn(RSTn), 
                                   .local_set(time_oflow), .async_clear(timec_oflow_clr),
                                    .o_value (timec_oflow));
        // CLK_SLOW_TC is not gated if needed for sync as well as results
        // note timec_oflow waits for clear (set when timing).
        assign tc_slow_gate = syn_tc_gate & ~event_pending[0] & ~timec_oflow;

        i3c_time_control time_ctrl(
          .RSTn          (RSTn), 
          .CLK_SLOW      (CLK_SLOW_TC), 
          .clk_SCL_n     (clk_SCL_n), 
          .timec_ena     (raw_TimeC),   // what is enabled - one hot
          .event_start   (ev_pend),     // start - held while true
          .was_nacked    (o_ibi_nacked),// if NACKed
          .sc1_stop      (ibi_timec_marks[0]),// marks on edges for SC1 and SC2
          .sc2_stop      (ibi_timec_marks[1]),
          .time_info_sel (ibi_timec_sel), // sel byte out >=1/2 cycle early
          .time_info_byte(timec_ibi_byte),// selected byte for output
          .ibi_timec     (ibi_timec),   // trigger IBI now
          .time_overflow (time_oflow),  // 1 if timer overflowed
          .slow_gate     (syn_tc_gate), // gate control for CLK_SLOW_TC
          .scan_no_rst   (scan_no_rst));
      end else begin
        assign tc_slow_gate    = 1;     // never used
        assign timec_ibi_byte  = 8'd0;
        assign ibi_timec       = 1'b0;
        assign timec_oflow     = 1'b0;
      end

    end else begin // no IBI, MR, HJ
      assign event_pending = 3'd0;
      assign force_sda     = 1'b0;
      assign hold_engine   = 1'b0;
      assign slow_gate     = 1'b0;
      assign tc_slow_gate  = 1'b0;
      assign o_ibi_done    = 1'b0;
      assign o_ibi_nacked  = 1'b0;
      assign abt_s0s1_err  = 1'b0;
      assign read_abort    = 1'b0;
      assign ibi_timec     = 1'b0;
    end

    if (ENA_IBI_MR_HJ[`EV_IBI_DAT_b]) begin : ibi_byte
      assign ibi_has_byte  = 1'b1;
      if (|ENA_TIMEC) 
        assign opt_ibi_byte  = ~|ibi_timec_sel[1:0] ? 
                               // IBI byte always used 1st and mandatory if BCR said so
                               (i_ibi_byte | (ibi_timec?8'h80:8'h00)) : 
                               // time c byte changes by mux
                               timec_ibi_byte;
      else
        assign opt_ibi_byte  = i_ibi_byte;
    end else begin
      assign ibi_has_byte  = 1'b0;
      assign opt_ibi_byte  = 8'd0;
    end

  endgenerate

  //  
  // logic below is a form of assertion to catch bad parameters or combinations
  // The sim or sythesis compiler will issue warnings on out-of-bounds
  //

  // ID must be some mix of constant and if some regs, then mux must match
  // Note that VID must not be 0 even if reg, since that forms the reset value
  localparam ID_TST = ~|ENA_ID48B || ENA_ID48B>`ID48B_CONST_NONE || ID_48B==48'd0 ||
                      (ENA_ID48B<=`ID48B_CONST_INST && ID_48B[32]);
  wire [0:0] bad_id;
  assign bad_id[ID_TST] = 1'b0;
  localparam IAR_TST = ID_AS_REGS>=32 || 
                       ID_AS_REGS[`IDREGS_INST_b]&(ENA_ID48B!=`ID48B_CONST_INST) ||
                       ID_AS_REGS[`IDREGS_RAND_b]&(ENA_ID48B!=`ID48B_CONST_PARTNO) ||
                       ID_AS_REGS[`IDREGS_VID_b] &(ENA_ID48B!=`ID48B_CONST_NONE);
  wire [0:0] bad_id_as_reg;
  assign bad_id_as_reg[IAR_TST] = 1'b0;
    // note cannot check DCR since allowed to be 0
    // below says if constant BCR, then has to be valid
  localparam BCR_TST = ~ID_AS_REGS[`IDREGS_BCR_b] &
                       (ID_BCR[7:6]!=2'b00 || // must match as slave
                        ////ID_BCR[5]!=|ENA_HDR || -- OK since DDR_OK can be used
                        ID_BCR[1]!=ENA_IBI_MR_HJ[`EV_IBI_b] ||
                        ID_BCR[2]!=ENA_IBI_MR_HJ[`EV_IBI_DAT_b] ||
                        (ENA_IBI_MR_HJ[`EV_IBI_DAT_b]&~ENA_IBI_MR_HJ[`EV_IBI_b]) ||
                        (|ENA_TIMEC&~ENA_IBI_MR_HJ[`EV_IBI_DAT_b]));
  wire [0:0] bad_bcr;
  assign bad_bcr[BCR_TST] = 1'b0;
  // now static addr
  localparam SA_INV = SADDR_P<3 || SADDR_P>=8'h7E ||
                      SADDR_P==8'h7C || SADDR_P==8'h7A || SADDR_P==8'h76 ||
                      SADDR_P==8'h6E || SADDR_P==8'h5E || SADDR_P==8'h3E;
  localparam SA_TST = ENA_SADDR>`SADDR_CONFIG ||
                      (ENA_SADDR!=`SADDR_CONST && |SADDR_P) || // not allowed
                      (ENA_SADDR==`SADDR_CONST && SA_INV);
  wire [0:0] bad_sa;
  assign bad_sa[SA_TST] = 1'b0;
  // now clock slow
  localparam CLK_TST = (|(CLK_SLOW_BITS|CLK_SLOW_MATCH) & ~|ENA_IBI_MR_HJ) ||
                       CLK_SLOW_MATCH>=(1<<CLK_SLOW_BITS);
  wire [0:0] bad_clk;
  assign bad_clk[CLK_TST] = 1'b0;
  // CCC handling
  localparam MAXES   = MAX_RDLEN|MAX_WRLEN;
  localparam ALLMX   = MAXES|MAX_DS_WR|MAX_DS_RD|MAX_DS_RDTURN;
  localparam CCC_TST = ENA_CCC_HANDLING>6'h3F ||
                       MAXES>16'hFFFF ||
                       (~ENA_CCC_HANDLING[`ENCCC_MAXES_b]&|ALLMX) ||
                       (ENA_CCC_HANDLING[`ENCCC_MAXES_b]&
                             (MAX_DS_WR>7||MAX_DS_RD>63||MAX_DS_RDTURN>=(1<<24)));
  wire [0:0] bad_ccc;
  assign bad_ccc[CCC_TST] = 1'b0;
  // time control - require async mode 0 if sync - for slave as well
  localparam TIMEC_TST = (ENA_TIMEC[2] & ~ENA_TIMEC[1]) |
                         (ENA_TIMEC[0] & ~ENA_TIMEC[1]) | 
                         (|ENA_TIMEC[5:3]);
  wire [0:0] bad_timec;
  assign bad_timec[TIMEC_TST] = 1'b0;
  // bus info - none for autonomous
  // FIFO type - none for autonomous

  // pin model
  localparam PINMODEL_TST = PIN_MODEL>2;
  wire [0:0] bad_pin_model;
  assign bad_pin_model[PINMODEL_TST] = 1'b0;

  // HDR only allows DDR but not TSP or TSL for now
  localparam HDR_TST = |ENA_HDR[2:1];
  wire [0:0] bad_hdr;
  assign bad_hdr[HDR_TST] = 1'b0;

  // Magic has to be enabled for
  localparam MAG_TST = |(MAGIC_RULES|MAGIC_MASK|MAGIC_MATCH)&~REG_RULES[5];
  wire [0:0] bad_magic;
  assign bad_magic[MAG_TST] = 1'b0;

endmodule
