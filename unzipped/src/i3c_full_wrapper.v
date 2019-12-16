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
//  File            : i3c_full_wrapper.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Mon Dec 16 11:09:12 2019 $
//  Revision        : $Revision: 1.76.1.3 $
//
//  IP Name         : i3c_full_wrapper 
//  Description     : MIPI I3C Outer wrapper for Net based use, inc CDC
//    This contains the full wrapper for I3C where the registers are handled 
//    by the system, but the CDC and data (FIFO internal or external) is 
//    supported. See also the APB wrapper for Slave and Master
//    As an outer wrapper, this module support 3 needs:
//    1. Instantiates the Slave and optionally slave-in-Master.
//    2. Handles clock domain crossing using the system clock (e.g. PCLK if
//       APB bus).
//    3. Manages the data buffering/FIFO as needed, including signals for
//       external FIFO.
//    The specifics of what this does are controlled by the parameters
//    as outlined in the micro-arch spec and also in the params file.
//  NOTE: this also has checks for parameters being incorrect or 
//        mismatched - this will issue warnings (no way to get errors).
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
//  ECO from tag 1.5 and earlier:
//  1. pin_PUR_out/oena removed
//  2. added is_slave
//  3. added tb_, fb_, m_tb_, and m_fb_ ports to tie off and strap


`include "i3c_params.v"                 // local parameters/constants

module i3c_full_wrapper #(
    // params are driven from upper layer to control how this block
    // is built and operates
    parameter ENA_ID48B       = `ID48B_CONST, // const vs. reg
    parameter  ID_48B         = 48'h0,  // must be filled in from above with some/all of 48-bit ID
    parameter ID_AS_REGS      = 12'd0,  // regs for ID and other uses (e.g. masks)
    parameter  ID_BCR         = 8'd0,   // filled in from above with BCR if not from regs
    parameter  ID_DCR         = 8'd0,   // filled in from above with DCR if not from regs
    parameter ENA_SADDR       = `SADDR_NONE, // none, const, reg/net
    parameter  SADDR_P        = 0,      // 7 bits as 6:0
    parameter ENA_MAPPED      = 5'd0,   // if extra DAs/SAs allowed, and related
    parameter  MAP_CNT        = 4'd1,   // number of extra DAs/SAs allowed
    parameter  MAP_I2CID      = 24'd0,  // !=0 if I2C extended with DevID
    parameter  MAP_DA_AUTO    = {5'd0,1'b0,1'b0,3'd0,1'b0,1'b0,1'b0},
    parameter  MAP_DA_DAA     = 0,      // if not MMR and PID/DCR !=0, is bit array      
    parameter ENA_IBI_MR_HJ   = 0,      // 0 if no events, else events as mask
    parameter  CLK_SLOW_BITS  = 6,      // number of bits needed for count for Bus Avail
    parameter  CLK_SLOW_MATCH = 6'd47,  // count: e.g. 47 for 1us if 48MHz CLK (0 rel)
    parameter  CLK_SLOW_HJMUL = 10'd1000,// number of MATCHes (const or reg) for 1ms (1 rel)
    parameter  ERROR_HANDLING = 3'd0,   // any special error handling, eg. read abort 
    parameter  ENA_TIMEC      = 6'b000010,    // clock_reg, res, res, mode 1, mode 0, sync
    parameter  TIMEC_FREQ_ACC = {8'd24,8'd10},// freq=12MHz (12.0=24) with 1.0% accuracy
    parameter ENA_CCC_HANDLING= 6'd0,   // passed down as to what to support
    parameter RSTACT_CONFIG   = 26'd0,  // Slave Reset RSTACT CCC config (see RSTA_xxx_b fields)
    parameter MAX_RDLEN       = 0,      // default S->M len max
    parameter MAX_WRLEN       = 0,      // default M->S len max
    parameter MAX_DS_WR       = 0,      // data speed limits for M->S
    parameter MAX_DS_RD       = 0,      // data speed limits for S->M
    parameter MAX_DS_RDTURN   = 0,      // latency needs for S->M read req
    parameter SEL_BUS_IF      = 5'd0,   // default to no bus - they can choose
    parameter FIFO_TYPE       = 0,      // if internal FIFO, the width (only 1 is allowed)
    parameter  EXT_FIFO       = 3'd0,   // if not 0, then external FIFO is selected
    parameter  ENA_TOBUS_FIFO = 0,      // depth of to-bus as power of 2 from 2 up
    parameter  ENA_FROMBUS_FIFO=0,      // depth of from-bus as power of 2 from 2 up
    parameter ENA_HDR         = 0,      // enables for HDR modes
    parameter BLOCK_ID        = 0,      // if not 0, allows ID reg
    parameter ENA_MASTER      = 0,      // 1 if using master
    parameter PIN_MODEL       = `PINM_COMBO, // combinatorial pin use
    parameter priv_sz         = PIN_MODEL[1] ? (ENA_HDR[0] ? 3 : 1) : 0,// wider if ext-pad+ddr
    parameter [7:0] PID_CNT   = MAP_DA_AUTO[`MAPDA_DAA_PID_lb+4:`MAPDA_DAA_PID_lb], // computed
    parameter FULL_DBG_MX     = 0       // debug observer
  )
  (
  // define clock and reset
  input               RSTn,             // reset from system
  input               CLK,              // system clock for regs and sync
  input               CLK_SLOW,         // clock to use for IBI forced
  output              slow_gate,        // 1 if may gate CLK_SLOW
  input               CLK_SLOW_TC,      // clock for time-control
  output              tc_slow_gate,     // 1 if may gate CLK_SLOW_TC. AND with slow_gate if same clock
  input               clk_FastTernary,  // Ternary clock if Ternary used
  // define pins
  input               pin_SCL_in,       // SCL: normally clock in, data if ternary
  output              pin_SCL_out,      // SCL: only driven if ternary
  output              pin_SCL_oena,     // SCL: only driven if ternary
  input               pin_SDA_in,       // SDA: M->S
    // Note: next 3 are used special when PIN_MODEL==`PINM_EXT_REG
    //       They feed 3 flops placed close to the pads
  output              pin_SDA_out,      // SDA: S->M on read
  output              pin_SDA_oena,     // SDA: S->M on read
  output  [priv_sz:0] pin_SDA_oena_rise,// special on SCL rising if EXT_REG, else 0
    // next one is only used if SDA & SCL pads have i2c 50ns Spike 
    // filters and they can be turned on/off via net.
  output              i2c_spike_ok,     // Is 1 to allow i2c spike filter
  output              i2c_hs_enabled,   // If enabled, indicates in I2C HS mode
  // now the configuration (stable) inputs. These should be tied off as 0 
  // when not used (selected by param).
  input               cf_SlvEna,        // only process bus if 1
  input               cf_SlvNack,       // do not ACK private msgs
  input         [7:0] cf_SlvSA,         // optional slave i2c address ([0]=1 if enabled)
  input         [3:0] cf_IdInst,        // Instance of ID when selected (not if partno used)
  input               cf_IdRand,        // random partno of ID if used
  input               cf_Offline,       // conditions SlvEna
  input        [31:0] cf_Partno,        // partno od ID if used
  input         [7:0] cf_IdBcr,         // BCR of DAA if not const
  input         [7:0] cf_IdDcr,         // DCR of DAA if not const
  input        [14:0] cf_IdVid,         // MIPI Vendor ID if not const
  input               cf_DdrOK,         // Allow DDR messages (if param allows)
  input               cf_TspOK,         // Allow TSP messages (if param allows)
  input               cf_TslOK,         // Allow TSL messages (if param allows)
  input        [11:0] cf_MaxRd,         // Max read in bytes (if CCC enabled)
  input        [11:0] cf_MaxWr,         // Max write in bytes (if CCC enabled)
  input        [23:0] cf_RstActTim,     // Slave Reset recovery times
  input         [7:0] cf_BAMatch,       // Bus Available count match (if IBI+BAM)
  input        [15:0] cf_TCclk,         // Time control clock info (if reg)
  input               cf_s0ignore,      // suppress S0 error
  input               cf_matchss,       // only set Start/Stop if matched=1
  input         [7:0] cf_SetDA,         // Main Master sets its own DA
  input [(MAP_CNT*10)-1:0] cf_SetMappedDASA, // mapped SAs/DAs if supported
  input         [2:0] cf_SetSA10b,      // if [1] is SA 10-bit
  input               cf_MasterAcc,     // only for M+S, slave OK to accept Mastership
  input               cf_IbiExtData,    // Extended data for IBI
  input         [3:0] cf_IbiMapIdx,     // Mapped index for IBI
  input         [1:0] cf_HdrCmd,        // enable for HDR Cmd as MMR
  input         [6:0] cf_CccMask,       // Mask enables for unhandled CCCs
  input         [8:0] cf_vgpio,         // VGPIO control
  output              vgpio_done,       // done pulse if VGPIO enabled
  output        [7:0] raw_vgpio_byte,   // byte value if "
  input   [MAP_CNT-1:0] map_daa_use,      // which MAP are auto-DAA
  input [(MAP_CNT*8)-1:0] map_daa_dcr,    // DCRs if MAP auto-DAA
  input [(MAP_CNT*PID_CNT)-1:0] map_daa_pid, // PID partials if MAP auto-DAA
  output              hdr_new_cmd,      // Pulse in PCLK domain (sync)
  output        [7:0] raw_hdr_cmd,      // raw but new_cmd syncs
  output              map_rstdaa,       // pulse in PCLK domain if rstdaa and MAP Auto
  output              map_setaasa,      // pulse in PCLK domain if enabled
  output              map_daa_ena,      // pulse in PCLK domain if map DASA/DAA
  output        [3:0] map_sa_idx,       // index if dasa_ena
  output        [7:1] map_daa_da,       // new DA
    // i2c extended stuff if used
  input         [2:0] i2c_dev_rev,      // revision if DeviceID is used (OR into const)
  output              i2c_sw_rst,       // SW reset
  // Now special states from SCL domain raw - these can be synchronized
  // on the fly for STATUS reads.
  output      [29:28] raw_ActState,     // activity state on bus from ENTASn
  output      [27:24] raw_EvState,      // event enabled states from ENEC/DISEC
  output        [2:0] raw_TimeC,        // time control enables - one hot
  output        [6:0] raw_Request,      // bus request in process w/details
  output        [7:0] raw_DynAddr,      // dynamic address if set - is stable
  output        [2:0] raw_DynChgCause,  // cause info - static after change
  output       [13:0] raw_timec_sync,   // SYNC tc handled by system. [13]=clk
  output        [3:0] raw_slvr_reset,   // SlaveRst controls
  input               iraw_rst_slvr_reset, // clear of SlaveRst controls in SCL domain
  input               iraw_slvr_irq,    // block reset from SCL domain
  output       [12:0] raw_match_idx,    // last 3 matching indexes of SA or DA if mapped
  output              raw_matched,      // SA or DA matched in SCL domain: used for wake

  // these next ones are to be safe-raw. That means either changed when
  // bus is STOPped or otherwise not doing a GETSTATUS or synchronized.
  input         [7:6] sraw_ActMode,     // System activity mode (or 0 if not used)
  input         [3:0] sraw_PendInt,     // Pending interrupt (or 0 if not used)
  input        [15:8] sraw_StatusRes,   // reserved bits of status (or 0 if not used) 
  // Now "registers" which are provided by or for outer wrapper (as needed) 
  // using CLK domain. So, synchronized as needed when interchanged with SCL
  // reg_ means provided from above. outp_ means we are pushing out to
  // upper layer. regflg and outpflg are flag controls (for changes)
  output       [19:8] outp_IntStates,   // interrupt status; see `IS_xx
  input         [2:0] reg_EvPend,       // Event request. [2] is flag for change
  input         [7:0] reg_EvIbiByte,    // optional byte (req if BCR said so)
  output              outp_EvNoCancel,  // 1 if not safe to cancel now
  output      [22:20] outp_EvDet,       // details of Event pending/done 
  output        [5:0] outp_GenErr,      // general errors from engines
  output       [11:8] outp_DataErr,     // data errors from engine. [11] is level held
  input         [5:0] msk_GenErr,       // mask for Status bit and IRQ
  input        [11:8] msk_DataErr,      // mask for Status bit and IRQ
  input        [19:8] reg_clrIntStates, // clear int status when bits are 1
  input         [5:0] reg_clrGenErr,    // clear errors when bits are 1 
  input        [11:8] reg_clrDataErr,   // clear errors when bits are 1 
  input               reg_holdOErr,     // cause int status for Overun errors
  output              outpflg_MaxRd,    // Master changed MaxRd
  output              outpflg_MaxWr,    // Master changed MaxWr
  output       [11:0] outp_MaxRW,       // Value from Master for Rd or Wr
  output              outp_to_master,   // switch to master now (if M+S)
  // now same CLK domain but related to data or FIFO
  input               reg_TbEnd,        // end of to-bus data
  input               reg_TbFlush,      // 1 cycle pulse to flush tb buff
  input               reg_FbFlush,      // 1 cycle pulse to flush fb buff
    // next 2 are only used if internal FIFO enabled
  input         [5:4] reg_TxTrig,       // trigger level for TX (tb)
  input         [7:6] reg_RxTrig,       // trigger level for RX (fb)
  output      [20:16] outp_TxCnt,       // current counter for TX (tb)
  output      [28:24] outp_RxCnt,       // current counter for RX (fb)
  output              outp_TxFull,      // TX FIFO is full (not int which is Trig)
  output              outp_RxEmpty,     // RX FIFO is empty (not int which is Trig)
    // next 3 will only be used to size(s) allowed by params
  input         [1:0] regflg_wr_cnt,    // tb: 0=none, 1 = wrote b
  input         [7:0] reg_wdata,        // tb: byte
  input         [1:0] regflg_rd_cnt,    // fb: 0=none, 1 = read b
  output        [7:0] outp_fb_data,     // fb: byte
  // Optional external FIFO: 
    // To-bus means from slave to master
  input               ixf_tb_avail,     // 1 if byte is available for tobus
  input               ixf_tb_last,      // byte is last for message
  input         [7:0] ixf_tb_data,      // actual data when avail=1
  output              oxf_tb_start,     // pulsed when ixf_tb_avail==1 and START
  output              oxf_tb_used,      // pulsed when tb_data used
    // From-bus means from master into slave
  input               ixf_fb_free,      // 1 if space to take a byte frombus
  output              oxf_fb_req,       // pulse when data to be taken (amd free=1)
  output        [7:0] oxf_fb_data,      // data frombus when req=1
  output              oxf_fb_eof,       // frame ended in repeated START or STOP
    // Next for IBI FIFO (for IBI EXTDATA) if used
  input        [10:0] ibi_wr_fifo,      // FIFO signals from Regs
  output              ibi_wr_ack,       // reg wr picked up
  // these next ones are only used when Master/Slave, else strapped
  input               is_slave,         // 1=in slave mode, else in master
  output              d_tb_data_valid,  // 1 if to-bus data ready
  output              tb_pclk_valid,    // PCLK version of d_tb_data_valid
  output        [7:0] d_tb_datab,       // to-bus data if valid
  output              d_tb_end,         // 1 if last byte to-bus
  input               m_tb_datab_ack,   // 1 SCL pulse Master accepts to-bus data
  output              fb_data_use,      // 1 if from-bus buffer not full
  input         [7:0] m_fb_datab,       // byte from-bus if use=1
  input               m_fb_datab_done,  // 1 SCL pulse Master says data ready
  // debug observer
  output[FULL_DBG_MX:0]full_debug_observ, // optional debug observer
  // now scan related
  input               scan_single_clock,// single clock domain from scan
  input               scan_clk,         // clock to use if in scan
  input               scan_no_rst,      // prevents layered reset
  input               scan_no_gates     // prevents clock gating
  );

  //
  // Wires - We define wires which are to/from registers and handleed 
  //         here or by instance.
  //

  wire                PCLK    = CLK;    // map to simplify
  wire                PRESETn = RSTn;   // "   "  "
  // the following are used from the CRTL register to start an event.
  wire          [2:0] event_pending;
  wire                force_sda;
  wire                ibi_has_byte;
  wire          [7:0] opt_ibi_byte;
  wire                ibi_timec;        // 1 if time control on
  wire          [1:0] ibi_timec_marks;  // SC1 and SC2 stop - pulses
  wire          [2:0] ibi_timec_sel;    // output selector
  wire                raw_ibi_in_extdata; // indicates if IBI ext data read
  wire                timec_oflow;
  wire                timec_oflow_clr;
  wire                hold_engine;      // hold during HJ
  wire                slvena_delay;
    // only used if IBI FIFO and EXTDATA
  wire                ibs_tb_data_valid;// has data for us
  wire          [7:0] ibs_tb_datab;     // data byte
  wire                ibs_tb_end;       // if ending byte
  wire                ibs_tb_datab_ack; // was taken by engine
  // next two are exported up for memory buffer CDC
  wire                clk_SCL;
  wire                clk_SCL_n;
  // these next ones are handled here for memory buffering
  wire                s_tb_datab_ack;
  wire                s_tb_data_valid;
  wire          [7:0] s_tb_datab;
  wire                s_tb_end;
  wire                d_tb_datab_ack, d2_tb_datab_ack;
  wire                tb_urun_nack;
  wire                tb_urun;
  wire                tb_term;
  wire          [7:0] fb_datab, s_fb_datab;
  wire                fb_datab_done, s_fb_datab_done;
  wire                fb_datab_err;
  wire          [2:0] fb_ddr_errs;      // HDR framing, parity, CRC
  wire                fb_s0s1_err;      // held 1 if in S0 or S1 error hold
  wire                brk_s0s1_err;     // output 1 in any domain to break S0/S1 error hold
  wire                abt_s0s1_err;     // break auto from 60us inaction
  wire                read_abort;       // read stalled
  wire                fb_orun;
  wire                local_fb_int;
  // linkage for FIFO or non-FIFO. Note byte oriented (FIFO may be 16/32)
  wire                localn_fb_ready;
  wire          [7:0] localn_fb_data;
  wire                localn_fb_ack;
  wire                avail_tb_ready;
  wire          [7:0] avail_tb_data;
  wire                avail_tb_ack; 
  wire                avail_tb_end; 
  wire                local_tb_int;
  // buffering but PCLK side
  wire          [3:0] set_fb_err;
  wire                set_rd_err;       // rd abort special
  wire                set_fb_orun;
  wire                set_tb_urun_nack;
  wire                set_tb_urun;
  wire                set_tb_term;
  wire                flg_maxrd, flg_maxwr;
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
  wire                opt_MasterAcc;
  wire          [6:0] raw_req;
  wire                clr_slvr_cause;
  wire                raw_vgpio_done;
  wire                dyn_addr_chg;
  wire                fb_hdr_exit;
  wire                scl_hdr_new_cmd;
  wire          [7:0] scl_hdr_cmd;
  wire                raw_setaasa;
  wire                raw_rstdaa;
  wire                raw_daa_ena;

  //
  // Allow export of debug observer data
  // -- default stub assigns 0, but this allows adding nets
  // -- Note that SLV_DBG_MX is defined by include file as
  //    is slv_debug_observ
  //
  `ifdef ENA_DBG_OBSERVE
   `include "dbg_observ_full.v"          // integrator replaces inc file contents
  `else
   localparam SLV_DBG_MX = 0;
   wire [SLV_DBG_MX:0] slv_debug_observ;
   assign full_debug_observ = 0;
  `endif
  //


  //
  // Stitching block for Master vs. Slave use of FIFO (i3c bus side)
  // -- Will prune away if slave only
  // -- Note stitching for optional IBI FIFO as well - Slave only
  assign d2_tb_datab_ack = is_slave ? s_tb_datab_ack : m_tb_datab_ack;
  assign fb_datab        = is_slave ? s_fb_datab      : m_fb_datab;
  assign fb_datab_done   = is_slave ? s_fb_datab_done : m_fb_datab_done;
  assign tb_pclk_valid   = |outp_TxCnt & 
                (~d2_tb_datab_ack | (outp_TxCnt!=4'b0001)); // used because no SCL if waiting

  // now linkages for IBI FIFO if used 
  // s_tb_.. is what goes to slave specifically
  // d_tb_.. is what comes from DATA FIFO and goes to master aways
  generate if (ENA_IBI_MR_HJ[`EV_EXTFIFO_b]) begin : ibi_fifo_mux
    assign s_tb_data_valid = raw_ibi_in_extdata ? ibs_tb_data_valid : d_tb_data_valid;
    assign s_tb_datab      = raw_ibi_in_extdata ? ibs_tb_datab      : d_tb_datab;
    assign s_tb_end        = raw_ibi_in_extdata ? ibs_tb_end        : d_tb_end;
    assign ibs_tb_datab_ack= raw_ibi_in_extdata ? s_tb_datab_ack    : 1'b0;
    assign d_tb_datab_ack  = raw_ibi_in_extdata ? 1'b0              : s_tb_datab_ack;
  end else begin
    assign s_tb_data_valid = d_tb_data_valid;
    assign s_tb_datab      = d_tb_datab;
    assign s_tb_end        = d_tb_end;
    assign d_tb_datab_ack  = d2_tb_datab_ack;
    assign ibs_tb_datab_ack= 1'b0;
  end endgenerate

  //
  // Data buffer management for from-bus
  // -- This handles the 2 buffer (ping pong) to one visible buffer map
  // -- Can Handle FIFO or singleton
  //
  i3c_data_frombus #(.FIFO_TYPE(FIFO_TYPE),.ENA_FROMBUS_FIFO(ENA_FROMBUS_FIFO),
                     .ANY_HDR(|ENA_HDR))
    frombus 
    (
    .RSTn             (PRESETn), 
    .CLK              (PCLK), 
    .SCL              (clk_SCL), 
    .SCL_n            (clk_SCL_n),
    .notify_fb_ready  (localn_fb_ready), 
    .notify_fb_data   (localn_fb_data), // byte
    .notify_fb_ack    (localn_fb_ack), 
    .fb_flush         (reg_FbFlush),
    .set_fb_err       (set_fb_err), 
    .set_fb_orun      (set_fb_orun), 
    .clear_fb_err     (reg_clrDataErr[11:8]),
    .clear_fb_orun    (reg_clrGenErr[0]), 
    .avail_byte_cnt   (outp_RxCnt),     // note bytes
    .avail_fb_empty   (outp_RxEmpty),   // truly empty and not just trig
    .rx_trig          (reg_RxTrig),     // if int FIFO
    .int_rx           (local_fb_int),
    .fb_data_use      (fb_data_use), 
    .fb_datab         (fb_datab), 
    .fb_datab_done    (fb_datab_done), 
    .fb_datab_err     (fb_datab_err), // we overload, but context separates 
    .fb_orun          (fb_orun),
    .fb_ddr_errs      (fb_ddr_errs),
    .fb_s0s1_err      (fb_s0s1_err),
    .brk_s0s1_err     (brk_s0s1_err),
    .scan_no_rst      (scan_no_rst)
  );  
  assign outp_fb_data  = localn_fb_data;
  // now frombus linkages for FIFO use if enabled
  generate if (FIFO_TYPE[`FIFO_EXT_b]==0 || EXT_FIFO==0) begin : no_fifo_fb
    assign localn_fb_ack   = |regflg_rd_cnt;
    assign oxf_fb_req      = 0;
    assign oxf_fb_data     = 0;
    assign oxf_fb_eof      = 0;
  end else begin : fifo_fb
    assign oxf_fb_req  = ixf_fb_free & localn_fb_ready; // pulse so taken
    assign oxf_fb_data = localn_fb_data; // byte to take
    assign localn_fb_ack = ixf_fb_free; // 1 if free space to take next
    // next pulses eof for 1 cycle on repeated START or STOP. Note that 
    // stop flows into start, but no issue as no data between
    SYNC_ASet_Seq2 sync_fb_eof(.CLK(PCLK), .RSTn(PRESETn), 
                               .async_set(int_start_seen | int_in_STOP), 
                               .local_clear(1'b1), .o_pulse(oxf_fb_eof));
  end endgenerate

  //
  // Data buffer management for to-bus
  // -- This handles the 2 buffer (ping pong) from one visible buffer map
  // -- Can Handle FIFO or singleton
  //
  i3c_data_tobus #(.FIFO_TYPE(FIFO_TYPE),.EXT_FIFO(EXT_FIFO),
                   .ENA_TOBUS_FIFO(ENA_TOBUS_FIFO))
    tobus 
    (
    .RSTn             (PRESETn), 
    .CLK              (PCLK), 
    .SCL              (clk_SCL), 
    .SCL_n            (clk_SCL_n),
    .avail_tb_ready   (avail_tb_ready),
    .avail_tb_data    (avail_tb_data),
    .avail_tb_ack     (avail_tb_ack),
    .avail_tb_end     (avail_tb_end),   // pulsed when last one with ready
    .tb_flush         (reg_TbFlush),
    .avail_tb_full    (outp_TxFull),    // truly full (trig or not) 
    .set_tb_urun_nack (set_tb_urun_nack),
    .set_tb_urun      (set_tb_urun),
    .set_tb_term      (set_tb_term),
    .clear_tb_urun_nack(reg_clrGenErr[2]),
    .clear_tb_urun    (reg_clrGenErr[1]),
    .clear_tb_term    (reg_clrGenErr[3]),
    .avail_byte_cnt   (outp_TxCnt),     // note bytes
    .tx_trig          (reg_TxTrig),     // trigger level if FIFO
    .int_tb           (local_tb_int),
    .tb_data_valid    (d_tb_data_valid), 
    .tb_datab         (d_tb_datab), 
    .tb_end           (d_tb_end), 
    .tb_datab_ack     (d_tb_datab_ack), 
    .tb_urun_nack     (tb_urun_nack), 
    .tb_urun          (tb_urun),
    .tb_term          (tb_term),
    .scan_no_rst      (scan_no_rst)
  );  
  // now linkages for FIFO if selected
  generate if (FIFO_TYPE[`FIFO_EXT_b]==0 || EXT_FIFO==0) begin : no_fifo_tb
    assign avail_tb_ready = |regflg_wr_cnt; 
    assign avail_tb_data  = reg_wdata[7:0];
    // avail_tb_ack not used by register interface
    assign avail_tb_end   = reg_TbEnd;  // pulsed when last one with ready
    assign oxf_tb_start   = 0;
    assign oxf_tb_used    = 0;
  end else begin : fifo_tb
    assign avail_tb_ready = ixf_tb_avail;   // byte available if 1
    assign avail_tb_data  = ixf_tb_data;    // byte to use
    assign oxf_tb_used    = avail_tb_ack;   // accepted tb byte 
    assign avail_tb_end   = ixf_tb_last;    // pulsed when last one with tb_ready
    // next is pulsed on avail and reset on next async start|stop (so can pulse again)
    // this seems to be the model wanted.
    SYNC_AClr_Seq2 sync_start(.CLK(PCLK), .RSTn(PRESETn), .local_set(ixf_tb_avail),  
                              .async_clear(int_start_seen | int_in_STOP), 
                              .o_pulse(oxf_tb_start));
  end endgenerate


  generate if (ENA_IBI_MR_HJ[`EV_EXTFIFO_b]) begin : ibi_fifo_inst
    //
    // IBI FIFO is ping-pong feed for IBI EXTDATA - if wanted
    //
    i3c_data_ibi_ext ibi_fifo
    (
      .RSTn             (PRESETn), 
      .CLK              (PCLK), 
      .SCL              (clk_SCL), 
      .SCL_n            (clk_SCL_n),
      .avail_tb_ready   (ibi_wr_fifo[8]),
      .avail_tb_data    (ibi_wr_fifo[7:0]),
      .avail_tb_end     (ibi_wr_fifo[9]),
      .avail_tb_ack     (ibi_wr_ack),
      .tb_flush         (ibi_wr_fifo[10]),
      .avail_byte_cnt   (),             // bytes - not used
      .tb_data_valid    (ibs_tb_data_valid), 
      .tb_datab         (ibs_tb_datab), 
      .tb_end           (ibs_tb_end), 
      .tb_datab_ack     (ibs_tb_datab_ack), 
      .scan_no_rst      (scan_no_rst)
    );
  end else begin
    assign ibs_tb_data_valid = 1'b0;
    assign ibs_tb_datab      = 8'd0;
    assign ibs_tb_end        = 1'b0;
    assign ibi_wr_ack        = 1'b0;
  end endgenerate
  

  //
  // The Slave instance handles all aspects of the slave in SCL domain
  //
  i3c_slave_wrapper #(.ENA_ID48B(ENA_ID48B),.ID_48B(ID_48B),
                      .ID_AS_REGS(ID_AS_REGS),.ID_BCR(ID_BCR),.ID_DCR(ID_DCR),
                      .ENA_SADDR(ENA_SADDR),.SADDR_P(SADDR_P),
                      .ENA_MAPPED(ENA_MAPPED),.MAP_CNT(MAP_CNT), .MAP_I2CID(MAP_I2CID),
                      .MAP_DA_AUTO(MAP_DA_AUTO), .MAP_DA_DAA(MAP_DA_DAA),
                      .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ),
                      .ENA_CCC_HANDLING(ENA_CCC_HANDLING),.RSTACT_CONFIG(RSTACT_CONFIG),
                      .MAX_RDLEN(MAX_RDLEN), .MAX_WRLEN(MAX_WRLEN), .MAX_DS_WR(MAX_DS_WR), 
                      .MAX_DS_RD(MAX_DS_RD), .MAX_DS_RDTURN(MAX_DS_RDTURN),
                      .ENA_HDR(ENA_HDR),.ENA_MASTER(ENA_MASTER),.PIN_MODEL(PIN_MODEL), 
                      .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
                      .SLV_DBG_MX(SLV_DBG_MX))
    slave 
    (
    .RSTn             (PRESETn), 
    .pin_SCL_in       (pin_SCL_in), 
    .pin_SCL_out      (pin_SCL_out), 
    .pin_SCL_oena     (pin_SCL_oena), 
    .pin_SDA_in       (pin_SDA_in), 
    .pin_SDA_out      (pin_SDA_out), 
    .pin_SDA_oena     (pin_SDA_oena), 
    .pin_SDA_oena_rise(pin_SDA_oena_rise), 
    .i2c_spike_ok     (i2c_spike_ok),
    .i2c_hs_enabled   (i2c_hs_enabled),
      // we hold off slave engine if not enabled, if master, or in HJ hold
    .slv_enable       (slvena_delay & is_slave & ~hold_engine), 
    .int_start_seen   (int_start_seen), 
    .int_start_err    (int_start_err), 
    .int_da_matched   (int_da_matched), 
    .int_sa_matched   (int_sa_matched), 
    .int_7e_matched   (int_7e_matched), 
    .int_ddr_matched  (int_ddr_matched),// if DDR enabled, else 0
    .int_in_STOP      (int_in_STOP), 
    .int_event_sent   (int_event_sent), 
    .int_event_ack    (int_event_ack),  // combinatorial from pin, so only use registered
    .int_in_evproc    (int_in_evproc),  // signals if in IBI now so no cancel
    .int_ccc_handled  (int_ccc_handled),// held if handled by lower block
    .int_ccc          (int_ccc),        // pulse if not handled by lower block
    .state_req        (raw_req),        // [6]=HDR, [5]=ENTDAA, [4]=Wr, [3]=Rd, [2]=CCChandle, [1]=W/R, [0]=busy
    .opt_state_AS     (raw_ActState),   // Act state if known
    .opt_ev_mask      (raw_EvState),    // Event mask if known
    .opt_TimeC        (raw_TimeC),      // one hot time control enables
    .opt_timec_sync   (raw_timec_sync[12:0]), // if SYNC handled by system (note: upd in SCL clock domain)
    .opt_slvr_reset   (raw_slvr_reset), // SlaveRst control
    .opt_match_idx    (raw_match_idx),  // map addr match if mapped used
    .i_rst_slvr_reset (iraw_rst_slvr_reset),
    .clr_slvr_cause   (clr_slvr_cause),
    .state_in_ccc     (state_in_ccc), 
    .state_in_daa     (state_in_daa), 
    .state_in_hdr     (state_in_hdr),
    .state_dir        (),               // held as in-read or in-write from header
    .dyn_addr         (raw_DynAddr), 
    .dyn_addr_chg     (dyn_addr_chg),   // pulses in SCL when DA changed
    .dyn_chg_cause    (raw_DynChgCause),
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
    .ibi_extdata      (cf_IbiExtData),
    .ibi_mapidx       (cf_IbiMapIdx),
    .ibi_in_extdata   (raw_ibi_in_extdata),
    .timec_oflow      (timec_oflow),
    .timec_oflow_clr  (timec_oflow_clr),
    .oclk_SCL         (clk_SCL),
    .oclk_SCL_n       (clk_SCL_n),
    .tb_data_valid    (s_tb_data_valid), 
    .tb_datab         (s_tb_datab), 
    .tb_end           (s_tb_end),
    .tb_datab_ack     (s_tb_datab_ack), 
    .tb_urun_nack     (tb_urun_nack), 
    .tb_urun          (tb_urun), 
    .tb_term          (tb_term),
    .fb_data_use      (fb_data_use), 
    .fb_datab         (s_fb_datab), 
    .fb_datab_done    (s_fb_datab_done), 
    .fb_datab_err     (fb_datab_err), 
    .fb_orun          (fb_orun), 
    .fb_ddr_errs      (fb_ddr_errs),
    .fb_s0s1_err      (fb_s0s1_err),
    .brk_s0s1_err     (brk_s0s1_err | abt_s0s1_err), // get out of S0/S1 err
    .rd_abort         (read_abort),
    .fb_hdr_exit      (fb_hdr_exit),
    .obit_cnt         (),               // not used
    .ccc_byte         (),               // not used
    .willbe_ccc       (),               // not used
    .cf_IdInst        (cf_IdInst), 
    .cf_IdRand        (cf_IdRand), 
    .cf_Partno        (cf_Partno), 
    .cf_IdBcr         (cf_IdBcr), 
    .cf_IdDcr         (cf_IdDcr),
    .cf_IdVid         (cf_IdVid),
    .cf_MaxRd         (cf_MaxRd),
    .cf_MaxWr         (cf_MaxWr),
    .cf_RstActTim     (cf_RstActTim),
    .cf_SlvNack       (cf_SlvNack),
    .cf_SdrOK         (1'b1),           // if 0, i2c only
    .cf_DdrOK         (cf_DdrOK), 
    .cf_TspOK         (cf_TspOK), 
    .cf_TslOK         (cf_TslOK),
    .cf_TCclk         (cf_TCclk),
    .cf_s0ignore      (cf_s0ignore),
    .cf_SetDA         (cf_SetDA),       // used for Main Master and override
    .cf_SetMappedDASA (cf_SetMappedDASA),// only if mapped SA/DA supported
    .cf_SetSA10b      (cf_SetSA10b),    // if [1] is SA 10-bit
    .cf_HdrCmd        (cf_HdrCmd),      // want HDR Cmd in MMR
    .cf_vgpio         (cf_vgpio),       // if VGPIO is CCC and match CCC
    .cf_CccMask       (cf_CccMask),     // to mask off unhandled CCCs
    .cf_MasterAcc     (cf_MasterAcc),   // only for M+S, slave OK to accept Mastership
    .map_daa_use      (map_daa_use),    // if MAP has auto-DAA regs
    .map_daa_dcr      (map_daa_dcr),    // "
    .map_daa_pid      (map_daa_pid),    // "
    .opt_MasterAcc    (opt_MasterAcc),  // held in SCL if master accepted
    .raw_vgpio_done   (raw_vgpio_done), // only used if ENA_MAPPED VGPIO
    .raw_vgpio_byte   (raw_vgpio_byte),
    .raw_hdr_newcmd   (scl_hdr_new_cmd),// pulses if new CMD and enabled for local
    .raw_hdr_cmd      (scl_hdr_cmd),    // synchronized by new_cmd
    .raw_setaasa      (raw_setaasa),    // sync if used
    .raw_rstdaa       (raw_rstdaa),     // sync if used
    .raw_daa_ena      (raw_daa_ena),    // pulse if DASA/DAA for MAP
    .map_sa_idx       (map_sa_idx),     // index in map if daa_ena
    .map_daa_da       (map_daa_da),     // new DA if daa_ena for map
    .i2c_dev_rev      (i2c_dev_rev),    // if extended i2c
    .i2c_sw_rst       (i2c_sw_rst),     // "
    .sraw_ActMode     (sraw_ActMode),
    .sraw_PendInt     (sraw_PendInt),
    .sraw_StatusRes   (sraw_StatusRes),
    .opt_ChgMaxRd     (flg_maxrd),
    .opt_ChgMaxWr     (flg_maxwr),
    .opt_MaxRdWr      (outp_MaxRW),
    .slv_debug_observ (slv_debug_observ),
    .scan_single_clock(scan_single_clock), 
    .scan_clk         (scan_clk), 
    .scan_no_rst      (scan_no_rst), 
    .scan_no_gates    (scan_no_gates)
  );
  assign raw_timec_sync[13] = clk_SCL; // clock if sync TC used in sys
  assign raw_Request = {raw_req[6:4], raw_req[3]&~raw_req[6],raw_req[2:0]};
  assign raw_matched = int_da_matched | int_sa_matched;

  //
  // CDC controls for various nets
  // -- handle registration and sync between SCL/SCL_n and PCLK
  // -- note that raw_request and opt_state_AS/opt_ev_mask are only 
  //    registedr on status read
  //
    // these below handle the clock crossing but are special. The
    // issue is that SCL is not free running. So, we use 2-phase
    // but then need another flop so not propagating the metastable
    // comparison. So, the "st_xxx" flops are used. Note that the
    // data ones are in tobus and frombus
  wire         st_start, st_start_err, st_stop, st_dachg, st_ccchand;
  wire         st_ddrmatch, st_ccc, st_event, st_slvrst;
  wire   [2:0] st_matched;
  wire  [19:8] IntStates;

  // IntStates are as follows:
  // [8]=START --> SCL
  assign IntStates[`IS_START]   = st_start;
  // [9]=MATCHED --> SCL
  assign IntStates[`IS_MATCHED] = |st_matched[1:0]; // DA and SA only
  // [10]=STOP --> SCL
  assign IntStates[`IS_STOP]    = st_stop;
  // [11]=RXPEND==From-bus --> our domain (CDC in fb block)
  assign IntStates[`IS_RXPEND]  = local_fb_int;
  // [12]=TXPEND==To-bus  --> our domain (CDC in tb block)
  assign IntStates[`IS_TXPEND]  = local_tb_int;
  // [13]=DACHG --> SCL
  assign IntStates[`IS_DACHG]   = st_dachg;
  // [14]=CCC --> SCL
  assign IntStates[`IS_CCC]     = st_ccc;
  // [15]=ErrWarn --> mix including from engine and from above
  assign IntStates[`IS_ERRWARN] = |(outp_GenErr & msk_GenErr) | 
                                  (|(outp_DataErr & msk_DataErr)) |
                                  reg_holdOErr;
  // [16]=DDRMATCH --> SCL
  assign IntStates[`IS_DDRMATCH]= st_ddrmatch;
  // [17]=CHANDLED --> SCL
  assign IntStates[`IS_CHANDLED]= st_ccchand;
  // [18]=EVENT --> SCL
  assign IntStates[`IS_EVENT]   = st_event;
  // [19]=SLVRST --> SCL
  assign IntStates[`IS_SLVRST]   = st_slvrst;

  assign outp_IntStates = IntStates;
  assign outp_GenErr    = {1'b0, st_start_err,set_tb_term,set_tb_urun_nack,
                          set_tb_urun,set_fb_orun};
  assign outp_DataErr   = {set_fb_err[3:1], // DDR framing/parity, CRC, S0/S1
                           set_fb_err[0]|set_rd_err}; // SDR parity and read abort

  // note use of SCL vs. SCL_n to pick up the pulse 1/2 cycle earlier
  
  // NOTE: these next ones are a bit confusing. The scl_  to clk_
  //       is 2-phase. The clk_ domain handshake waits for the 
  //       st_ flags to go to 1 and then they change. This is to avoid
  //       the case of clk_ changing too fast. This is all done because
  //       SCL is not free running and so may stop. PCLK is going to
  //       run longer and so we can flip to match.

  // We sync start and start error (since cleared by start)
  // 2-phase between clk/scl
  // Note naming: SYNC_= synchronizing, 2PH_=2-phase, S2P_=SCL to CLK, STATE=state with clr in
  //              LVL_=level trigger, so extra flop needed to detect edge
  //              LVLH_=level trigger when high
  SYNC_2PH_S2C_STATE synch_int_start(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                     .trig_scl(int_start_seen & (~cf_matchss|(|st_matched[1:0]))), 
                                     .out_clk(st_start), .clear_clk(reg_clrIntStates[`IS_START]));
     // start error is SCL changing before SDA and is simply noted
  wire starterr_rst_n = PRESETn & (~fb_hdr_exit | scan_no_rst);// Exit looks like start err
  `Observe(observe_starterr_rst_n, clk_SCL, ~fb_hdr_exit) // optional DFT observer
  SYNC_2PH_LVLH_S2C_STATE synch_int_start_err(.rst_n(starterr_rst_n), .scl(clk_SCL), .clk(PCLK), 
                                         .trig_scl(int_start_err), 
                                         .out_clk(st_start_err), .clear_clk(reg_clrGenErr[4]));

  // now matched (DA or SA) - we keep all 3 sep for nets
  // again 2-phase
  SYNC_2PH_S2C_STATE synch_da_match(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                    .trig_scl(int_da_matched), 
                                    .out_clk(st_matched[0]), .clear_clk(reg_clrIntStates[`IS_MATCHED]));
  SYNC_2PH_S2C_STATE synch_sa_match(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                    .trig_scl(int_sa_matched), 
                                    .out_clk(st_matched[1]), .clear_clk(reg_clrIntStates[`IS_MATCHED]));
    // note that 7E is cleared by matched bit from STATUS even though not used by status
  SYNC_2PH_S2C_STATE synch_7e_match(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                    .trig_scl(int_7e_matched), 
                                    .out_clk(st_matched[2]), .clear_clk(reg_clrIntStates[`IS_MATCHED]));

  // STOP detect is from seeing stop. But, note that in_STOP is
  // held, so we have to use LVL sync to track the change.
  SYNC_2PH_LVLH_S2C_STATE synch_stop(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                    .trig_scl(int_in_STOP&(~cf_matchss|(|st_matched[1:0]))),// held vs. pulse
                                    .out_clk(st_stop), .clear_clk(reg_clrIntStates[`IS_STOP]));

  // DA changed. Old method did not catch SETNEWDA, so we now export change itself
  SYNC_2PH_S2C_STATE synch_dachg(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                      .trig_scl(dyn_addr_chg),  
                                      .out_clk(st_dachg), .clear_clk(reg_clrIntStates[`IS_DACHG]));

  // DDR matched when command has our DA
  SYNC_2PH_S2C_STATE synch_ddrmatch(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                    .trig_scl(int_ddr_matched),
                                    .out_clk(st_ddrmatch), .clear_clk(reg_clrIntStates[`IS_DDRMATCH]));

  // CCC handled by lower block is an interrupt, whereas in_CCC is a state
  // So, we again use LVL since handled is held while true
  SYNC_2PH_LVLH_S2C_STATE synch_ccchand(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                       .trig_scl(int_ccc_handled),  // held vs. pulse
                                       .out_clk(st_ccchand), .clear_clk(reg_clrIntStates[`IS_CHANDLED]));

  // CCC not handled is an interrupt and level, although it can end sooner
  SYNC_2PH_LVLH_S2C_STATE synch_ccc(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                       .trig_scl(int_ccc),  // held vs. pulse
                                       .out_clk(st_ccc), .clear_clk(reg_clrIntStates[`IS_CCC]));

  generate
    if (RSTACT_CONFIG[`RSTA_ENA_b]) begin : slvrst_irq
      // SlaveReset block reset - sync into PCLK domain for reg
      SYNC_2PH_LVLH_S2C_STATE synch_slvrst(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK),
                                           .trig_scl(iraw_slvr_irq), .out_clk(st_slvrst),
                                           .clear_clk(reg_clrIntStates[`IS_SLVRST]));
      SYNC_AClr_C2S sync_slvrst_clr(.CLK(PCLK), .RSTn(PRESETn), 
                                    .local_set(reg_clrIntStates[`IS_SLVRST]), 
                                    .async_clear(1'b1), .o_value(clr_slvr_cause));
    end else begin
      assign st_slvrst      = 1'b0;     // not used
      assign clr_slvr_cause = 1'b0;     // "
    end
  endgenerate 

  generate
    if (ENA_MAPPED[`MAP_VGPIO_b]) begin : vgpio_sync
      SYNC_2PH_LVLH_S2C_STATE sync_vgpio_done(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK),
                                           .trig_scl(raw_vgpio_done), .out_clk(vgpio_done),
                                           .clear_clk(vgpio_done)); // pulse - self clear
    end else begin
      assign vgpio_done = 1'b0;         // not used
    end
  endgenerate 

  generate 
    // MAX limits on message sizes
    if (ENA_CCC_HANDLING[`ENCCC_MAXES_b]) begin : maxes_sync
      SYNC_Pulse_S2C sync_maxrd(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                .local_set(flg_maxrd), .o_pulse(outpflg_MaxRd));
      SYNC_Pulse_S2C sync_maxwr(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                .local_set(flg_maxwr), .o_pulse(outpflg_MaxWr));
    end else begin
      assign outpflg_MaxRd = 0;
      assign outpflg_MaxWr = 0;
    end
  endgenerate 

  // Now to-master transition from GETACCMST
  generate
    if (ENA_MASTER) begin : mstacc_sync
      // we get opt_MasterAcc from SCL domain to indicate accpeted
      // GETACCMST command. But, we cannot do anything with it until
      // we see a stop. Input clears when we change cf_MasterAccept
      // Note that input will clear on Sr as well (as it should)
      SYNC_S2C sync_to_mast(.rst_n(PRESETn), .clk(PCLK), 
                            .scl_data(opt_MasterAcc&int_in_STOP), 
                            .out_clk(outp_to_master));
    end else begin
      assign outp_to_master = 1'b0; // not used if not master
    end


    if (ID_AS_REGS[`IDREGS_HDRCMD_b]) begin : hdrcmd_new
      SYNC_Pulse_S2C sync_hdr_new(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                  .local_set(scl_hdr_new_cmd),
                                  .o_pulse(hdr_new_cmd));
      // now we register a copy of the 8 bits for the register. This is in SCL
      // domain so we do not worry about the PCLK freq
      reg [7:0] hdr_cmd_hold;
      always @ (posedge clk_SCL or negedge PRESETn)
        if (!PRESETn)
          hdr_cmd_hold <= 8'd0;
        else if (scl_hdr_new_cmd)
          hdr_cmd_hold <= scl_hdr_cmd; 
      assign raw_hdr_cmd = hdr_cmd_hold;
    end else begin
      assign hdr_new_cmd = 1'b0;
      assign raw_hdr_cmd = 8'd0;
    end

    if (MAP_DA_AUTO[`MAPDA_AASA_b]) begin : syn_map_setaasa
      SYNC_Pulse_S2C sync_setaasa(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                  .local_set(raw_setaasa), 
                                  .o_pulse(map_setaasa));
    end else begin
      assign map_setaasa = 1'b0;
    end

    if (|MAP_DA_AUTO[2:0]) begin : syn_map_rstdaa
      SYNC_Pulse_S2C sync_rstdaa(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                  .local_set(raw_rstdaa), 
                                  .o_pulse(map_rstdaa));
    end else begin
      assign map_rstdaa = 1'b0;
    end

    if (MAP_DA_AUTO[`MAPDA_DASA_b]|MAP_DA_AUTO[`MAPDA_DAA_b]) begin : map_da_ena
      SYNC_Pulse_S2C sync_mapdaaena(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                  .local_set(raw_daa_ena), 
                                  .o_pulse(map_daa_ena));
    end else begin
      assign map_daa_ena = 1'b0;
    end
  endgenerate


  //
  // Event management within CDC
  //

  // This portion will convert an event request between CLK and SCL
  generate 
    if (ENA_IBI_MR_HJ != 0) begin : sync_events

      // next two hold pulse from scl_n
      reg              scl_event_sent;
      reg              scl_event_ack;
      reg        [2:0] clk_ev_det_r;    // details passed up into CLK domain
      wire       [1:0] ev_det_as_code, pclk_ev_det_as_code;
      reg              ev_det_chg;
      wire       [7:0] timec_ibi_byte;

      // translate the request down into SCL_n domain
      // we do not register since 1st SCL will be START, which we need
      // instead, [2] of the net is delayed by 1 CLK, so [1:0] are
      // safe if it is 1. Further, we only strobe it to SCL domain when
      // in STOP, so it is only visible in 1st cycle of START; so seen
      // or missed; if missed, seen next START (or we force).
      // only allow flag of valid on 1st cycle of START
      // note: [1:0]=1 if IBI, 2=MR, 3=HJ
      wire        allow_ev = ((reg_EvPend[1:0]==2'd3) & raw_EvState[27]) |
                             ((reg_EvPend[1:0]==2'd2) & raw_EvState[25]) |
                             ((reg_EvPend[1:0]==2'd1) & raw_EvState[24]);
      assign event_pending = {(reg_EvPend[2]&int_in_STOP&allow_ev), 
                               (reg_EvPend[1:0] & {2{allow_ev}})};

      // Note that sent and ACK are registered on same clock edges, but
      // we resolve the potential metastability issue by having the PCLK
      // domain take 2 of its cycles to pick up the change.
      // These are cleared when registered into other clock domain.
      // Note that int_event_ack is combinatorial on pin, so must only
      // be used in registered scl_event_ack form. 
      wire ev_rst_n = PRESETn & (~clk_ev_det_r[2] | scan_no_rst);
      always @ (posedge clk_SCL or negedge ev_rst_n)
        if (!ev_rst_n) begin
          scl_event_sent <= 1'b0;
          scl_event_ack  <= 1'b0;
        end else if (int_event_sent) begin
          scl_event_sent <= 1'b1;
          scl_event_ack  <= int_event_ack;
        end else if (~scl_event_sent & scl_event_ack)
          scl_event_ack  <= 1'b0;       // avoid metastable timing on clear
      // we decode the return. Note that once event_pending goes back to
      // 0, we clear the event_sent data
      assign ev_det_as_code = (scl_event_sent & scl_event_ack)       ? 2'd3 :
                              (scl_event_sent & ~scl_event_ack)      ? 2'd2 :
                              (~scl_event_sent& |event_pending[1:0]) ? 2'd1 :
                                                                       2'd0;

      // this next just synchronizes between SCL and PCLK domain. But, does
      // not try to deal with metastability between two values. We do that
      // by picking up change in one PCLK cycle and then updating in 2nd.
      // Since input does not change until clk_ev_det_r[2] does, we have
      // the time (and the SCL is unpredictable when it will happen).
      SYNC_S2C #(.WIDTH(2)) sync_ev_det_as_code(.rst_n(PRESETn), .clk(PCLK), 
                                        .scl_data(ev_det_as_code), 
                                        .out_clk(pclk_ev_det_as_code));
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          ev_det_chg <= 1'b0;
        else if (|(clk_ev_det_r[1:0] ^ pclk_ev_det_as_code))
          ev_det_chg <= 1'b1;
        else
          ev_det_chg <= 1'b0;

      // now translate the details back into PCLK domain
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          clk_ev_det_r      <= 3'd0;
        else if (&clk_ev_det_r[1:0]) begin // done, so wait for them to write new
          if (~clk_ev_det_r[2])
            clk_ev_det_r[2] <= 1'b1;    // buy a cycle for reg to self clear
          else if (reg_EvPend[2])       // clear when they write new
            clk_ev_det_r    <= 3'd0;
        end else if (reg_EvPend == 3'b100) // clear state req
          clk_ev_det_r      <= 3'd0;
        else if (ev_det_chg)
          if (~|clk_ev_det_r[1:0])
            clk_ev_det_r    <= {1'b0,pclk_ev_det_as_code};// simply reflect it back up
          else if (pclk_ev_det_as_code[1])
            clk_ev_det_r    <= {1'b0,pclk_ev_det_as_code};// pick up NACK or ACK
      assign outp_EvDet    = clk_ev_det_r[2:0];

      // st_event
      SYNC_2PH_LVLH_S2C_STATE synch_event(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                          .trig_scl(&ev_det_as_code), // held vs. pulse
                                          .out_clk(st_event), .clear_clk(reg_clrIntStates[`IS_EVENT]));

      // sync no cancel of IBI (or MR)
      SYNC_S2C sync_no_cancel (.rst_n(PRESETn), .clk(PCLK), .scl_data(int_in_evproc), 
                               .out_clk(outp_EvNoCancel));


      // now the counters for IBI, MR, S0/S1 breakout, Read stall detect, and HJ
      wire syn_s0s1;
      wire syn_inread;
      wire raw_rd_abort;
      wire sync_gate;
      reg  enabled;
      wire delay_ena, exit_pulse, delay_ena_done;
        // S0S1 error is synchronized to slow clock
      SYNC_S2C sync_s0s1_err(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(fb_s0s1_err), 
                             .out_clk(syn_s0s1));
        // slave_enable from offline uses S0/S1 type delay to lock onto bus
      // slave enable delay simply exists to allow offline reacquire
      always @ (posedge PCLK or negedge PRESETn)
        if (~PRESETn) begin
          enabled <= 1'b0;
        end else if (cf_SlvEna ^ enabled) begin
          if (~cf_Offline | ~cf_SlvEna) begin
            enabled <= cf_SlvEna;
          end else if (delay_ena_done | exit_pulse) begin
            enabled <= 1'b1;
          end
        end
      assign slvena_delay = enabled;
      // next indicates if trying to turn on SlaveEna with Offline, so we
      // wait for 60us of no bus activity or ExitPattern
      SYNC_S2C ena_ol_sync(.rst_n(PRESETn), .clk(CLK_SLOW), 
                           .scl_data(cf_SlvEna & (cf_SlvEna^enabled)), 
                           .out_clk(delay_ena));
        // hdr_exit is a pulse in SDA domain, so we use as a reset 
      wire exit_rst_n = PRESETn & (~fb_hdr_exit | scan_no_rst);
      reg exit_match;
      always @ (posedge PCLK or negedge exit_rst_n)
        if (!exit_rst_n) 
          exit_match <= 1'b1;
        else if (exit_match)
          exit_match <= 1'b0;
      assign exit_pulse = exit_match;
      SYNC_C2S done_60_sync(.rst_n(PRESETn), .scl(PCLK), .clk_data(abt_s0s1_err), 
                            .out_scl(delay_ena_done));

        // in read and not i2c is synchronized to slow clock - SDR and HDR
      SYNC_S2C sync_in_read(.rst_n(RSTn), .clk(CLK_SLOW), 
                            .scl_data(raw_DynAddr[0]&raw_req[3]), 
                            .out_clk(syn_inread));
        // read abort causes reset of state machine, so no sync. Note HDR and SDR read
      assign read_abort = ERROR_HANDLING[`ERR_RDABT_b] & raw_rd_abort & raw_req[3]; 
      SYNC_2PH_S2C_STATE synch_fb_err0(.rst_n(RSTn), .scl(CLK_SLOW), .clk(PCLK), 
                                   .trig_scl(ERROR_HANDLING[`ERR_RDABT_b] & raw_rd_abort), 
                                   .out_clk(set_rd_err), .clear_clk(reg_clrDataErr[8]));
        // slow gate has to allow clock for sync above
        // So combination of synchronized and unsync
      wire raw_gate = ~(cf_SlvEna & (cf_SlvEna^enabled)) & ~fb_s0s1_err &
                      ~(raw_DynAddr[0]&raw_req[3]) & 
                      ~(ERROR_HANDLING[`ERR_RDABT_b] & raw_rd_abort);
      assign slow_gate = sync_gate & raw_gate;


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
          .run_60           (syn_s0s1 | delay_ena), 
          .run_100          (syn_inread & ERROR_HANDLING[`ERR_RDABT_b]),// in read for i3c or DDR
          .int_in_STOP      (int_in_STOP), 
          .force_sda        (force_sda),
          .done_60          (abt_s0s1_err), // no sync needed since reset
          .done_100         (raw_rd_abort), 
          .hold_engine      (hold_engine)   // hold engine off until HJ emits start
        );

      // IBI byte is not possible unless IBI
      if (ENA_IBI_MR_HJ[`EV_IBI_DAT_b]) begin : ibi_byte
        assign ibi_has_byte  = 1'b1;
        assign opt_ibi_byte  = ~|ibi_timec_sel[1:0] ? 
                               // IBI byte always used 1st 
                               (reg_EvIbiByte | (ibi_timec?8'h80:8'h00)) : 
                               // time c byte changes by mux
                               timec_ibi_byte;
      end else begin
        assign ibi_has_byte  = 1'b0;
        assign opt_ibi_byte  = 8'd0;
      end

      // next pulls in time control if enabled (only with IBI of course)
      if (|ENA_TIMEC) begin : timec_handling
        wire                    ev_pend;
        wire                    time_oflow;
        wire                    syn_tc_gate;
        // sync to SLOW clock from PCLK - 1 bit as true or not true
        // is held signal
        SYNC_C2S #(.WIDTH(1)) sync_IBI_pend(.rst_n(RSTn), .scl(CLK_SLOW_TC), 
                                    .clk_data(event_pending[0]), .out_scl(ev_pend));
        SYNC_AClr_C2S sync_tcoflow(.CLK(CLK_SLOW_TC), .RSTn(RSTn), .local_set(time_oflow), 
                                   .async_clear(timec_oflow_clr), .o_value(timec_oflow));
        // CLK_SLOW_TC is not gated if needed for sync as well as results
        // note timec_oflow waits for clear (set when timing).
        assign tc_slow_gate = syn_tc_gate & ~event_pending[0] & ~timec_oflow;

        i3c_time_control time_ctrl(
          .RSTn          (RSTn), 
          .CLK_SLOW      (CLK_SLOW_TC), 
          .clk_SCL_n     (clk_SCL_n), 
          .timec_ena     (raw_TimeC),   // what is enabled - one hot
          .event_start   (ev_pend),     // start - held while true
          .was_nacked    (clk_ev_det_r[2:0]==3'b010),// if NACKed
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
      assign outp_EvDet    = 3'd0;
      assign st_event      = 1'b0;
      assign slow_gate     = 1'b1;      // never used
      assign tc_slow_gate  = 1'b1;      // never used
      assign ibi_has_byte  = 1'b0;
      assign opt_ibi_byte  = 8'd0;
      assign ibi_timec     = 1'b0;
      assign timec_oflow   = 1'b0;
      assign abt_s0s1_err  = 1'b0;
      assign read_abort    = 1'b0;
      assign set_rd_err    = 1'b0;      // no read abort err
      assign slvena_delay  = cf_SlvEna; // no offline
    end

  endgenerate

  //  
  // logic below is a form of assertion to catch bad parameters or combinations
  // The sim or sythesis compiler will issue warnings on out-of-bounds
  //

  // ID must be some mix of constant and if some regs, then mux must match
  // Note that VID must not be 0 even if reg, since that forms the reset value
  localparam ID_TST = ~|ENA_ID48B || ENA_ID48B>`ID48B_CONST_NONE || ID_48B==48'd0 ||
                      (ENA_ID48B==`ID48B_CONST_NONE && ~ID_AS_REGS[`IDREGS_VID_b]) ||
                      (ENA_ID48B<=`ID48B_CONST_INST && ID_48B[32]);
  wire [0:0] bad_id;
  assign bad_id[ID_TST] = 1'b0;
  localparam IAR_TST = |ID_AS_REGS[11] || 
                       ID_AS_REGS[`IDREGS_INST_b]&(ENA_ID48B!=`ID48B_CONST_INST) ||
                       ID_AS_REGS[`IDREGS_RAND_b]&(ENA_ID48B< `ID48B_CONST_PARTNO) ||
                       ID_AS_REGS[`IDREGS_VID_b] &(ENA_ID48B!=`ID48B_CONST_NONE) ||
                       ID_AS_REGS[`IDREGS_DAWR_b]&|ENA_MASTER ||
                       (ID_AS_REGS[`IDREGS_VGPIO_b]&~ENA_MAPPED[`MAP_VGPIO_b]) ||
                       ID_AS_REGS[`IDREGS_SLVENA_b]&|ENA_MASTER; // SLVENA not OK for M+S
  wire [0:0] bad_id_as_reg;
  assign bad_id_as_reg[IAR_TST] = 1'b0;
    // note cannot check DCR since allowed to be 0
    // below says if constant BCR, then has to be valid
  localparam BCR_TST = ~ID_AS_REGS[`IDREGS_BCR_b] &
                       (ID_BCR[7:6]!=(ENA_MASTER?2'b01:2'b00) || // must match
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
                             (MAX_DS_WR>7||MAX_DS_RD>63||MAX_DS_RDTURN>=(1<<24))) ||
                         // If v1.1 then SlaveReset is required (and GETCAPS)
                       (ENA_CCC_HANDLING[`ENCCC_V11MIN]!=RSTACT_CONFIG[`RSTA_ENA_b]);
  wire [0:0] bad_ccc;
  assign bad_ccc[CCC_TST] = 1'b0;
  // time control - below async mode 0 if sync - we enforce for Slave
  localparam TIMEC_TST = (ENA_TIMEC[2] & ~ENA_TIMEC[1]) |
                         (ENA_TIMEC[0] & ~ENA_TIMEC[1]) | 
                         (|ENA_TIMEC[4:3]);
  wire [0:0] bad_timec;
  assign bad_timec[TIMEC_TST] = 1'b0;
  // bus info
  localparam SELBUS_TST = (SEL_BUS_IF>=(1<<5));
  wire [0:0] bad_selbus;
  assign bad_selbus[SELBUS_TST] = 1'b0;
  // FIFO type
  localparam FIFO_TST = FIFO_TYPE>=(1<<4) || &FIFO_TYPE[1:0] ||
                        (FIFO_TYPE[2] & ~FIFO_TYPE[0]) ||
                        (FIFO_TYPE[3] & FIFO_TYPE[0]) ||
                        (~|FIFO_TYPE[1:0] & |(ENA_TOBUS_FIFO|ENA_FROMBUS_FIFO)) ||
                        (FIFO_TYPE[0] & ~|(ENA_TOBUS_FIFO|ENA_FROMBUS_FIFO)) ||
                        (~FIFO_TYPE[`FIFO_EXT_b] & |EXT_FIFO) ||
                        (FIFO_TYPE[`FIFO_EXT_b] & ~|EXT_FIFO) || // have to say which
                        EXT_FIFO>2;
  wire [0:0] bad_fifo;
  assign bad_fifo[FIFO_TST] = 1'b0;
  // pin model. No ext reg if master.
  localparam PINMODEL_TST = (PIN_MODEL>2) || (PIN_MODEL[1]&|ENA_MASTER);
  wire [0:0] bad_pin_model;
  assign bad_pin_model[PINMODEL_TST] = 1'b0;

  // HDR only allows DDR but not TSP or TSL for now
  localparam HDR_TST = |ENA_HDR[2:1];
  wire [0:0] bad_hdr;
  assign bad_hdr[HDR_TST] = 1'b0;

  // Mapped for slave
  localparam MAP_TST = (~ENA_MAPPED[0] & |ENA_MAPPED[4:1]) ||
                       (|MAP_CNT[3:1] & ~ENA_MAPPED[0]) ||
                       (MAP_CNT>8) ||
                       (|MAP_I2CID & ~ENA_MAPPED[0]) ||
                         // next prevents 10b addr and VGPIO at same time
                       (ENA_MAPPED[`MAP_I2C_SA10_b] & ENA_MAPPED[`MAP_VGPIO_b]) ||
                       (ENA_MAPPED[1] & ~|MAP_CNT) ||
                       (~ENA_MAPPED[0]&|MAP_DA_AUTO[12:9]) ||
                       ((~MAP_DA_AUTO[2]|MAP_DA_AUTO[6])&|MAP_DA_DAA) ||
                       (~MAP_DA_AUTO[2]&|MAP_DA_AUTO[12:9]) ||
                       (MAP_DA_AUTO[6]&(MAP_DA_AUTO[12:8]>18)) ||
                       (&MAP_DA_AUTO[7:6]&(MAP_DA_AUTO[12:8]>10));
  wire [0:0] bad_map;
  assign bad_map[MAP_TST] = 1'b0;

  //
  // Special assertions to catch free version using params not supported
  //
  //FREE_VERSION_CUT - check for params not supported in free version
  `ifndef IS_NOT_FREE
   // assert on unsupported params. Need to check if complete list
   localparam FREE_TST = |ENA_MAPPED[`MAP_VGPIO_b:`MAP_I2C_SA10_b] ||
                         (|MAP_DA_AUTO[`MAPDA_DAA_DCR_b:`MAPDA_DASA_b]) ||
                         (|ENA_HDR) || (|ENA_TIMEC);
   //TODO: need DMA test since no DMA allowed
   wire [0:0] not_in_free;
   assign not_in_free[FREE_TST] = 1'b0;
  `endif


endmodule
