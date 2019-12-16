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
//  File            : i3c_slave_wrapper.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Wed Dec 11 18:20:40 2019 $
//  Revision        : $Revision: 1.75.1.1 $
//
//  IP Name         : i3c_slave_wrapper
//  Description     : MIPI I3C Slave wrapper purely in SCL clock domain
//    This is the slave wrapper. It handles all of the pieces of the
//    slave operation, but not the system side. The outer system wrappers
//    are used such that only one is selected depending on whether
//    a bus is used or not, and whether Master and Slave or just Slave.
//    This slave wrapper uses the parameters of its instantiation to 
//    determine what to instantiate and glue together (e.g. HDR-DDR). 
//    There are some  required pieces such as slave engine, exit detector, 
//    slave DAA  handling, and minimal CCC handling (for DA use).
//    Note that CDC handling is always above this level.
//
//  ----------------------------------------------------------------------------
//                    Revision History
//  ----------------------------------------------------------------------------
//
//
//  ----------------------------------------------------------------------------
//                    Implementation details
//  ----------------------------------------------------------------------------
//  See the programmer's model spec as well as MIPI I3C spec
//
//    *****************
//    CRITICAL NOTE!!!!
//    *****************
//    It is important to replace CLOCK_SOURCE_xxx modules with uses
//    of protected logic which is clock appropriate in being glitch
//    free and in the case of an FPGA is a suitable clock buffer.
//    The backend (PD) tools should use CLOCK_SOURCE_xx in the 
//    constraints for both sourcing for clock nets, but also for
//    clock relationships, so the tools do not waste effort. That is,
//    clk_SCL and clk_SCL_n are of course related by being 1/2 clock 
//    apart (so 180 phase). clk_SDA is async to clk_SCL but is also
//    "safe" with regards to pin_SCL_in. 
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_slave_wrapper #(
    // params are driven from upper layer to control how this block
    // is built and operates
    parameter ENA_ID48B       = `ID48B_CONST, // const vs. reg
    parameter  ID_48B         = 48'h0,  // must be filled in from above with some/all of 48-bit ID
    parameter ID_AS_REGS      = 12'd0,  // [0]=IDINST,[1]=ISRAND,[2]=IDDCR,[3]=IDBCR,[4]=VID,[5]=DAwr
    parameter  ID_BCR         = 8'd0,   // filled in from above with BCR if not from regs
    parameter  ID_DCR         = 8'd0,   // filled in from above with DCR if not from regs
    parameter ENA_SADDR       = `SADDR_NONE, // none, const, reg/net
    parameter  SADDR_P        = 0,      // 7 bits as 7:1
    parameter ENA_MAPPED      = 5'd0,   // if extra DAs/SAs allowed, and related
    parameter  MAP_CNT        = 4'd1,   // number of extra DAs/SAs allowed
    parameter  MAP_I2CID      = 24'd0,// !=0 if I2C extended with DevID
    parameter  MAP_DA_AUTO    = {5'd0,1'b0,1'b0,3'd0,1'b0,1'b0,1'b0},
    parameter  MAP_DA_DAA     = 0,      // if not MMR and PID/DCR !=0, is bit array      
    parameter ENA_IBI_MR_HJ   = 0,      // 0 if no events, else events as mask
    parameter ENA_CCC_HANDLING= 6'd0,   // passed down as to what to support
    parameter RSTACT_CONFIG   = 26'd0,  // Slave Reset RSTACT CCC config (see RSTA_xxx_b fields)
    parameter MAX_RDLEN       = 0,      // default S->M len max
    parameter MAX_WRLEN       = 0,      // default M->S len max
    parameter MAX_DS_WR       = 0,      // data speed limits for M->S
    parameter MAX_DS_RD       = 0,      // data speed limits for S->M
    parameter MAX_DS_RDTURN   = 0,      // latency needs for S->M read req
    parameter ENA_HDR         = 0,      // HDR modes support
    parameter ENA_MASTER      = 0,      // If master - for CCC support
    parameter PIN_MODEL       = `PINM_COMBO,  // combinatorial pin use
    parameter ENA_TIMEC       = 6'b000010,    // if reg, res, res, mode 1, mode 0, sync
    parameter TIMEC_FREQ_ACC  = {8'd24,8'd10},// freq=12MHz (12.0=24) with 1.0% accuracy
    parameter priv_sz         = PIN_MODEL[1] ? (ENA_HDR[0] ? 3 : 1) : 0,// wider if ext-pad+ddr
    parameter [7:0] PID_CNT   = MAP_DA_AUTO[`MAPDA_DAA_PID_lb+4:`MAPDA_DAA_PID_lb], // computed
    parameter SLV_DBG_MX      = 0       // debug observer
  )
  (
  input               RSTn,             // master Reset
  // need system slow clock and optional high speed clock
  // if bus, then bus interface as well
  // -- else output SCL as clock

  // define pins - slave treats SCL as input only except ternary
  input               pin_SCL_in,       // SCL: normally clock in, data if ternary
  output              pin_SCL_out,      // SCL: only driven if ternary
  output              pin_SCL_oena,     // SCL: only driven if ternary
  input               pin_SDA_in,       // SDA: M->S
   // next 3 are used special when PIN_MODEL==`PINM_EXT_REG
  output              pin_SDA_out,      // SDA: S->M on read
  output              pin_SDA_oena,     // SDA: S->M on read
  output  [priv_sz:0] pin_SDA_oena_rise,// special on SCL rising if EXT_REG, else 0
    // next one is only used if SDA & SCL pads have i2c 50ns Spike 
    // filters and they can be turned on/off via net.
  output              i2c_spike_ok,     // Is 1 to allow i2c spike filter
  output              i2c_hs_enabled,   // If enabled, indicates in I2C HS mode
  // the main slave enable is the release from the registers (if any)
  input               slv_enable,       // held 1 if enabled
  // now some signals that indicate states to allow upper layer or system
  // act on them or interrupt. Sequence is int_start_seen, then possibly one of the
  // matching addresses (and fb_data_done), then data and possibly state_in_CCC, and
  // finally in_STOP. Note that DA changed int in upper layer
  output              int_start_seen,   // 1 cycle pulse when START or repeated START seen
  output              int_start_err,    // held when SDA=1 after STOP - cleared on START
  output              int_da_matched,   // 1 cycle pulse if matched our dynamic address
  output              int_sa_matched,   // 1 cycle pulse if matched our static address (if any)
  output              int_7e_matched,   // 1 cycle pulse if matched i3c broadcast address
  output              int_ddr_matched,  // 1 cycle pulse if matched DDR command to DA
  output              int_in_STOP,      // STOP state (held until START)
  output              int_event_sent,   // 1 cycle pulse when event (event_pending) out
  output              int_event_ack,    // 1 cycle pulse with int_event_sent if ACK vs. NACK
  output              int_in_evproc,    // 1 if processing IBI now
  output              int_ccc_handled,  // held if CCC handled by us
  output              int_ccc,          // 1 cycle pulse if CCC not handled by us
  output              state_in_ccc,     // held while true - context
  output              state_in_daa,     // held while true - informative
  output              state_in_hdr,     // held while true - informative
  output        [6:0] state_req,        // status: [6]=HDR, [5]=ENTDAA, [4]=Wr, [3]=Rd, [2]=CCChandle, [1]=W/R, [0]=busy
  output              state_dir,        // held 1 if header is RnW, else 0
  output        [1:0] opt_state_AS,     // act state if known
  output        [3:0] opt_ev_mask,      // event mask if known
  output        [2:0] opt_TimeC,        // enabled time control if any - one hot
  output       [12:0] opt_timec_sync,   // for SYNC time control use
  output        [3:0] opt_slvr_reset,   // for SlaveRst control
  output       [12:0] opt_match_idx,    // last 3 matching Map addr if used
  input               i_rst_slvr_reset, // in SCL domain
  input               clr_slvr_cause,   // clear cause - sync to SCL
      // note: state_bus_act is ~int_in_STOP
  // now related to addressing
  output        [7:0] dyn_addr,         // dynamic address we got from Master
  input         [7:0] opt_static_addr,  // static address to match in [7:1] if [0]=1 (SADDR controls)
  output              dyn_addr_chg,     // change from ENTDAA, SETDASA, RSTDAA, SETNEWDA
  output        [2:0] dyn_chg_cause,    // cause of last change
  // now related to events like IBI, MR, and HJ if used 
  input         [2:0] event_pending,    // !=0 if event (MR=2, IBI=1, HJ=3) pending
  input               force_sda,        // force SDA as long as SCL=1. This looked at time
  input               ibi_has_byte,     // 1 if extra byte attached to IBI
  input         [7:0] opt_ibi_byte,     // byte to send if we have one
  input               ibi_timec,        // 1 if time control on for IBI
  output        [1:0] ibi_timec_marks,  // SC1 and SC2 stop - pulses
  output        [2:0] ibi_timec_sel,    // mux selector
  input               ibi_extdata,      // if read data after IBI byte
  input         [3:0] ibi_mapidx,       // mapped index for IBI
  output              ibi_in_extdata,   // indicates reading IBI extdata (SCL domain)
  input               timec_oflow,
  output              timec_oflow_clr,
  // we export SCL and SCL_n up to allow for to-bus and from-bus
  output              oclk_SCL,         // SCL as clock
  output              oclk_SCL_n,       // SCL as clock
  // now common buffers for to-bus. We use clk_SCL_n for the operational
  // clock for ack. The valid model is 01 (lower), 10 (upper), or 00 (none).
  // ack will allow it to advance (can read using SCL).
  input               tb_data_valid,    // b1 if full, b0 if empty
  input         [7:0] tb_datab,         // input byte from system if valid==1
  input               tb_end,           // no more data
  output              tb_datab_ack,     // ack for 1 SCL_n clock when consumed
  output              tb_urun_nack,     // pulse for 1 SCL_n if NACKed due to no data
  output              tb_urun,          // underrun and not end
  output              tb_term,          // terminated by master
  // now common buffers for from-bus. Layer above this uses clk_SCL for the operational 
  // clock to indicate buffer to fill. The use model is 01 (build lower), 10 (build upper), 
  // or 0 (is still full so if data in, will orun). 
  // Done pulsed by this block on clk_SCL_n. The data_use should not change until done.
  input               fb_data_use,      // b1 if OK, b0 if full
  output        [7:0] fb_datab,         // output byte to system if done=1 and use=1
  output              fb_datab_done,    // done pulsed for one clk_SCL_n when written
  output              fb_datab_err,     // set with done if i3c parity error
  output              fb_orun,          // pulse for one clk_SCL_n if over-run (full when data in)
  output        [2:0] fb_ddr_errs,      // DDR framing, parity, CRC
  output              fb_s0s1_err,      // held 1 if in S0 or S1 error hold
  input               brk_s0s1_err,     // pulse 1 in scl domain to break S0/S1 error hold
  input               rd_abort,         // abort read due to clock stall (>60us)
  output              fb_hdr_exit,      // HDR Exit pattern seen (pulse in SDA domain!)
  // next is just exported bit index and ccc byte and willbe_ccc - for specialized uses
  output        [2:0] obit_cnt,         // only use if needed and understand meaning
  output        [7:0] ccc_byte,         // CCC byte, with special meaning if DDR
  output              willbe_ccc,       // CCC in next cycle (except with parity error)
  // now nets from registers or outer system, depending on ID_AS_REGS
  input         [3:0] cf_IdInst,        // Instance of ID when selected (not if partno used)
  input               cf_IdRand,        // random partno of ID if used
  input        [31:0] cf_Partno,        // partno od ID if used
  input         [7:0] cf_IdBcr,         // BCR of DAA if not const
  input         [7:0] cf_IdDcr,         // DCR of DAA if not const
  input        [14:0] cf_IdVid,         // Vendor ID if not const
  // more such nets whose use depends on params (CCC, HDR, etc)
  input        [11:0] cf_MaxRd,         // Max read return
  input        [11:0] cf_MaxWr,         // Max write for master
  input        [23:0] cf_RstActTim,     // Slave Reset recovery times
  input               cf_SlvNack,       // Nack messages (temporary)
  input               cf_SdrOK,         // Allow I3C SDR, else i2c only
  input               cf_DdrOK,         // Allow DDR messages
  input               cf_TspOK,         // Allow TSP messages
  input               cf_TslOK,         // Allow TSL messages
  input        [15:0] cf_TCclk,         // Time control clock info
  input               cf_s0ignore,      // Ignore S0 errors
  input         [7:0] cf_SetDA,         // Main Master sets its own DA
  input [(MAP_CNT*10)-1:0] cf_SetMappedDASA, // mapped SAs/DAs if supported
  input         [2:0] cf_SetSA10b,      // if [1] is SA 10-bit
  input         [1:0] cf_HdrCmd,        // enable for HDR Cmd as MMR
  input         [6:0] cf_CccMask,       // Mask enables for unhandled CCCs
  input         [8:0] cf_vgpio,         // VGPIO control
  input               cf_MasterAcc,     // only for M+S, slave OK to accept Mastership
  input   [MAP_CNT-1:0] map_daa_use,      // which MAP are auto-DAA
  input [(MAP_CNT*8)-1:0] map_daa_dcr,      // DCRs if MAP auto-DAA
  input [(MAP_CNT*PID_CNT)-1:0] map_daa_pid, // PID partials if MAP auto-DAA
  output              opt_MasterAcc,    // pulses 1 in SCL if master accepted
  output              raw_vgpio_done,   // done pulse if VGPIO enabled
  output        [7:0] raw_vgpio_byte,   // byte value if "
  output              raw_hdr_newcmd,   // pulses in SCL domain if new CMD
  output        [7:0] raw_hdr_cmd,      // raw but new_cmd - used if cf_HdrCmd
  output              raw_setaasa,      // if MAP Auto
  output              raw_rstdaa,       // RSTDAA for map use (if auto DAA)
  output              raw_daa_ena,      // pulse if map auto-DASA match or auto-DAA match
  output        [3:0] map_sa_idx,       // index if raw_daa_ena
  output        [7:1] map_daa_da,       // new DA
    // i2c extended stuff if used
  input         [2:0] i2c_dev_rev,      // revision if DeviceID is used (OR into const)
  output              i2c_sw_rst,       // SW reset
  // these next ones are to be safe-raw. That means either changed when
  // bus is STOPped or otherwise not doing a GETSTATUS or synchronized.
  input         [7:6] sraw_ActMode,     // System activity mode (or 0 if not used)
  input         [3:0] sraw_PendInt,     // Pending interrupt (or 0 if not used)
  input        [15:8] sraw_StatusRes,   // reserved bits of status (or 0 if not used) 
  // now returned states from CCC if handled
  output              opt_ChgMaxRd,
  output              opt_ChgMaxWr,
  output       [11:0] opt_MaxRdWr,      // value when one of above changes
  // debug observer
  output[SLV_DBG_MX:0]slv_debug_observ, // optional observer
  // now scan related
  input               scan_single_clock,// single clock domain from scan
  input               scan_clk,         // clock to use if in scan
  input               scan_no_rst,      // prevents layered reset
  input               scan_no_gates     // prevents clock gating
  );

  //
  // Table of Contents
  //
  // 1. We define wires for inter connecting blocks and for clock gen
  // 2. We generate clocks from SCL and SDA pins.
  // 3. We handle some special nets such as SDR_hold
  // 4. We instantiate the engine, daa, ccc, and exit pattern detect blocks
  // 5. We optionally instantiate the HDR blocks
  //

  // 1. We define wires for inter connecting blocks and for clock gen
  wire                HDR_exit;
  wire                in_HDR_mode;      // CCC ENTHDRn seen
  wire                HDR_restart;
  wire                HDR_restart_ack;
  wire                SDR_hold;
  reg                 sdr_hold_r;
  wire                daa_inp_drv;
  wire                daa_inp_bit;
  wire                daa_done;
  wire                daa_acknack;
  wire          [7:0] state_in_CCC_v;
  wire                state_accslv;
  wire                state_req_rd;
  wire                state_req_wr;
  wire                state_daa;
  wire                state_is_read;
  wire          [1:0] next_ccc;
  wire          [1:0] next_hdr;         // [1]=ddr, [0]=hdr
  wire                in_ddr;
  wire                ccc_handled;
  wire                ccc_handling;
  wire                ccc_uh_mask;
  wire                ccc_get;
  wire                ccc_is_read;
  wire                ccc_real_START;
  wire          [7:0] done_buff;
  wire                done_rdy;
  wire                ccc_tb_now;
  wire                ccc_tb_data;
  wire                ccc_tb_continue;
  wire          [1:0] ccc_vgpio_done;
  wire          [7:0] ccc_vgpio;
  wire                is_9th;
  wire                daa_active;
  wire                daa_act_ok;
  wire         [63:0] daa_id;
  wire                set_da;
  wire          [7:1] new_da;
  wire          [7:1] map_da;
  wire                raw_dasa_ena;
  wire                raw_map_daa;
  wire          [3:0] map_dasa_idx;
  wire          [3:0] daa_map_idx;
  wire                opt_setaasa;
  wire                match_da, match_sa;
  wire          [6:0] ccc_counter;
  wire                stb_urun_nack;
  wire                stb_urun;
  wire                stb_term;
  wire                fb_s0s1;
  reg                 s0s1_err;
    // HDR-DDR stuff below. Only used if DDR enabled
  wire          [1:0] ddr_fb_ctrl;
  wire          [1:0] ddr_fb;
  wire          [1:0] ddr_tb_ctrl;
  wire          [1:0] ddr_tb_bits; 
  wire          [1:0] ddr_tb_oe;
  wire          [7:0] byte_1st;
  wire          [7:0] byte_2nd;
  wire                dtb_urun_nack;
  wire                dtb_urun;
  wire                dtb_term;
  wire                ddr_read;

  wire                clk_SCL_n;
  wire                clk_SCL;
  wire                clk_SDA;
  wire                clk_SDA_n;

  //
  // Allow export of debug observer data
  // -- default stub assigns 0, but this allows adding nets
  // -- Note that ENG_DBG_MX is defined by include file as
  //    is eng_debug_observ
  //
  `ifdef ENA_DBG_OBSERVE
   `include "dbg_observ_slv.v"   // integrator replaces inc file contents
  `else
   localparam ENG_DBG_MX = 0;
   wire [ENG_DBG_MX:0] eng_debug_observ;
   assign slv_debug_observ = 0;
  `endif
  //


  // 2. We generate clocks from SCL and SDA pins.
  // Four clocks for clock domain: from pins except in scan
  // These below are clock worthy: glitch free and names used for clock source
  // Note that the relationship of SCL to SCL_n is defined, as is SCL to SDA
  CLOCK_SOURCE_MUX source_scl  (.use_scan_clock(scan_single_clock), .scan_clock(scan_clk), 
                                .pin_clock(pin_SCL_in), .clock(clk_SCL));
  CLOCK_SOURCE_INV source_scl_n(.scan_no_inv(scan_single_clock), .good_clock(clk_SCL), 
                                .clock(clk_SCL_n));
  CLOCK_SOURCE_MUX source_sda  (.use_scan_clock(scan_single_clock), .scan_clock(scan_clk), 
                                .pin_clock(pin_SDA_in), .clock(clk_SDA));
  CLOCK_SOURCE_INV source_sda_n(.scan_no_inv(scan_single_clock), .good_clock(clk_SDA), 
                                .clock(clk_SDA_n));
    // export the SCL clocks for data CDC
  assign oclk_SCL   = clk_SCL; 
  assign oclk_SCL_n = clk_SCL_n; 

  generate if (1 /*~|ENA_HDR[`HDR_TSL_b:`HDR_TSP_b]*/) begin : no_hdr_ternary
    assign pin_SCL_out     = 0;
    assign pin_SCL_oena    = 0;         // we never drive SCL except Ternary
  end else begin
    // FUTURE: add logic for HDR Ternary if decide to support
  end endgenerate

  // 3. We handle some special nets such as SDR_hold
  // SDR_hold is used for when in HDR (waiting on exit pattern,
  //   whether we support HDR or not) and for enable and for 
  //   S0/S1 errors which wait on HDR exit pattern.
  //   Note that slv_enable both resets the registered hold
  //   if turned off, and it also prevents SDR use (any use if off)
  wire hold_RSTn = RSTn & ((~HDR_exit & slv_enable) | scan_no_rst);
  wire s0s1_RSTn = hold_RSTn & (~(brk_s0s1_err & s0s1_err) | scan_no_rst);
  always @ (posedge clk_SCL_n or negedge s0s1_RSTn)
    if (!s0s1_RSTn)
      sdr_hold_r  <= 1'b0;              // i2c/SDR is the default
    else if (in_HDR_mode | next_hdr[0] | s0s1_err)
      sdr_hold_r  <= 1'b1;              // wait for exit pattern (which resets)
  assign SDR_hold = sdr_hold_r | &next_hdr[1:0] | // DDR is early
                    ~slv_enable;        // ignore unless enabled

  // S0 and S1 errors lock up waiting on HDR Exit. We allow app to break as well
  always @ (posedge clk_SCL_n or negedge s0s1_RSTn)
    if (!s0s1_RSTn)
      s0s1_err <= 1'b0;
    else if (fb_s0s1)
      s0s1_err <= 1'b1;
  assign fb_s0s1_err = s0s1_err;

  // 4. We instantiate the engine, daa, ccc, and exit pattern detect blocks
  //
  // the slave engine is the workhorse of the slave - protocol for SDR
  // -- Also has knowledge of some CCCs and modes and states as needed
  // -- Is suppressed by HDR mode, using exit pattern detector to wake it
  // -- Has some HDR DDR knowledge
  //
  i3c_sdr_slave_engine #(.ENA_SADDR(ENA_SADDR), .SADDR_P(SADDR_P),
                         .ENA_MAPPED(ENA_MAPPED),.MAP_CNT(MAP_CNT),
                         .MAP_DA_AUTO(MAP_DA_AUTO), .MAP_DA_DAA(MAP_DA_DAA),
                         .MAP_I2CID(MAP_I2CID),
                         .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ), .ENA_HDR(ENA_HDR),
                         .PIN_MODEL(PIN_MODEL),.ENG_DBG_MX(ENG_DBG_MX))
    engine
    (
    .clk_SCL_n        (clk_SCL_n), 
    .clk_SCL          (clk_SCL), 
    .clk_SDA          (clk_SDA), 
    .RSTn             (RSTn), 
    .SDR_hold         (SDR_hold), 
    .cf_SdrOK         (cf_SdrOK),
    .pin_SCL_in       (pin_SCL_in), 
    .pin_SDA_in       (pin_SDA_in), 
    .pin_SDA_out      (pin_SDA_out), 
    .pin_SDA_oena     (pin_SDA_oena), 
    .pin_SDA_oena_rise(pin_SDA_oena_rise), 
    .i2c_spike_ok     (i2c_spike_ok),
    .i2c_hs_enabled   (i2c_hs_enabled),
    .int_start_seen   (int_start_seen), 
    .int_start_err    (int_start_err), 
    .int_da_matched   (match_da), // masked below if handled 
    .int_sa_matched   (match_sa), // "
    .int_7e_matched   (int_7e_matched), 
    .int_in_STOP      (int_in_STOP), 
    .int_event_sent   (int_event_sent), 
    .int_event_ack    (int_event_ack), 
    .int_in_evproc    (int_in_evproc),
    .cf_SlvNack       (cf_SlvNack),
    .tb_data_valid    (tb_data_valid), 
    .tb_datab         (tb_datab), 
    .tb_end           (tb_end), 
    .tb_datab_ack     (tb_datab_ack), 
    .tb_urun_nack     (stb_urun_nack), 
    .tb_urun          (stb_urun), 
    .tb_term          (stb_term),
    .fb_data_use      (fb_data_use), 
    .fb_datab         (fb_datab), 
    .fb_datab_done    (fb_datab_done), 
    .fb_datab_err     (fb_datab_err), 
    .fb_orun          (fb_orun), 
    .fb_s0s1_err      (fb_s0s1),
    .rd_abort         (rd_abort),
    .state_in_CCC     (state_in_CCC_v), 
    .next_ccc         (next_ccc),
    .next_hdr         (next_hdr),
    .in_ddr           (in_ddr),
    .ccc_handling     (ccc_handling),
    .int_ccc_handled  (int_ccc_handled), // for us specifically
    .ccc_get          (ccc_get),
    .ccc_is_read      (ccc_is_read),
    .ccc_real_START   (ccc_real_START),
    .ccc_handled      (ccc_handled),
    .ccc_uh_mask      (ccc_uh_mask),
    .int_ccc          (int_ccc),
    .state_accslv     (state_accslv),
    .state_rd         (state_req_rd),
    .state_wr         (state_req_wr),
    .state_dir        (state_is_read),
    .in_daa_mode      (state_daa),
    .done_buff        (done_buff), 
    .done_rdy         (done_rdy), 
    .ccc_tb_now       (ccc_tb_now),
    .ccc_tb_data      (ccc_tb_data),
    .ccc_tb_continue  (ccc_tb_continue),
    .is_9th           (is_9th),
    .opt_s0ignore     (cf_s0ignore),
    .daa_active       (daa_active), 
    .daa_act_ok       (daa_act_ok),
    .daa_inp_drv      (daa_inp_drv), 
    .daa_inp_bit      (daa_inp_bit), 
    .daa_acknack      (daa_acknack),
    .daa_done         (daa_done), 
    .dyn_addr         (dyn_addr), 
    .opt_static_addr  (opt_static_addr),// will be net, config, or const 0 from above
    .SetMappedDASA    (cf_SetMappedDASA),// if mapped SA/DA
    .SetSA10b         (cf_SetSA10b),    // if [1] is SA 10-bit
    .cf_vgpio         (cf_vgpio[8]),    // if CCC based (so not separate DA)
    .vgpio_done       (raw_vgpio_done),
    .vgpio_byte       (raw_vgpio_byte),
    .ccc_vgpio_done   (ccc_vgpio_done),
    .ccc_vgpio        (ccc_vgpio),
    .hdr_newcmd       (raw_hdr_newcmd), // pulses in SCL domain if new CMD
    .hdr_cmd          (raw_hdr_cmd),    // cmd value when newcmd
    .raw_dasa_ena     (raw_dasa_ena),   // pulse if auto-DASA match
    .map_sa_idx       (map_dasa_idx),   // index if raw_dasa_ena
    .opt_match_idx    (opt_match_idx),  // last 3 matched if map used
    .i2c_dev_rev      (i2c_dev_rev),    // if extended i2c
    .i2c_sw_rst       (i2c_sw_rst),     // "
    .event_pending    (event_pending), 
    .force_sda        (force_sda), 
    .ibi_has_byte     (ibi_has_byte), 
    .opt_ibi_byte     (opt_ibi_byte),
    .ibi_timec        (ibi_timec),
    .ibi_timec_marks  (ibi_timec_marks),
    .ibi_timec_sel    (ibi_timec_sel),
    .ibi_extdata      (ibi_extdata),
    .ibi_mapidx       (ibi_mapidx),
    .ibi_in_extdata   (ibi_in_extdata),
    .ddr_fb_ctrl      (ddr_fb_ctrl),
    .ddr_fb_MSB       (ddr_fb[0]),
    .ddr_fb_LSB       (ddr_fb[1]),
    .ddr_tb_ctrl      (ddr_tb_ctrl),
    .ddr_tb_bits      (ddr_tb_bits), 
    .ddr_tb_oe        (ddr_tb_oe), 
    .obit_cnt         (obit_cnt), 
    .byte_1st         (byte_1st), 
    .eng_debug_observ (eng_debug_observ),
    .byte_2nd         (byte_2nd),
    .scan_no_rst      (scan_no_rst)
    );
  assign ccc_byte      = byte_1st;
  assign willbe_ccc    = |next_ccc;
  assign state_in_ccc  = |state_in_CCC_v[`CF_DIRECT:`CF_BCAST]; // broadcast or direct
  assign state_in_daa  = state_in_CCC_v[`CF_DAA_M]; 
  assign in_HDR_mode   = state_in_CCC_v[`CF_HDR_M];
  assign state_in_hdr  = in_HDR_mode; 
  assign state_dir     = state_is_read | (in_HDR_mode & ddr_read);
  // status: [6]=HDR, [5]=ENTDAA, [4]=write, [3]=read/IBI, [2]=CCChandle, [1]=W/R, [0]=busy
  assign state_req = {state_in_hdr, state_daa, state_req_wr, 
                      state_in_hdr?ddr_read:state_req_rd, ccc_handled&~int_in_STOP, 
                      state_accslv, ~int_in_STOP};
    // only cause match interrupt if not handled
  assign int_da_matched= match_da & ~ccc_handled;
  assign int_sa_matched= match_sa & ~ccc_handled;
  assign raw_daa_ena   = raw_dasa_ena | raw_map_daa;
  assign map_daa_da    = raw_map_daa ? map_da : new_da; // picks up new DA for DASA/DAA change
  assign map_sa_idx    = raw_map_daa ? daa_map_idx : map_dasa_idx;

  //
  // DAA slave support - supports ENTDAA/RSTDAA handling with help of engine
  // -- Also has inputs from CCC for SETNEWDA and SETDASA
  // -- This block owns the DA (Dynamic address)
  //
  i3c_daa_slave #(.ENA_ID48B(ENA_ID48B), .ID_48B(ID_48B), 
                  .ENA_MAPPED(ENA_MAPPED),.MAP_CNT(MAP_CNT),.MAP_I2CID(MAP_I2CID),
                  .MAP_DA_AUTO(MAP_DA_AUTO), .MAP_DA_DAA(MAP_DA_DAA),
                  .ID_AS_REGS(ID_AS_REGS), .ID_BCR(ID_BCR),
                  .ID_DCR(ID_DCR), .PIN_MODEL(PIN_MODEL))
    daa 
    (
    .clk_SCL_n        (clk_SCL_n), 
    .clk_SCL          (clk_SCL), 
    .RSTn             (RSTn), 
    .pin_SDA_in       (pin_SDA_in), 
    .state_in_CCC     (state_in_CCC_v), 
    .daa_active       (daa_active), 
    .daa_act_ok       (daa_act_ok),
    .daa_inp_drv      (daa_inp_drv), 
    .daa_inp_bit      (daa_inp_bit), 
    .daa_done         (daa_done), 
    .daa_acknack      (daa_acknack),
    .dyn_addr         (dyn_addr), 
    .dyn_addr_chg     (dyn_addr_chg),
    .dyn_chg_cause    (dyn_chg_cause),
    .id64_cnt         (ccc_counter),
    .daa_out          (daa_id),
    .raw_map_daa      (raw_map_daa),
    .daa_map_idx      (daa_map_idx),
    .map_da           (map_da),
    .map_chg          (raw_dasa_ena|raw_setaasa),
    .set_da           (set_da & ~raw_dasa_ena), // set unless map
    .new_da           (new_da),
    .set_aasa         (opt_setaasa),
    .old_sa           (opt_static_addr),
    .cf_IdInst        (cf_IdInst), 
    .cf_IdRand        (cf_IdRand), 
    .cf_Partno        (cf_Partno), 
    .cf_IdBcr         ({cf_IdBcr[7:6],cf_IdBcr[5]|cf_DdrOK,cf_IdBcr[4:0]}), // no Ternary
    .cf_IdDcr         (cf_IdDcr),
    .cf_IdVid         (cf_IdVid),
    .cf_DdrOK         (cf_DdrOK),
    .cf_SetDA         (cf_SetDA),
    .map_daa_use      (map_daa_use),
    .map_daa_dcr      (map_daa_dcr),
    .map_daa_pid      (map_daa_pid),
    .SetMappedDASA    (cf_SetMappedDASA),
    .scan_no_rst      (scan_no_rst)
    );

  //
  // CCC handling for Slave to support as many CCCs as requested by param
  // -- Minimally handles SETDASA and SETNEWDA
  // -- Can be enabled for others
  //
  i3c_ccc_slave #(.ENA_CCC_HANDLING(ENA_CCC_HANDLING), .ID_AS_REGS(ID_AS_REGS),
                  .RSTACT_CONFIG(RSTACT_CONFIG),.MAX_DS_WR(MAX_DS_WR), 
                  .MAX_DS_RD(MAX_DS_RD), .MAX_DS_RDTURN(MAX_DS_RDTURN),
                  .ENA_MASTER(ENA_MASTER), .ENA_HDR(ENA_HDR), .PIN_MODEL(PIN_MODEL),
                  .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC))
    ccc 
    (
    .clk_SCL_n        (clk_SCL_n), 
    .clk_SCL          (clk_SCL), 
    .RSTn             (RSTn), 
    .pin_SDA_in       (pin_SDA_in), 
    .state_in_CCC     (state_in_CCC_v), 
    .next_ccc         (next_ccc & {1'b1,~fb_datab_err}),// mask bcast on parity err
    .dyn_addr         (dyn_addr), 
    .int_start_seen   (int_start_seen), 
    .int_da_matched   (match_da), 
    .int_7e_matched   (int_7e_matched), 
    .int_in_STOP      (int_in_STOP), 
    .ccc_tb_now       (ccc_tb_now),
    .ccc_tb_data      (ccc_tb_data),
    .ccc_tb_continue  (ccc_tb_continue),
    .is_9th           (is_9th),
    .idata_byte       (done_buff), 
    .idata_done       (done_rdy),
    .daa_active       (daa_active), 
    .ccc_counter      (ccc_counter),
    .daa_id           (daa_id),
    .set_da           (set_da),
    .new_da           (new_da),
    .ccc_handling     (ccc_handling),   // will be handling the CCC
    .ccc_handled      (ccc_handled),    // is handling the CCC
    .int_ccc_handled  (int_ccc_handled),
    .ccc_uh_mask      (ccc_uh_mask),    // mask for unhandled
    .ccc_get          (ccc_get),
    .ccc_is_read      (ccc_is_read),
    .ccc_real_START   (ccc_real_START),
    .opt_state_AS     (opt_state_AS),   // activity state if handled
    .opt_ev_mask      (opt_ev_mask),    // event mask if known
      // now read and write lengths when enabled
    .cf_MaxRd         (cf_MaxRd),
    .cf_MaxWr         (cf_MaxWr),
    .cf_RstActTim     (cf_RstActTim),
    .cf_vgpio         (cf_vgpio),       // used if VGPIO is CCC
    .cf_CccMask       (cf_CccMask),     // select which unhandled CCCs to pass up
    .ccc_vgpio_done   (ccc_vgpio_done),
    .ccc_vgpio        (ccc_vgpio),
    .opt_ChgMaxRd     (opt_ChgMaxRd),
    .opt_ChgMaxWr     (opt_ChgMaxWr),
    .opt_MaxRdWr      (opt_MaxRdWr),
    .opt_TimeC        (opt_TimeC),
    .opt_timec_oflow  (timec_oflow),
    .opt_timec_oflow_clr(timec_oflow_clr),
    .cf_TCclk         (cf_TCclk),
    .opt_timec_sync   (opt_timec_sync),
    .cf_MasterAcc     (cf_MasterAcc),   // only for M+S, slave OK to accept Mastership
    .opt_MasterAcc    (opt_MasterAcc),
    .opt_slvr_reset   (opt_slvr_reset), // SlaveRst control
    .i_rst_slvr_reset (i_rst_slvr_reset),
    .clr_slvr_cause   (clr_slvr_cause),
    .opt_setaasa      (opt_setaasa),
    .no_setaasa       (&event_pending[1:0]), // no SETAASA if HJ
    .bus_err          ((fb_datab_done & fb_datab_err) | (|fb_ddr_errs)),
    .event_pending    (event_pending),
    .sraw_ActMode     (sraw_ActMode),
    .sraw_PendInt     (sraw_PendInt),
    .sraw_StatusRes   (sraw_StatusRes),
    .scan_no_rst      (scan_no_rst)
  );
  assign raw_setaasa = MAP_DA_AUTO[`MAPDA_AASA_b] & opt_setaasa; // only if enabled
  assign raw_rstdaa  = |MAP_DA_AUTO[2:0] &
                          (state_in_CCC_v[`CF_GRP_b] == `CFG_RSTDAA_US);

  //
  // Exit Pattern detector to exit HDR
  // -- Is working at all times
  // -- Is critical when CCC state shows we are in HDR as that locks SDR mode
  //
  i3c_exit_detector #(.ENA_HDR(ENA_HDR))
    exit_detect
    (
    .clk_SDA_n        (clk_SDA_n), 
    .clk_SDA          (clk_SDA), 
    .clk_SCL          (clk_SCL), 
    .RSTn             (RSTn), 
    .pin_SCL_in       (pin_SCL_in), 
    .in_HDR_mode      (in_HDR_mode), 
    .HDR_restart_ack  (HDR_restart_ack), 
    .oHDR_exit        (HDR_exit), 
    .oHDR_restart     (HDR_restart),
    .scan_no_rst      (scan_no_rst)
  );
  assign fb_hdr_exit = HDR_exit;


  //
  // 5. We optionally instantiate the HDR blocks
  //

  //
  // HDR-DDR is instantiated here if enabled
  // -- Note that it uses the SDR engine to do a lot of its work
  // 
  generate if (ENA_HDR[`HDR_DDR_b]) begin : ddr_instance
    i3c_hdr_ddr_slave #(.PIN_MODEL(PIN_MODEL))
      hdr_ddr
      (
      .clk_SCL_n        (clk_SCL_n), 
      .clk_SCL          (clk_SCL), 
      .RSTn             (RSTn), 
      .pin_SDA_in       (pin_SDA_in), 
      .in_DDR           (in_ddr&daa_id[8+5]), // daa_id[8+5] to be sure HDR allowed
      .HDR_restart      (HDR_restart), 
      .restart_ack      (HDR_restart_ack),
      .HDR_exit         (HDR_exit), 
      .rd_abort         (rd_abort),     // kill if clock stops too long
      .dyn_addr         (dyn_addr), 
      .ddr_fb_ctrl      (ddr_fb_ctrl), 
      .ddr_fb           (ddr_fb), 
      .ddr_tb_ctrl      (ddr_tb_ctrl), 
      .ddr_tb_bits      (ddr_tb_bits), 
      .ddr_tb_oe        (ddr_tb_oe), 
      .bit_cnt          (obit_cnt[1:0]), // only lower 2 since goes by 2s 
      .byte_1st         (byte_1st), 
      .byte_2nd         (byte_2nd), 
      .tb_datab         (tb_datab),
      .tb_empty         (~tb_data_valid), 
      .tb_end           (tb_end),
      .int_ddr_matched  (int_ddr_matched),
      .ddr_read         (ddr_read),
      .fb_ddr_errs      (fb_ddr_errs),
      .tb_term          (dtb_term),
      .tb_urun_nack     (dtb_urun_nack),
      .tb_urun          (dtb_urun),
      .cf_HdrCmd        (cf_HdrCmd),      // if HDR CMD to go out to MMR
      .hdr_newcmd       (raw_hdr_newcmd), // pulses in SCL domain if new CMD
      .scan_no_rst      (scan_no_rst)
      );
    // underrun is OR of SDR and DDR underrun
    assign tb_urun_nack = stb_urun_nack | dtb_urun_nack;
    assign tb_urun      = stb_urun | dtb_urun;
    assign tb_term      = stb_term | dtb_term;
  end else begin
    assign ddr_fb_ctrl = 0;
    assign ddr_fb      = 0;
    assign ddr_tb_ctrl = 0;
    assign ddr_tb_bits = 0;
    assign ddr_tb_oe   = 0;
    assign HDR_restart_ack = 1'b1;      // only used if HDR is suppported
    assign int_ddr_matched = 0;
    assign ddr_read    = 0;
    assign fb_ddr_errs = 0;
    assign tb_urun_nack= stb_urun_nack;
    assign tb_urun     = stb_urun;
    assign tb_term     = stb_term;
    assign raw_hdr_newcmd = 0;
  end endgenerate

endmodule

