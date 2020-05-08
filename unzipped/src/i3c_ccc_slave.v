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
//  File            : i3c_ccc_slave.v
//  Organisation    : MCO
//  Tag             : 1.0.2
//  Tag             : 1.1.11
//  Date            : $Date: Thu Nov 14 18:30:38 2019 $
//  Revision        : $Revision: 1.64 $
//
//  IP Name         : i3c_ccc_slave
//  Description     : MIPI I3C Slave CCC handling (req and optional by param)
//    This small module processes the CCC commands it is enabled for.
//    All slaves support ENTDAA and RSTDAA, but thos are handled in 
//    i3c_daa_slave.
//    This block minimally supports required CCCs. It works with
//    i3c_daa_slave in support of ENTDAA, RSTDAA, SETDASA, and SETNEWDA 
//    Beyond that it can handle Events, Activity states, status
//    requests, max values, etc.
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
//    This block is fed data from the i3c_sdr_slave_engine, which detects
//    CCC commands to at least the level of direct and broadcast, but also
//    parses a few others that either relate to a mode or a state. It 
//    handles end of command (on start, stop, start with 7E, etc).
// 
//    Command processing works in 4 ways, except ENTDAA and ENDHDR,
//    neither of which is handled by this block:
//    1. Broadcast SET: 
//         (re)START,7E/W,{1'b0,cmd[6:0]},opt_data,reSTART_or_STOP
//       All Slaves see the same broadcast and the cmd_byte is matched.
//       Any data follows and then it is done.
//    2. Direct SET: 
//         (re)START,7E/W,{1'b1,cmd[6:0]},reSTART,da/W,opt_data,reSTART_or_STOP
//       and
//         (re)START,7E/W,{1'b1,cmd[6:0]},def_byte,reSTART,da/W,reSTART_or_STOP
//       Each slave has to check each "da" to see if for it. If so, it 
//       must ACK. Then it sees the data for it (if any)
//       We know if it was for us via the input int_da_matched.
//       Note that defining byte has to be held to be emitted with da match.
//    3. Direct GET: 
//         (re)START,7E/W,{1'b1,cmd[6:0]},reSTART,da/R,<data>,reSTART_or_STOP
//       and
//         (re)START,7E/W,{1'b1,cmd[6:0]},def_byte,reSTART,da/R,<data>,reSTART_or_STOP
//       Each slave has to check each "da" to see if for it. If so, it 
//       must ACK and then return the data associated with the GET.
//       We know if it was for us via the input int_da_matched.
//       Note that defining byte has to be held to be emitted with da match.
//    4. Direct SETDASA command:
//         (re)START,7E/W,SETDASA_D,reSTART,sa/W,DA,START_or_STOP
//         (re)START,7E/W,SETDASA_D,reSTART,01/W,01,START_or_STOP
//       Each slave matches its Static address if it has one. It also
//       tries to match the reserved 01 for point-to-point comms.
//       If it matches, it issues an ACK and reads the new DA.
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_ccc_slave #(
    parameter ENA_CCC_HANDLING= 6'd0,   // passed down as to what to support
    parameter ID_AS_REGS      = 12'd0,  // Used for VGPIO
    parameter RSTACT_CONFIG   = 26'd0,  // Slave Reset RSTACT CCC config (see RSTA_xxx_b fields)
    parameter MAX_DS_WR = 0,            // data speed limits for M->S
    parameter MAX_DS_RD = 0,            // data speed limits for S->M
    parameter MAX_DS_RDTURN = 0,        // latency needs for S->M read req
    parameter ENA_HDR   = 0,            // which HDR modes if any we support
    parameter ENA_MASTER= 0,            // If master - for CCC support
    parameter PIN_MODEL = `PINM_COMBO,  // combinatorial pin use
    parameter ENA_TIMEC = 6'b000010,    // if reg, res, res, mode 1, mode 0, sync
    parameter TIMEC_FREQ_ACC = {8'd24,8'd10}// freq=12MHz (12.0=24) with 1.0% accuracy
  )
  (
  // 1st Clock and Reset and pin in from outer parts
  input               clk_SCL_n,        // SCL falling edge: start and launching data
  input               clk_SCL,          // SCL rising edge: sample data and read T bit
  input               RSTn,             // master Reset
  input               pin_SDA_in,       // SDA pin as read
  // now context we care about
  input         [7:0] state_in_CCC,     // engine defines detected CCC states/modes: see `CF_xxx
  input         [1:0] next_ccc,         // for broadcast no-data CCCs
  input         [7:0] dyn_addr,         // dynamic address to match in [7:1] if [0]=1
  input               int_start_seen,   // 1 cycle pulse when START or repeated START seen
  input               int_da_matched,   // 1 cycle pulse if matched our dynamic address (and valid)
  input               int_7e_matched,   // 1 cycle pulse if matched i3c broadcast address
  input               int_in_STOP,      // STOP state (held until START)
  // now outbound data when GET CCC is used
  output              ccc_tb_now,       // held when we are outputting data
  output              ccc_tb_data,      // current data bit
  output              ccc_tb_continue,  // 1 if continue, else 0 if done
  input               is_9th,           // controls counting for 9th T bit
  // we are fed the data byte when we indicate we want it
  input         [7:0] idata_byte,       // byte of data associated with CCC
  input               idata_done,       // 1 cycle when data byte valid
  // now the controls for DAA to update dynamic address
  input               daa_active,       // used for DAA for ID count
  output        [6:0] ccc_counter,      // used for CCCs and DAA
  input        [63:0] daa_id,           // for GET CCCs
  output              set_da,           // pulsed to force DA to change
  output        [7:1] new_da,           // DA to set
  output              ccc_handling,     // if will be handling CCC
  output              ccc_handled,      // we are handling CCC
  output              int_ccc_handled,  // handling, but only if was for us if direct
  output              ccc_uh_mask,      // mask for unhandled
  output              ccc_get,          // 1 if GET
  input               ccc_is_read,      // i3c resd - needed for SETGET
  input               ccc_real_START,   // to cancel RSTACT
  // now outputs and inputs for CCC commands we may handle
  output        [1:0] opt_state_AS,     // act state if known
  output        [3:0] opt_ev_mask,      // event mask if known
    // read and write length. Cf is current state (and starting)
  input        [11:0] cf_MaxRd,
  input        [11:0] cf_MaxWr,
  input        [23:0] cf_RstActTim,     // Slave Reset recovery times
  input         [8:0] cf_vgpio,         // VGPIO control
  input         [6:0] cf_CccMask,       // Mask enables for unhandled CCCs
  output        [1:0] ccc_vgpio_done,   // when VGPIO is a CCC
  output        [7:0] ccc_vgpio,
  output              opt_ChgMaxRd,
  output              opt_ChgMaxWr,
  output       [11:0] opt_MaxRdWr,      // value when one of above changes
  output        [2:0] opt_TimeC,        // enabled time control if any - one hot
  input               opt_timec_oflow,  // counter overflow
  output              opt_timec_oflow_clr, // clear bit
  input        [15:0] cf_TCclk,         // Time control clock info
  output       [12:0] opt_timec_sync,   // sync info if [10:8]!=0 - change on [11]=1
  output        [3:0] opt_slvr_reset,   // reserved for future - only full/none now
  input               i_rst_slvr_reset, // in SCL domain
  input               clr_slvr_cause,   // cleared cause - clears rst
  output              opt_setaasa,      // pulses on SETAASA (SA->DA)
  input               no_setaasa,       // Disabled if Hot Join
  input               cf_MasterAcc,     // only for M+S, slave OK to accept Mastership
  output              opt_MasterAcc,    // pulses 1 in SCL if master accepted
  input               bus_err,          // pulsed if FB prot error
  input         [2:0] event_pending,    // if IBI or P2P or HJ
  // safe-raw below means we can use as is
  input         [7:6] sraw_ActMode,     // System activity mode (or 0 if not used)
  input         [3:0] sraw_PendInt,     // Pending interrupt (or 0 if not used)
  input        [15:8] sraw_StatusRes,   // reserved bits of status (or 0 if not used) 
  input               scan_no_rst       // prevents layered reset
  );

  //
  // Table of contents
  //
  // 1. Use of macros to set params to be real or collapsed
  // 2. Simple state machine
  // 3. Mechanism to determine if we will handle
  // 4. Handlers for CCCs we support
  // 5. Master counter used for CCCs and DAA mode
  //

  reg           [6:0] ccc_handle_r;     // set to value if to be handled
  reg           [6:0] ccc_init;
  wire          [6:0] load;             // cnt loader
  wire                rst_daa_n;
  wire                is_get;
  reg           [6:0] cnt;              // general counter for CCC and DAA
  wire          [6:0] use_cnt;
  reg                 prot_err;         // holds protocol error until read
  reg           [7:0] def_byte;         // only used if v1.1

  // we define the mapped commands. Note the ugly use of `define is
  // because generate has its own scope, so we cannot use it for
  // params (only wires).
  // Note that bit [5] is 1 if GET, [6] is 1 if SETGET
  `define CBA(val) ENA_CCC_HANDLING[`ENCCC_BASIC_b]?val:7'd0
  `define CMX(val) ENA_CCC_HANDLING[`ENCCC_MAXES_b]?val:7'd0
  `define CAA(val) ENA_CCC_HANDLING[`ENCCC_AASA]?val:7'd0
  `define CV1(val) ENA_CCC_HANDLING[`ENCCC_V11MIN]?val:7'd0
  `define CMM(val) |ENA_MASTER?val:7'd0 
  `define CMT(val) |ENA_TIMEC?val:7'd0 
  localparam CHAN_NONE  =      7'h00;
  localparam CHAN_DAA   =      7'h01;   // using details from engine
  localparam CHAN_ENEC  = `CBA(7'h02);  // enable events
  localparam CHAN_DISEC = `CBA(7'h03);  // disabled events
  localparam CHAN_SETAS0= `CBA(7'h04);  // set Act State
  localparam CHAN_SETAS1= `CBA(7'h05);  // set Act State
  localparam CHAN_SETAS2= `CBA(7'h06);  // set Act State
  localparam CHAN_SETAS3= `CBA(7'h07);  // set Act State
  localparam CHAN_SMXWL = `CMX(7'h08);  // set max write len
  localparam CHAN_SMXRL = `CMX(7'h09);  // set max read len
  localparam CHAN_ENTTM = `CMM(7'h0A);  // entering test mode
  localparam CHAN_DEFSLV= `CMM(7'h0B);  // define slaves: will need N length model
  localparam CHAN_BRDGT = `CMM(7'h0C);  // bridge target
  localparam CHAN_SXTIM = `CMT(7'h0D);  // set xtime (time control)
  localparam CHAN_RSTACT= `CV1(7'h4E);  // define SlaveRst rules - SETGET type
  localparam CHAN_SAASA = `CAA(7'h0F);  // SETAASA makes DA from SA
    // GET below
  localparam CHAN_STATUS=      7'h20;   // get status
  localparam CHAN_GPID  = `CBA(7'h21);  // get pid
  localparam CHAN_GBCR  = `CBA(7'h22);  // get bcr
  localparam CHAN_GDCR  = `CBA(7'h23);  // get dcr
  localparam CHAN_GHDR  = `CBA(7'h24);  // get HDR cap
  localparam CHAN_GMXWL = `CMX(7'h25);  // get max write len
  localparam CHAN_GMXRL = `CMX(7'h26);  // get max read len
  localparam CHAN_GMXDS = `CMX(7'h27);  // max data speed limit
  localparam CHAN_GMST  = `CMM(7'h28);  // master handoff
  localparam CHAN_GXTIM = `CMT(7'h29);  // get time control

  // Load is only used for direct commands
  localparam LOAD_DAA   = 7'h7F;
  localparam LOAD_ENEC  = `CBA(7'd9);
  localparam LOAD_DISEC = `CBA(7'd9);
  localparam LOAD_STATUS= 7'd16;
  localparam LOAD_GPID  = `CBA(7'd48);
  localparam LOAD_GBCR  = `CBA(7'd8);
  localparam LOAD_GDCR  = `CBA(7'd8);
  localparam LOAD_GHDR  = `CBA(7'd8);
  localparam LOAD_SMXWL = `CMX(7'd16);
  localparam LOAD_SMXRL = `CMX(7'd16);
  localparam LOAD_GMXWL = `CMX(7'd16);
  localparam LOAD_GMXRL = `CMX(7'd16);
  localparam LOAD_GMXDS = |MAX_DS_RDTURN?`CMX(7'd40):`CMX(7'd16); 
  localparam LOAD_GMST  = `CMM(7'h08); // NACK if not accepted
  localparam LOAD_GXTIM = `CMT(7'd32); // 4 bytes
  localparam LOAD_RSTACT= `CV1(7'd8);


  // Entry to CCC looks like:
  // 0. int_start_seen
  // 1. int_7E or int_da
  // 2. state_in_CCC[`CF_BCAST] or [`CF_DIRECT] == 1
  //    -- Same cycle as idata_done
  //    -- state_in_CCC[`CF_GRP_b] may be non-0 if it is a CCC that we
  //       have to support like SETNEWDA or SETDASA
  // 3. If broadcast, then data follows using idata_done
  //    -- ends with start_seen
  // 4. If direct, then wait for start_seen
  // 5. Will be int_da if normal CCC direct
  //    -- SETDASA is special, so engine changes state_in_CCC[`CF_GRP_b]
  //       if matched our static address or 01.
  // 6. Now data using idata_done until start_seen.
  //     

  localparam CST_NO_CCC     = 3'd0;
  localparam CST_BCAST_CCC  = 3'd1;     // if not one of ours, wait for start_seen
  localparam CST_DIRECT_CCC = 3'd2;     // if not one of ours, wait for int_7e or START  parameter
  localparam CST_NEWDA      = 3'd3;
  localparam CST_DASA       = 3'd4;     // special, so wait for state_in_CCC to change

  reg     [2:0] ccc_state;
  reg     [2:0] ccc_next_state;

  always @ (posedge clk_SCL_n or negedge RSTn)
    if (!RSTn)
      ccc_state <= CST_NO_CCC;
    else if (~|state_in_CCC[`CF_DIRECT:`CF_BCAST])
      ccc_state <= CST_NO_CCC;
    else
      ccc_state <= ccc_next_state;

  always @ ( * )
    case (ccc_state)
    CST_NO_CCC:                         // not in CCC; wait until we are
      ccc_next_state = 
          state_in_CCC[`CF_BCAST]  ? CST_BCAST_CCC  : 
          state_in_CCC[`CF_DIRECT] ? CST_DIRECT_CCC :
                                     CST_NO_CCC;

    CST_BCAST_CCC:                      // not a known one
      ccc_next_state = 
          int_start_seen ? CST_NO_CCC : // broadcast ends on start
                           CST_BCAST_CCC;

    CST_DIRECT_CCC: 
      ccc_next_state = 
          int_7e_matched ? CST_NO_CCC : // direct ends on 7E or STOP
          ((state_in_CCC[`CF_GRP_b]==`CFG_NEWDA) & int_da_matched) ? CST_NEWDA : 
          (state_in_CCC[`CF_GRP_b]> `CFG_DASA) ? CST_DASA : // SA or 01
          CST_DIRECT_CCC;

    CST_NEWDA:                          // direct NEWDA for us
      // we wait for data_done to get the new DA
      ccc_next_state = 
          idata_done ? CST_DIRECT_CCC :
                       CST_NEWDA;
      
    CST_DASA:                           // direct SETDASA for us
      // we wait for data_done to get the new DA
      ccc_next_state = 
          idata_done ? CST_DIRECT_CCC : CST_DASA;

    default:
      ccc_next_state = CST_NO_CCC;

    endcase

  // determine whether we support the CCC. 
  wire new_ccc = (ccc_state==CST_NO_CCC) & (ccc_next_state==CST_NO_CCC);
  always @ (posedge clk_SCL_n or negedge RSTn)
    if (!RSTn) 
      ccc_handle_r   <= CHAN_NONE;
    else if (new_ccc) begin
      if (|next_ccc) begin
        ccc_handle_r <= ccc_init;
      end else
        ccc_handle_r <= CHAN_NONE;
    end 
  assign is_get          = (ccc_handle_r[6]&ccc_is_read) | // SETGET
                           ccc_handle_r[5]; // GET vs. SET
  assign ccc_get         = is_get;
 
  // Note that the case is misleading since we map any not
  // actually supported to CHAN_NONE and so they collapse
  always @ ( * ) 
    casez (idata_byte)                   // only meaningful on 1st byte
    8'h06,8'h07,8'h86,
     8'h87,8'h88:      ccc_init = CHAN_DAA;    
    8'h90:             ccc_init = CHAN_STATUS; 
    {1'b?,7'h00}:      ccc_init = CHAN_ENEC;   
    {1'b?,7'h01}:      ccc_init = CHAN_DISEC;  
    {1'b?,7'h02}:      ccc_init = CHAN_SETAS0; 
    {1'b?,7'h03}:      ccc_init = CHAN_SETAS1; 
    {1'b?,7'h04}:      ccc_init = CHAN_SETAS2; 
    {1'b?,7'h05}:      ccc_init = CHAN_SETAS3; 
    {1'b?,7'h09}:      ccc_init = CHAN_SMXWL;  
    {1'b?,7'h0A}:      ccc_init = CHAN_SMXRL;  
    8'h8B:             ccc_init = CHAN_GMXWL;  
    8'h8C:             ccc_init = CHAN_GMXRL;  
    8'h94:             ccc_init = CHAN_GMXDS;   
    8'h8D:             ccc_init = CHAN_GPID;   
    8'h8E:             ccc_init = CHAN_GBCR;   
    8'h8F:             ccc_init = CHAN_GDCR;   
    8'h95:             ccc_init = CHAN_GHDR; // is now GETCAPS in v1.1
    /* we do not handle these in HW - could in the future perhaps
    8'h0B:             ccc_init = CHAN_ENTTM;
    8'h08:             ccc_init = CHAN_DEFSLV;
    8'h93:             ccc_init = CHAN_BRDGT;
    */
    8'h28:             ccc_init = CHAN_SXTIM;
    8'h98:             ccc_init = CHAN_SXTIM;
    8'h2A:             ccc_init = CHAN_RSTACT;
    8'h9A:             ccc_init = CHAN_RSTACT;
    8'h29:             ccc_init = ~no_setaasa ? CHAN_SAASA : 5'd0;
    8'h91:             ccc_init = cf_MasterAcc ? CHAN_GMST : 5'd0;
    8'h99:             ccc_init = CHAN_GXTIM;
    default:           ccc_init = CHAN_NONE;   
    endcase


  // the ccc_mask is used to stop unhandled CCCs from being send to
  // app
  generate if (ID_AS_REGS[`IDREGS_CCCMSK_b]) begin : ccc_unhandled
  wire base_msk   = cf_CccMask[0] & (idata_byte[6:0]<=8'h1F);
  wire basebx_msk = cf_CccMask[1] & (idata_byte>=8'h20) & (idata_byte<=8'h48);
    wire basedx_msk = cf_CccMask[2] & (idata_byte>=8'hA0) & (idata_byte<=8'hBF);
  wire metb_msk   = cf_CccMask[3] & (idata_byte>=8'h49) & (idata_byte<=8'h64);
  wire metd_msk   = cf_CccMask[4] & (idata_byte>=8'hC0) & (idata_byte<=8'hE3);
  wire vendb_msk  = cf_CccMask[5] & (idata_byte>=8'h65) & (idata_byte<=8'h7F);
    wire vendd_msk  = cf_CccMask[6] & (idata_byte>=8'hE4) & (idata_byte<=8'hFF);
  assign ccc_uh_mask = base_msk | basebx_msk | basedx_msk | 
                       metb_msk | metd_msk | vendb_msk | vendd_msk;
  end else begin
    assign ccc_uh_mask = 1'b1;
  end endgenerate
  // load is highest bit + 1. Only used for direct
  assign load = ({7{ccc_handle_r==CHAN_DAA}}    & LOAD_DAA) |
                ({7{ccc_handle_r==CHAN_STATUS}} & LOAD_STATUS) |
                ({7{ccc_handle_r==CHAN_ENEC}}   & LOAD_ENEC) |
                ({7{ccc_handle_r==CHAN_DISEC}}  & LOAD_DISEC) |
                ({7{ccc_handle_r==CHAN_SMXWL}}  & LOAD_SMXWL) |
                ({7{ccc_handle_r==CHAN_SMXRL}}  & LOAD_SMXRL) |
                ({7{ccc_handle_r==CHAN_GMXWL}}  & LOAD_GMXWL) |
                ({7{ccc_handle_r==CHAN_GMXRL}}  & LOAD_GMXRL) |
                ({7{ccc_handle_r==CHAN_GMXDS}}  & LOAD_GMXDS) |
                ({7{ccc_handle_r==CHAN_GPID}}   & LOAD_GPID) |
                ({7{ccc_handle_r==CHAN_GBCR}}   & LOAD_GBCR) |
                ({7{ccc_handle_r==CHAN_GDCR}}   & LOAD_GDCR) |
                ({7{ccc_handle_r==CHAN_GHDR}}   & LOAD_GHDR) |
                ({7{ccc_handle_r==CHAN_GMST}}   & LOAD_GMST) |
                ({7{ccc_handle_r==CHAN_GXTIM}}  & LOAD_GXTIM)|
                ({7{ccc_handle_r==CHAN_RSTACT}} & LOAD_RSTACT);


  // ccc_handling is combinatorial 1st since we have to use it to
  // stop from storing byte for app. Then registered as is ccc_handled
  assign ccc_handling    = ((ccc_state==CST_NO_CCC) & (ccc_next_state==CST_NO_CCC)) ?
                            (|next_ccc & |ccc_init) : |ccc_handle_r;
  assign ccc_handled     = |ccc_handle_r;
  assign int_ccc_handled = ccc_handled & 
                           (state_in_CCC[`CF_BCAST] | int_da_matched |
                            (ccc_state==CST_DASA));

  // we set DA only if the T bit parity matches (idata_done). Note bit[0] unused here
  assign set_da = ((ccc_state == CST_NEWDA) | (ccc_state == CST_DASA)) &
                  idata_done; 
  assign new_da = idata_byte[7:1];

  // next registers protocol errors for GET_STATUS
  always @ (posedge clk_SCL_n or negedge RSTn)
    if (!RSTn)
      prot_err <= 1'b0;
    else if (bus_err)
      prot_err <= 1'b1;                 // remember protocol errors
    else if ((ccc_handle_r==CHAN_STATUS) & ccc_tb_now & (use_cnt[3:0]==4'd1))
      prot_err <= 1'b0;                 // clear on read

  // we forward some nets which are optional
  wire [15:0] max_wrlen;
  wire [15:0] max_rdlen;
  wire [LOAD_GMXDS:0] max_ds;
  wire [31:0] xtime;
  wire  [7:0] get_slvr_value;
  // now the built in GET ones not related to DA
  wire [15:0] status     = {sraw_StatusRes[14:8],sraw_ActMode[7:6],
                            prot_err,1'b0,
                            |sraw_PendInt ? sraw_PendInt[3:0] : 
                            (event_pending[1:0]==2'd1) ? 4'd1 : 4'd0, 
                            sraw_StatusRes[15]}; // for wraparound index

  generate if (PIN_MODEL == `PINM_COMBO) begin : combo_for_get
    // combo SDA_oe/out, so we used all registered states.
    assign use_cnt = cnt;
  end else begin : reg_for_get
    // registered SDA_oe/out, so we have to use combo pre-cnt for GET since it 
    // is registered on the same cycle as the output is registered.
    assign use_cnt = (~daa_active & (ccc_handle_r!=CHAN_DAA) & |ccc_handle_r) ?
                              ((int_da_matched & (ccc_state == CST_DIRECT_CCC)) ?
                               load : (|cnt & ~is_9th) ? (cnt-7'd1) : cnt) :
                             cnt;
  end endgenerate
 
  assign ccc_tb_now = (is_get & |use_cnt);
  wire        get_status = (ccc_handle_r==CHAN_STATUS) & status[use_cnt[3:0]];
  wire [16:0] dcr_bcr    = {daa_id[15:0],1'b0};
  wire [48:0] prov_id    = {daa_id[63:16],1'b0};
  wire        get_bcr    = (ccc_handle_r==CHAN_GBCR) & dcr_bcr[8+use_cnt[3:0]];
  wire        get_dcr    = (ccc_handle_r==CHAN_GDCR) & dcr_bcr[use_cnt[3:0]];
  wire        get_pid    = (ccc_handle_r==CHAN_GPID) & prov_id[use_cnt[5:0]]; // Provisional ID part
  wire        get_caps;  // is GET_HDR if v1.0
  generate if (ENA_CCC_HANDLING[`ENCCC_V11MIN]) begin : def_get_caps
    // TODO: need to add defining byte option
    wire [7:0]cap1       = {7'd0, dcr_bcr[8+5+1] & ENA_HDR[`HDR_DDR_b]}; // HDR - DDR if enabled
                          // flow ctrl  group  version
    wire [7:0]cap2       = {1'b0, 1'b0, 2'b00, 4'd1}; // v1.1
                         // DB below is defining byte suppor for status and getcaps
                         //pnd_rd BT crc32 DB stat DB caps DTDT ibi supp  ML
    wire [7:0]cap3       = {1'b0, 1'b0,    1'b0,   1'b0,   1'b0,    1'b0, 1'b0};
    wire [7:0]cap4       = {1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0};
    wire[32:0]caps       = {cap4, cap3, cap2, cap1, 1'b0};
    assign    get_caps   = (ccc_handle_r==CHAN_GHDR) & caps[use_cnt[5:0]];
  end else begin
    wire      get_hdr    = (use_cnt[3:0]==4'd1) & dcr_bcr[8+5+1] &
                              ENA_HDR[`HDR_DDR_b]; // only DDR for now
    assign    get_caps   = (ccc_handle_r==CHAN_GHDR) & get_hdr;
  end endgenerate
  wire        get_mxwrlen= (ccc_handle_r==CHAN_GMXWL)& max_wrlen[use_cnt[3:0]];
  wire        get_mxrdlen= (ccc_handle_r==CHAN_GMXRL)& max_rdlen[use_cnt[3:0]];
  wire        get_mxds   = (ccc_handle_r==CHAN_GMXDS)& max_ds[use_cnt[(|MAX_DS_RDTURN?5:3):0]];
  wire [32:0] xtime_z    = {xtime,1'b0};
  wire        get_xtime  = (ccc_handle_r==CHAN_GXTIM)& xtime_z[use_cnt[5:0]];
  wire  [8:0] mst_acc    = {dyn_addr[7:1],~^dyn_addr[7:1],1'b0};
  wire        get_mstacc = (ccc_handle_r==CHAN_GMST) & |ENA_MASTER & mst_acc[use_cnt[3:0]];
  wire  [8:0] slvr_val   = {get_slvr_value,1'b0};
  wire        get_rstact = (ccc_handle_r==CHAN_RSTACT) & slvr_val[use_cnt[3:0]];
  assign ccc_tb_data     = ccc_tb_now &
                             (get_status | get_bcr | get_dcr | get_pid | get_xtime |
                              get_caps | get_mxwrlen | get_mxrdlen | get_mxds | 
                              get_mstacc | get_rstact);
  assign ccc_tb_continue = |use_cnt[5:1];

  // now optional ones in the basic class
  generate 
    if (ENA_CCC_HANDLING[`ENCCC_BASIC_b]) begin : basic_ccc
      reg        [1:0] act_state;
      reg        [3:0] event_mask;
      assign opt_state_AS = act_state;
      assign opt_ev_mask  = event_mask;

      // Activity state setting is broadcast and direct; we 
      // peel off AS from bottom 2 bits of CCC
      // Note: the direct version is different in that has no data
      always @ (posedge clk_SCL_n or negedge RSTn)
        if (!RSTn)
          act_state   <= 2'd0;
        else if (next_ccc[`CF_BCAST-1] & (ccc_init[6:2] == (CHAN_SETAS0>>2)) &
                 idata_done)
          act_state   <= ccc_init[1:0];
        else if ((ccc_handle_r[6:2] == (CHAN_SETAS0>>2)) & int_da_matched)
          act_state   <= ccc_handle_r[1:0];

      // Event mask set or clear is broadcast and direct; we 
      // get a byte with the set or clr bits
      // Note direct carries the byte to enable or disable
      always @ (posedge clk_SCL_n or negedge RSTn)
        if (!RSTn)
          event_mask   <= 4'b1011;      // enabled by default
        else if (idata_done & ((ccc_state==CST_BCAST_CCC) | (cnt==7'h1))) begin
          if (ccc_handle_r == CHAN_ENEC)
            event_mask <= event_mask | idata_byte[3:0];
          else if (ccc_handle_r == CHAN_DISEC)
            event_mask <= event_mask & ~idata_byte[3:0];
        end
    end else begin
      assign opt_state_AS = 2'd0;
      assign opt_ev_mask  = 4'b1011;
    end
  endgenerate

  generate 
    if (|ENA_TIMEC) begin : xtime_ccc
    //FREE_VERSION_CUT - remove time-control from free version
    end else 
    begin
      assign xtime     = 32'd0;
      assign opt_TimeC = 3'b000;
      assign opt_timec_oflow_clr = 1'b0;
      assign opt_timec_sync = 12'd0;
    end
  endgenerate

  generate
    if (RSTACT_CONFIG[`RSTA_ENA_b]) begin : rstact_supp
      reg [2:0] slvr_rstact;
      reg [3:0] def_byte_sr;
      localparam CUS_RST = (8'h40|RSTACT_CONFIG[`RSTA_VAL_CUS_b]); // custom
      
      wire [2:0] idata_map = ({3{idata_byte[6:0]==7'h00}}   & `RACT_NONE) |
                             ({3{idata_byte[6:0]==7'h02}}   & `RACT_FULL) | 
                             ({3{idata_byte[6:0]==CUS_RST}} & `RACT_CUST & 
                              {3{RSTACT_CONFIG[`RSTA_CUS_b]}}); // custom ena and matched
      // Note that this is a defining byte when Direct, so this
      // is more complex than the normal scheme; the idata_done
      // will signal for the byte following the CCCC, so we have
      // to understand it.
      always @ (posedge clk_SCL_n or negedge RSTn)
        if (!RSTn) begin
          slvr_rstact     <= `RACT_DEF;
          def_byte_sr     <= 3'd0;
        end else if (i_rst_slvr_reset | ccc_real_START)
          slvr_rstact     <= `RACT_DEF;  // back to default
        else if ((ccc_handle_r==CHAN_RSTACT) & ~ccc_is_read & ~new_ccc)
          if (idata_done & (ccc_state==CST_BCAST_CCC)) // broadcast
            slvr_rstact   <= idata_map;  // direct assign
          else if (ccc_state==CST_DIRECT_CCC) begin
            if (idata_done & ~int_da_matched)
              def_byte_sr <= {idata_byte[7], idata_map};
            else if (~def_byte_sr[3] & int_da_matched & ~is_get)
              slvr_rstact <= def_byte_sr[2:0]; // pick up request if not get 
          end

      // clear reset is RSTACT or GETSTATUS; RSTACT can be GET or SET
      wire clr_reset = (idata_done | (ccc_tb_now & is_9th)) & 
                        ((ccc_handle_r==CHAN_RSTACT) | (ccc_handle_r==CHAN_STATUS));
      // assign opt_slvr_reset = {clr_reset|clr_slvr_cause, slvr_rstact}; -- would stop escalate on peripheral reset
      assign opt_slvr_reset = {clr_reset, slvr_rstact};
      wire [7:0] map_slvr = ({8{slvr_rstact==`RACT_DEF}}  & 8'h01) | 
                            ({8{slvr_rstact==`RACT_FULL}} & 8'h02) |
                            ({8{slvr_rstact==`RACT_NONE}} & 8'h00) |
                            ({8{slvr_rstact==`RACT_CUST}} & CUS_RST);
      assign get_slvr_value = ~def_byte_sr[3] ? map_slvr :
                              (def_byte_sr[2:0]==`RACT_DEF)  ? cf_RstActTim[7:0] :
                              (def_byte_sr[2:0]==`RACT_FULL) ? cf_RstActTim[15:8] :
                              (def_byte_sr[2:0]==`RACT_CUST) ? cf_RstActTim[23:16] :
                                                               8'd0;
    end else begin
      assign opt_slvr_reset = 4'd0;
      assign get_slvr_value = 8'h00;    // not used, so no meaning
    end
  endgenerate

  generate
    if (ID_AS_REGS[`IDREGS_VGPIO_b]) begin : vgpio_ccc_supp
      //FREE_VERSION_CUT - remove VGPIO from free version
    end else 
    begin
      assign ccc_vgpio      = 8'h00;
      assign ccc_vgpio_done = 2'b00;
    end
  endgenerate
 
  // SETAASA is standalone since in BASIC as one adder
  assign opt_setaasa    = ENA_CCC_HANDLING[`ENCCC_AASA] &
                          (ccc_handle_r == CHAN_SAASA);

  generate
    if (ENA_MASTER) begin : mstacc_ccc
      reg mast_accepted;
      always @ (posedge clk_SCL_n or negedge RSTn)
        if (!RSTn) 
          mast_accepted <= 1'b0;
        else if ((ccc_handle_r==CHAN_GMST) & ~|use_cnt[3:0])
          mast_accepted <= 1'b1;        // set at end
        else if (~cf_MasterAcc)
          mast_accepted <= 1'b0;        // clear when master switches
        else if (int_start_seen)
          mast_accepted <= 1'b0;        // killed by Sr vs. P

      assign opt_MasterAcc = mast_accepted;
    end else begin
      assign opt_MasterAcc = 1'b0;
    end
  endgenerate

  generate 
    if (ENA_CCC_HANDLING[`ENCCC_MAXES_b]) begin : maxes_ccc
      // for the GETs, we build the bit walkers and they get ORed in
      // above with other GETs. for SET, we just do here.
      reg [11:0] max_chg;
      reg        chgw, chgr;
      reg  [1:0] lcnt;

      assign max_wrlen = {3'd0,cf_MaxWr,1'b0}; // note wraparound addr
      assign max_rdlen = {3'd0,cf_MaxRd,1'b0}; // note wraparound addr
      if (|MAX_DS_RDTURN) // RDTURN is LSB order
        assign max_ds    = {MAX_DS_WR[7:0],MAX_DS_RD[7:0],MAX_DS_RDTURN[7:0],
                            MAX_DS_RDTURN[15:8],MAX_DS_RDTURN[23:16],1'b0};
      else
        assign max_ds    = {MAX_DS_WR[6:0],MAX_DS_RD[7:0],MAX_DS_WR[7]};

      // Set Max len is broadcast and direct; we get 2 bytes with
      // the broadcast command, and direct is for us only
      always @ (posedge clk_SCL_n or negedge RSTn)
        if (!RSTn) begin
          max_chg     <= 12'd0;
          chgw        <= 1'b0;
          chgr        <= 1'b0;
          lcnt        <= 2'd0;
        end else if ((ccc_handle_r == CHAN_SMXWL) | (ccc_handle_r == CHAN_SMXRL)) begin
          if (idata_done & 
              ((ccc_state==CST_BCAST_CCC) | (~|cnt[2:0] & (ccc_state==CST_DIRECT_CCC)))) begin
            if (~|lcnt) begin
              max_chg[11:8] <= idata_byte[3:0];
              lcnt    <= 2'd1;
            end else if (lcnt == 2'd1) begin
              max_chg[7:0]  <= idata_byte;
              if (ccc_handle_r == CHAN_SMXWL)
                chgw  <= 1'b1;
              else
                chgr  <= 1'b1;
              lcnt    <= 2'd2;
            end // FUTURE: if we allow larger IBI byte counts, pick up 3rd byte for read
          end else begin
            chgw      <= 1'b0;
            chgr      <= 1'b0;
          end
        end else begin
          lcnt        <= 2'd0;
          max_chg     <= 12'd0;
          chgw        <= 1'b0;
          chgr        <= 1'b0;
        end
      assign opt_ChgMaxRd = chgr;
      assign opt_ChgMaxWr = chgw;
      assign opt_MaxRdWr  = max_chg;

    end else begin
      assign opt_ChgMaxRd = 1'b0;
      assign opt_ChgMaxWr = 1'b0;
      assign opt_MaxRdWr  = 12'd0;
      assign max_wrlen    = 16'd0;
      assign max_rdlen    = 16'd0;
      assign max_ds       = 25'd0;
    end
  endgenerate


  // counter handles the CCC commands and also the DAA mode ID process
  // We reset when not in use so cannot cause problems (including on STOP)
  assign rst_daa_n = RSTn & (|state_in_CCC[`CF_ONLY_b] | scan_no_rst);
  `Observe(observe_daa_rst, clk_SCL, |state_in_CCC) // optional DFT observer

  always @ (posedge clk_SCL_n or negedge rst_daa_n)
    if (!rst_daa_n)
      cnt     <= 7'h00;
    else if (~daa_active) begin
      if (ccc_handle_r == CHAN_DAA)
        cnt   <= 7'h7F;                 // prepared for DAA
      else if (|ccc_handle_r) begin
        // one we handle
        if (int_da_matched & (ccc_state == CST_DIRECT_CCC)) begin
          cnt <= load;
        end else if (|cnt & ~is_9th)
          cnt <= cnt - 7'd1;            // down count for other uses
      end
    end else if (|cnt)
      cnt     <= cnt - 7'd1;            // down count
  assign ccc_counter = cnt;


endmodule


