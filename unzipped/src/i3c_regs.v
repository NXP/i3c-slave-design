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
//  File            : i3c_regs.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Wed Dec 11 18:20:38 2019 $
//  Revision        : $Revision: 1.79.1.1 $
//
//  IP Name         : i3c_regs
//  Description     : MIPI I3C Memory Mapped registers for local processor use
//    This contains the memory mapped registers over an APB or APB like
//    bus (can be mapped from AHB or AXI for example). 
//    Actual registers based on parameters selected.
//    NOTE: this does not use PWDATA in 1st cycle, so can be mapped
//          from AHB or similar 2 phase bus easily. Can also be a
//          1 cycle write (PSEL+PENA) if wanted.
//    NOTE: this *does* rely on Read being 2 cycles for certain registers
//          because it uses the 1st cycle to register potentually 
//          metastable status bits (pre-read). If a 1 cycle read is needed,
//          then those bits should be masked off or use models should ensure
//          no combinatorial uses of those bits.
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
//  Note use of generate for features configured in or out - this makes it easier
//  for coverage vs. just pruned out logic.
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_regs #(
    // params are driven from upper layer to control how this block
    // is built and operates
    parameter ENA_ID48B       = `ID48B_CONST, // const vs. reg
    parameter  ID_48B         = 48'h0,  // must be filled in from above with some/all of 48-bit ID
    parameter ID_AS_REGS      = 12'd0,  // reg for ID and other uses (e.g. Masks)
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
    parameter  CLK_SLOW_BITS  = 6,      // number of bits needed for count for Bus Avail
    parameter  CLK_SLOW_MATCH = 6'd47,  // count: e.g. 47 for 1us if 48MHz CLK (0 rel)
    parameter  CLK_SLOW_HJMUL = 10'd1000,// number of MATCHes (const or reg) for 1ms (1 rel)
    parameter  ERROR_HANDLING = 3'd0,
    parameter ENA_CCC_HANDLING= 6'd0,   // passed down as to what to support
    parameter RSTACT_CONFIG   = 26'd0,  // Slave Reset RSTACT CCC config (see RSTA_xxx_b fields)
    parameter MAX_RDLEN       = 0,      // default S->M len max
    parameter MAX_WRLEN       = 0,      // default M->S len max
    parameter MAX_DS_WR       = 0,      // data speed limits for M->S
    parameter MAX_DS_RD       = 0,      // data speed limits for S->M
    parameter MAX_DS_RDTURN   = 0,      // latency needs for S->M read req
    parameter SEL_BUS_IF      = 0,      // what bus support
    parameter FIFO_TYPE       = 0,      // if FIFO, the width
    parameter  EXT_FIFO       = 3'd0,   // if not 0, then external FIFO is selected
    parameter  ENA_TOBUS_FIFO = 0,      // depth of to-bus
    parameter  ENA_FROMBUS_FIFO=0,      // depth of from-bus
    parameter ENA_HDR         = 0,      // enables for HDR modes
    parameter ENA_TIMEC       = 6'b000010,    // clock_reg, res, res, mode 1, mode 0, sync
    parameter TIMEC_FREQ_ACC  = {8'd24,8'd10},// freq=12MHz (12.0=24) with 1.0% accuracy
    parameter BLOCK_ID        = 0,      // if not 0, returns an ID at FFC
    parameter ENA_MASTER      = 0,      // if 1, then we support master
    parameter [7:0] PID_CNT   = MAP_DA_AUTO[`MAPDA_DAA_PID_lb+4:`MAPDA_DAA_PID_lb] // computed
  )
  (
  input               PRESETn,          // reset from system
  input               PCLK,             // system clock for regs and sync
  // define APB bus
  input               PSEL,             // select (with penable)
  input               PENA,             // actual read or write cycle enable
  input        [11:2] PADDR,            // address - word aligned
  input               PWRITE,           // is write (data phase and addr are same)
  output       [31:0] PRDATA,           // read reg
  input        [31:0] PWDATA,           // write reg
  output              PREADY,           // Stall
  output              wr_err,           // write to RO regs error
  input         [1:0] ign_mwrite,       // special (master only) to suppress regwrite
  // now the registers that are output from block to SCL domain
  // some may be unused if not enabled by param
  output              cf_SlvEna,        // only process bus if 1
  output              cf_SlvNack,       // do not ACK private msgs
  output        [7:0] cf_SlvSA,         // optional slave i2c address
  output        [3:0] cf_IdInst,        // Instance of ID when selected (not if partno used)
  output              cf_IdRand,        // random partno of ID if used
  output              cf_Offline,       // if 1 when SlvEna to 1, then check bus state
  output       [31:0] cf_Partno,        // partno od ID if used
  output        [7:0] cf_IdBcr,         // BCR of DAA if not const
  output        [7:0] cf_IdDcr,         // DCR of DAA if not const
  output       [14:0] cf_IdVid,         // MIPI Vendor ID if not const
  output              cf_DdrOK,         // Allow DDR messages
  output              cf_TspOK,         // Allow TSP messages
  output              cf_TslOK,         // Allow TSL messages
  output       [11:0] cf_MaxRd,         // Max read in bytes
  output       [11:0] cf_MaxWr,         // Max write in bytes
  output       [23:0] cf_RstActTim,     // Optional time values (vs. param)
  output        [7:0] cf_BAMatch,       // Match for Bus Avail counter
  output       [15:0] cf_TCclk,         // Time control clock info if reg based
  output              cf_s0ignore,      // suppress S0 error
  output              cf_matchss,       // Start/Stop on Matched=1 only
  output              cf_IbiExtData,    // IBI extended data
  output        [3:0] cf_IbiMapIdx,     // Mapped DA for IBI
  output        [2:0] cf_i2c_dev_rev,   // Rev of i2c device ID if used
  output        [1:0] cf_HdrCmd,        // If HDR CMD as MMR (R,W)
  output        [8:0] cf_vgpio,         // control for VGPIO as CCC
  output        [6:0] cf_CccMask,       // mask of unhandled CCCs
  output  [MAP_CNT-1:0] map_daa_use,      // which MAP are auto-DAA
  output[(MAP_CNT*8)-1:0] map_daa_dcr,      // DCRs if MAP auto-DAA
  output[(MAP_CNT*PID_CNT)-1:0] map_daa_pid, // PID partials if MAP auto-DAA
    // next relates to ID_ASREGS allowing DA override. The one after
    // for mapped DAs and SAs if enabled
  output        [7:0] SetDA,            // set DA if configured for it
  output [(MAP_CNT*10)-1:0] SetMappedDASA, // list of DAs or SAs if used
  output        [2:0] SetSA10b,         // 10-bit SA in index 1 if used
  // next two are for master+slave builds
  input               is_slave,         // 1 if slave vs. master
  input               master_comp,      // special for Master only for DMA
  // Now special states from SCL domain raw - we sync in Status reg
  // read. Some may be synchronized in upper layer for other uses
  input       [29:28] raw_ActState,     // activity state on bus
  input       [27:24] raw_EvState,      // event states (HJ,P2P,MR,IBI)
  input         [2:0] raw_TimeC,        // enabled time control if any - one hot
  input         [6:0] raw_Request,      // bus request in process w/details
  input         [7:0] raw_DynAddr,      // dynamic address if set - is stable
  input         [2:0] raw_DynChgCause,  // Indicates how DA set
  input        [12:0] raw_match_idx,    // last 3 matching indexes if mapped addrs
  // Now registers which are input and output in same PCLK domain
  // So, synchronized as needed above this layer
  output       [19:8] reg_IntEna,       // interrupts we have enabled
  output        [5:0] reg_DmaCtrl,      // dma control bits if used
  input        [19:8] inp_IntStates,    // interrupt status; see `IS_xx
  output        [2:0] reg_EvPend,       // Event request. [2] is flag for change
  output        [7:0] reg_EvIbiByte,    // optional byte (req if BCR said so)
  input               inp_EvNoCancel,   // stops cancel if IBI happening now
  input       [22:20] inp_EvDet,        // details of Event pending/done [with 18]
  input         [5:0] inp_GenErr,       // general errors from engines
  input        [11:8] inp_DataErr,      // data errors from engines
  output        [5:0] msk_GenErr,       // mask for Status bit and IRQ
  output       [11:8] msk_DataErr,      // mask for Status bit and IRQ
  input               inp_err_loc,      // used for DMA stop
  output       [19:8] reg_clrIntStates, // clear int status when bits are 1
  output        [5:0] reg_clrGenErr,    // clear errors when bits are 1 
  output       [11:8] reg_clrDataErr,   // clear errors when bits are 1 
  output              reg_holdOErr,     // hold int err if overruns are 1
  input               inpflg_MaxRd,     // Master changed MaxRd
  input               inpflg_MaxWr,     // Master changed MaxWr
  input        [11:0] inp_MaxRW,        // Value from Master for Rd or Wr
  output              reg_TbEnd,        // end of to-bus data
  output              reg_TbFlush,      // 1 cycle flush internal buff pulse
  output              reg_FbFlush,      // 1 cycle flush internal buff pulse
  input               inp_dma_last_tb,  // PCLK domain from DMA
    // next 4 have special meaning if internal FIFO used
  output        [5:4] reg_TxTrig,       // trigger level for TX (tb)
  output        [7:6] reg_RxTrig,       // trigger level for RX (fb)
  input       [20:16] inp_TxCnt,        // current counter for TX (tb)
  input       [28:24] inp_RxCnt,        // current counter for RX (fb)
  input               inp_TxFull,       // TX FIFO is full
  input               inp_RxEmpty,      // RX FIFO is empty
  output        [1:0] regflg_wr_cnt,    // tb: 0=none, 1 = wrote b. [1] not used
  output        [7:0] reg_wdata,        // tb: byte
  output        [1:0] regflg_rd_cnt,    // fb: 0=none, 1 = read b. [1] not used
  input         [7:0] inp_fb_data,      // fb: B
  output        [8:7] reg_ActMode,      // System activity mode (or 0 if not used)
  output        [3:0] reg_PendInt,      // Pending interrupt (or 0 if not used)
  output       [15:8] reg_StatusRes,    // reserved bits of status (or 0 if not used) 
  input               hdr_new_cmd,      // Pulse in PCLK domain (sync)
  input         [7:0] raw_hdr_cmd,      // raw but new_cmd syncs
  input               map_rstdaa,       // pulse in PCLK if MAP auto and rstdaa happened
  input               map_setaasa,      // pulse in PCLK if SETAASA and MAP auto
  input               map_daa_ena,      // pulse in PCLK domain if MAP DASA/DAA
  input         [3:0] map_sa_idx,       // index if daa_ena
  input         [7:1] map_daa_da,       // new DA if daa_ena
  output       [10:0] ibi_wr_fifo,      // 4 signals for IBI push if enabled
  input               ibi_wr_ack,       // 1 signal for ack
  // next two are just for Master is used, else ignored
  output              exp_owrite_err,
  output              exp_oread_err
  `ifdef USE_D_RESET
  ,input        d_reset_r               // special D input reset for one project
  `endif
  );

  wire                is_read, is_pre_read, is_write;
  reg          [14:0] pre_reg;          // for metastable bits
  wire                ma_config, ma_stat, ma_ctrl, ma_intset, ma_intclr;
  wire                ma_intmasked, ma_errwarn, ma_dmactrl, ma_datactrl;
  wire                ma_wdatab, ma_wdatabe, ma_rdatab, ma_capable, ma_capable2;
  wire                ma_dynaddr, ma_maxlimits, ma_idpartno, ma_idext, ma_idvid;
  wire                ma_wdatah, ma_wdatahe, ma_rdatah; // includes the sdr/ddr cases
  wire                ma_wibidata, ma_wdatab1, ma_tcclk, ma_msgmap, ma_rsttim, ma_id;
  wire                ma_vgpio, ma_hdrcmd, ma_cccmask, ma_errwarnmask;
  wire                ma_mapctrl0, ma_mapctrln;
  wire                ma_merrwarn;      // special for master only
  wire         [31:0] dmac, maxl, idpart, idext;
  wire         [14:0] idvid;
  // now the regs (mapped or _r) that we write
  reg                 slv_ena;
  reg                 slv_nack;
  wire        [31:25] opt_slv_saddr;    // 0 if not enabled for
  reg                 s0ignore_r;
  reg                 matchss_r;
  wire                opt_idrand;
  reg                 slv_offline;
  wire                opt_ddrok, opt_tspok, opt_tslok;
  wire          [2:0] opt_ctrl_ev;
  wire         [15:8] opt_ctrl_ibidata;
  wire        [19:16] opt_pendint;
  wire        [21:20] opt_actmode;
  wire        [31:24] opt_statusres;
  wire          [7:0] opt_bamatch;
  wire         [15:0] opt_tcclk;
  wire          [5:4] opt_txtrig;
  wire          [7:6] opt_rxtrig;
  wire                opt_ibi_wr_empty;
  wire         [23:0] opt_rsttimes;
  wire          [1:0] opt_hdrcmd;
  wire          [1:0] hdr_cmd;
  wire         [15:8] opt_vgpio_match;
  wire                opt_vgpio_ccc;
  wire          [7:0] write_data;       // reg is byte
  wire          [7:0] read_buff;        // 8 bits for read data
  wire          [7:0] read_buff2;
  wire         [19:8] int_ena;
  reg                 tb_end_r;         // written reg or high bit of data
  reg                 tb_flush_r;
  reg                 fb_flush_r;
  wire         [31:0] capable;          // built from params
  wire         [31:0] capable2;         // "
  reg                 oread_err;
  reg                 owrite_err;
  wire          [3:0] MapLastIdx;
  wire          [4:0] MapLastDet;       // Nack,SA10,SA
  wire          [7:0] MapLastDA;
  wire         [31:0] mapctrl_reg;      // for display of map reg

  // bus properties
  assign is_read     = PSEL & PENA  & ~PWRITE;
  assign is_pre_read = PSEL & ~PENA & ~PWRITE;
  assign is_write    = PSEL & PENA  & PWRITE;
  assign PREADY  = 1;                   // unused

  // see which register is being read or written
  generate                              // some are only if allowed
    assign ma_config     = PADDR == (12'h004>>2);
    assign ma_stat       = PADDR == (12'h008>>2);
    assign ma_ctrl       = PADDR == (12'h00C>>2);
    if (SEL_BUS_IF[`SBIF_IRQ_b]) begin : allow_irq
      assign ma_intset   = PADDR == (12'h010>>2);
      assign ma_intclr   = PADDR == (12'h014>>2);
      assign ma_intmasked= PADDR == (12'h018>>2);
    end else begin
      assign ma_intset   = 1'b0;
      assign ma_intclr   = 1'b0;
      assign ma_intmasked= 1'b0;
    end
    assign ma_errwarn    = PADDR == (12'h01C>>2);
    assign ma_merrwarn   = ENA_MASTER[0]&(PADDR == (12'h09C>>2));
    assign ma_dmactrl    = SEL_BUS_IF[`SBIF_DMA_b] &
                           ((PADDR == (12'h020>>2)) | (ENA_MASTER[0]&(PADDR == (12'hA0>>2))));
    assign ma_datactrl   = (PADDR == (12'h02C>>2)) | (ENA_MASTER[0]&(PADDR == (12'hAC>>2)));
    assign ma_wdatab     = (PADDR == (12'h030>>2)) | (ENA_MASTER[0]&(PADDR == (12'hB0>>2)));
    assign ma_wdatab1    = (PADDR == (12'h054>>2)) | (ENA_MASTER[0]&(PADDR == (12'hCC>>2)));
    assign ma_wdatabe    = (PADDR == (12'h034>>2)) | (ENA_MASTER[0]&(PADDR == (12'hB4>>2)));
    assign ma_rdatab     = (PADDR == (12'h040>>2)) | (ENA_MASTER[0]&(PADDR == (12'hC0>>2)));
    assign ma_wdatah     = (SEL_BUS_IF[`SBIF_HALF_b] & (PADDR == (12'h038>>2))) | 
                           (ENA_MASTER[0]&(PADDR == (12'hB8>>2))) |
                           (ENA_MASTER[0]&(PADDR == (12'hD0>>2))) |
                           (ENA_MASTER[0]&(PADDR == (12'hD8>>2)));
    assign ma_wdatahe    = (SEL_BUS_IF[`SBIF_HALF_b] & (PADDR == (12'h03C>>2)))|
                           (ENA_MASTER[0]&(PADDR == (12'hBC>>2)));
    assign ma_rdatah     = (SEL_BUS_IF[`SBIF_HALF_b] & (PADDR == (12'h048>>2))) | 
                           (ENA_MASTER[0]&(PADDR == (12'hC8>>2))) |
                           (ENA_MASTER[0]&(PADDR == (12'hD4>>2))) |
                           (ENA_MASTER[0]&(PADDR == (12'hDC>>2)));
    assign ma_wibidata   = (ENA_IBI_MR_HJ[`EV_EXTFIFO_b]&(PADDR == (12'h050>>2)));
    assign ma_capable    = PADDR == (12'h060>>2);
    assign ma_capable2   = PADDR == (12'h05C>>2);
    assign ma_dynaddr    = PADDR == (12'h064>>2);
    assign ma_maxlimits  = ENA_CCC_HANDLING[`ENCCC_MAXES_b] & (PADDR == (12'h068>>2));
    assign ma_idpartno   = ((ENA_ID48B == `ID48B_CONST_PARTNO) |
                            (ENA_ID48B == `ID48B_CONST_NONE)) &
                           (PADDR == (12'h06C>>2));
    assign ma_idext      = (ID_AS_REGS[`IDREGS_BCR_b] | ID_AS_REGS[`IDREGS_DCR_b] |
                            (ENA_ID48B == `ID48B_CONST_INST) | (|MAP_I2CID)) &
                           (PADDR == (12'h070>>2));
    assign ma_rsttim     = RSTACT_CONFIG[`RSTA_MMR_b] & (PADDR == (12'h100>>2));
    assign ma_idvid      = ID_AS_REGS[`IDREGS_VID_b] & (PADDR == (12'h074>>2));
    assign ma_tcclk      = ENA_TIMEC[`TC_FREQ_REG] & (PADDR == (12'h078>>2));
    assign ma_msgmap     = ENA_MAPPED[`MAP_ENA_b] & (PADDR == (12'h07C>>2));
    assign ma_vgpio      = ID_AS_REGS[`IDREGS_VGPIO_b] & (PADDR == (12'h104>>2));
    assign ma_hdrcmd     = ID_AS_REGS[`IDREGS_HDRCMD_b] & (PADDR == (12'h108>>2));
    assign ma_cccmask    = ID_AS_REGS[`IDREGS_CCCMSK_b] & (PADDR == (12'h10C>>2));
    assign ma_errwarnmask= ID_AS_REGS[`IDREGS_MSK_EW_b] & (PADDR == (12'h110>>2));
    localparam MPMX = 12'h11C+(MAP_CNT*4);
    assign ma_mapctrln   = ENA_MAPPED[`MAP_ENA_b] & 
                             ((PADDR >= (12'h120>>2)) & (PADDR <= (MPMX>>2)));
    assign ma_mapctrl0   = ENA_MAPPED[`MAP_ENA_b] & (PADDR == (12'h11C>>2));
    assign ma_id         = PADDR == (12'hFFC>>2);
  endgenerate


  // write error is from writes to RO regs. Note that partially RO regs are not
  // checked for mask, so only pure RO included.
  assign wr_err = is_write & 
                  (ma_intmasked | ma_rdatab | ma_rdatah | ma_capable | ma_capable2 |
                   ma_msgmap | ma_id);
  // return value - note that CDC handled in different ways depending on field
  assign PRDATA = {32{is_read}} & (
    ({32{ma_config}}   & {cf_SlvSA[7:1],1'b0,cf_BAMatch,4'd0,hdr_cmd,cf_Offline,
                          cf_IdRand,1'b0,cf_TslOK,cf_TspOK,cf_DdrOK,cf_s0ignore,
                          cf_matchss,cf_SlvNack,cf_SlvEna}) |
    ({32{ma_stat}}     & {pre_reg[14:7],2'd0,inp_EvDet[21:20],inp_IntStates[19:8],
                          1'b0,pre_reg[6:0]}) |
    ({32{ma_ctrl}}     & {opt_statusres,2'b00,opt_actmode, opt_pendint,
                          opt_ctrl_ibidata[15:8],cf_IbiMapIdx,cf_IbiExtData,1'b0,
                          opt_ctrl_ev[1:0]}) |
    ({32{ma_intset}}   & {12'd0,int_ena,8'd0}) |
    ({32{ma_intclr}}   & {12'd0,int_ena,8'd0}) |
    ({32{ma_intmasked}}& {12'd0,int_ena[19:8]&inp_IntStates[19:8],8'd0}) |
    ({32{ma_errwarn}}  & {14'd0,owrite_err,oread_err,4'd0,inp_DataErr,2'd0,inp_GenErr}) |
    ({32{ma_datactrl}} & {inp_RxEmpty,inp_TxFull,1'b0,inp_RxCnt,3'd0,inp_TxCnt, 
                          8'd0,opt_rxtrig,opt_txtrig, 2'b00,2'b00}) |
    ({32{ma_rdatab}}   & {24'd0,read_buff}) |
    ({32{ma_rdatah}}   & {16'd0,read_buff,read_buff2}) |
    ({32{ma_wibidata}} & {31'd0, opt_ibi_wr_empty}) |
    ({32{ma_capable}}  & {capable}) |
    ({32{ma_capable2}} & {capable2}) |
    ({32{ma_dynaddr}}  & (|MapLastIdx ?
                          {15'd0,MapLastDet,MapLastIdx,MapLastDA} :
                          {15'd0,SetDA[0],5'd0, 
                           SetDA[0]?{3'd0,SetDA[7:0]} :
                                    {pre_reg[10:8],{8{pre_reg[0]}} & pre_reg[7:0]}})) |
    // below are optional, but will be 0 if not used
    ({32{ma_dmactrl}}  & {dmac}) |
    ({32{ma_maxlimits}}& {maxl}) |
    ({32{ma_idpartno}} & {idpart}) |
    ({32{ma_idext}}    & {idext}) |
    ({32{ma_idvid}}    & {17'd0,idvid}) |
    ({32{ma_tcclk}}    & {16'd0,opt_tcclk}) |
    ({32{ma_msgmap}}   & {12'd0,pre_reg[11:8],4'd0,pre_reg[7:4],3'd0,pre_reg[12],pre_reg[3:0]}) |
    ({32{ma_rsttim}}   & {8'd0,opt_rsttimes[23:0]}) |
    ({32{ma_vgpio}}    & {16'd0,opt_vgpio_match[15:8],7'd0,opt_vgpio_ccc}) |
    ({32{ma_hdrcmd}}   & {opt_hdrcmd[1:0],22'd0,raw_hdr_cmd[7:0]}) | // raw protected by flag
    ({32{ma_cccmask}}  & {25'd0,cf_CccMask[6:0]}) |
    ({32{ma_errwarnmask}}&{14'd0,6'd0,msk_DataErr,2'd0,msk_GenErr}) |
    ({32{ma_mapctrl0}} & {15'd0,SetDA[0],5'd0, 
                           SetDA[0]?{3'd0,SetDA[7:0]} :
                                    {pre_reg[10:8],{8{pre_reg[0]}} & pre_reg[7:0]}}) |
    ({32{ma_mapctrln}} & mapctrl_reg) |
    ({32{ma_id}}       & {BLOCK_ID})    // ID and revision if provided
    );


  // pre_reg holds possibly metastable bits in 1st cycle of read,
  // to return in 2nd cycle (safely)
  wire [1:0] timec = raw_TimeC[0] ? (|raw_TimeC[2:1] ? 2'd3 : 2'd1) : 
                     |raw_TimeC[2:1] ? 2'd2 : 2'd0;
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) 
      pre_reg   <= 15'd0;               // only what must be registered
    else if (is_pre_read)
      if (ma_stat)                      // register live data
        pre_reg[14:0] <= {timec,raw_ActState[29:28],
                          ~raw_EvState[27:24] & 4'b1011, // no P2P
                          raw_Request[6:0]};
      else if (ma_dynaddr | ma_mapctrl0)
        pre_reg[10:0]  <= {raw_DynChgCause,raw_DynAddr};
      else if (ma_msgmap)
        pre_reg[12:0] <= raw_match_idx;

  //
  // Optional read regs and fields of regs
  //

  // below are config nets only used if enabled, so no harm used this way
  assign cf_SlvEna    = slv_ena;
  assign cf_SlvNack   = slv_nack;
  assign cf_SlvSA[7:1]= opt_slv_saddr;  // optional i2c SA
  assign cf_SlvSA[0]  = |opt_slv_saddr; // only valid if not 0
  assign cf_IdInst    = idext[3:0];
  assign cf_i2c_dev_rev= idext[6:4];
  assign cf_IdRand    = opt_idrand;
  assign cf_Offline   = slv_offline;
  assign cf_Partno    = idpart;
  assign cf_IdBcr     = idext[23:16];
  assign cf_IdDcr     = idext[15:8];
  assign cf_IdVid     = idvid;
  assign cf_DdrOK     = opt_ddrok;
  assign cf_TspOK     = opt_tspok;
  assign cf_TslOK     = opt_tslok;
  assign cf_MaxRd     = maxl[11:0];
  assign cf_MaxWr     = maxl[27:16];
  assign cf_RstActTim = opt_rsttimes;
  assign cf_BAMatch   = opt_bamatch;
  assign cf_TCclk     = opt_tcclk;
  assign cf_s0ignore  = s0ignore_r;
  assign cf_matchss   = matchss_r;
  assign cf_vgpio     = {opt_vgpio_ccc,opt_vgpio_match[15:8]};
  // now these below are active nets staying in our domain
  // some are optional and so will never change
  assign reg_IntEna   = int_ena;
  assign reg_DmaCtrl  = dmac[5:0];
  assign reg_TbEnd    = tb_end_r;       // datactrl or wdata high bit
  assign reg_TbFlush  = tb_flush_r;     // 1 cycle to flush internal buff
  assign reg_FbFlush  = fb_flush_r;     // 1 cycle to flush internal buff
  assign reg_TxTrig   = opt_txtrig;
  assign reg_RxTrig   = opt_rxtrig;
  assign reg_wdata    = write_data;     // byte or byte end
  assign reg_clrGenErr  = (is_write & ma_errwarn) ? PWDATA[5:0]  : 6'd0;
  assign reg_clrDataErr = (is_write & ma_errwarn) ? PWDATA[11:8] : 4'd0;
  assign reg_holdOErr = oread_err | owrite_err;
  assign reg_EvPend   = slv_ena ? opt_ctrl_ev : 3'd0;
  assign reg_EvIbiByte= opt_ctrl_ibidata;
  assign reg_PendInt  = opt_pendint;
  assign reg_ActMode  = opt_actmode;
  assign reg_StatusRes= opt_statusres;

  //
  // Optional ones - write and read
  //

  // we have a special `define for D-input reset, which is not normally used. It
  // is a `define so it is isolated completely
  `ifdef USE_D_RESET
    `define D_RESET d_reset_r
  `else
    `define D_RESET 0
  `endif
  generate 
    // opt static addr of config
    if (ENA_SADDR == `SADDR_CONFIG) begin : saddr_reg
      reg       [7:1] saddr_r;
      assign opt_slv_saddr = saddr_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  saddr_r <= 7'd0;
        else if (`D_RESET)             saddr_r <= 7'd0;
        else if (is_write & ma_config) saddr_r <= PWDATA[31:25];
    end else begin
      assign opt_slv_saddr = 7'd0;
    end

    // Bus available count match
    if (ENA_IBI_MR_HJ[`EV_BAMATCH_b]) begin : bamatch_reg
      reg [CLK_SLOW_BITS-1:0] bamatch_r;
      assign opt_bamatch = bamatch_r; // may not be more than 8 bits
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  bamatch_r <= CLK_SLOW_MATCH[CLK_SLOW_BITS-1:0]; 
        else if (`D_RESET)             bamatch_r <= CLK_SLOW_MATCH[CLK_SLOW_BITS-1:0]; 
        else if (is_write & ma_config) bamatch_r <= PWDATA[(16+CLK_SLOW_BITS-1):16];
    end else begin
      assign opt_bamatch = 0;
    end

    // Time control clock info if reg based
    if (ENA_TIMEC[`TC_FREQ_REG]) begin : tcclk_reg
      reg [15:0] tcclk_r;
      assign opt_tcclk = tcclk_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                 tcclk_r <= TIMEC_FREQ_ACC[15:0]; 
        else if (`D_RESET)            tcclk_r <= TIMEC_FREQ_ACC[15:0]; 
        else if (is_write & ma_tcclk) tcclk_r <= PWDATA[15:0];
    end else begin
      assign opt_tcclk = 16'd0;
    end

    // if Id rand is a field of config
    if (ID_AS_REGS[`IDREGS_RAND_b]) begin : rand_reg
      reg             idrand_r;
      assign opt_idrand = idrand_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  idrand_r <= 1'b0; 
        else if (`D_RESET)             idrand_r <= 1'b0; 
        else if (is_write & ma_config) idrand_r <= PWDATA[8];
    end else begin
      assign opt_idrand = 1'b0;
    end

    // 3 HDR forms with OK regs in config
    if (ENA_HDR[`HDR_DDR_b]) begin : ddr_reg
      reg               ddr_ok_r;
      assign opt_ddrok = ddr_ok_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  ddr_ok_r <= 1'b0; 
        else if (`D_RESET)             ddr_ok_r <= 1'b0; 
        else if (is_write & ma_config) ddr_ok_r <= PWDATA[4];
    end else begin
      assign opt_ddrok = 1'b0;
    end
    if (0 /*ENA_HDR[`HDR_TSP_b]*/) begin : tsp_reg
      reg               tsp_ok_r;
      assign opt_tspok = tsp_ok_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  tsp_ok_r <= 1'b0; 
        else if (`D_RESET)             tsp_ok_r <= 1'b0; 
        else if (is_write & ma_config) tsp_ok_r <= PWDATA[5];
    end else begin
      assign opt_tspok = 1'b0;
    end
    if (0 /*ENA_HDR[`HDR_TSL_b]*/) begin : tsl_reg
      reg               tsl_ok_r;
      assign opt_tslok = tsl_ok_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  tsl_ok_r <= 1'b0; 
        else if (`D_RESET)             tsl_ok_r <= 1'b0; 
        else if (is_write & ma_config) tsl_ok_r <= PWDATA[6];
    end else begin
      assign opt_tslok = 1'b0;
    end

    // Event pend and optionally IBI byte
    if (ENA_IBI_MR_HJ != 0) begin : ctrl_reg
      reg         [3:0] ctrl_ev_r; // 3:2 allows delay to flag
      reg               is_da;

      // we only permit events if allowed and correct change (0->ev and ev->0)
      wire ev_chg_ok = ~|PWDATA[1:0] |
                       (~|ctrl_ev_r[1:0] &
                          (((PWDATA[1:0]==2'd1)&ENA_IBI_MR_HJ[`EV_IBI_b]) |
                           ((PWDATA[1:0]==2'd2)&ENA_IBI_MR_HJ[`EV_MR_b])  |
                           ((PWDATA[1:0]==2'd3)&ENA_IBI_MR_HJ[`EV_HJ_b])));
      // below: HJ only if no DA, IBI and MR only if DA
      wire ev_allowed = ~|PWDATA[1:0] | (&PWDATA[1:0] & ~is_da) | (^PWDATA[1:0] & is_da);

      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          ctrl_ev_r        <= 4'd0;  
        else if (`D_RESET)
          ctrl_ev_r        <= 4'd0;  
        else if (inp_EvDet[22:20] == 3'd3) // one clock as 3, then 7
          ctrl_ev_r        <= 4'd0;        // EvDet will clear too
        else if (is_write & ma_ctrl) begin // note lower pri than test above
          if (ev_chg_ok & ~inp_EvNoCancel & ev_allowed)
            ctrl_ev_r[3:0] <= {2'b10,PWDATA[1:0]}; // 0 cancels if active
        end else if (ctrl_ev_r[3])
          ctrl_ev_r[3:2]   <= 2'b01;    // now signal SCL domain
      assign opt_ctrl_ev      = ctrl_ev_r[2:0];

      // next registers raw state in case changing 
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          is_da <= 1'b0;  
        else if (`D_RESET)
          is_da <= 1'b0;  
        else if (PSEL & ~PENA & PWRITE)
          is_da <= raw_DynAddr[0];

      if (ENA_MAPPED[`MAP_ENA_b]) begin : ibi_map_idx
        reg       [7:0] midx_r; // index of Mapped device if enabled
        wire [((MAP_CNT+1)*8)-1:0] addrs = {SetMappedDASA[(MAP_CNT*10)-1:(MAP_CNT*2)],raw_DynAddr};
          // make sure index is valid. If DA not enabled, then use base
        wire      [3:0] idx = (PWDATA[7:4]>MAP_CNT) ? 4'd0 : PWDATA[7:4];
        wire      [3:0] vidx = addrs[idx<<3] ? idx : 4'd0;
        always @ (posedge PCLK or negedge PRESETn)
          if (!PRESETn)
            midx_r <= 4'd0;  
          else if (`D_RESET)
            midx_r <= 4'd0;  
          else if (is_write & ma_ctrl) 
            midx_r <= vidx;
        assign cf_IbiMapIdx = midx_r;
      end else begin
        assign cf_IbiMapIdx = 4'd0;
      end
    end else begin
      assign opt_ctrl_ev   = 3'd0;
      assign cf_IbiMapIdx  = 4'd0;
    end
    // IBI data only if enabled
    if (ENA_IBI_MR_HJ[`EV_IBI_DAT_b]) begin : ibidata_reg
      reg        [15:8] ctrl_ibidata_r;
      reg               ctrl_extd_r;
      assign opt_ctrl_ibidata = ctrl_ibidata_r;
      assign cf_IbiExtData    = ctrl_extd_r;

      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          ctrl_ibidata_r   <= 8'd0;
          ctrl_extd_r      <= 1'b0;
        end else if (`D_RESET) begin
          ctrl_ibidata_r   <= 8'd0;
          ctrl_extd_r      <= 1'b0;
        end else if (is_write & ma_ctrl) begin
          if (~inp_EvNoCancel)
            ctrl_ibidata_r <= PWDATA[15:8];
          ctrl_extd_r      <= PWDATA[3];
        end
    end else begin
      assign opt_ctrl_ibidata = 8'd0;
      assign cf_IbiExtData    = 1'b0;
    end

    // CTRL fields for PENDINT and ACTSTATE
    if (ENA_CCC_HANDLING[`ENCCC_STATINF_b]) begin : status_fields
      reg       [19:16] ctrl_pendint_r;
      reg       [21:20] ctrl_actmode_r;
      assign opt_pendint = ctrl_pendint_r;
      assign opt_actmode = ctrl_actmode_r;

      // GETSTATUS fields
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          ctrl_pendint_r <= 4'd0;
          ctrl_actmode_r <= 2'd0;
        end else if (`D_RESET) begin
          ctrl_pendint_r <= 4'd0;
          ctrl_actmode_r <= 2'd0;
        end else if (is_write & ma_ctrl) begin
          ctrl_pendint_r <= PWDATA[19:16];
          ctrl_actmode_r <= PWDATA[21:20];
        end
    end else begin
      assign opt_pendint = 4'd0;
      assign opt_actmode = 2'd0;
    end

    // CTRL fields for VENDINFO
    if (ENA_CCC_HANDLING[`ENCCC_STATVEND_b]) begin : status_vend
      reg       [31:24] ctrl_statusres_r;
      assign opt_statusres = ctrl_statusres_r;

      // GETSTATUS fields
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) 
          ctrl_statusres_r <= 8'd0;
        else if (`D_RESET) 
          ctrl_statusres_r <= 8'd0;
        else if (is_write & ma_ctrl) 
          ctrl_statusres_r <= PWDATA[31:24];
    end else begin
      assign opt_statusres = 8'd0;
    end

    // opt dma control if supported
    if (SEL_BUS_IF[`SBIF_DMA_b]) begin : dma_reg
      reg         [5:0] dma_r;
       // note on below: they should set MATCHSS to use this
      wire              is_slv_done = is_slave & inp_IntStates[`IS_MATCHED] & 
                                      (inp_IntStates[`IS_STOP] | inp_IntStates[`IS_START]);
      wire              is_mst_done = ~is_slave & master_comp;
      assign dmac = {26'd0, dma_r};
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          dma_r        <= 6'h10;
        else if (`D_RESET)
          dma_r        <= 6'h10;
        else if (is_write & ma_dmactrl) begin
          dma_r[3:0]   <= PWDATA[3:0];  // DMA setting
          if (SEL_BUS_IF[`SBIF_HALF_b])
            dma_r[5:4] <= &PWDATA[5:4] ? 2'd0 : PWDATA[5:4];
        end else if (is_slv_done | is_mst_done | inp_err_loc) begin
          // Slave: auto-clear on START/STOP. Should only be used with matchss
          // Master: auto-clear on complete
          // below should be if statements but Spyglass lint cannot handle
          dma_r[3:2] <= (dma_r[3:2]==2'd1) ? 2'd0 : dma_r[3:2];
          dma_r[1:0] <= (dma_r[1:0]==2'd1) ? 2'd0 : dma_r[1:0];
        end
    end else begin
      assign dmac = 32'd0;                // unsupported
    end


    // max limits is for max read and write
    // note that MXDS is handled by constants (based on BCR[0])
    if (ENA_CCC_HANDLING[`ENCCC_MAXES_b]) begin : ccc_max
      reg        [11:0] maxrd_r, maxwr_r;
      assign maxl = {4'd0,maxwr_r,4'd0,maxrd_r};
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          maxrd_r <= MAX_RDLEN;
          maxwr_r <= MAX_WRLEN;
        end else if (`D_RESET) begin
          maxrd_r <= MAX_RDLEN;
          maxwr_r <= MAX_WRLEN;
        end else if (inpflg_MaxRd)      // M written (but sync)
          maxrd_r <= inp_MaxRW;
        else if (inpflg_MaxWr)          // M written (but sync)
          maxwr_r <= inp_MaxRW;
        else if (is_write & ma_maxlimits) begin
          // note that if app writes when Master is, app loses
          // so it should read back if that is a risk
          maxrd_r <= PWDATA[11:0];
          maxwr_r <= PWDATA[27:16];
        end
    end else begin
      assign maxl = 32'd0;
    end

    wire [7:0] custim = RSTACT_CONFIG[`RSTA_CUS_b] ? 
                        RSTACT_CONFIG[`RSTA_TIM_PER_b] : // use peripheral
                        8'd0;
    if (RSTACT_CONFIG[`RSTA_MMR_b]) begin : rstact_tim
      reg   [23:0] rsttimes_r;
      assign opt_rsttimes = rsttimes_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                   // init from params as starting values
          rsttimes_r <= {custim, 
                         RSTACT_CONFIG[`RSTA_TIM_SYS_b], 
                         RSTACT_CONFIG[`RSTA_TIM_PER_b]};
        else if (`D_RESET)             
          rsttimes_r <= {custim, 
                         RSTACT_CONFIG[`RSTA_TIM_SYS_b], 
                         RSTACT_CONFIG[`RSTA_TIM_PER_b]};
        else if (is_write & ma_rsttim) 
          rsttimes_r <= PWDATA[23:0];
    end else begin
      assign opt_rsttimes = {custim, 
                             RSTACT_CONFIG[`RSTA_TIM_SYS_b], 
                             RSTACT_CONFIG[`RSTA_TIM_PER_b]};
    end

    // VGPIO can be CCC as well as mapped
    if (ID_AS_REGS[`IDREGS_VGPIO_b] & ENA_MAPPED[`MAP_VGPIO_b]) begin : vpgio_reg
      reg [7:0] vgpio_match_r;          // CCC to match
      reg       vgpio_ccc_r;            // 1 if use CCC else map
      assign opt_vgpio_match = vgpio_match_r;
      assign opt_vgpio_ccc   = vgpio_ccc_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          vgpio_match_r <= 8'd0;
          vgpio_ccc_r   <= 1'b0;
        end else if (`D_RESET) begin
          vgpio_match_r <= 8'd0;
          vgpio_ccc_r   <= 1'b0;
        end else if (is_write & ma_vgpio) begin
          vgpio_match_r <= PWDATA[15:8];
          vgpio_ccc_r   <= PWDATA[0];
        end
    end else begin
      assign opt_vgpio_match[15:8] = 8'd0;
      assign opt_vgpio_ccc         = 1'b0; // only by Map
    end

    // HDR Command in MMR if enabled (param and CONFIG bit)
    if (ID_AS_REGS[`IDREGS_HDRCMD_b]) begin : hdrcmd_reg
      reg  [1:0] hdr_cmd_r;
      reg   hdr_new_r, hdr_oflow_r;

      assign hdr_cmd   = hdr_cmd_r;
      assign cf_HdrCmd = ~^hdr_cmd_r  ? 2'b00 : // 3 is not allowed
                         hdr_cmd_r[0] ? 2'b11 : // W and R to MMR
                                        2'b10;  // R only to MMR
      assign opt_hdrcmd = {hdr_new_r, hdr_oflow_r};

      // we get a pulse on new command (synchronized)
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          hdr_new_r     <= 1'b0;
          hdr_oflow_r   <= 1'b0;
        end else if (`D_RESET) begin
          hdr_new_r     <= 1'b0;
          hdr_oflow_r   <= 1'b0;
        end else if (hdr_new_cmd) begin
          hdr_new_r     <= 1'b1;
          if (hdr_new_r)
            hdr_oflow_r <= 1'b1;
        end else if (is_read & ma_hdrcmd) begin
          // clear on read
          hdr_new_r     <= 1'b0;
          hdr_oflow_r   <= 1'b0;
        end

      // enable from CONFIG for HDR command 
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          hdr_cmd_r <= 2'b00;
        else if (`D_RESET)
          hdr_cmd_r <= 2'b00;
        else if (is_write & ma_config) 
          hdr_cmd_r <= ~&PWDATA[11:10] ? PWDATA[11:10] : 2'b00;

    end else begin
      assign cf_HdrCmd  = 1'b0;         // never passed up
      assign opt_hdrcmd = 2'd0;
      assign hdr_cmd    = 2'd0;
    end

    // CCC Mask for unhandled CCC if used
    if (ID_AS_REGS[`IDREGS_CCCMSK_b]) begin : cccmask_reg
      reg [6:0] ccc_mask_r;
      assign cf_CccMask = ccc_mask_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) 
          ccc_mask_r <= 7'h7F;          // all enabled by default
        else if (`D_RESET) 
          ccc_mask_r <= 7'h7F;
        else if (is_write & ma_cccmask) 
          ccc_mask_r <= PWDATA[6:0];
    end else begin
      assign cf_CccMask = 7'h7F;        // all enabled by default
    end

    if (ID_AS_REGS[`IDREGS_MSK_EW_b]) begin : ewmask_reg
      reg [5:0]  msk_gen_r;
      reg [11:8] msk_data_r;
      assign msk_GenErr  = msk_gen_r;
      assign msk_DataErr = msk_data_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          msk_gen_r  <= 6'h3F;          // all enabled by default
          msk_data_r <= 4'hF;           // all enabled by default
        end else if (`D_RESET) begin
          msk_gen_r  <= 6'h3F;
          msk_data_r <= 4'hF;
        end else if (is_write & ma_errwarnmask) begin
          msk_gen_r  <= PWDATA[5:0];
          msk_data_r <= PWDATA[11:8];
        end
    end else begin
      assign msk_GenErr  = 6'h3F;
      assign msk_DataErr = 4'hF;
    end

    // Part number, whether real or random
    if ((ENA_ID48B == `ID48B_CONST_PARTNO) |
        (ENA_ID48B == `ID48B_CONST_NONE)) begin : partno_reg
      reg        [31:0] partno_r;
      assign idpart = partno_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                    partno_r   <= 32'd0;
        else if (`D_RESET)               partno_r   <= 32'd0;
        else if (is_write & ma_idpartno) partno_r <= PWDATA;
    end else begin
      assign idpart = 32'd0;
    end

    // IDEXT with some combo of BCR, DCR, and Instace
    assign idext[31:24] = 8'd0;
    if (ID_AS_REGS[`IDREGS_BCR_b]) begin : bcr_reg
      reg         [7:0] bcr_r;
      assign idext[23:16] = bcr_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  bcr_r <= 8'd0;
        else if (`D_RESET)             bcr_r <= 8'd0;
        else if (is_write & ma_idext)  bcr_r <= PWDATA[23:16];
    end else begin
      assign idext[23:16] = 8'h0;
    end
    if (ID_AS_REGS[`IDREGS_DCR_b]) begin : dcr_reg
      reg         [7:0] dcr_r;
      assign idext[15:8] = dcr_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  dcr_r <= 8'd0;
        else if (`D_RESET)             dcr_r <= 8'd0;
        else if (is_write & ma_idext)  dcr_r <= PWDATA[15:8];
    end else begin
      assign idext[15:8] = 8'h0;
    end
    if (ENA_ID48B == `ID48B_CONST_INST) begin : inst_reg
      reg         [3:0] inst_r;
      assign idext[3:0] = inst_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  inst_r <= 8'd0;
        else if (`D_RESET)             inst_r <= 8'd0;
        else if (is_write & ma_idext)  inst_r <= PWDATA[3:0];
    end else begin
      assign idext[3:0] = 4'd0;
    end
    if (|MAP_I2CID) begin : i2c_revision
      reg [2:0] i2c_rev_r;
      assign idext[7:4] = {1'b0,i2c_rev_r};
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  i2c_rev_r <= 3'd0;
        else if (`D_RESET)             i2c_rev_r <= 3'd0;
        else if (is_write & ma_idext)  i2c_rev_r <= PWDATA[6:4];
    end else begin
      assign idext[7:4] = 4'd0;
    end

    if (ID_AS_REGS[`IDREGS_VID_b]) begin : vid_reg
      reg          [14:0] vid_r;
      assign idvid [14:0] = vid_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  vid_r <= ID_48B[47:33]; // reset to default
        else if (`D_RESET)             vid_r <= ID_48B[47:33]; // reset to default
        else if (is_write & ma_idvid)  vid_r <= PWDATA[14:0];
    end else begin
      assign idvid = 15'd0;
    end

    // next is optional DA override for retention save over power down
    // they can write when slave is disabled. But, if master changes DA,
    // we clear. This also allows mapped DAs and SAs. Note that the two
    // mechanisms are distinct and allows for one or the other or both.
    if (ID_AS_REGS[`IDREGS_DAWR_b]) begin : wrda_reg
      reg [7:1] wr_da;
      reg       hold_wr_da;             // clears on DA change
      reg       force_rst;
      assign SetDA = force_rst ? 8'hFF : {wr_da, hold_wr_da};
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          wr_da      <= 7'd0;
          hold_wr_da <= 1'b0;
          force_rst  <= 1'b0;
        end else if (`D_RESET) begin
          wr_da      <= 7'd0;
          hold_wr_da <= 1'b0;
          force_rst  <= 1'b0;
        end else if (is_write & ma_dynaddr & ~|PWDATA[11:8] & 
                     (~slv_ena | ENA_MAPPED[`MAP_ENA_b]) & 
                     PWDATA[0] & (PWDATA[31:16]==16'hA4D9)) begin
          // can only write if not enabled and if key is used
          wr_da      <= PWDATA[7:1];
          hold_wr_da <= 1'b1;
        end else if (is_write & ma_dynaddr & ENA_MAPPED[`MAP_ENA_b] & 
                     ~|PWDATA[15:0] & (PWDATA[31:16]==16'hCB19))
          force_rst  <= 1'b1;
        else if (inp_IntStates[`IS_DACHG])
          hold_wr_da <= 1'b0;
        else if (force_rst)
          force_rst  <= 1'b0;
    end else begin
      assign SetDA = 8'd0;
    end
    if (ENA_MAPPED[`MAP_ENA_b]) begin : wrda_map
      // we start with mapped regs - note in PCLK domain
      // we have addresses (ad) and extra info
      reg [(MAP_CNT*8)-1:0] map_regs_ad;
      reg [(MAP_CNT*2)-1:0] map_regs_det;
      reg             [3:0] map_last_idx; // used for read back
      assign SetMappedDASA = {map_regs_ad,map_regs_det};
      genvar i;
      for (i = 1; i <= MAP_CNT; i = i + 1) begin : map_writes
        always @ (posedge PCLK or negedge PRESETn)
          if (!PRESETn) begin
            map_regs_ad[((i*8)-1) -: 8]  <= 0;
            map_regs_det[((i*2)-1) -: 2] <= 0;
          end else if (`D_RESET) begin
            map_regs_ad[((i*8)-1) -: 8]  <= 0;
            map_regs_det[((i*2)-1) -: 2] <= 0;
          end else if (map_rstdaa) begin // will always be 0 if not enabled
            // NOTE: if they reset and was static, we do not correctly switch
            // back to static; should we hold an extra bit to remember???
            if (~map_regs_det[((i*2)-2)])
              map_regs_ad[((i*8)-8)]<= 1'b0; // we reset to off
          end else if (map_setaasa) begin // will always be 0 if not enabled
            map_regs_det[((i*2)-2)] <= 1'b0; // if static, now dynamic
          end else if (map_daa_ena & (map_sa_idx==i)) begin // will always be 0 if not enabled
            map_regs_det[((i*2)-2)] <= 1'b0; // dynamic address vs. static
            map_regs_ad[((i*8)-1) -: 8]  <= {map_daa_da,1'b1};
          end else if (is_write & ma_dynaddr & (PWDATA[11:8]==i) &
                     (PWDATA[31:16]==16'hA4D9)) begin
            // WARNING: next two would be skipped if the above DASA or AASA was active in the cycle
            // can only write if map idx!=0 and if key is used
            map_regs_ad[((i*8)-1) -: 8]  <= PWDATA[7:0];
            map_regs_det[((i*2)-1) -: 2] <= {1'b0,PWDATA[12]};
          end else if (is_write & ma_dynaddr & (PWDATA[11:8]==i) &
                      (PWDATA[31:16]==16'hA731)) begin
            map_regs_det[((i*2)-1) -: 1] <= ~PWDATA[0]; // [0]=1 if ACK, 0=NACK
          end else if (is_write & ma_mapctrln & (PADDR[4:2]==(i-1))) begin
            // WARNING: next two would be skipped if the above DASA or AASA was active in the cycle
            map_regs_ad[((i*8)-1) -: 8]  <= PWDATA[7:0];
            map_regs_det[((i*2)-1) -: 2] <= {PWDATA[12],PWDATA[8]};
          end
      end
      // now 10 bit static address for i2c
      if (ENA_MAPPED[`MAP_I2C_SA10_b]) begin : static_10b
        //FREE_VERSION_CUT - remove extended MAP from free version
      end else begin
        assign SetSA10b = 3'd0;
      end

      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) 
          map_last_idx <= 4'd0;
        else if (`D_RESET) 
          map_last_idx <= 4'd0;
        else if (is_write & ma_dynaddr) // select IDX for readback if wanted
          map_last_idx <= {4{|PWDATA[11:8] & ~|PWDATA[31:16]}} & PWDATA[11:8];
      wire [1:0] last_ns= map_regs_det[(map_last_idx<<1)-1 -: 2];
      assign MapLastIdx = map_last_idx;
      assign MapLastDet = {last_ns[1],{3{map_last_idx==4'd1}}&SetSA10b,last_ns[0]};
      assign MapLastDA  = map_regs_ad[(map_last_idx<<3)-1 -: 8];
      wire [5:0] mapr   = PADDR[4:2]; // index select by addr using MAPCTRLn
      wire [7:0] mapp   = (mapr * PID_CNT);
      wire [1:0] mapns  = map_regs_det[(mapr<<1) +: 2];
      wire [7:0] maddr  =  map_regs_ad[(mapr<<3) +: 8];
      wire [31:14] mpid;
      wire [31:24] mdcr;
      wire       mdause;
      if (MAP_DA_AUTO[`MAPDA_DAA_MMR_b]) begin : daa_mrr_rd
        //FREE_VERSION_CUT - remove extended MAP from free version
      end else begin
        assign mpid     = 0; 
        assign mdcr     = 0;
        assign mdause   = 1'b0;
      end
      // We build return string for multple MAPCTRL regs
      assign mapctrl_reg= {mdcr,mpid[23:14],mdause,mapns[1],
                           {3{mapr==4'd0}}&SetSA10b,mapns[0],maddr};
      if (MAP_DA_AUTO[`MAPDA_DAA_MMR_b]) begin : map_daa_mmr
        //FREE_VERSION_CUT - remove extended MAP from free version
      end else begin
        assign map_daa_use = 0;
        assign map_daa_dcr = 0;
        assign map_daa_pid = 0;
      end
    end else begin
      assign SetMappedDASA = 0;
      assign SetSA10b      = 0;
      assign MapLastIdx    = 0;
      assign MapLastDet    = 0;
      assign MapLastDA     = 0;
      assign mapctrl_reg   = 0;
      assign map_daa_use   = 0;
      assign map_daa_dcr   = 0;
      assign map_daa_pid   = 0;
    end

    // allow interrupt enable from regs intset, intclr
    if (SEL_BUS_IF[`SBIF_IRQ_b]) begin : int_reg
      reg        [19:8] int_ena_r;
      assign int_ena          = int_ena_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          int_ena_r[18:8]   <= 11'd0;
          int_ena_r[19]     <= RSTACT_CONFIG[`RSTA_ENA_b]; // SLVRST is mask enabled by default
        end else if (`D_RESET) begin
          int_ena_r[18:8]   <= 11'd0;
          int_ena_r[19]     <= RSTACT_CONFIG[`RSTA_ENA_b]; // SLVRST is mask enabled by default
        end else if (is_write)
          if (ma_intset) begin
            int_ena_r[18:8] <= int_ena_r[18:8] | PWDATA[18:8];
            int_ena_r[19]   <= int_ena_r[19] | (PWDATA[19]&RSTACT_CONFIG[`RSTA_ENA_b]);
          end else if (ma_intclr) 
            int_ena_r       <= int_ena_r & ~PWDATA[19:8];
    end else begin
      assign int_ena          = 12'd0;
    end
    // the status bits can be cleared either way
    assign reg_clrIntStates = (is_write & ma_stat) ? PWDATA[19:8] : 12'd0;

    // To-bus (TX) FIFO or non-fifo
    // We always use since ping/pong still uses 0 vs. non-0
    reg       [5:4] txtrig_r;
    assign opt_txtrig = txtrig_r;
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn)
        txtrig_r <= 2'd3;             // default is 3 (anything)
      else if (`D_RESET)
        txtrig_r <= 2'd3;
      else if (is_write & ma_datactrl & PWDATA[3]) // note unlock bit
        txtrig_r <= PWDATA[5:4];

    // From-bus (RX) FIFO or non-fifo
    // We always use since ping/pong still uses 0 vs. non-0
    reg       [7:6] rxtrig_r;
    assign opt_rxtrig = rxtrig_r;
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn) 
        rxtrig_r <= 2'd0;
      else if (`D_RESET) 
        rxtrig_r <= 2'd0;
      else if (is_write & ma_datactrl & PWDATA[3]) // note unlock bit
        rxtrig_r <= PWDATA[7:6];

  endgenerate

  //
  // Non-optional write to regs (optional fields handled later)
  //

  //
  // CONFIG @0x004
  //
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) begin
      slv_ena    <= ID_AS_REGS[`IDREGS_SLVENA_b]; // may configure enabled by def
      slv_nack   <= 1'b0;
      s0ignore_r <= 1'b0;
      matchss_r  <= 1'b0;
      slv_offline<= 1'b0;
    end else if (`D_RESET) begin
      slv_ena    <= ID_AS_REGS[`IDREGS_SLVENA_b]; 
      slv_nack   <= 1'b0;
      s0ignore_r <= 1'b0;
      matchss_r  <= 1'b0;
      slv_offline<= 1'b0;
    end else if (is_write & ma_config) begin
      slv_ena    <= PWDATA[0];
      slv_nack   <= PWDATA[1];
      matchss_r  <= PWDATA[2];
      s0ignore_r <= PWDATA[3];
      slv_offline<= PWDATA[9];
    end 

  //
  // WDATAB and BE
  //
  reg           [7:0] write_data_r;
  reg           [7:0] write_data2_r;
  reg           [1:0] rd_cnt;
  reg           [1:0] wr_cnt;
  reg           [1:0] wr_again;

  assign write_data = wr_again[0] ? write_data2_r : write_data_r;
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) begin
      write_data_r   <= 0;
      write_data2_r  <= 0;
      wr_again       <= 2'b00;
      wr_cnt         <= 2'b00;
      owrite_err     <= 1'b0;
    end else if (`D_RESET) begin
      write_data_r   <= 0;
      write_data2_r  <= 0;
      wr_again       <= 2'b00;
      wr_cnt         <= 2'b00;
      owrite_err     <= 1'b0;
    end else if (is_write & (ma_wdatab | ma_wdatabe | ma_wdatab1 | &ign_mwrite)) begin
      // note: special for writing DDR msg moves CMD only (not half)
      if (inp_TxFull)
        owrite_err   <= 1'b1;
      else begin 
        // we use B and BE
        write_data_r <= PWDATA[7:0];
        wr_cnt       <= 2'b01;
      end
    end else if (is_write & ~ign_mwrite[0] & (ma_wdatah | ma_wdatahe)) begin
      // this will collapse away if not enabled (ma_wdatah always 0)
      if (inp_TxFull)
        owrite_err   <= 1'b1;
      else begin
        // we use B and BE
        write_data2_r<= PWDATA[7:0];
        write_data_r <= PWDATA[15:8];
        wr_cnt       <= 2'b01;
        wr_again     <= {ma_wdatahe|PWDATA[16]|inp_dma_last_tb,1'b1};
      end
    end else if (is_write & (ma_errwarn|ma_merrwarn) & PWDATA[17])
      owrite_err     <= 1'b0;           // W1C
    else if (wr_again[0]) begin
      // 2nd byte from half word write
      if (inp_TxFull)                   // note can error on 2nd
        owrite_err   <= 1'b1;
      wr_again[0]    <= 1'b0;
    end else begin
      wr_cnt         <= 2'b00;          // 1 clock pulse
      if (is_pre_read & ma_rdatah & ~inp_RxEmpty)
        write_data2_r<= read_buff;      // 1st half (is LSB)
    end

  // write count valid for 1 cycle after registered
  assign regflg_wr_cnt = wr_cnt;
  // we only read if not empty (we locally handle error)
  assign regflg_rd_cnt = (((is_read | is_pre_read) & ma_rdatah & ~inp_RxEmpty) |
                          (is_read & ma_rdatab & ~inp_RxEmpty)) ? 2'd1 : 2'd0;
  assign read_buff     = inp_RxEmpty ? 8'd0 : inp_fb_data[7:0];
  assign read_buff2    = write_data2_r;   // holds low half
  assign exp_owrite_err= owrite_err;
  assign exp_oread_err = oread_err;

  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) begin
      rd_cnt      <= 2'b00;
      oread_err   <= 1'b0;
    end else if (`D_RESET) begin
      rd_cnt      <= 2'b00;
      oread_err   <= 1'b0;
    end else if (is_read & ma_rdatab) begin
      if (inp_RxEmpty)
        oread_err <= 1'b1;
      else
        rd_cnt    <= 2'b01;
    end else if ((is_read | is_pre_read) & ma_rdatah) begin
      // see write_data2_r for extra byte
      if (inp_RxEmpty)
        oread_err <= 1'b1;
      else
        rd_cnt    <= 2'b01;
    end else if (is_write & (ma_errwarn|ma_merrwarn) & PWDATA[16])
      oread_err   <= 1'b0;              // W1C
    else
      rd_cnt      <= 2'b00;

  // The END bit is handled separately
  // to-bus end can be set from its own bit as well as WDATA
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) 
      tb_end_r     <= 1'b0;
    else if (`D_RESET) 
      tb_end_r     <= 1'b0;
    else if (is_write) begin
      if (ma_wdatabe)                   // end data reg
        tb_end_r   <= 1'b1;
      else if (ma_wdatab & (PWDATA[8] | PWDATA[16] | inp_dma_last_tb))
        tb_end_r   <= 1'b1;             // end marker bits
      else if (ma_wdatab1 & inp_dma_last_tb)
        tb_end_r   <= 1'b1;             // special for DMA
      else
        tb_end_r   <= 1'b0;
    end else if (wr_again == 2'b11)     // wdatahe
      tb_end_r     <= 1'b1;             // end applies to 2nd byte
    else
      tb_end_r     <= 1'b0; 
      
  // to-bus flush bit flushes the internal to-bus state
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) 
      tb_flush_r <= 1'b0;
    else if (`D_RESET) 
      tb_flush_r <= 1'b0;
    else if (is_write & ma_datactrl & PWDATA[0])
      tb_flush_r <= 1'b1;
    else if (tb_flush_r)
      tb_flush_r <= 1'b0;

  // from-bus flush bit flushes the internal from-bus state
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) 
      fb_flush_r <= 1'b0;
    else if (`D_RESET) 
      fb_flush_r <= 1'b0;
    else if (is_write & ma_datactrl & PWDATA[1])
      fb_flush_r <= 1'b1;
    else if (fb_flush_r)
      fb_flush_r <= 1'b0;

  generate if (ENA_IBI_MR_HJ[`EV_EXTFIFO_b]) begin : ibi_fifo_mux
    reg       rdy_r, valid_r, end_r, flush_r;
    reg [7:0] data_r;

    assign ibi_wr_fifo      = {flush_r, end_r, valid_r, data_r};
    assign opt_ibi_wr_empty = ~rdy_r;

    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn)  begin
        rdy_r     <= 1'b0;              // ready for app to write new
        end_r     <= 1'b0;
        flush_r   <= 1'b0;
        data_r    <= 8'd0;
      end else if (`D_RESET)  begin
        rdy_r     <= 1'b0;
        end_r     <= 1'b0;
        flush_r   <= 1'b0;
        data_r    <= 8'd0;
      end else if (is_write & ma_wibidata) begin
        if (PWDATA[31])
          flush_r <= 1'b1;
        else begin
          rdy_r   <= 1'b1;
          end_r   <= PWDATA[8] | PWDATA[16];
          data_r  <= PWDATA[7:0];
          flush_r <= 1'b0;
        end
      end else if (rdy_r & ibi_wr_ack)  // data copied
        rdy_r     <= 1'b0;
      else if (flush_r)
        flush_r   <= 1'b0;

    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn)
        valid_r   <= 1'b0;              // valid to push to FIFO
      else if (`D_RESET)
        valid_r   <= 1'b0;
      else if (is_write & ma_wibidata) 
        valid_r   <= 1'b1;
      else
        valid_r   <= 1'b0;
  end else begin
    assign ibi_wr_fifo      = 11'd0;
    assign opt_ibi_wr_empty = 1'b0;
  end endgenerate


  //
  // now capabilities from params
  //
  assign capable[1:0]   = ENA_ID48B[1:0]; // 0 means 4 (all from regs)
  assign capable[5:2]   = ID_AS_REGS[3:0];
  assign capable[8:6]   = ENA_HDR[2:0]; // DDR, BT, reserved
  assign capable[9]     = ENA_MASTER[0];
  assign capable[11:10] = ENA_SADDR[1:0];
  assign capable[15:12] = ENA_CCC_HANDLING[3:0];
  assign capable[20:16] = ENA_IBI_MR_HJ[4:0];
  assign capable[21]    = |ENA_TIMEC;
  assign capable[22]    = 1'b0;
  assign capable[25:23] = EXT_FIFO[2:0];
  wire [2:0] tmptb, tmpfb;
  assign tmptb          = ENA_TOBUS_FIFO[2:0]-2'd1;
  assign capable[27:26] = |ENA_TOBUS_FIFO[2:0] ? tmptb[1:0] : 2'd0;
  assign tmpfb          = ENA_FROMBUS_FIFO[2:0]-2'd1;
  assign capable[29:28] = |ENA_FROMBUS_FIFO[2:0] ? tmpfb[1:0] : 2'd0;
  assign capable[30]    = SEL_BUS_IF[`SBIF_IRQ_b];
  assign capable[31]    = SEL_BUS_IF[`SBIF_DMA_b];
  assign capable2[3:0]  = ENA_MAPPED ? MAP_CNT[3:0] : 4'd0;
  assign capable2[4]    = ENA_MAPPED[`MAP_I2C_SA10_b];
  assign capable2[5]    = ENA_MAPPED[`MAP_I2C_SLVRST_b];
  assign capable2[6]    = |MAP_I2CID; 
  assign capable2[7]    = 1'b0;
  assign capable2[8]    = 1'b1; // should this be a param?
  assign capable2[9]    = ENA_IBI_MR_HJ[`EV_EXTFIFO_b];
  assign capable2[15:10]= 0;
  assign capable2[16]   = ENA_CCC_HANDLING[`ENCCC_V11MIN];
  assign capable2[17]   = RSTACT_CONFIG[`RSTA_ENA_b];
  assign capable2[19:18]= 2'd0; // no group yet
  assign capable2[20]   = 1'b0; 
  assign capable2[21]   = ENA_CCC_HANDLING[`ENCCC_AASA];
  assign capable2[22]   = 1'b0;
  assign capable2[23]   = 1'b0;
  assign capable2[31:24]= 0;


endmodule
