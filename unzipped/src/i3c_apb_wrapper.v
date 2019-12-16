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
//  File            : i3c_apb_wrapper.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Wed Dec 11 18:20:40 2019 $
//  Revision        : $Revision: 1.74.1.1 $
//
//  IP Name         : i3c_apb_wrapper 
//  Description     : MIPI I3C Slave support with APB bus for MMRs
//    This contains the wrapper for I3C use with an APB bus for registers.
//    See the other wrappers if not using APB.
//    As an outer wrappers, this module support 4 needs:
//    1. Instantiates the Slave 
//    2. Instantiates the register interface using APB
//    3. Handles clock domain crossing using the system clock (PCLK in
//       this case), although down in the full wrapper.
//    4. Manages the data buffering/FIFO as needed, although down in the
//       full wrapper.
//    The specifics of what this does are controlled by the parameters
//    as outlined in the micro-arch spec and also in the params file.
//
//  ----------------------------------------------------------------------------
//                    Revision History
//  ----------------------------------------------------------------------------
//
//
//  ----------------------------------------------------------------------------
//                    Implementation details
//  ----------------------------------------------------------------------------
//  See I3C Slave micro-arch spec and MIPI I3C spec, as well as the 
//  programmers guide for MMR information.
//  ----------------------------------------------------------------------------
//  ECO from tag 1.5 and earlier:
//  1. pin_PUR_out/oena removed

`include "i3c_params.v"                 // local parameters/constants

module i3c_apb_wrapper #(
    // params are driven from upper layer to control how this block
    // is built and operates
    parameter ENA_ID48B       = `ID48B_CONST, // const vs. reg
    parameter  ID_48B         = 48'h0,  // must be filled in from above with some/all of 48-bit ID
    parameter ID_AS_REGS      = 12'd0,  // Registers for ID and others uses (e.g. Masks)
    // note that BCR is below so can use other params
    parameter  ID_DCR         = 8'd0,   // filled in from above with DCR if not from regs
    parameter ENA_SADDR       = `SADDR_NONE, // none, const, reg/net
    parameter  SADDR_P        = 0,      // 7 bits as 6:0
    parameter ENA_MAPPED      = 5'd0,   // if extra DAs/SAs allowed plus related
    parameter  MAP_CNT        = 4'd1,   // number of extra DAs/SAs allowed
    parameter  MAP_I2CID      = 24'd0,  // !=0 if I2C extended with DevID
               // DA_AUTO: PID[pos:0],DCR, MMR, res  DAA, AASA,DASA,
    parameter  MAP_DA_AUTO    = {5'd1,1'b0,1'b0,3'd0,1'b0,1'b0,1'b0},
    parameter  MAP_DA_DAA     = 0,      // if not MMR and PID/DCR !=0, is bit array      
    parameter ENA_IBI_MR_HJ   = 0,      // 0 if no events, else events as mask
    parameter  CLK_SLOW_BITS  = 6,      // number of bits needed for count for Bus Avail
    parameter  CLK_SLOW_MATCH = 6'd47,  // count: e.g. 47 for 1us if 48MHz CLK (0 rel)
    parameter  CLK_SLOW_HJMUL = 10'd1000,// number of MATCHes (const or reg) for 1ms (1 rel)
      // next is error handling. Read Abort is default if IBI is enabled
    parameter  ERROR_HANDLING = 3'd0|((|ENA_IBI_MR_HJ)<<`ERR_RDABT_b), 
    parameter  ENA_TIMEC      = 6'b000010,  // clock_reg, res, res, mode 1, mode 0, sync
    parameter  TIMEC_FREQ_ACC = {8'd0,8'd0},// freq H number of 0.5MHz, then acc as mul of 0.1%
    parameter ENA_CCC_HANDLING= 6'd0,   // passed down as to what to support
    parameter RSTACT_CONFIG   = 26'd0,  // Slave Reset RSTACT CCC config (see RSTA_xxx_b fields)
    parameter MAX_RDLEN       = 0,      // default S->M len max
    parameter MAX_WRLEN       = 0,      // default M->S len max
    parameter MAX_DS_WR       = 0,      // data speed limits for M->S
    parameter MAX_DS_RD       = 0,      // data speed limits for S->M
    parameter MAX_DS_RDTURN   = 0,      // latency needs for S->M read req
    parameter SEL_BUS_IF      = 5'h07,  // default to full APB but not DMA or half (unless FIFO)
    parameter  DMA_TYPE       = 2'd0,   // 0=req/ack bef, 2=req/ack after, 3=req/ack 4-phase
    parameter FIFO_TYPE       = 4'b10_00,// type of FIFO used (including external) and holding
    parameter  EXT_FIFO       = 3'd0,   // type of external FIFO if used
    parameter  ENA_TOBUS_FIFO = 0,      // depth of to-bus as power of 2 from 2 up
    parameter  ENA_FROMBUS_FIFO=0,      // depth of from-bus as power of 2 from 2 up
    parameter ENA_HDR         = 0,      // enables for HDR modes
    parameter BLOCK_ID        = 0,      // if not 0, allows ID reg
    parameter ENA_MASTER      = 0,      // 1 if using master
    parameter PIN_MODEL       = `PINM_COMBO, // combinatorial pin use
      // BCR auto filled in from params. But, cannot do offline or bridge
    parameter  ID_BCR         = (|ENA_MASTER<<6) | (|ENA_HDR<<5) | ((ENA_IBI_MR_HJ&8'h03)<<1) |
                                (|(MAX_DS_RDTURN|MAX_DS_WR|MAX_DS_RD)), // limits at [0]
    parameter priv_sz         = PIN_MODEL[1] ? (ENA_HDR[0] ? 3 : 1) : 0,// wider if ext-pad+ddr
    parameter TOP_DBG_MX      = 0       // for debug observer
  )
  (
  // define clock and reset
  input               PRESETn,          // reset from system
  input               PCLK,             // system clock for regs and sync
  input               CLK_SLOW,         // may be same as PCLK or slower
  output              slow_gate,        // 1 if CLK_SLOW may be gated
  input               CLK_SLOW_TC,      // time-control clockl may be same as CLK_SLOW
  output              tc_slow_gate,     // 1 if CLK_SLOW_TC may be gated; AND with slow_gate if common clock
    // FUTURE: input  clk_FastTernary,  // Ternary clock if Ternary used
  // define APB bus
  input               PSEL,             // select (with penable)
  input               PENA,             // actual read or write cycle enable
  input        [11:0] PADDR,            // address - word aligned
  input               PWRITE,           // is write (data phase and addr are same)
  output       [31:0] PRDATA,           // read reg
  input        [31:0] PWDATA,           // write reg
  output              PREADY,           // Stall
  output              PSLVERR,          // optional if ERROR_HANDLING enables
  // define IRQ and DMA - No-connect if not used
  output              irq,              // interrupt when an int_pin - held
  output              dma_req_tb,       // request for to-bus data
  output              dma_req_fb,       // request to get from-bus data
     // DMA ack use depends on how DMA in system works:
     // - if DMA_TYPE=0, then ACK is 1 clock (PCLK) and precedes the MMR
     //   access (e.g. WDATAB and RDATAB).
     // - if DMA_TYPE=2, then ACK is 1 clock and with/after the MMR access.
     // - if DMA_TYPE=3, then REQ/ACK are 4-phase with ACK with/after MMR
     //   access.
     // - If DMA_TYPE=1 is reserved for now (trigger is deprecated).
     // Note: dma_req_tb_ack_last should be strapped 0 if not used. Else, it 
     // must be held 1 into WDATAB/WDATAH write by DMA when last byte (END). 
     // This will treat as WDATABE/WDATAHE.
  input               dma_req_tb_ack,   // ack of to-bus req
  input               dma_req_fb_ack,   // ack of from-bus req
  input               dma_req_tb_ack_last,// NOTE: strap 0 if not using
  output              wakeup,           // held with irq if enabled (to wake Deepsleep)
    // next two may be combined in integration - as one wakeup
  output              raw_wakeup_irq,   // wake on SA/DA match in SCL: to start PCLK for enabled Int
  output              raw_wakeup_dma,   // " same but if dma req enabled
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
  // optional nets into or from system, when not from regs
  // No-connect outputs and strap inputs 0 if not being used
  input         [7:0] ext_SlvSA,        // inp Slave addr [0]=1 if valid
  output        [7:0] out_SlvDA,        // DA if any, [0]=1 if value
    // note: below in SCL clock domain. [11]->1 on changes; rest stable after
  output       [13:0] raw_timec_sync,   // SYNC tc handled by system. [13]=clk
  // Next 3 are for SlaveReset if enabled/used. Reset detector is external to block
  output        [3:0] raw_slvr_reset,   // SlaveRst RSTACT action in [2:0], clear in [3]
  input               iraw_rst_slvr_reset, // clear of SlaveRst controls in SCL domain
    // note: Slave Reset for block connected to reset detector if handled by
    //       Application Interrupt, else strap 0 and handle by system control. 
  input               iraw_slvr_irq,    // for block reset request: sets IRQ and cause
  output              raw_i2c_slvrst,   // if extended i2c - slave reset
    // next two are only used with ENA_MAPPED[VGPIO]
  output              vgpio_done,       // done pulse if VGPIO enabled
  output        [7:0] raw_vgpio_byte,   // byte value if "
  // Optional external FIFO: 
    // To-bus means from slave to master
  input               ixf_tb_avail,     // 1 if byte is available for tobus
  input               ixf_tb_last,      // byte is last for message
  input         [7:0] ixf_tb_data,      // actual data when avail=1
  output               oxf_tb_start,     // pulsed when ixf_tb_avail==1 and START
  output              oxf_tb_used,      // pulsed when tb_data used
    // From-bus means from master into slave
  input               ixf_fb_free,      // 1 if space to take a byte frombus
  output              oxf_fb_req,       // pulse when data to be taken (amd free=1)
  output        [7:0] oxf_fb_data,      // data frombus when req=1
  output              oxf_fb_eof,       // frame ended in repeated START or STOP
  // Now general ports for 'debug' type observation
  `ifdef ENA_DBG_OBSERVE
  output[TOP_DBG_MX:0]out_debug_observ, // observer output
  `endif
  // special below for D input reset
  `ifdef USE_D_RESET
  input               d_reset_r,
  `endif
  // now scan related
  input               scan_single_clock,// single clock domain from scan_clk for pin-based clocks
  input               scan_clk,         // clock to use for pin clocks if in scan
  input               scan_no_rst,      // prevents layered reset
  input               scan_no_gates     // prevents arch clock gating
  );
  // below for optional MAP Auto DAA
  localparam   [7:0] PID_CNT = MAP_DA_AUTO[`MAPDA_DAA_PID_lb+4:`MAPDA_DAA_PID_lb];

  // bus special
  wire   wr_err;                        // set in reg interface
  assign PSLVERR = wr_err & ERROR_HANDLING[`ERR_WR_RO_b];

  //
  // Wires - We define wires which are to/from registers and handleed 
  //         here or by instance.
  //

  // config are values from registers which are unchanging
  wire                cf_SlvEna;
  wire                cf_SlvNack;
  wire          [7:0] cf_SlvSA;
  wire          [3:0] cf_IdInst;
  wire                cf_IdRand;
  wire                cf_Offline;
  wire         [31:0] cf_Partno;
  wire          [7:0] cf_IdBcr;
  wire          [7:0] cf_IdDcr;
  wire         [14:0] cf_IdVid;
  wire                cf_DdrOK;
  wire                cf_TspOK;
  wire                cf_TslOK;
  wire         [11:0] cf_MaxRd;
  wire         [11:0] cf_MaxWr;
  wire         [23:0] cf_RstActTim;
  wire          [7:0] cf_BAMatch;
  wire         [15:0] cf_TCclk;
  wire                cf_s0ignore;
  wire                cf_matchss;
  wire          [1:0] cf_HdrCmd;
  wire          [6:0] cf_CccMask;
  wire          [8:0] cf_vgpio;
  wire    [MAP_CNT-1:0] map_daa_use;      // which MAP are auto-DAA
  wire  [(MAP_CNT*8)-1:0] map_daa_dcr;      // DCRs if MAP auto-DAA
  wire  [(MAP_CNT*PID_CNT)-1:0] map_daa_pid; // PID partials if MAP auto-DAA

    // next is special case of changed while slave is not enabled
    // along with mapped regs if used
  wire          [7:0] SetDA;
  wire [(MAP_CNT*10)-1:0] SetMappedDASA;
  wire          [2:0] SetSA10b;
  wire                cf_IbiExtData;
  wire          [3:0] cf_IbiMapIdx;
  wire          [2:0] cf_i2c_dev_rev;
  // Now special states from SCL domain raw (sync done by reg block)
  wire        [21:20] raw_ActState;
  wire        [19:16] raw_EvState;
  wire          [2:0] raw_TimeC;
  wire          [6:0] raw_Request;
  wire          [7:0] raw_DynAddr;
  wire          [2:0] raw_DynChgCause;
  wire         [12:0] raw_match_idx;
  wire                raw_matched;
  wire                hdr_new_cmd;
  wire          [7:0] raw_hdr_cmd;
  wire                map_rstdaa;
  wire                map_setaasa;
  wire                map_daa_ena;
  wire          [3:0] map_sa_idx;
  wire          [7:1] map_daa_da;
  // now registers in and out which are from same PCLK domain
  wire         [19:8] reg_IntEna;
  wire          [5:0] reg_DmaCtrl;
  wire         [19:8] inp_IntStates;
  wire          [2:0] reg_EvPend;
  wire          [7:0] reg_EvIbiByte;
  wire                inp_EvNoCancel;
  wire        [22:20] inp_EvDet;
  wire          [5:0] inp_GenErr;
  wire         [11:8] inp_DataErr;
  wire          [5:0] msk_GenErr;
  wire         [11:8] msk_DataErr;
  wire         [19:8] reg_clrIntStates;
  wire          [5:0] reg_clrGenErr;
  wire         [11:8] reg_clrDataErr;
  wire                reg_holdOErr;
  wire                inpflg_MaxRd;
  wire                inpflg_MaxWr;
  wire         [11:0] inp_MaxRW;
  wire                reg_TbEnd;
  wire                reg_TbFlush;
  wire                reg_FbFlush;
    // next 4 are only used if FIFO enabled
  wire          [5:4] reg_TxTrig;
  wire          [7:6] reg_RxTrig;
  wire        [20:16] inp_TxCnt;
  wire        [28:24] inp_RxCnt;
  wire                inp_TxFull;
  wire                inp_RxEmpty;
    // next 3 will only be used to size(s) allowed by params
  wire          [1:0] regflg_wr_cnt;
  wire          [7:0] reg_wdata;
  wire          [1:0] regflg_rd_cnt;
  // the next two are used for reg interface
  wire          [7:0] notify_fb_data;
  wire          [8:7] reg_ActMode;
  wire          [3:0] reg_PendInt;
  wire         [15:8] reg_StatusRes;
  // Now IBI FIFO from regs
  wire         [10:0] ibi_wr_fifo;
  wire                ibi_wr_ack;

  //
  // Allow export of debug observer data
  // -- default stub assigns 0, but this allows adding nets
  // -- Note that FULL_DBG_MX is defined by include file as
  //    is full_debug_observ
  //
  `ifdef ENA_DBG_OBSERVE
   `include "dbg_observ_top.v"   // integrator replaces inc file contents 
  `else
   localparam FULL_DBG_MX = 0;
   wire [FULL_DBG_MX:0] full_debug_observ;
  `endif
  //


  // IRQ and DMA (if used)
  assign irq        = |(reg_IntEna & inp_IntStates); // signals if any masked and pending
  assign wakeup     = |(reg_IntEna[12:11] & inp_IntStates[12:11]); // data/FIFO wakes
  assign raw_wakeup_irq = raw_matched & reg_IntEna[9]; // wake if irq enabled
  assign raw_wakeup_dma = raw_matched & |reg_DmaCtrl[3:0];// wake if DMA enabled

  generate if (SEL_BUS_IF[`SBIF_DMA_b]) begin : DMA_supp
    wire [1:0] rx_fullness, tx_avail;
    i3c_dma_control #(.DMA_TYPE(DMA_TYPE)) dma_ctrl(
       .PRESETn         (PRESETn), 
       .PCLK            (PCLK), 
       .dma_ctrl        (reg_DmaCtrl), 
       .rx_trig         (inp_IntStates[`IS_RXPEND]), 
       .tx_trig         (inp_IntStates[`IS_TXPEND]), 
       .rx_fullness     (rx_fullness), 
       .tx_avail        (tx_avail),
       .pread           (PSEL&PENA&~PWRITE),// special for re-arm RX
       .dma_req_tb      (dma_req_tb), 
       .dma_req_fb      (dma_req_fb), 
       .dma_req_tb_ack  (dma_req_tb_ack), 
       .dma_req_fb_ack  (dma_req_fb_ack)
       );
    // next is to stop errors when this generate is not used (still parsed)
    localparam FAKE_TB_WID = |ENA_TOBUS_FIFO ? ENA_TOBUS_FIFO : 2; // stop errors if 0 
    // compute the fullness and avail counts
    // count for RX says if 0, 1, 2, or more than 2 bytes in there
    assign rx_fullness = |inp_RxCnt[28:26] ? 2'd3 : inp_RxCnt[25:24];
    // count for TX is more complex since it is bytes
    // available, to allow for 0, 1, 2, or more than 2 avail
    // For ping-pong case (not common if using DMA), it uses 2
    // if empty, 0 if full, and 1 for not full. 3 is not possible.
    assign tx_avail    = ~|ENA_TOBUS_FIFO ? 
                         (~|inp_TxCnt[17:16]?2'd2:inp_TxCnt[17]?2'd0:2'd1) :
                         (inp_TxCnt[16+FAKE_TB_WID] ? 2'd0 :
                          &inp_TxCnt[(16+FAKE_TB_WID-1):16] ? 2'd1 :
                          reg_DmaCtrl[5] ? // need to treat 3B same as 2B (halfword)
                            ((inp_TxCnt[(16+FAKE_TB_WID-1):16]>=((1<<FAKE_TB_WID)-3)) ?
                             2'd2 : 2'd3) :
                            ((inp_TxCnt[(16+FAKE_TB_WID-1):16]==((1<<FAKE_TB_WID)-2)) ?
                             2'd2 : 2'd3));
  end else begin
    assign dma_req_tb  = 1'b0;
    assign dma_req_fb  = 1'b0;
  end
  endgenerate

  // next is special D input reset scheme used by one project
  wire RSTn;
 `ifdef USE_D_RESET
    assign RSTn = PRESETn & (~d_reset_r | scan_no_rst); // is async reset for other than PCLK domain
 `else
    assign RSTn = PRESETn;
 `endif

  //
  // Full wrapper handles Slave, CDC, and data buffering for FIFO
  //
  i3c_full_wrapper #(.ENA_ID48B(ENA_ID48B),.ID_48B(ID_48B),
                     .ID_AS_REGS(ID_AS_REGS),.ID_BCR(ID_BCR),.ID_DCR(ID_DCR),
                     .ENA_SADDR(ENA_SADDR),.SADDR_P(SADDR_P),
                     .ENA_MAPPED(ENA_MAPPED),.MAP_CNT(MAP_CNT),.MAP_I2CID(MAP_I2CID),
                       .MAP_DA_AUTO(MAP_DA_AUTO), .MAP_DA_DAA(MAP_DA_DAA),
                     .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ),
                       .CLK_SLOW_BITS(CLK_SLOW_BITS),.CLK_SLOW_MATCH(CLK_SLOW_MATCH),
                       .CLK_SLOW_HJMUL(CLK_SLOW_HJMUL), .ERROR_HANDLING(ERROR_HANDLING),
                     .ENA_CCC_HANDLING(ENA_CCC_HANDLING), .RSTACT_CONFIG(RSTACT_CONFIG),
                       .MAX_RDLEN(MAX_RDLEN),
                       .MAX_WRLEN(MAX_WRLEN), .MAX_DS_WR(MAX_DS_WR), .MAX_DS_RD(MAX_DS_RD),
                       .MAX_DS_RDTURN(MAX_DS_RDTURN),
                     .SEL_BUS_IF(SEL_BUS_IF),
                     .FIFO_TYPE(FIFO_TYPE),
                       .EXT_FIFO(EXT_FIFO),
                       .ENA_TOBUS_FIFO(ENA_TOBUS_FIFO),.ENA_FROMBUS_FIFO(ENA_FROMBUS_FIFO),
                     .ENA_HDR(ENA_HDR),
                     .BLOCK_ID(BLOCK_ID),
                     .PIN_MODEL(PIN_MODEL),
                     .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
                     .FULL_DBG_MX(FULL_DBG_MX))
  full_wrap
  (
    .RSTn             (RSTn),           // is just PRESETn normally
    .CLK              (PCLK), 
    .CLK_SLOW         (CLK_SLOW),
    .slow_gate        (slow_gate),
    .CLK_SLOW_TC      (CLK_SLOW_TC),
    .tc_slow_gate     (tc_slow_gate), // they will AND with slow_gate if common clock
    .clk_FastTernary  (1'b0), // FUTURE: clk_FastTernary), 
    .pin_SCL_in       (pin_SCL_in), 
    .pin_SCL_out      (pin_SCL_out), 
    .pin_SCL_oena     (pin_SCL_oena), 
    .pin_SDA_in       (pin_SDA_in), 
    .pin_SDA_out      (pin_SDA_out), 
    .pin_SDA_oena     (pin_SDA_oena), 
    .pin_SDA_oena_rise(pin_SDA_oena_rise), 
    .i2c_spike_ok     (i2c_spike_ok),
    .i2c_hs_enabled   (i2c_hs_enabled),
    .cf_SlvEna        (cf_SlvEna), 
    .cf_SlvNack       (cf_SlvNack), 
     // Static address will be mapped below as well, so const not really needed
    .cf_SlvSA         ((ENA_SADDR==`SADDR_CONST) ? {SADDR_P[6:0],1'b1} :
                       (ENA_SADDR==`SADDR_CONFIG)? cf_SlvSA  :
                       (ENA_SADDR==`SADDR_NET)   ? ext_SlvSA :
                       8'h0),           // from param or none 
    .cf_IdInst        (cf_IdInst), 
    .cf_IdRand        (cf_IdRand), 
    .cf_Offline       (cf_Offline),
    .cf_Partno        (cf_Partno), 
    .cf_IdBcr         (cf_IdBcr), 
    .cf_IdDcr         (cf_IdDcr), 
    .cf_IdVid         (cf_IdVid),
    .cf_DdrOK         (cf_DdrOK), 
    .cf_TspOK         (cf_TspOK), 
    .cf_TslOK         (cf_TslOK), 
    .cf_MaxRd         (cf_MaxRd), 
    .cf_MaxWr         (cf_MaxWr), 
    .cf_RstActTim     (cf_RstActTim),
    .cf_BAMatch       (cf_BAMatch),
    .cf_TCclk         (cf_TCclk),
    .cf_s0ignore      (cf_s0ignore),
    .cf_matchss       (cf_matchss),
    .cf_SetDA         (SetDA),          // if [0]=1, wants to override DA (when not enabled)
    .cf_SetMappedDASA (SetMappedDASA),  // if mapped DAs/SAs supported
    .cf_SetSA10b      (SetSA10b),       // if [1] is SA 10bit
    .cf_MasterAcc     (1'b0),           // only for M+S: slave OK to accept Mastership
    .cf_IbiExtData    (cf_IbiExtData),	// extended data after IBI byte
    .cf_IbiMapIdx     (cf_IbiMapIdx),   // mapped index for IBI
    .cf_HdrCmd        (cf_HdrCmd),      // enable for HDR Cmd as MMR
    .cf_CccMask       (cf_CccMask),     // Mask enables for unhandled CCCs
    .cf_vgpio         (cf_vgpio),       // VGPIO control
    .vgpio_done       (vgpio_done),     // only used if ENA_MAPPED VGPIO
    .raw_vgpio_byte   (raw_vgpio_byte),
    .map_daa_use      (map_daa_use),    // if MAP has auto-DAA regs
    .map_daa_dcr      (map_daa_dcr),    // "
    .map_daa_pid      (map_daa_pid),    // "
    .hdr_new_cmd      (hdr_new_cmd),    // pulse in PCLK domain
    .raw_hdr_cmd      (raw_hdr_cmd),    // pulse in PCLK domain
    .map_rstdaa       (map_rstdaa),     // pulse in PCLK domain if rstdaa and Map auto
    .map_setaasa      (map_setaasa),    // pulse in PCLK domain and set aasa for Map
    .map_daa_ena      (map_daa_ena),    // pulse in PCLK if MAP DASA/DAA
    .map_sa_idx       (map_sa_idx),     // index in map if daa_ena
    .map_daa_da       (map_daa_da),     // new DA if DASA/DAA for map
    .i2c_dev_rev      (cf_i2c_dev_rev), // from regs if used
    .i2c_sw_rst       (raw_i2c_slvrst), // i2c slave reset if used
    .outp_to_master   (),               // never used
    .raw_ActState     (raw_ActState), 
    .raw_EvState      (raw_EvState), 
    .raw_TimeC        (raw_TimeC),
    .raw_timec_sync   (raw_timec_sync), // if SYNC supported by system
    .raw_slvr_reset   (raw_slvr_reset),
    .iraw_rst_slvr_reset(iraw_rst_slvr_reset),
    .iraw_slvr_irq    (iraw_slvr_irq),  // block reset from SCL domain
    .raw_Request      (raw_Request), 
    .raw_DynAddr      (raw_DynAddr), 
    .raw_DynChgCause  (raw_DynChgCause),
    .raw_match_idx    (raw_match_idx),
    .raw_matched      (raw_matched),
    .sraw_ActMode     (reg_ActMode),
    .sraw_PendInt     (reg_PendInt),
    .sraw_StatusRes   (reg_StatusRes),
    .outp_IntStates   (inp_IntStates), 
    .reg_EvPend       (reg_EvPend), 
    .reg_EvIbiByte    (reg_EvIbiByte), 
    .outp_EvNoCancel  (inp_EvNoCancel),
    .outp_EvDet       (inp_EvDet), 
    .outp_GenErr      (inp_GenErr), 
    .outp_DataErr     (inp_DataErr), 
    .msk_GenErr       (msk_GenErr),
    .msk_DataErr      (msk_DataErr),
    .reg_clrIntStates (reg_clrIntStates), 
    .reg_clrGenErr    (reg_clrGenErr), 
    .reg_clrDataErr   (reg_clrDataErr), 
    .reg_holdOErr     (reg_holdOErr),
    .outpflg_MaxRd    (inpflg_MaxRd), 
    .outpflg_MaxWr    (inpflg_MaxWr), 
    .outp_MaxRW       (inp_MaxRW), 
    .reg_TbEnd        (reg_TbEnd), 
    .reg_TbFlush      (reg_TbFlush), 
    .reg_FbFlush      (reg_FbFlush), 
    .reg_TxTrig       (reg_TxTrig), 
    .reg_RxTrig       (reg_RxTrig), 
    .outp_TxCnt       (inp_TxCnt), 
    .outp_RxCnt       (inp_RxCnt), 
    .outp_TxFull      (inp_TxFull),
    .outp_RxEmpty     (inp_RxEmpty),
    .regflg_wr_cnt    (regflg_wr_cnt), 
    .reg_wdata        (reg_wdata), 
    .regflg_rd_cnt    (regflg_rd_cnt), 
    .outp_fb_data     (notify_fb_data), 
    .ixf_fb_free      (ixf_fb_free),
    .oxf_fb_req       (oxf_fb_req),
    .oxf_fb_data      (oxf_fb_data),
    .oxf_fb_eof       (oxf_fb_eof),
    .ixf_tb_avail     (ixf_tb_avail),
    .ixf_tb_last      (ixf_tb_last),
    .ixf_tb_data      (ixf_tb_data),
    .oxf_tb_start     (oxf_tb_start),
    .oxf_tb_used      (oxf_tb_used),
    // Next for IBI FIFO (for IBI EXTDATA) if used
    .ibi_wr_fifo      (ibi_wr_fifo),
    .ibi_wr_ack       (ibi_wr_ack),
    // these below are only used when Master supported, so strapped here
    .is_slave         (1'b1),
    .d_tb_data_valid  (),
    .tb_pclk_valid    (),
    .d_tb_datab       (),
    .d_tb_end         (),
    .m_tb_datab_ack   (1'b0),
    .fb_data_use      (),
    .m_fb_datab       (8'd0),
    .m_fb_datab_done  (1'b0),
    // debug observer
    .full_debug_observ(full_debug_observ),
    // last is scan/DFT related
    .scan_single_clock(scan_single_clock), 
    .scan_clk         (scan_clk), 
    .scan_no_rst      (scan_no_rst), 
    .scan_no_gates    (scan_no_gates)
  );
  assign out_SlvDA    = raw_DynAddr;


  //
  // Registers are programmer's model registers
  //
  i3c_regs  #(.ENA_ID48B(ENA_ID48B),.ID_48B(ID_48B),
              .ID_AS_REGS(ID_AS_REGS),.ID_BCR(ID_BCR),.ID_DCR(ID_DCR),
              .ENA_SADDR(ENA_SADDR),.SADDR_P(SADDR_P),
              .ENA_MAPPED(ENA_MAPPED),.MAP_CNT(MAP_CNT),.MAP_I2CID(MAP_I2CID),
                .MAP_DA_AUTO(MAP_DA_AUTO), .MAP_DA_DAA(MAP_DA_DAA),
              .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ), .ERROR_HANDLING(ERROR_HANDLING),
                .CLK_SLOW_BITS(CLK_SLOW_BITS),.CLK_SLOW_MATCH(CLK_SLOW_MATCH),
              .ENA_CCC_HANDLING(ENA_CCC_HANDLING), .MAX_RDLEN(MAX_RDLEN),
                .MAX_WRLEN(MAX_WRLEN), .MAX_DS_WR(MAX_DS_WR), .MAX_DS_RD(MAX_DS_RD),
                .MAX_DS_RDTURN(MAX_DS_RDTURN), .RSTACT_CONFIG(RSTACT_CONFIG),
              .SEL_BUS_IF(SEL_BUS_IF),
              .FIFO_TYPE(FIFO_TYPE),
                .EXT_FIFO(EXT_FIFO),
                .ENA_TOBUS_FIFO(ENA_TOBUS_FIFO),
              .ENA_FROMBUS_FIFO(ENA_FROMBUS_FIFO),
              .ENA_HDR(ENA_HDR),
              .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
              .BLOCK_ID(BLOCK_ID)) 
    regs 
    (
    .PRESETn          (PRESETn), 
    .PCLK             (PCLK), 
    .PSEL             (PSEL), 
    .PENA             (PENA), 
    .PADDR            (PADDR[11:2]), 
    .PWRITE           (PWRITE), 
    .PRDATA           (PRDATA), 
    .PWDATA           (PWDATA), 
    .PREADY           (PREADY), 
    .wr_err           (wr_err),
    .ign_mwrite       (2'b00),
    .cf_SlvEna        (cf_SlvEna), 
    .cf_SlvNack       (cf_SlvNack), 
    .cf_SlvSA         (cf_SlvSA), 
    .cf_IdInst        (cf_IdInst), 
    .cf_IdRand        (cf_IdRand), 
    .cf_Offline       (cf_Offline),
    .cf_Partno        (cf_Partno), 
    .cf_IdBcr         (cf_IdBcr), 
    .cf_IdDcr         (cf_IdDcr), 
    .cf_IdVid         (cf_IdVid),
    .cf_DdrOK         (cf_DdrOK), 
    .cf_TspOK         (cf_TspOK), 
    .cf_TslOK         (cf_TslOK), 
    .cf_MaxRd         (cf_MaxRd), 
    .cf_MaxWr         (cf_MaxWr), 
    .cf_RstActTim     (cf_RstActTim),
    .cf_BAMatch       (cf_BAMatch),
    .cf_TCclk         (cf_TCclk),
    .cf_s0ignore      (cf_s0ignore),
    .cf_matchss       (cf_matchss),
    .cf_IbiExtData    (cf_IbiExtData),  // extended data after IBI
    .cf_IbiMapIdx     (cf_IbiMapIdx),   // mapped index for IBI
    .cf_i2c_dev_rev   (cf_i2c_dev_rev), // if extended i2c device ID
    .cf_HdrCmd        (cf_HdrCmd),      // enable for HDR Cmd as MMR
    .cf_CccMask       (cf_CccMask),     // Mask enables for unhandled CCCs
    .cf_vgpio         (cf_vgpio),       // VGPIO control
    .map_daa_use      (map_daa_use),    // if MAP has auto-DAA regs
    .map_daa_dcr      (map_daa_dcr),    // "
    .map_daa_pid      (map_daa_pid),    // "
    .SetDA            (SetDA),          // if [0]=1, wants to override DA (when not enabled)
    .SetMappedDASA    (SetMappedDASA),  // if mapped DAs/SAs supported
    .SetSA10b         (SetSA10b),       // if [1] is SA 10-bit
    .is_slave         (1'b1),           // always slave mode
    .master_comp      (1'b0),           // not used since slave only
    .raw_ActState     (raw_ActState), 
    .raw_EvState      (raw_EvState), 
    .raw_TimeC        (raw_TimeC),
    .raw_Request      (raw_Request), 
    .raw_DynAddr      (raw_DynAddr), 
    .raw_DynChgCause  (raw_DynChgCause),
    .raw_match_idx    (raw_match_idx),  // last 3 matching addressses
    .inp_dma_last_tb  (dma_req_tb_ack_last),
    .reg_IntEna       (reg_IntEna), 
    .reg_DmaCtrl      (reg_DmaCtrl), 
    .inp_IntStates    (inp_IntStates),
    .reg_EvPend       (reg_EvPend),
    .reg_EvIbiByte    (reg_EvIbiByte),
    .inp_EvNoCancel   (inp_EvNoCancel),
    .inp_EvDet        (inp_EvDet),
    .inp_GenErr       (inp_GenErr), 
    .inp_DataErr      (inp_DataErr), 
    .msk_GenErr       (msk_GenErr),
    .msk_DataErr      (msk_DataErr),
    .inp_err_loc      (inp_IntStates[`IS_ERRWARN]),
    .reg_clrIntStates (reg_clrIntStates),
    .reg_clrGenErr    (reg_clrGenErr),
    .reg_clrDataErr   (reg_clrDataErr),
    .reg_holdOErr     (reg_holdOErr),
    .inpflg_MaxRd     (inpflg_MaxRd), 
    .inpflg_MaxWr     (inpflg_MaxWr), 
    .inp_MaxRW        (inp_MaxRW), 
    .reg_TbEnd        (reg_TbEnd), 
    .reg_TbFlush      (reg_TbFlush), 
    .reg_FbFlush      (reg_FbFlush), 
    .reg_TxTrig       (reg_TxTrig), 
    .reg_RxTrig       (reg_RxTrig), 
    .inp_TxCnt        (inp_TxCnt), 
    .inp_RxCnt        (inp_RxCnt), 
    .inp_TxFull       (inp_TxFull),
    .inp_RxEmpty      (inp_RxEmpty),
    .regflg_wr_cnt    (regflg_wr_cnt), 
    .reg_wdata        (reg_wdata), 
    .regflg_rd_cnt    (regflg_rd_cnt),
    .inp_fb_data      (notify_fb_data), // might only use 8 or 16
    .reg_ActMode      (reg_ActMode),
    .reg_PendInt      (reg_PendInt),
    .reg_StatusRes    (reg_StatusRes),
    .hdr_new_cmd      (hdr_new_cmd),    // pulse in PCLK domain
    .raw_hdr_cmd      (raw_hdr_cmd),    // pulse in PCLK domain
    .map_rstdaa       (map_rstdaa),     // pulse in PCLK domain if rstdaa and Map auto
    .map_setaasa      (map_setaasa),    // pulse in PCLK domain if set aasa
    .map_daa_ena      (map_daa_ena),    // pulse in PCLK if MAP DASA/DAA
    .map_sa_idx       (map_sa_idx),     // index in map if daa_ena
    .map_daa_da       (map_daa_da),     // new DA if map DASA/DAA
    .ibi_wr_fifo      (ibi_wr_fifo),
    .ibi_wr_ack       (ibi_wr_ack),
    .exp_owrite_err   (),               // this and next for master, so ignored
    .exp_oread_err    ()
    `ifdef USE_D_RESET
    ,.d_reset_r        (d_reset_r)      // only if D input reset used
    `endif
  );

endmodule
