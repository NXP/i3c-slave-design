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
//  File            : i3c_daa_slave.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Wed Dec 11 18:20:38 2019 $
//  Revision        : $Revision: 1.72.1.1 $
//
//  IP Name         : i3c_daa_slave
//  Description     : MIPI I3C Dynamic address (CCC) handling
//    This small module processes the ENTDAA ID/BCD/DCR to support the
//    Slave getting its I3C Dynamic Address (when not done via SETDASA).
//    It feeds the bits 1 at a time and also reads the dynamic address
//    from the Master. It also works with the i3c_ccc_slave to handle
//    the other DA related CCCs including RSTDAA, SETNEWDA, SETDASA
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
//    This block is fed data from the outer wrapper and also from the 
//    i3c_sdr_slave_engine. The slave engine detects the ENTDAA,
//    RSTDAA, and a few others commands. But, the other commands are
//    handled by the CCC block.
//    This block exists in large part to hide the many options for how
//    to handle the DAA mode. It also handles the counting and other
//    details of the ENTDAA mechanism.
//    This block composes the 48-bit ID from one of 3 sources:
//    1. All constant from parameters.
//    2. All constant from parameters except Instance, which comes from
//       net or register (IDINST).
//    3. Constant for MIPI VID, but PARTNO register has part number.
//    This block also composes the BCR and DCR each from one of 2 sources:
//    1. Constant from parameters.
//    2. Provided by IDEXT register.
//    The model for feeding the ID is as follows:
//    - When state_in_CCC[CF_DAA_M]==1, then we are active
//      - When o_daa_act=1, we are to feed data, changing on SCL
//      - When o_daa_act=0, we reset out counter (we lost)
//    - We emit 64 bits (48b ID plus BCR and DCR)
//    - Then, we read the 7 bit DA and parity.
//    - If DA+Parity is OK, we ACK in next cycle.
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // local parameters/constants

module i3c_daa_slave #(
    // params: ID and CCC ones not here since handled in separate module
    //         localparams with meaning of params is included below
    parameter         ENA_ID48B = `ID48B_CONST, // const vs. reg
    parameter         ID_48B    = 48'h0,// must be filled in from above with some/all of 48-bit ID
    parameter         ID_AS_REGS= 12'd0,// [0]=IDINST,[1]=ISRAND,[2]=IDDCR,[3]=IDBCR,[4]=VID,[5]=DAwr
    parameter         ID_BCR    = 8'd0, // filled in from above with BCR if not from regs
    parameter         ID_DCR    = 8'd0, // filled in from above with DCR if not from regs
    parameter         ENA_MAPPED= 5'd0, // if extra DAs/SAs allowed plus related
    parameter         MAP_CNT   = 4'd1, // number of extra DAs/SAs allowed
    parameter         MAP_I2CID = 24'd0,// !=0 if I2C extended with DevID
                      //     PID[pos:0],DCR, MMR, res  DAA, AASA,DASA,
    parameter         MAP_DA_AUTO={5'd0,1'b0,1'b0,3'd0,1'b0,1'b0,1'b0},
    parameter         MAP_DA_DAA= 0,    // if not MMR and PID/DCR !=0, is bit array      
    parameter         PIN_MODEL = `PINM_COMBO, // combinatorial pin use
    // one below is computed
    parameter  [7:0]  PID_CNT = MAP_DA_AUTO[`MAPDA_DAA_PID_lb+4:`MAPDA_DAA_PID_lb]
  )
  (
  // 1st Clock and Reset and pin in from outer parts
  input               clk_SCL_n,        // SCL falling edge: start and launching data
  input               clk_SCL,          // SCL rising edge: sample data and read T bit
  input               RSTn,             // master Reset
  input               pin_SDA_in,       // SDA pin as read
  input         [7:0] state_in_CCC,     // engine defines detected CCC states/modes: see CF_xxx
  input               daa_active,       // engine indicates when processing DAA; goes low if lost
  output              daa_act_ok,       // for MAP uses of DAA
  output              daa_inp_drv,      // 1 if output expected
  output              daa_inp_bit,      // bit to output if daa_inp_drv==1
  output              daa_done,         // indicates DA out and on ACK/NACK cycle
  output              daa_acknack,      // in ACK/NACK of ENTDAA new DA
  output        [7:0] dyn_addr,         // dynamic address once set
  output              dyn_addr_chg,     // pulsed for one SCL when DA changes
  output        [2:0] dyn_chg_cause,    // cause of last change
  input         [6:0] id64_cnt,         // counter for 64b of ID and then DA and ACK when DAA
  output       [63:0] daa_out,          // exported for CCC GET commmands
  output              raw_map_daa,      // if MAP DAA was used (see also index)
  output        [3:0] daa_map_idx,      // index that goes with it
  output        [7:1] map_da,           // MAP assigned DA to use
  input               map_chg,          // MAP change happened
  // request to change DA from CCC
  input               set_da,           // pulsed to force DA to change
  input         [7:1] new_da,           // DA to set
  input               set_aasa,         // SA to DA special
  input         [7:0] old_sa,           // SA to use for SETAASA
  // now nets from registers or outer system, depending on ID_AS_REGS
  input         [3:0] cf_IdInst,        // from regs/net or not used
  input               cf_IdRand,        // from reg if used
  input        [31:0] cf_Partno,        // from reg if used
  input         [7:0] cf_IdBcr,         // from reg if used
  input         [7:0] cf_IdDcr,         // from reg if used
  input        [14:0] cf_IdVid,         // from reg if used
  input               cf_DdrOK,         // to OR into BCR
  input         [7:0] cf_SetDA,         // Set DA for Main Master
  input   [MAP_CNT-1:0] map_daa_use,      // which MAP are auto-DAA
  input [(MAP_CNT*8)-1:0] map_daa_dcr,      // DCRs if MAP auto-DAA
  input [(MAP_CNT*PID_CNT)-1:0] map_daa_pid, // PID partials if MAP auto-DAA
  input [(MAP_CNT*10)-1:0] SetMappedDASA,// mapped SAs/DAs if supported
  input               scan_no_rst       // prevents layered reset
  );

  // counter [2:0]=bits in byte (7 to 0), [5:3]=ID[0:5],BCR,DCR using 7 to 0, [6]=1 for ID, 0=DA
  reg           [7:0] i3c_addr;         // dynamic address
  reg                 parity;
  wire                parity_matched;
  wire                map_parity;
  reg           [1:0] delay_i3c;
  wire                rst_i3c_n;
  wire         [63:0] daa_out_0;        // for DA map=0


  // output when contesting ENTDAA except if lost and while getting new DA
  assign daa_inp_bit    = daa_acknack ? ~parity_matched : daa_out[id64_cnt[5:0]];
  assign daa_inp_drv    = daa_active & state_in_CCC[`CF_DAA_M] &
                            (id64_cnt[6] | daa_acknack);
  assign parity_matched = parity == ^i3c_addr[7:1] ^ 1'b1;
  wire valid_aasa       = set_aasa & |old_sa & ~i3c_addr[0];
    // note that only MainMaster would set cf_SetDA except for retention use; neither live
  assign dyn_addr       = cf_SetDA[0] ? cf_SetDA : i3c_addr;
  assign dyn_addr_chg   = (state_in_CCC[`CF_GRP_b] == `CFG_RSTDAA_US) |
                          set_da | delay_i3c[1] | (set_aasa & |old_sa & ~i3c_addr[0]) |
                          raw_map_daa | map_chg;

  generate if (ID_AS_REGS[`IDREGS_DAWR_b]) begin : set_chg_cause
    reg    [2:0] chg_cause; 
    assign dyn_chg_cause = chg_cause;
    always @ (posedge clk_SCL or negedge RSTn)
      if (~RSTn)
        chg_cause   <= 3'd0;
      else if (raw_map_daa|map_chg)
        chg_cause   <= 3'd4;            // only if MAP Auto is enabled
      else if (dyn_addr_chg) begin
        if ((state_in_CCC[`CF_GRP_b] == `CFG_RSTDAA_US))
          chg_cause <= 3'd3;            // reset with RSTDAA
        else if (set_da | set_aasa)
          chg_cause <= 3'd2;            // SETDASA, SETAASA, SETNEWDA
        else
          chg_cause <= 3'd1;            // ENTDAA
      end     
  end else begin
    assign dyn_chg_cause = 2'b00;
  end endgenerate
 

  // for mapped DA/SA use, we let them blow off DA from register
  // interface using reset. This allows a new DA in ENTDAA mode. The
  // model is to copy to map and then reset.
  assign rst_i3c_n = RSTn & (scan_no_rst | ~&cf_SetDA[7:0]);
  `Observe(observe_DA_rst, clk_SCL, ~&cf_SetDA[7:0]) // optional DFT observer
  generate if (PIN_MODEL == `PINM_COMBO) begin : rec_da_combo
    // Note that this is from clk_SCL and not SCL_n to Read on rising
    always @ (posedge clk_SCL or negedge rst_i3c_n)
      if (!rst_i3c_n) begin
        i3c_addr                    <= 8'd0;
        parity                      <= 1'b0;
        delay_i3c                   <= 2'b00;
      end else if (state_in_CCC[`CF_GRP_b] == `CFG_RSTDAA_US)
        i3c_addr[0]                 <= 1'b0; // lose DA
      else if (set_da)                  // set from CCC (NEWDA/DASA)
        i3c_addr                    <= {new_da, 1'b1};
      else if (|delay_i3c) begin
        // we delay officially being i3c 1 cycle to ACK it
        i3c_addr[0]                 <= 1'b1;
        delay_i3c                   <= {delay_i3c[0],1'b0}; // 2 cycles to clear
      end else if (i3c_addr[0])
        ;                               // never for us if assigned
      else if (valid_aasa)              // set DA from SA if valid SA
        i3c_addr                    <= {old_sa[7:1],1'b1};
      else if (daa_active & ~id64_cnt[6]) begin
        if (id64_cnt[3]) begin
          if (|id64_cnt[2:0])
            i3c_addr[id64_cnt[2:0]] <= pin_SDA_in;
          else
            parity                  <= pin_SDA_in; // 8th bit is parity in DAA
        end else 
          delay_i3c[0]              <= parity_matched;
      end 

    assign daa_done       = ~id64_cnt[6] & (id64_cnt[3:0]==4'd7); // on ACK/NACK
    assign daa_acknack    = daa_done;
  end else begin : rec_da_reg
    // Note that this is from clk_SCL and not SCL_n to Read on rising
    // For registered uses, the flip from output (on falling) to input (on rising)
    // needs a 1 cycle gap unlike combo, so we have to use different numbers
    always @ (posedge clk_SCL or negedge rst_i3c_n)
      if (!rst_i3c_n) begin
        i3c_addr                    <= 8'd0;
        parity                      <= 1'b0;
        delay_i3c                   <= 2'b00;
      end else if (state_in_CCC[`CF_GRP_b] == `CFG_RSTDAA_US)
        i3c_addr[0]                 <= 1'b0; // lose DA
      else if (set_da)                  // set from CCC (NEWDA/DASA)
        i3c_addr                    <= {new_da, 1'b1};
      else if (|delay_i3c) begin
        // S3 error if parity not matched - skip and try again
        i3c_addr[0]                 <= parity_matched;
        delay_i3c                   <= {delay_i3c[0],1'b0};
      end else if (i3c_addr[0])
        ;                               // never for us if assigned
      else if (set_aasa & |old_sa)      // set DA from SA if valid SA
        i3c_addr                    <= {old_sa[7:1],1'b1};
      else if (daa_active) begin
        if (~id64_cnt[6]) begin // M sends DA
          if (id64_cnt[3]) begin
            if (~&id64_cnt[2:0])
              i3c_addr[id64_cnt[2:0]+1]<= pin_SDA_in;
          end else if (&id64_cnt[2:0]) begin
            parity                  <= pin_SDA_in;
            delay_i3c[0]            <= 1'b1;
          end
        end
      end 

    assign daa_done       = ~id64_cnt[6] & (id64_cnt[3:0]==4'd6); // on ACK/NACK
    assign daa_acknack    = ~id64_cnt[6] & (id64_cnt[3:0]==4'd7); // to reg ACK
  end endgenerate


  // now we extract the bits we want for the ID using id64_cnt
  generate 
    if (ENA_ID48B==`ID48B_CONST) begin : id_all_const
      assign daa_out_0[63:16] = ID_48B;    // all constants
    end
    if (ENA_ID48B==`ID48B_CONST_INST) begin : id_const_but_inst
      assign daa_out_0[63:16] = {ID_48B[47:16],cf_IdInst[3:0],ID_48B[11:0]};
    end
    if (ENA_ID48B==`ID48B_CONST_PARTNO) begin : id_mostly_reg
      assign daa_out_0[63:16] = {ID_48B[47:33],cf_IdRand,cf_Partno[31:0]};
    end
    if (ENA_ID48B==`ID48B_CONST_NONE) begin : id_all_reg
      assign daa_out_0[63:16] = {cf_IdVid[14:0],cf_IdRand,cf_Partno[31:0]};
    end
  endgenerate

  // now we extract the bits we want for the BCR and DCR
  generate 
    if (ID_AS_REGS[`IDREGS_BCR_b]) begin : bcr_reg
      assign daa_out_0[15:8] = cf_IdBcr[7:0];
    end else begin : bcr_const
      // Note: should we OR in the features to BCR from the
      // params or rely on them to get it right?
      assign daa_out_0[15:8] = {ID_BCR[7:5]|cf_DdrOK,ID_BCR[4:0]};
    end
    if (ID_AS_REGS[`IDREGS_DCR_b]) begin : dcr_reg
      assign daa_out_0[7:0] = cf_IdDcr[7:0];
    end else begin : dcr_const
      assign daa_out_0[7:0] = ID_DCR[7:0];
    end
  endgenerate 

  generate
    genvar i;
    if (ENA_MAPPED[`MAP_ENA_b] & MAP_DA_AUTO[`MAPDA_DAA_b]) begin : auto_daa
      //FREE_VERSION_CUT - remove extended MAP from free version
    end else begin
      // No auto DAA for map
      assign daa_out     = daa_out_0;
      assign daa_act_ok  = 1'b0;
      assign daa_map_idx = 0;
      assign raw_map_daa = 1'b0;
      assign map_da      = 7'd0;
      assign map_parity  = 1'b0;
    end
  endgenerate


endmodule


