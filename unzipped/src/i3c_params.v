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
//  File            : i3c_params.v
//  Organisation    : MCO
//  Tag             : 1.1.11.a.0.1
//  Date            : $Date: Sat Oct 12 20:30:21 2019 $
//  Revision        : $Revision: 1.64 $
//
//  IP Name         : i3c_params 
//  Description     : MIPI I3C related parameters/constants
//    This contains the defined params as named constants. Note that this
//    is not related to the parametrization of the block per se, but does
//    have the constants used by parameters including the bit breakouts.
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
//    Note the use of `define vs. localparam
//  ----------------------------------------------------------------------------

  //
  // Instantiation Parameters and the possible values/meanings
  // -- See micro-arch spec for more details
  //

  // The allowed parameters are as follows:
  // - ENA_ID48B: set to select how ID is defined. Note BCR/DCR separate
  // -  constants from ID_48B, ID_BCR, ID_DCR as needed
  // - ID_AS_REGS: controls which are SW controlled registers vs nets or constants
  // - ENA_SADDR: selects if there is an i2c static address and how provided if so
  //    constant from SADDR_P as needed
  // - ENA_CCC_HANDLING: selects which CCCs are handled by the block other than
  //                     the builtin ENTDAA, SETDASA, RSTDAA, SETNEWDA
  //    constants from MAX_DS_WR, MAX_DS_RD, MX_DS_RDTURN
  // - ENA_IBI_MR_HJ: enables event generation for interrupts, mastership/p2p, 
  //                     and hot join, as well as timing rule for Bus Avail match
  // - SEL_BUS_IF: related to use of bus and memory mapped registers
  // - PIN_MODEL: controls how pins are handled
  // - FIFO_TYPE: controls use of FIFO, whether internal or external
  // - EXT_FIFO: controls external FIFO type
  // - RSTACT: related to SlaveReset
  //
    // ENA_ID48B
  `define ID48B_CONST         3'd1      // all constant
  `define ID48B_CONST_INST    3'd2      // constant except instance
  `define ID48B_CONST_PARTNO  3'd3      // VID const, Partno is not
  `define ID48B_CONST_NONE    3'd4      // all MMR/nets
    // ID_AS_REGS
  `define IDREGS_INST_b       0         // bit [0]=IDINST is reg
  `define IDREGS_RAND_b       1         // bit [1]=IDRAND is reg and used
  `define IDREGS_DCR_b        2         // bit [2]=IDDCR is reg and used
  `define IDREGS_BCR_b        3         // bit [3]=IDBCR is reg and used
  `define IDREGS_VID_b        4         // bit [4]=IDVID (Vendor ID) is reg and used
  `define IDREGS_DAWR_b       5         // bit [5]=DA writable (override)
  `define IDREGS_SLVENA_b     6         // bit [6]=SLVENA is 1 by default
  `define IDREGS_HDRCMD_b     7         // bit [7]=HDRCMD reg is allowed
  `define IDREGS_VGPIO_b      8         // bit [8]=VGPIO reg is created to control VGPIO
  `define IDREGS_CCCMSK_b     9         // bit [9]=CCCMASK reg is created for masking CCCs
  `define IDREGS_MSK_EW_b     10        // bit [10]=ERRWARNMASK reg is created for masking ERRWARN
    // ENA_SADDR
  `define SADDR_NONE          2'd0      // no static addr
  `define SADDR_CONST         2'd1      // static addr is const
  `define SADDR_NET           2'd2      // static addr is net
  `define SADDR_CONFIG        2'd3      // static addr is field in config
    // ENA_CCC_HANDLING
  `define ENCCC_BASIC_b       0         // bit [0]=Basic CCCs handled (e.g. AS, EN, Status, etc)
  `define ENCCC_MAXES_b       1         // bit [1]=Max CCCs handles (Rd, Wr, DS)
  `define ENCCC_STATINF_b     2         // bit [2]=GETSTATUS info as register field
  `define ENCCC_STATVEND_b    3         // bit [3]=GETSTATUS vendor field as register field
  `define ENCCC_AASA          4         // bit [4]=SETAASA
  `define ENCCC_V11MIN        5         // bit [5]=Min v1.1 (GETCAP, ...)
    // ENA_IBI_MR_HJ
  `define EV_IBI_b            0         // bit [0]=IBI
  `define EV_IBI_DAT_b        1         // bit [1]=IBI with data
  `define EV_MR_b             2         // bit [2]=MR or Peer-to-Peer
  `define EV_HJ_b             3         // bit [3]=HJ
  `define EV_BAMATCH_b        4         // bit [4]=use BAMATCH register
  `define EV_EXTFIFO_b        5         // bit [5]=IBI Extended data FIFO (sep)
    // ERROR_HANDLING
  `define ERR_RDABT_b         0         // bit [0]=ReadAbort for i3c SDR and DDR
  `define ERR_WR_RO_b         1         // bit [1]=1 if PSLVERR on Wr to RO regs
    // Time control enable bits
  `define TC_SYNC_b           0         // bit [0]=Sync time control 
  `define TC_MODE0_b          1         // bit [1]=Async Mode 0 time control 
  `define TC_MODE1_b          2         // bit [2]=Async Mode 1 time control 
  `define TC_SYNC_INT_b       4         // bit [4]=Support SYNC internally (vs. ext)
  `define TC_FREQ_REG         5         // bit [5]=Use reg for GETXTIME freq/acc
    // Time control SYNC values (mapped from commands)
  `define SYNC_NONE           3'd0      // no last command
  `define SYNC_ST             3'd1      // ST command
  `define SYNC_DT             3'd2      // Delay time - with byte
  `define SYNC_TPH            3'd3      // TPH 1st byte
  `define SYNC_TPH2           3'd4      // TPH 2nd byte if any
  `define SYNC_TU             3'd5      // RR - with byte
  `define SYNC_ODR            3'd6      // ODR - with byte
    // HDR enable
  `define HDR_DDR_b           0         // bit [0]=DDR supported
  `define HDR_BT_b            2:1       // bit [2:1]=1 for BT1, 2=BT1,2, 3=BT1,2,4
    // SEL_BUS_IF
  `define SBIF_APB_b          0         // bit [0] enabled APB
  `define SBIF_REGS_b         1         // bit [1] general regs vs. nets
  `define SBIF_IRQ_b          2         // bit [2] Interrupt regs
  `define SBIF_DMA_b          3         // bit [3] DMA support
  `define SBIF_HALF_b         4         // bit [4] WDATAH and RDATAH
    // DMA_TYPE (when selected by SBIF_DMA_b)
  `define DMA_ACK1BEF         2'd0      // ACK before read, so wait for read
  `define DMA_TRIG            2'd1      // request is trigger: not used
  `define DMA_ACK1AFT         2'd2      // ACK with/after, so go on current state
  `define DMA_4PHASE          2'd3      // 4-phase handshake
    // PIN_MODEL
  `define PINM_COMBO          0         // Basic combinatorial use
  `define PINM_REG            1         // Basic registered use
  `define PINM_EXT_REG        2         // extended register use with regs near pads
    // FIFO_TYPE
  `define FIFO_INT_b          0         // bit [0]=internal (so [1]=0)
  `define FIFO_EXT_b          1         // bit [1]=external (so [0]=0)
  `define FIFO_TBHOLD_b       2         // bit [2]=to-bus holding buff used
  `define FIFO_FBHOLD_b       3         // bit [3]=from-bus holding buff used
    // EXT_FIFO
  `define EXT_FIFO_NORMAL     2'd1      // Normal avail/free tracking
  `define EXT_FIFO_REQ        2'd2      // Request model
    // RSTACT
  `define RSTA_ENA_b          0         // if 1, RSTACT amd SlaveReset enabled
  `define RSTA_MMR_b          1         // if 1, times from MMR vs. param
  `define RSTA_CUS_b          2         // if 1, custom RSTACT
  `define RSTA_TIM_PER_b      11:4      // Time needed to reset peripheral
  `define RSTA_TIM_SYS_b      19:12     // Time needed to reset system
  `define RSTA_VAL_CUS_b      25:20     // value for custom (ored to 0x40)
    // ENA_MAPPED
    // Note that mapped used for i2c as well as extra (virtual) DA and SA
    //      Some i2c features are a sep param, but only if MAP enabled
  `define MAP_ENA_b           0         // allow multiple DA/SA
  `define MAP_I2C_SA10_b      1         // 10-bit address
  `define MAP_I2C_SLVRST_b    2         // I2C slave reset
  `define MAP_I2C_HS_b        3         // High speed - just the detector
  `define MAP_VGPIO_b         4         // special vgpio mechanism
    // MAP_DA_AUTO
    // Map automation builds up DAs using HW
  `define MAPDA_DASA_b        0         // 1 if Auto SETDASA
  `define MAPDA_AASA_b        1         // 1 if Auto SETAASA (on map)
  `define MAPDA_DAA_b         2         // 1 if Auto ENTDAA (see below)
  `define MAPDA_DAA_MMR_b     6         // 1 if DAA PID/DCR is MMR
  `define MAPDA_DAA_DCR_b     7         // 1 if DCR swapped
  `define MAPDA_DAA_PID_lb    8         // 5 bits for PID[n:0] or 0 if none
      // below computes width of each element of MAP_DA_DAA if not MMR
  `define MAP_DACONST_w(mda)  ((mda[MAPDA_DAA_DCR_b]?8:0)+\
                               (mda[MAPDA_PID_lb+:5]?(mda[MAPDA_PID_lb+:5]+1):0)
  
  //
  // CCC related values
  //

   // params for CCC detection flags
  `define CF_POSS         0             // possible CCC since 7E/W seen
  `define CF_ONLY_b       4:0           // range to test for POSS only
  `define CF_ONLY_POSS    5'd1          // value of poss only
  `define CF_BCAST        1             // in broadcast command
  `define CF_DIRECT       2             // in direct command
  `define CF_DAA_M        3             // in DAA mode (7E/R will activate each)
  `define CF_HDR_M        4             // in an ENTHDR mode (Exit Pattern will exit)
  `define CF_GRP_b        7:5           // index of CCC detected
  `define CFG_NEWDA       3'd1          // in SETNEWDA direct command
  `define CFG_RSTDAA_D    3'd2          // RSTDAA direct command
  `define CFG_RSTDAA_US   3'd3          // RSTDAA matches us or is bcast
  `define CFG_HDRDDR      3'd4          // DDR is only one we break out
          // 5 to 7 for DASA
  `define CFG_DASA        3'd5          // in SETDASA direct command
  `define CFG_DASA_SA     3'd6          // matched our SA
  `define CFG_DASA_P2P    3'd7          // matched 01 (point to point)

  // params for CCCs
  `define CCC_ENEC        8'h00         // Bcast enable Events
  `define CCC_ENEC_D      8'h80         // Direct enable Events
  `define CCC_DISEC       8'h01         // Bcast disable Events
  `define CCC_DISEC_D     8'h81         // Direct disable Events
  `define CCC_ENTAS0      8'h02         // Bcast Enter Activity State Normal
  `define CCC_ENTAS0_D    8'h82         // Direct Enter Activity State Normal
  `define CCC_ENTAS1      8'h03         // Bcast Enter Activity State low
  `define CCC_ENTAS1_D    8'h83         // Direct Enter Activity State low
  `define CCC_ENTAS2      8'h04         // Bcast Enter Activity State lower
  `define CCC_ENTAS2_D    8'h84         // Direct Enter Activity State lower
  `define CCC_ENTAS3      8'h05         // Bcast Enter Activity State lowest
  `define CCC_ENTAS3_D    8'h85         // Direct Enter Activity State lowest
  `define CCC_RSTDAA      8'h06         // Bcast reset DAA
  `define CCC_RSTDAA_D    8'h86         // Direct reset DAA
  `define CCC_ENTDAA      8'h07         // Bcast enter DAA mode
  `define CCC_SETDASA_D   8'h87         // Direct Assign a DA from an SA or special
  `define CCC_SETNEWDA_D  8'h88         // Direct set a new Dynamic addr
  `define CCC_SETMXWL     8'h09         // set Max write len
  `define CCC_SETMXWL_D   8'h89         // set Max write len
  `define CCC_SETMXRL     8'h0A         // set Max read len
  `define CCC_SETMXRL_D   8'h8A         // set Max read len
  `define CCC_GETMXWL_D   8'h8B         // get Max write len
  `define CCC_GETMXRL_D   8'h8C         // get Max read len
  `define CCC_GETPID_D    8'h8D         // get Provisional ID
  `define CCC_GETBCR_D    8'h8E         // get BCR
  `define CCC_GETDCR_D    8'h8F         // get DCR
  `define CCC_GETSTATUS_D 8'h90         // get status byte pair
  `define CCC_GETMXDS_D   8'h94         // get Max data speed
  `define CCC_GETHDRCAP_D 8'h95         // get HDR capabilities
  `define CCC_ENTHDR_MSK  (8'h20>>3)    // mask for any ENTHDR (low 3 bits are which)
  `define CCC_SETXTIME    8'h28         // Broadcast set time control
  `define CCC_SETXTIME_D  8'h98         // Direct set time control
  `define CCC_GETXTIME_D  8'h99         // get time control info
  // now 1.0.1 and 1.1 version CCCs
  `define CCC_RSTACT      8'h2A         // Set SlaveRst behavior for all
  `define CCC_RSTACT_D    8'h9A         // Set or Get SlaveRst reset for a slave
  `define CCC_SETAASA     8'h29         // Broadcast only SA->DA


  //
  // Interrrupt states (status)
  //
  `define IS_START        8             // Start seen
  `define IS_MATCHED      9             // matched DA or SA
  `define IS_STOP         10            // stop seen
  `define IS_RXPEND       11            // RX (fb) or RX FIFO trigger or fb DMA done
  `define IS_TXPEND       12            // TX (tb) or TX FIFO trigger or tb DMA done
  `define IS_DACHG        13            // Dynamic addr added/lost
  `define IS_CCC          14            // CCC not handled (not same as REQ CCC)
  `define IS_ERRWARN      15            // One or more errors seen
  `define IS_DDRMATCH     16            // DDR command matched us
  `define IS_CHANDLED     17            // CCC handled by the block
  `define IS_EVENT        18            // Event sent
  `define IS_SLVRST       19            // Slave Reset of block

  //
  // Slave Reset action request value (what action to take)
  //
  `define RACT_DEF        3'd0          // perpheral by default, unless escalate
  `define RACT_FULL       3'd1          // reser chip
  `define RACT_NONE       3'd2          // do nothing this one time
  `define RACT_CUST       3'd4          // custom (if enabled)

  //
  // Special observer for DFT - optional
  //
  `ifdef ENA_OBSERVER
   `define Observe(oreg, clk, mon) reg oreg /* cadence preserve sequence*/; \
        always @ (posedge clk) oreg <= mon; /* clk is scan_clk or single sourced */
  `else
   `define Observe(oreg, clk, mon) /* unused */
  `endif


