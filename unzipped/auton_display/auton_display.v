// This contains a display task to show the Autonomous Slave
// params in context. This must be included into the same
// file as the params are visible (e.g. a test bench) 

  `ifdef NO_MAGIC
    localparam MAGIC_RULES=0;
    localparam MAGIC_MASK=0;
    localparam MAGIC_MATCH=0;
  `endif
  integer oi, mi, mr;
  task AutonomousDisplayRules(); begin
    $display("Autonomous Rules decoded from params");
    $display("Rules:");
    if (REG_RULES[0]) $display("  [%0d] Write only allowed at start of Run (e.g. REG_RUN[index]==0)", 0);
    else              $display("  [%0d] Write allowed in middle of a Run", 0);
    if (REG_RULES[1]) $display("  [%0d] Writes ignored after end of Run (e.g. when next hitting REG_RUN[index]==0", 1);
    else              $display("  [%0d] Writes can go over Run boundaries", 1);
    if (REG_RULES[2]) $display("  [%0d] Only emits oraw_reg_touch[] signal on start of Run (e.g. REG_RUN[index]==0)", 2);
    else              $display("  [%0d] oraw_reg_touch[] signal used on each register", 2);
    if (REG_RULES[3]) $display("  [%0d] Only emits oraw_reg_touch[] signal on Writes, but not reads", 3);
    else              $display("  [%0d] Emits oraw_reg_touch[] signal on both Write and Read", 3);
    if (REG_RULES[4]) $display("  [%0d] Reset Index to 0 on START (but not repeated START)", 4);
    else              $display("  [%0d] Index is not reset on START, so Read following goes from last use", 4);
    if (REG_RULES[5]) $display("  [%0d] Use magic-reg bus for all or some of the indexes", 5);
    else              $display("  [%0d] Magic-reg bus is not used", 5);
    if (~REG_RULES[5] & |MAGIC_RULES) $display("WARNING: Magic rules set but magic-regs not enabled");
    if (REG_RULES[5]) begin
      $display("Magic reg rules:");
      if (MAGIC_RULES[0]) $display("  [%0d] Magic regs for R and W", 0);
      else                $display("  [%0d] Magic regs for Write only; read using normal REG_READABLE", 0);
      if (MAGIC_RULES[1]) $display("  [%0d] Magic regs bus for unhandled CCCs", 1);
      else                $display("  [%0d] Magic regs bus is not used for unhandled CCCs", 1);
      if (~|MAGIC_MASK & ~|MAGIC_MATCH) $display("Magic-regs bus used for all register indexes");
      else                              $display("Magic-regs bus maps: (index & %X)==%X", MAGIC_MASK, MAGIC_MATCH);
    end
    if (REG_RULES[7:6]==0) $display("  [7:6] DDR rule is CMD=index");
    else if (REG_RULES[7:6]==1) $display("  [7:6] DDR rule is CMD=index except 7F which uses byte after");
    else if (REG_RULES[7:6]==2) $display("  [7:6] DDR rule is index is byte after, CMD ignored");
    else                        $display("ERROR: [7:6] DDR rule INVALID");
    if (REG_RULES[8]) $display("  [%0d] NACK if read from invalid register", 8);
    else              $display("  [%0d] Return FF if read from invalid register", 8);
    $display("Maximum register index=%0d. This uses %0d bits to hold index.", MAX_REG, 
          1+((MAX_REG<4)?1:(MAX_REG<8)?2:(MAX_REG<16)?3:(MAX_REG<32)?4:(MAX_REG<64)?5:(MAX_REG<128)?6:7));
    $display("Registers and virtual registers:");
    mr = 0;
    for (oi = 0; oi <= MAX_REG; oi = oi + 1) begin
      if (REG_RULES[5] & ((oi & MAGIC_MASK) == MAGIC_MATCH)) begin
        if (!mr) begin $display("=============Magic=========");
                       $display("%0d: Magic Regs block starts here", oi); end
        mr = 1;
        if (REG_WRITABLE[oi] | REG_RUN[oi] | (REG_REMAP[((oi*8)+7) -: 8] != 8'hFF) |
            REG_BLEND[oi] | (REG_MASK[((oi*8)+7) -: 8] != 8'hFF))
          $display("WARNING: invalid mix of REG_xxx and MAGIC_xxx at index %0d", oi);
        if (REG_READABLE[oi] & MAGIC_RULES[0]) $display("WARNING: Invalid REG_READABLE when magic-reg is RW at index %0d", oi);
        else if (~REG_READABLE[oi] & ~MAGIC_RULES[0]) $display("Note: magic-reg at index %0d is write-only", oi);
      end else begin
        if (~REG_RUN[oi]) begin
          if (REG_WRITABLE[oi] | REG_READABLE[oi] | (oi == 0))
            $display("------------------------------Run-Start----");
          else if (REG_WRITABLE[oi-1] | REG_READABLE[oi-1])
            $display("------------------------------Run-End------");
        end
        // we check for remap, both as vrtual and if we are source
        if (REG_REMAP[((oi*8)+7) -: 8] != 8'hFF) begin
          $display("%3d: Virtual reg - Cluster remapped from %0d", oi, REG_REMAP[((oi*8)+7) -: 8]);
          if (oi < MAX_REG && REG_RUN[oi+1])
            $display("WARNING: RUN will not continue past the cluster to %0d", oi+1);
        end
        for (mi = 0; mi <= MAX_REG; mi = mi + 1) 
          if (REG_REMAP[((mi*8)+7) -: 8] == oi && oi != 8'hFF) begin
            $display("%3d: virtual reg at %0d maps to here", oi, mi);
            if (REG_REMAP[((oi*8)+7) -: 8] != 8'hFF)
              $display("WARNING: remap from %0d will not be remapped from here", mi);
          end
        if (REG_WRITABLE[oi] & ~REG_READABLE[oi])
          if (REG_MASK[((oi*8)+7) -: 8] != 8'hFF)
            $display("%3d: Masked R/W reg masked with %X; local flops", oi, REG_MASK[((oi*8)+7) -: 8]);
          else
            $display("%3d: Normal R/W reg; local flops", oi);
        else if (~REG_WRITABLE[oi] & REG_READABLE[oi])
          if (REG_MASK[((oi*8)+7) -: 8] != 8'hFF)
            $display("%3d: Masked RO reg masked with %X; system flops", oi, REG_MASK[((oi*8)+7) -: 8]);
          else
            $display("%3d: Normal RO reg; system flops", oi);
        else if (~REG_WRITABLE[oi] & ~REG_READABLE[oi])
          $display("%3d: none", oi);
        else begin
          if (REG_BLEND[oi])
            $display("%3d: Blended R/W reg with write mask %X; mix of local and system flops", oi, REG_MASK[((oi*8)+7) -: 8]);
          else
            if (REG_MASK[((oi*8)+7) -: 8] != 8'hFF)
              $display("%3d: Masked 2-level register masked with %X; write local flops, read system flops", oi, REG_MASK[((oi*8)+7) -: 8]);
            else
              $display("%3d: Normal 2-level register; write local flops, read system flops", oi);
        end
      end
    end

  end endtask

