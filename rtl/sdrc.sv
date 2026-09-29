// This file is part of rmSdram, an SDRAM controller for the MiSTer SDRAM boards.
// Copyright (c) 2026 Umberto Parisi (rmonic79)
//
// rmSdram is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
// later version. It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details: LICENSE in the root of the repository, or <https://www.gnu.org/licenses/>.
//
// A commercial license, for use without the obligations of the GPL (for example in closed-source products), is
// available from the author, who can also be contacted for other licensing needs.

//============================================================================
// sdrc -- synchronous SDRAM controller for the MiSTer SDRAM boards
// (SPEC_controller.md). Clean room: from the AS4C32M16SB datasheet, the board
// schematic and the pin delays measured on a compiled netlist only; the
// single chip boards from the AS4C16M16SA datasheet (Rev. 3.0), the
// W9825G6KH datasheet (Rev. A04) and the schematic of the 32 MB board
// (sdram_xs 2.2).
//
// Features: power-up, refresh, N_PORTS access ports, one shared write buffer.
//   - open row per bank (2 chips x 4 banks): a new row closes the old one;
//   - burst length 1 in the mode register, one column command per halfword;
//   - one operation on the data bus at a time, the next one loaded in the
//     cycle the current one issues its last command;
//   - the next operation (SPEC 4.4): refresh at the debt limit, urgent ports,
//     aged requests (reads, then the head of the write buffer), full write
//     buffer, open rows, the rest, and the refresh in the cycles with
//     nothing else to do (SPEC 4.5);
//   - banks overlapped (SPEC 4.4): in every cycle the current operation
//     leaves the command bus free, the candidate at the top of that list
//     gets its bank prepared (PRECHARGE of another row, ACTIVE of its row),
//     or the banks of the chip of a pending refresh get closed, so that when
//     its turn comes only the column commands are left. The current
//     operation always goes first and is never delayed by a preparation;
//   - command stage (SPEC 4.7), CMD_STAGE = 1: every command is decided one
//     cycle before it is on the pins, so the pin registers are fed by
//     registers only; one cycle more of read latency, the same distances
//     between commands. CMD_STAGE = 0 puts the decided command on the pins
//     at the edge that ends its decision, for clocks where the logic before
//     the pins fits in one cycle: no cycle more of read latency.
// Board (SPEC 2.6): two chips share every line; sdram_ncs selects the chip
// (0 = U1, 1 = U2, through an inverter); DQML = A11 and DQMH = A12; CKE is
// tied high. A NOP is RAS#/CAS#/WE# high (the CS cannot deselect both).
// Boards (N_CHIPS, COL_BITS; all have 4 banks of 8192 rows):
//   N_CHIPS 2, COL_BITS 10: the 128 MB board, two AS4C32M16SB (the default);
//   N_CHIPS 1, COL_BITS 10: one AS4C32M16SB, 64 MB;
//   N_CHIPS 1, COL_BITS 9:  one AS4C16M16SA or W9825G6KH, the 32 MB board
//                           (sdram_xs 2.2: CS# from sdram_ncs, DQML = A11,
//                           DQMH = A12, CKE high). It also runs on the other
//                           boards: on the 128 MB board sdram_ncs low is U1.
// With one chip sdram_ncs stays low, and the power-up and the refresh are
// those of that chip only. The W9825G6KH gives tRRD, tWR and tMRD as two
// clocks: the instance passes T_RRD_PS, T_WR_PS and T_MRD_PS of at least
// two periods, and INIT_REFS = 8 (eight power-up refreshes).
//
// Ports (SPEC 4.1), all on clk. The address is the linear halfword address
// of the memory, the same for every port; MAP turns it into bank, chip, row
// and column, one map for all the ports (two ports that share memory must
// see the same cells). With C = COL_BITS:
//   MAP 0: bank h[25:24], chip h[23], row h[22:10], col h[9:0]   (2 chips)
//          bank h[C+14:C+13], row h[C+12:C], col h[C-1:0]        (1 chip)
//   MAP 1: bank h[12:11], chip h[10], row h[25:13], col h[9:0]   (2 chips)
//          bank h[C+1:C], row h[C+14:C+2], col h[C-1:0]          (1 chip)
// The bits above the memory (h[25] with one chip and C = 10, h[25:24] with
// C = 9) are not used: those addresses fold onto the memory.
// The column is always h[C-1:0]: a burst that runs past the last column goes
// on in the next 2^C halfwords (a block, below), mapped again.
//   write: one dword, wdata/wbe, at an even halfword address, taken in the
//          cycle req and port_ready are high, into the write buffer. Writes
//          of several ports in the same cycle are taken one at a time, in
//          turn. A write with wbe = 0 is taken and dropped.
//   read:  len halfwords (1 .. 2^LEN_BITS-1, LEN_BITS up to COL_BITS + 1:
//          1023 (LEN_BITS 10) with either column, up to 2047 with COL_BITS
//          10) from any halfword address, back
//          on rdata with rvalid[port], in the order of that port's requests.
//          One read slot per port: a read is taken when its slot is free.
//          A read with len = 0 is taken and dropped.
//   urgent: a real-time port raises it when it must be served first (4.8).
// Port capabilities (constants, bit g for port g): RD_PORTS and WR_PORTS are
// the ports that read and that write, URG_PORTS those meant to be always
// urgent. What a port never does is not built: no read slot for a port that
// only writes, no write intake for one that only reads, and no age counter
// nor row-hit order for a URG_PORTS port: its reads go in the urgent pass
// only with its urgent input high, so tie that input high. The defaults build
// everything.
// Order (SPEC 4.2): a read waits for the writes taken before it when one of
// them touches a block (2^COL_BITS halfwords, 1024 with the default) the read
// touches, the blocks compared on the address bits the map uses (with one
// chip an address above the memory is the same block as the one it folds
// onto); a write of a port is not
// taken while a read of that port is still in its slot. A read and a write
// of two different ports have no order between them when the write is taken
// in the same cycle as the read or after it.
//
// Timings are parameters in picoseconds (datasheet p.22 Table 16, -7 grade by
// default) turned into cycles of this clock: cycles = ceil(ps / period) (tREFI: floor).
// Every "at least K cycles since that command" is kept as a history of K-1
// bits (bit i: the command was i+1 cycles ago) and holds when no bit of the
// last K-1 cycles is set; a K of 1 leaves nothing to keep.
//
// Read capture (SPEC 4.7): the edge of clk at which the data pins are taken
// is chosen at elaboration, from the datasheet (tAC, tOH) and the delays of
// the pins in the two corners (SDRAM_CLK after the fall of clk, input
// setup/hold): the first half period, counted from the edge that loads the
// READ command, inside the window valid in BOTH corners. If there is none the
// simulation stops (CAP_H_FORCE picks one by hand, for measurements only).
//============================================================================
module sdrc #(
	parameter int CLK_KHZ      = 66667,
	parameter int N_PORTS      = 1,
	parameter int LEN_BITS     = 4,          // read length in halfwords, 1 .. 2^LEN_BITS-1 (LEN_BITS <= COL_BITS + 1)
	parameter int MAP          = 1,
	parameter int N_CHIPS      = 2,          // chips on the board: 2 (sdram_ncs selects one) or 1 (see Boards)
	parameter int COL_BITS     = 10,         // column address bits of the chip: 10 or 9 (see Boards)
	parameter int WB_DEPTH     = 4,          // dwords of the write buffer, a power of two >= 2
	parameter int REF_MAX      = 4,          // refresh debt at which the refresh goes first
	parameter int AGE_MAX      = 64,         // cycles a request waits before it goes before the open rows
	parameter int T_RC_PS      = 63000,
	parameter int T_RFC_PS     = 63000,
	parameter int T_RCD_PS     = 21000,
	parameter int T_RP_PS      = 21000,
	parameter int T_RRD_PS     = 14000,
	parameter int T_MRD_PS     = 14000,
	parameter int T_RAS_PS     = 42000,
	parameter int T_WR_PS      = 14000,
	parameter int T_AC2_PS     = 6000,       // tAC max, CL=2
	parameter int T_AC3_PS     = 5400,       // tAC max, CL=3 (-7)
	parameter int T_OH_PS      = 2500,
	parameter int T_REFI_PS    = 7800000,    // max average interval, 8192 per 64 ms
	parameter int T_INIT_PS    = 200000000,  // 200 us of stable clock (p.23 Note 11)
	parameter int INIT_REFS    = 2,          // "a minimum of 2 Auto-Refresh" (Note 11)
	// pins of a compiled netlist on the DE10-nano, SDRAM_CLK on an ordinary output path (bench/pin_datasheet.tcl)
	parameter int TCKO_SLOW_PS = 8601,       // SDRAM_CLK rise after the fall of clk, slow 85 C
	parameter int TCKO_FAST_PS = 3121,       // same, fast 0 C
	parameter int TSU_SLOW_PS  = -584,       // SDRAM_DQ input setup, worst of rise/fall
	parameter int TH_SLOW_PS   = 3737,
	parameter int TSU_FAST_PS  = 248,
	parameter int TH_FAST_PS   = 1716,
	parameter int CAP_H_FORCE  = -1,         // -1: computed; else the half period by hand
	parameter int CMD_STAGE    = 1,          // 1: the command stage (see the header); 0: none
	parameter logic [N_PORTS-1:0] RD_PORTS  = '1,   // ports that read (see Port capabilities)
	parameter logic [N_PORTS-1:0] WR_PORTS  = '1,   // ports that write
	parameter logic [N_PORTS-1:0] URG_PORTS = '0    // ports always urgent: tie their urgent input high (see Port capabilities)
)(
	input             clk,
	input             rst_n,

	output reg        ready,              // power-up sequence done on every chip

	// access ports
	input      [N_PORTS-1:0]               req,
	input      [N_PORTS-1:0]               we,
	input      [N_PORTS-1:0]               urgent,
	input      [N_PORTS*26-1:0]            addr,     // port g: addr[g*26 +: 26]
	input      [N_PORTS*LEN_BITS-1:0]      len,      // port g: len[g*LEN_BITS +: LEN_BITS]
	input      [N_PORTS*32-1:0]            wdata,    // port g: wdata[g*32 +: 32]
	input      [N_PORTS*4-1:0]             wbe,      // port g: wbe[g*4 +: 4]
	output     [N_PORTS-1:0]               port_ready,
	output reg [15:0]                      rdata,
	output reg [N_PORTS-1:0]               rvalid,

	// pins (registered; SDRAM_CLK is not produced here, see README §8)
	output            sdram_ncs,
	output            sdram_nras,
	output            sdram_ncas,
	output            sdram_nwe,
	output      [1:0] sdram_ba,
	output     [12:0] sdram_a,
	output      [1:0] sdram_dqm,          // {DQMH, DQML} pins, for boards that wire them (on the
	                                      // 128 MB board they are not connected: DQM is A12/A11)
	inout      [15:0] sdram_dq
);
	//------------------------------------------------------------ timings
	localparam int PERIOD_PS = 1000000000 / CLK_KHZ;
	localparam int HALF_PS   = PERIOD_PS / 2;
	function automatic int cyc(input int ps);
		return (ps + PERIOD_PS - 1) / PERIOD_PS;
	endfunction
	localparam int C_RP   = cyc(T_RP_PS);
	localparam int C_RFC  = cyc(T_RFC_PS);
	localparam int C_RCD  = cyc(T_RCD_PS);
	localparam int C_RC   = cyc(T_RC_PS);
	localparam int C_RAS  = cyc(T_RAS_PS);
	localparam int C_WR   = cyc(T_WR_PS);
	localparam int C_RRD  = cyc(T_RRD_PS);
	localparam int C_MRD  = cyc(T_MRD_PS);
	localparam int C_REFI = T_REFI_PS / PERIOD_PS;       // floor: never later than tREFI
	localparam int C_INIT = cyc(T_INIT_PS);
	// CAS latency: 2 when the period allows it (tCK(CL=2) >= 10 ns, Table 16)
	localparam int CL     = (PERIOD_PS >= 10000) ? 2 : 3;
	localparam int T_AC_PS = (CL == 2) ? T_AC2_PS : T_AC3_PS;
	localparam [12:0] MODE = {3'b000, 1'b0, 2'b00, 3'(CL), 1'b0, 3'b000};  // BL 1, sequential, burst write

	//--------------------------------------------------- read capture edge
	// The READ loaded at a rising edge (time 0) is sampled by the chips at
	// HALF + TCKO (the next rise of SDRAM_CLK); its word is valid at the pins
	// from the edge CL-1 after that + tAC to the edge CL after + tOH.
	function automatic int max2(input int x, input int y); return (x > y) ? x : y; endfunction
	function automatic int min2(input int x, input int y); return (x < y) ? x : y; endfunction
	function automatic int cap_h_calc();
		int s_slow, e_slow, s_fast, e_fast, lo, hi, res;
		s_slow = (CL - 1) * PERIOD_PS + HALF_PS + TCKO_SLOW_PS + T_AC_PS;
		e_slow = CL * PERIOD_PS + HALF_PS + TCKO_SLOW_PS + T_OH_PS;
		s_fast = (CL - 1) * PERIOD_PS + HALF_PS + TCKO_FAST_PS + T_AC_PS;
		e_fast = CL * PERIOD_PS + HALF_PS + TCKO_FAST_PS + T_OH_PS;
		lo = max2(s_slow + TSU_SLOW_PS, s_fast + TSU_FAST_PS);
		hi = min2(e_slow - TH_SLOW_PS, e_fast - TH_FAST_PS);
		// from the top down: the last h kept is the first valid one
		res = -1;
		for(int h = 31; h >= 1; h--)
			if(h * HALF_PS >= lo && h * HALF_PS <= hi) res = h;
		return res;
	endfunction
	localparam int CAP_H_AUTO = cap_h_calc();
	localparam int CAP_H      = (CAP_H_FORCE >= 0) ? CAP_H_FORCE : CAP_H_AUTO;
	localparam int CAP_FALL   = CAP_H % 2;
	// rising edge (P0 + CAP_M) at which the captured word enters the core
	localparam int CAP_M      = (CAP_FALL != 0) ? (CAP_H + 1) / 2 : CAP_H / 2;

	//------------------------------------------------- command histories
	// "at least K cycles since the command": K-1 bits of history, bit i set
	// when the command was i+1 cycles ago; recent(K) are the bits that must
	// all be clear. K <= 1 needs no history (the mask is zero).
	function automatic int shw(input int k); return (k > 1) ? k - 1 : 1; endfunction
	function automatic [31:0] recent(input int k);
		return (k <= 1) ? 32'd0 : ((32'd1 << (k - 1)) - 32'd1);
	endfunction
	localparam int ACTW = shw(max2(max2(C_RC, C_RAS), C_RCD));   // ACTIVE of a bank
	localparam int PREW = shw(C_RP);                             // PRECHARGE of a bank
	localparam int WRW  = shw(C_WR);                             // WRITE on a bank
	localparam int CHW  = shw(C_RRD);                            // ACTIVE on a chip
	localparam int RDW  = shw(CL + 1);                           // READ (bus turnaround)
	localparam int RFCW = shw(C_RFC);                            // AUTO REFRESH on a chip
	localparam [ACTW-1:0] R_RCD = ACTW'(recent(C_RCD));
	localparam [ACTW-1:0] R_RAS = ACTW'(recent(C_RAS));
	localparam [ACTW-1:0] R_RC  = ACTW'(recent(C_RC));
	localparam [PREW-1:0] R_RP  = PREW'(recent(C_RP));
	localparam [WRW-1:0]  R_WR  = WRW'(recent(C_WR));
	localparam [CHW-1:0]  R_RRD = CHW'(recent(C_RRD));
	localparam [RDW-1:0]  R_RD  = RDW'(recent(CL + 1));
	localparam [RFCW-1:0] R_RFC = RFCW'(recent(C_RFC));
	// the same masks one cycle ahead: |(h & (R >> 1)) is "still blocked in
	// the next cycle" for a history h that will shift by one
	localparam [PREW-1:0] R_RPn  = PREW'(recent(C_RP) >> 1);
	localparam [ACTW-1:0] R_RCn  = ACTW'(recent(C_RC) >> 1);
	localparam [CHW-1:0]  R_RRDn = CHW'(recent(C_RRD) >> 1);
	localparam [RFCW-1:0] R_RFCn = RFCW'(recent(C_RFC) >> 1);
	// tRRD in cycles: 1 (an ACTIVE never blocks another), 2 (blocks the next
	// cycle only), more (a preparation waits for the row of the operation)
	localparam bit RRD1 = (C_RRD <= 1);
	localparam bit RRD2 = (C_RRD == 2);

	// widths: each counter just holds the largest value it is compared with
	localparam int WAITW = $clog2(C_INIT + 1);        // power-up waits, the longest is C_INIT
	localparam int REFW  = max2($clog2(C_REFI), 1);   // tREFI counter, 0 .. C_REFI-1
	localparam int AGEW  = $clog2(AGE_MAX + 1);       // ages, stop at AGE_MAX
	localparam int NRW   = max2($clog2(INIT_REFS + 1), 1);
	localparam int RQD   = max2(CAP_M, 2);            // stages of the read pipeline
	localparam int PW    = (N_PORTS > 1) ? $clog2(N_PORTS) : 1;
	localparam int WDW   = $clog2(2 * N_PORTS);       // index width of the doubled write candidates, >= PW
	localparam int WB_AW = $clog2(WB_DEPTH);
	localparam int WBW   = WB_AW + 1;           // width of the buffer pointers and count
	localparam int CB    = COL_BITS;            // column bits; the block is h[CB +: 16], above the column
	// order check: width of column + length (one bit more when a read can run
	// past two blocks), and the block bits the map uses (one chip: lin[15] is
	// above the memory, not used)
	localparam int RSW   = (LEN_BITS > CB) ? CB + 2 : CB + 1;
	localparam [15:0] LIN_USED = (N_CHIPS == 2) ? 16'hFFFF : 16'h7FFF;

	// synthesis translate_off
	initial begin
		$display("sdrc: CLK %0d kHz, period %0d ps, CL %0d, cycles tRP %0d tRFC %0d tRCD %0d tRC %0d tRAS %0d tWR %0d tRRD %0d tMRD %0d tREFI %0d init %0d",
		         CLK_KHZ, PERIOD_PS, CL, C_RP, C_RFC, C_RCD, C_RC, C_RAS, C_WR, C_RRD, C_MRD, C_REFI, C_INIT);
		$display("sdrc: read capture half period %0d (%0s edge, %0d ps after the READ), computed %0d",
		         CAP_H, (CAP_FALL != 0) ? "falling" : "rising", CAP_H * HALF_PS, CAP_H_AUTO);
		$display("sdrc: %0d ports, MAP %0d, write buffer %0d, reads up to %0d halfwords, refresh first at debt %0d, age %0d, command stage %0d, %0d chip(s), %0d column bits",
		         N_PORTS, MAP, WB_DEPTH, (1 << LEN_BITS) - 1, REF_MAX, AGE_MAX, CMD_STAGE, N_CHIPS, COL_BITS);
		if(CAP_H < 1) $fatal(1, "sdrc: no capture edge valid in both corners at this clock");
		if(CL != 2) $fatal(1, "sdrc: a clock above 100 MHz would need CL=3, which is not supported (read DQM on A11/A12, SPEC 2.6 rule 4)");
		if(WB_DEPTH < 2 || (1 << WB_AW) != WB_DEPTH) $fatal(1, "sdrc: WB_DEPTH must be a power of two >= 2");
		if(REF_MAX < 1 || REF_MAX > 8) $fatal(1, "sdrc: REF_MAX out of 1..8");
		if(AGE_MAX < 1 || AGE_MAX > 255) $fatal(1, "sdrc: AGE_MAX out of 1..255");
		if(LEN_BITS > CB + 1) $fatal(1, "sdrc: LEN_BITS > COL_BITS + 1: a read could touch four blocks, the order check looks at three");
		if(N_CHIPS != 1 && N_CHIPS != 2) $fatal(1, "sdrc: N_CHIPS must be 1 or 2");
		if(COL_BITS != 9 && COL_BITS != 10) $fatal(1, "sdrc: COL_BITS must be 9 or 10");
		if(N_CHIPS == 2 && COL_BITS != 10) $fatal(1, "sdrc: two chips only with COL_BITS 10 (the 128 MB board)");
	end
	// synthesis translate_on

	//----------------------------------------------------------- data pins
	wire        dq_oe;     // the output registers of the data pins (command stage below)
	wire [15:0] dq_out;
	assign sdram_dq = dq_oe ? dq_out : 16'hzzzz;

	// command stage: the command decided in a cycle is held in these core
	// registers and copied into the pin registers at the next edge, so that
	// the pin registers (in the I/O cells, far from the logic) are fed by a
	// register and nothing else. Every command leaves one cycle after its
	// decision, all alike: the distances between commands, and so every
	// timing of the chip, are those of the decisions; the read data comes
	// back one cycle later, and the read pipeline starts from the cycle the
	// READ is on the pins (rd_q). With CMD_STAGE = 0 there is no stage: the
	// q_* registers are the pin registers, a command is on the pins at the
	// edge that ends its decision and the read pipeline starts from that edge
	// (rd_now).
	reg        q_ncs = 1'b0;
	reg  [2:0] q_cmd = 3'b111;
	reg  [1:0] q_ba = 2'd0;
	reg [12:0] q_a = 13'h1800;
	reg  [1:0] q_dqm = 2'b11;
	reg        q_oe = 1'b0;
	reg [15:0] q_dout = 16'h0000;
	generate
		if(CMD_STAGE != 0) begin : g_stage
			reg        p_ncs = 1'b0;
			reg  [2:0] p_cmd = 3'b111;
			reg  [1:0] p_ba = 2'd0;
			reg [12:0] p_a = 13'h1800;
			reg  [1:0] p_dqm = 2'b11;
			reg        p_oe = 1'b0;
			reg [15:0] p_dout = 16'h0000;
			always @(posedge clk) begin
				p_ncs  <= q_ncs;
				p_cmd  <= q_cmd;
				p_ba   <= q_ba;
				p_a    <= q_a;
				p_dqm  <= q_dqm;
				p_oe   <= q_oe;
				p_dout <= q_dout;
			end
			assign sdram_ncs = p_ncs;
			assign {sdram_nras, sdram_ncas, sdram_nwe} = p_cmd;
			assign sdram_ba  = p_ba;
			assign sdram_a   = p_a;
			assign sdram_dqm = p_dqm;
			assign dq_oe     = p_oe;
			assign dq_out    = p_dout;
		end
		else begin : g_direct
			assign sdram_ncs = q_ncs;
			assign {sdram_nras, sdram_ncas, sdram_nwe} = q_cmd;
			assign sdram_ba  = q_ba;
			assign sdram_a   = q_a;
			assign sdram_dqm = q_dqm;
			assign dq_oe     = q_oe;
			assign dq_out    = q_dout;
		end
	endgenerate

	reg [15:0] dq_fall;
	always @(negedge clk) dq_fall <= sdram_dq;

	//------------------------------------------------------- commands (p.9)
	localparam [2:0] CMD_NOP = 3'b111, CMD_ACT = 3'b011, CMD_RD  = 3'b101, CMD_WR = 3'b100,
	                 CMD_PRE = 3'b010, CMD_REF = 3'b001, CMD_MRS = 3'b000;

	// the pins start as a NOP with DQM high (the initial values of the q_* and
	// p_* registers): with the CS that selects the chip, pins at zero would be
	// a MODE REGISTER SET
	initial begin
		ready = 1'b0; rvalid = '0; rdata = 16'h0;
	end

	// the block above the column -> {bank[1:0], chip, row[12:0]}; one chip:
	// chip 0, and lin[15] is above the memory
	function automatic [15:0] map_chunk(input [15:0] lin);
		if(N_CHIPS == 2) return (MAP == 1) ? {lin[2:1], lin[0], lin[15:3]} : lin;
		else             return (MAP == 1) ? {lin[1:0], 1'b0, lin[14:2]} : {lin[14:13], 1'b0, lin[12:0]};
	endfunction
	// {chip, bank}: index of the bank machines
	function automatic [2:0] bank_ix(input [15:0] m);
		return {m[13], m[15:14]};
	endfunction

	//----------------------------------------------------- sequencer state
	localparam [2:0] S_WAIT = 3'd0, S_IREF = 3'd2, S_MRS = 3'd3, S_RUN = 3'd4;
	reg  [2:0] st = S_WAIT;
	reg [WAITW-1:0] wait_c = WAITW'(C_INIT);  // the 200 us hold also without a reset
	reg        chip = 1'b0;            // chip of the next power-up step
	reg [NRW-1:0] nref = '0;

	task automatic issue(input chip_i, input [2:0] c, input [1:0] b, input [12:0] ad);
		q_ncs <= chip_i;
		q_cmd <= c;
		q_ba  <= b;
		q_a   <= ad;
		// own registers (each output register drives one pin, so it goes into
		// the I/O cell): the mask of a column command or of a NOP, as on A12/A11;
		// low with the other commands once running, high before
		q_dqm <= (c == CMD_RD || c == CMD_WR || c == CMD_NOP) ? ad[12:11] :
		         (st == S_RUN) ? 2'b00 : 2'b11;
	endtask

	// refresh debt per chip: refreshes due and not yet given to the engine
	reg [REFW-1:0] refi_c = '0;
	reg  [3:0] debt  [2];
	reg [RFCW-1:0] rfc_s [2];              // AUTO REFRESH history, per chip (tRFC)
	wire       refi_tick = (st == S_RUN) && (refi_c == REFW'(C_REFI - 1));

	//------------------------------------------------ banks (2 chips x 4)
	reg        bk_open [8];
	reg [12:0] bk_row  [8];
	reg [ACTW-1:0] bk_act [8];           // ACTIVE history (tRCD, tRAS, tRC)
	reg [PREW-1:0] bk_pre [8];           // PRECHARGE history (tRP)
	reg [WRW-1:0]  bk_wr  [8];           // WRITE history (tWR)
	reg [CHW-1:0]  ch_act [2];           // ACTIVE history of the chip (tRRD)
	reg [RDW-1:0]  since_rd = '0;        // READ history (bus turnaround before a WRITE)

	// per bank, from the histories: an ACTIVE (tRP, tRC, tRRD and tRFC of the
	// chip) or a PRECHARGE (tRAS, tWR) is allowed in this cycle
	reg [7:0] bk_can_act, bk_can_pre;
	always @(*) begin
		for(int i = 0; i < 8; i++) begin
			bk_can_act[i] = ~|(bk_pre[i] & R_RP) && ~|(bk_act[i] & R_RC) &&
			                ~|(ch_act[i >> 2] & R_RRD) && ~|(rfc_s[i >> 2] & R_RFC);
			bk_can_pre[i] = ~|(bk_act[i] & R_RAS) && ~|(bk_wr[i] & R_WR);
		end
	end
	// per chip, for the refresh: open banks (any, exactly one and which),
	// all the open ones can be precharged, no bank within tRP of a PRECHARGE
	// (needed by PRECHARGE ALL, which touches every bank, and by AUTO
	// REFRESH), and the AUTO REFRESH allowed (tRP of the banks, tRFC)
	reg [1:0] ch_any, ch_one, ch_canpre, ch_rpok, ch_canref;
	reg [1:0] ch_oneb [2];
	reg [2:0] ch_n;                       // temporary
	always @(*) begin
		for(int c = 0; c < 2; c++) begin
			ch_n = 3'd0; ch_oneb[c] = 2'd0; ch_canpre[c] = 1'b1; ch_rpok[c] = 1'b1;
			for(int b = 0; b < 4; b++) begin
				if(bk_open[c * 4 + b]) begin
					ch_n = ch_n + 3'd1; ch_oneb[c] = 2'(b);
					if(!bk_can_pre[c * 4 + b]) ch_canpre[c] = 1'b0;
				end
				if(|(bk_pre[c * 4 + b] & R_RP)) ch_rpok[c] = 1'b0;
			end
			ch_any[c]    = (ch_n != 3'd0);
			ch_one[c]    = (ch_n == 3'd1);
			ch_canref[c] = ch_rpok[c] && ~|(rfc_s[c] & R_RFC);
		end
	end

	//----------------------------------------------------- write buffer
	reg [25:0]    wb_addr [WB_DEPTH];
	(* ramstyle = "logic" *) reg [31:0] wb_data [WB_DEPTH];   // registers, not a memory block
	reg  [3:0]    wb_be   [WB_DEPTH];
	reg [WB_AW:0] wb_rp = '0, wb_wp = '0;          // one bit more than the index
	reg [AGEW-1:0] wb_age = '0;                   // cycles the head has waited, up to AGE_MAX
	wire [WB_AW:0] wb_cnt   = wb_wp - wb_rp;
	wire           wb_empty = (wb_cnt == '0);
	wire           wb_full  = (wb_cnt == WBW'(WB_DEPTH));
	wire [WB_AW-1:0] wb_h   = wb_rp[WB_AW-1:0];
	wire           wb_aged  = (wb_age >= AGEW'(AGE_MAX));

	//------------------------------------------------ read slots, per port
	reg [N_PORTS-1:0]  rd_v;
	reg [25:0]         rd_a   [N_PORTS];
	reg [LEN_BITS-1:0] rd_n   [N_PORTS];
	reg [WB_AW:0]      rd_old [N_PORTS];    // writes taken before it still in the buffer, when
	                                        // one of them touched its block; else 0
	reg [AGEW-1:0]     rd_age [N_PORTS];    // cycles in the slot, up to AGE_MAX

	//----------------------------------------------------- current operation
	localparam [1:0] OP_RD = 2'd0, OP_WR = 2'd1, OP_REF = 2'd2;
	reg                op_v = 1'b0;
	reg  [1:0]         op_k;
	reg  [PW-1:0]      op_port;
	reg [15:0]         op_lin;         // the block of the access (address above the column)
	reg [CB-1:0]       op_col;
	reg [LEN_BITS-1:0] op_left;        // column commands still to issue
	reg [31:0]         op_wdata;
	reg  [3:0]         op_wbe;
	reg                op_hi;          // write: the next beat is the high halfword
	reg                ref_chip;       // chip of a refresh operation

	wire [15:0] op_m    = map_chunk(op_lin);
	wire  [1:0] op_bank = op_m[15:14];
	wire        op_chip = op_m[13];
	wire [12:0] op_row  = op_m[12:0];
	wire  [2:0] op_bi   = bank_ix(op_m);
	wire        op_rdwr = op_v && (op_k != OP_REF);
	wire        op_ref  = op_v && (op_k == OP_REF);
	wire        op_row_ok = bk_open[op_bi] && (bk_row[op_bi] == op_row);
	// the operation will issue its ACTIVE in the next cycle unless an ACTIVE
	// on its chip in this cycle blocks it (tRRD of two cycles)
	wire        op_act_next = op_rdwr && !bk_open[op_bi] &&
	                          ~|(bk_pre[op_bi] & R_RPn) && ~|(bk_act[op_bi] & R_RCn) &&
	                          ~|(ch_act[op_chip] & R_RRDn) && ~|(rfc_s[op_chip] & R_RFCn);

	//--------------------------------------------------- write intake
	// one write per cycle into the buffer: among the ports that ask to write
	// and have no read in their slot, the first one from wrr on (in turn).
	// wrot is the candidates rotated so that bit k is port (wrr + k) mod N.
	reg  [PW-1:0]        wrr = '0;
	wire [N_PORTS-1:0]   wcand = req & we & ~rd_v & WR_PORTS;
	wire [2*N_PORTS-1:0] wdbl  = {wcand, wcand};
	wire [N_PORTS-1:0]   wrot  = wdbl[WDW'(wrr) +: N_PORTS];
	reg          wsel_v;
	reg [PW-1:0] wsel;
	integer      wi;                   // port looked at (temporary)
	always @(*) begin
		wsel_v = 1'b0; wi = 0;
		for(int k = N_PORTS - 1; k >= 0; k--)
			if(wrot[k]) begin wsel_v = 1'b1; wi = k; end
		wi = wi + {{(32-PW){1'b0}}, wrr};
		if(wi >= N_PORTS) wi = wi - N_PORTS;
		wsel = PW'(wi);
	end
	// the write of this cycle: req[wsel] && port_ready[wsel] && we[wsel]
	wire wtake = (st == S_RUN) && !wb_full && wsel_v;

	genvar gp;
	generate
		for(gp = 0; gp < N_PORTS; gp = gp + 1) begin : g_ready
			assign port_ready[gp] = (st == S_RUN) &&
			                        (we[gp] ? (WR_PORTS[gp] && !wb_full && wsel_v && wsel == gp)
			                                : (RD_PORTS[gp] && !rd_v[gp]));
		end
	endgenerate

	// a read being taken touches a write in the buffer: one of its blocks is
	// that of a write still there. Its blocks are the first one, the next one
	// when column + length runs past the last column, and (reads longer than
	// a block) the one after when it runs past two, so each entry is compared
	// as it is, minus one and minus two, on the block bits the map uses.
	reg [N_PORTS-1:0]  rcf_c;
	reg [WB_DEPTH-1:0] wb_val;          // entry j holds a write (temporary)
	reg [15:0]         wb_km1 [WB_DEPTH];  // its block minus one (temporary)
	reg [15:0]         wb_km2 [WB_DEPTH];  // its block minus two (temporary; reads longer than a block)
	reg   [WB_AW-1:0]  rc_off;          // place of entry j in the buffer (temporary)
	reg     [RSW-1:0]  rc_sum;          // column of the read plus its length (temporary)
	reg                rc_x;            // the read runs past the last column of its first block (temporary)
	reg                rc_x2;           // and past that of the next block (temporary)
	reg        [15:0]  rc_blk;          // first block of the read, bits the map uses (temporary)
	always @(*) begin
		for(int j = 0; j < WB_DEPTH; j++) begin
			rc_off    = WB_AW'(j) - wb_h;
			wb_val[j] = (WBW'(rc_off) < wb_cnt);
			wb_km1[j] = wb_addr[j][CB +: 16] - 16'd1;
			wb_km2[j] = wb_addr[j][CB +: 16] - 16'd2;
		end
		for(int g = 0; g < N_PORTS; g++) begin
			rc_sum = RSW'({1'b0, addr[g*26 +: CB]}) + RSW'(len[g*LEN_BITS +: LEN_BITS]);
			if(LEN_BITS > CB) begin
				rc_x  = (rc_sum > RSW'(1 << CB));
				rc_x2 = (rc_sum > RSW'(2 << CB));
			end
			else begin
				rc_x  = rc_sum[CB] && (rc_sum[CB-1:0] != '0);
				rc_x2 = 1'b0;
			end
			rc_blk   = addr[g*26+CB +: 16] & LIN_USED;
			rcf_c[g] = 1'b0;
			for(int j = 0; j < WB_DEPTH; j++)
				if(wb_val[j] && ((wb_addr[j][CB +: 16] & LIN_USED) == rc_blk ||
				                 (rc_x  && (wb_km1[j] & LIN_USED) == rc_blk) ||
				                 (rc_x2 && (wb_km2[j] & LIN_USED) == rc_blk)))
					rcf_c[g] = 1'b1;
		end
	end

	//--------------------------------------------------- next operation
	localparam [1:0] PK_NONE = 2'd0, PK_REF = 2'd1, PK_RD = 2'd2, PK_WR = 2'd3;
	reg  [1:0]    pk;
	reg [PW-1:0]  pk_port;
	reg           pk_chip;
	reg [N_PORTS-1:0] rok, rhit, rage;   // temporaries of the choice below
	reg        [15:0] rpm [N_PORTS];     // {bank, chip, row} of the read of each port
	reg        [15:0] wpm;               // the same for the head of the write buffer
	reg               whit, done;
	reg        [15:0] pk_m;              // {bank, chip, row} of the candidate chosen
	always @(*) begin
		for(int g = 0; g < N_PORTS; g++) begin
			rpm[g]  = map_chunk(rd_a[g][CB +: 16]);
			rok[g]  = RD_PORTS[g] && rd_v[g] && (rd_old[g] == '0);
			rhit[g] = !URG_PORTS[g] && bk_open[bank_ix(rpm[g])] && bk_row[bank_ix(rpm[g])] == rpm[g][12:0];
			rage[g] = !URG_PORTS[g] && (rd_age[g] >= AGEW'(AGE_MAX));
		end
		wpm   = map_chunk(wb_addr[wb_h][CB +: 16]);
		whit  = bk_open[bank_ix(wpm)] && bk_row[bank_ix(wpm)] == wpm[12:0];

		pk = PK_NONE; pk_port = '0; pk_chip = 1'b0; done = 1'b0;
		if(debt[0] >= 4'(REF_MAX))      begin pk = PK_REF; pk_chip = 1'b0; done = 1'b1; end
		else if(debt[1] >= 4'(REF_MAX)) begin pk = PK_REF; pk_chip = 1'b1; done = 1'b1; end
		for(int g = 0; g < N_PORTS; g++)
			if(!done && rok[g] && urgent[g]) begin pk = PK_RD; pk_port = PW'(g); done = 1'b1; end
		for(int g = 0; g < N_PORTS; g++)
			if(!done && rok[g] && rage[g]) begin pk = PK_RD; pk_port = PW'(g); done = 1'b1; end
		if(!done && !wb_empty && wb_aged) begin pk = PK_WR; done = 1'b1; end
		if(!done && wb_full) begin pk = PK_WR; done = 1'b1; end
		for(int g = 0; g < N_PORTS; g++)
			if(!done && rok[g] && rhit_q[g]) begin pk = PK_RD; pk_port = PW'(g); done = 1'b1; end
		if(!done && !wb_empty && whit_q) begin pk = PK_WR; done = 1'b1; end
		for(int g = 0; g < N_PORTS; g++)
			if(!done && rok[g]) begin pk = PK_RD; pk_port = PW'(g); done = 1'b1; end
		if(!done && !wb_empty) begin pk = PK_WR; done = 1'b1; end
		if(!done && debt[0] != 4'd0) begin pk = PK_REF; pk_chip = 1'b0; done = 1'b1; end
		if(!done && debt[1] != 4'd0) begin pk = PK_REF; pk_chip = 1'b1; done = 1'b1; end

		pk_m = (pk == PK_WR) ? wpm : rpm[pk_port];
	end

	// the choice of the cycle before, registered: the preparation of the
	// next operation decides its command from these registers and from the
	// state of the banks now, so that no path runs from the choice to the
	// pins within one cycle. A stale choice costs nothing: what the bank
	// needs and what the histories allow are checked again in the cycle the
	// command goes out, and a candidate loaded meanwhile is the current
	// operation, whose bank the preparation never touches. The command is
	// what its bank needs (ACTIVE when closed, PRECHARGE when another row is
	// open), allowed now, never on the bank of the current operation nor on
	// the chip of a refresh in progress, and never an ACTIVE that would delay
	// the ACTIVE of the current operation (tRRD)
	reg  [1:0]  pq = PK_NONE;
	reg         pq_chip;
	reg [15:0]  pq_m;
	wire  [2:0] pq_bi  = bank_ix(pq_m);
	wire        pq_acc = (pq == PK_RD || pq == PK_WR) &&
	                     !(op_rdwr && pq_bi == op_bi) && !(op_ref && pq_m[13] == ref_chip);
	wire        q_act  = pq_acc && !bk_open[pq_bi] && bk_can_act[pq_bi] &&
	                     (RRD1 || !op_rdwr || pq_m[13] != op_chip || (RRD2 ? !op_act_next : op_row_ok));
	wire        q_pre  = pq_acc && bk_open[pq_bi] && bk_row[pq_bi] != pq_m[12:0] && bk_can_pre[pq_bi];

	// read pipeline: bit k is a READ loaded k+1 rising edges ago, with its
	// port; rd_q / rdp_q are the READ decided in the cycle before, on the
	// pins in this one (the command stage, CMD_STAGE = 1)
	reg [RQD-1:0] rdq_v = '0;
	reg [PW-1:0]  rdq_p [RQD];
	reg           rd_now;
	reg           rd_q = 1'b0;
	reg [PW-1:0]  rdp_q;
	// row hits of the cycle before, for the order of the choice only: a
	// candidate whose row was open one cycle ago goes before the others.
	// Whether a row IS open is always read from the banks in the cycle a
	// command is decided (the current operation, the preparation).
	reg [N_PORTS-1:0] rhit_q = '0;
	reg               whit_q = 1'b0;
	reg           op_done;
	reg           wb_pop;
	reg  [1:0]    ld_ref;              // refresh given to the engine in this cycle, per chip
	reg  [7:0]    act_ev, pre_ev, wr_ev;   // commands issued in this cycle, per bank (temporaries)
	reg  [1:0]    ref_ev;              // AUTO REFRESH issued in this cycle, per chip (temporary)
	reg           busy;                // the current operation issued a command in this cycle (temporary)
	reg           prep_v;              // refresh preparation: a bank to close (temporaries)
	reg  [1:0]    prep_b;
	reg           w_lo, w_hi;          // write: the halves with bytes to write (temporaries)

	always @(posedge clk) begin
		if(!rst_n) begin
			st <= S_WAIT; wait_c <= WAITW'(C_INIT); chip <= 1'b0; nref <= '0;
			ready <= 1'b0;
			issue(1'b0, CMD_NOP, 2'd0, 13'h1800);
			q_oe <= 1'b0;
			refi_c <= '0; debt[0] <= 4'd0; debt[1] <= 4'd0;
			rfc_s[0] <= '0; rfc_s[1] <= '0;
			for(int i = 0; i < 8; i++) begin
				bk_open[i] <= 1'b0; bk_act[i] <= '0; bk_pre[i] <= '0; bk_wr[i] <= '0;
			end
			ch_act[0] <= '0; ch_act[1] <= '0; since_rd <= '0;
			wb_rp <= '0; wb_wp <= '0; wb_age <= '0; wrr <= '0;
			rd_v <= '0;
			for(int g = 0; g < N_PORTS; g++) begin
				rd_old[g] <= '0; rd_age[g] <= '0;
			end
			op_v <= 1'b0; pq <= PK_NONE; rhit_q <= '0; whit_q <= 1'b0;
			rdq_v <= '0; rd_q <= 1'b0; rvalid <= '0;
		end
		else begin
			// default: NOP; DQM high during the power-up, low afterwards
			issue(q_ncs, CMD_NOP, 2'd0, (st == S_RUN) ? 13'h0000 : 13'h1800);
			pq <= pk; pq_chip <= pk_chip; pq_m <= pk_m;
			rhit_q <= rhit & rd_v; whit_q <= whit && !wb_empty;
			q_oe    <= 1'b0;
			rd_now  = 1'b0;
			op_done = 1'b0;
			wb_pop  = 1'b0;
			ld_ref  = 2'b00;
			act_ev  = 8'd0; pre_ev = 8'd0; wr_ev = 8'd0; ref_ev = 2'b00;

			if(st == S_RUN) refi_c <= refi_tick ? '0 : refi_c + 1'b1;

			if(wait_c != '0) wait_c <= wait_c - 1'b1;
			else case(st)
				S_WAIT: begin	// 200 us of NOP done: PRECHARGE ALL on U1, then U2 (one chip: U1 only)
					issue(chip, CMD_PRE, 2'd0, 13'h1C00);   // A10 = all banks (DQM high)
					if(chip || N_CHIPS == 1) begin chip <= 1'b0; st <= S_IREF; wait_c <= WAITW'(C_RP - 1); end
					else chip <= 1'b1;
				end
				S_IREF: begin	// INIT_REFS AUTO REFRESH on each chip, tRFC apart per chip
					issue(chip, CMD_REF, 2'd0, 13'h1800);
					if(chip || N_CHIPS == 1) begin
						chip <= 1'b0; nref <= nref + 1'b1;
						wait_c <= WAITW'(C_RFC - N_CHIPS);   // two chips: the other chip used one cycle
						if(nref + 1'b1 == NRW'(INIT_REFS)) st <= S_MRS;
					end
					else chip <= 1'b1;
				end
				S_MRS: begin	// MODE REGISTER SET on U1, then U2 (one chip: U1 only)
					issue(chip, CMD_MRS, 2'd0, MODE);
					if(chip || N_CHIPS == 1) begin chip <= 1'b0; st <= S_RUN; wait_c <= WAITW'(C_MRD - 1); end
					else chip <= 1'b1;
				end
				S_RUN: begin
					ready <= 1'b1;
					busy = 1'b0;
					if(op_v) begin
						case(op_k)
						OP_REF: begin
							// all banks of the chip closed, then AUTO REFRESH: one open
							// bank is closed alone, more with PRECHARGE ALL, which touches
							// also the banks just closed by a preparation (tRP of all)
							if(ch_any[ref_chip]) begin
								if(ch_canpre[ref_chip] && (ch_one[ref_chip] || ch_rpok[ref_chip])) begin
									if(ch_one[ref_chip]) issue(ref_chip, CMD_PRE, ch_oneb[ref_chip], 13'h0000);
									else                 issue(ref_chip, CMD_PRE, 2'd0, 13'h0400);   // A10 = all banks of this chip
									// PRECHARGE ALL precharges every bank of the chip, the
									// idle ones too (datasheet p.9): tRP for all four
									for(int b = 0; b < 4; b++) begin
										bk_open[{ref_chip, 2'(b)}] <= 1'b0;
										if(bk_open[{ref_chip, 2'(b)}] || !ch_one[ref_chip]) pre_ev[{ref_chip, 2'(b)}] = 1'b1;
									end
									busy = 1'b1;
								end
							end
							else if(ch_canref[ref_chip]) begin
								issue(ref_chip, CMD_REF, 2'd0, 13'h0000);
								ref_ev[ref_chip] = 1'b1;
								op_done = 1'b1;
								busy = 1'b1;
							end
						end
						default: begin	// OP_RD / OP_WR
							if(!bk_open[op_bi]) begin
								// ACTIVE: tRP, tRC, tRRD of the chip, tRFC of the chip
								if(bk_can_act[op_bi]) begin
									issue(op_chip, CMD_ACT, op_bank, op_row);
									bk_open[op_bi] <= 1'b1; bk_row[op_bi] <= op_row;
									act_ev[op_bi] = 1'b1;
									busy = 1'b1;
								end
							end
							else if(bk_row[op_bi] != op_row) begin
								// another row: PRECHARGE this bank (tRAS, tWR)
								if(bk_can_pre[op_bi]) begin
									issue(op_chip, CMD_PRE, op_bank, 13'h0000);
									bk_open[op_bi] <= 1'b0;
									pre_ev[op_bi] = 1'b1;
									busy = 1'b1;
								end
							end
							else if(~|(bk_act[op_bi] & R_RCD) &&
							        (op_k != OP_WR || ~|(since_rd & R_RD))) begin
								// column command (tRCD; a WRITE waits for the read data to leave the pins)
								if(op_k == OP_WR) begin
									// A12 = DQMH, A11 = DQML, A10 = 0 (no auto precharge)
									issue(op_chip, CMD_WR, op_bank,
									      {~(op_hi ? op_wbe[3] : op_wbe[1]), ~(op_hi ? op_wbe[2] : op_wbe[0]), 1'b0, 10'(op_col)});
									q_oe   <= 1'b1;
									q_dout <= op_hi ? op_wdata[31:16] : op_wdata[15:0];
									op_hi  <= 1'b1;
									wr_ev[op_bi] = 1'b1;
								end
								else begin
									issue(op_chip, CMD_RD, op_bank, {3'b000, 10'(op_col)});
									rd_now = 1'b1;
								end
								// past the last column: the next block, mapped again
								op_col <= op_col + 1'b1;
								if(&op_col) op_lin <= op_lin + 16'd1;
								op_left <= op_left - LEN_BITS'(1);
								if(op_left == LEN_BITS'(1)) op_done = 1'b1;
								busy = 1'b1;
							end
						end
						endcase
					end

					// the free command bus prepares the next operation (SPEC 4.4): the
					// candidate chosen in the cycle before (pq) gets its row opened,
					// after the close of another row of its bank, while the current
					// operation waits or transfers; with a refresh chosen, the banks
					// of its chip are closed, one at a time when the operation runs
					// on that chip, all at once otherwise. Every check is on the state
					// of this cycle.
					if(!busy) begin
						if(q_act) begin
							issue(pq_m[13], CMD_ACT, pq_m[15:14], pq_m[12:0]);
							bk_open[pq_bi] <= 1'b1; bk_row[pq_bi] <= pq_m[12:0];
							act_ev[pq_bi] = 1'b1;
						end
						else if(q_pre) begin
							issue(pq_m[13], CMD_PRE, pq_m[15:14], 13'h0000);
							bk_open[pq_bi] <= 1'b0;
							pre_ev[pq_bi] = 1'b1;
						end
						else if(pq == PK_REF) begin
							if(op_rdwr && op_chip == pq_chip) begin
								// the lowest open bank that is not the one of the operation
								prep_v = 1'b0; prep_b = 2'd0;
								for(int b = 3; b >= 0; b--)
									if(bk_open[{pq_chip, 2'(b)}] && {pq_chip, 2'(b)} != op_bi &&
									   bk_can_pre[{pq_chip, 2'(b)}]) begin
										prep_v = 1'b1; prep_b = 2'(b);
									end
								if(prep_v) begin
									issue(pq_chip, CMD_PRE, prep_b, 13'h0000);
									bk_open[{pq_chip, prep_b}] <= 1'b0;
									pre_ev[{pq_chip, prep_b}] = 1'b1;
								end
							end
							else if(!(op_ref && ref_chip == pq_chip) && ch_any[pq_chip] && ch_canpre[pq_chip] &&
							        (ch_one[pq_chip] || ch_rpok[pq_chip])) begin
								if(ch_one[pq_chip]) issue(pq_chip, CMD_PRE, ch_oneb[pq_chip], 13'h0000);
								else                issue(pq_chip, CMD_PRE, 2'd0, 13'h0400);
								// PRECHARGE ALL: tRP for all four banks (see above)
								for(int b = 0; b < 4; b++) begin
									bk_open[{pq_chip, 2'(b)}] <= 1'b0;
									if(bk_open[{pq_chip, 2'(b)}] || !ch_one[pq_chip]) pre_ev[{pq_chip, 2'(b)}] = 1'b1;
								end
							end
						end
					end

					// next operation: taken when there is none, or in the cycle the
					// current one issues its last command (no idle cycle between)
					if(!op_v || op_done) begin
						op_v <= 1'b0;
						case(pk)
						PK_REF: begin
							op_v <= 1'b1; op_k <= OP_REF; ref_chip <= pk_chip;
							ld_ref[pk_chip] = 1'b1;
						end
						PK_RD: begin
							op_v <= 1'b1; op_k <= OP_RD; op_port <= pk_port;
							op_lin <= rd_a[pk_port][CB +: 16]; op_col <= rd_a[pk_port][CB-1:0];
							op_left <= rd_n[pk_port];
							rd_v[pk_port] <= 1'b0;
						end
						PK_WR: begin
							w_lo = (wb_be[wb_h][1:0] != 2'b00);
							w_hi = (wb_be[wb_h][3:2] != 2'b00);
							op_v <= 1'b1; op_k <= OP_WR;
							op_lin <= wb_addr[wb_h][CB +: 16]; op_col <= {wb_addr[wb_h][CB-1:1], !w_lo};
							op_left <= LEN_BITS'(w_lo) + LEN_BITS'(w_hi);
							op_wdata <= wb_data[wb_h]; op_wbe <= wb_be[wb_h];
							op_hi <= !w_lo;
							wb_rp <= wb_rp + 1'b1;
							wb_pop = 1'b1;
						end
						default: ;
						endcase
					end
				end
				default: st <= S_WAIT;
			endcase

			// histories of the commands issued in this cycle
			for(int i = 0; i < 8; i++) begin
				bk_act[i] <= (bk_act[i] << 1) | ACTW'(act_ev[i]);
				bk_pre[i] <= (bk_pre[i] << 1) | PREW'(pre_ev[i]);
				bk_wr[i]  <= (bk_wr[i] << 1)  | WRW'(wr_ev[i]);
			end
			for(int c = 0; c < 2; c++) begin
				ch_act[c] <= (ch_act[c] << 1) | CHW'(|act_ev[c*4 +: 4]);
				rfc_s[c]  <= (rfc_s[c] << 1)  | RFCW'(ref_ev[c]);
				// one chip: no debt for U2, never refreshed
				debt[c]   <= (c < N_CHIPS) ? debt[c] + (refi_tick ? 4'd1 : 4'd0) - (ld_ref[c] ? 4'd1 : 4'd0) : 4'd0;
			end
			since_rd <= (since_rd << 1) | RDW'(rd_now);

			// ports: the write of one port into the buffer, reads into their slots
			if(wtake) begin
				if(wbe[wsel*4 +: 4] != 4'd0) begin
					wb_addr[wb_wp[WB_AW-1:0]] <= addr[wsel*26 +: 26];
					wb_data[wb_wp[WB_AW-1:0]] <= wdata[wsel*32 +: 32];
					wb_be[wb_wp[WB_AW-1:0]]   <= wbe[wsel*4 +: 4];
					wb_wp <= wb_wp + 1'b1;
				end
				wrr <= (wsel == PW'(N_PORTS - 1)) ? '0 : wsel + 1'b1;
			end
			for(int g = 0; g < N_PORTS; g++) begin
				if(req[g] && port_ready[g] && !we[g]) begin
					if(len[g*LEN_BITS +: LEN_BITS] != '0) begin
						rd_v[g]   <= 1'b1;
						rd_a[g]   <= addr[g*26 +: 26];
						rd_n[g]   <= len[g*LEN_BITS +: LEN_BITS];
						rd_old[g] <= rcf_c[g] ? (wb_cnt - WBW'(wb_pop)) : '0;
						rd_age[g] <= '0;
					end
				end
				else if(rd_v[g]) begin
					if(wb_pop && rd_old[g] != '0) rd_old[g] <= rd_old[g] - 1'b1;
					if(!rage[g] && !URG_PORTS[g]) rd_age[g] <= rd_age[g] + 1'b1;
				end
			end
			wb_age <= (wb_empty || wb_pop) ? '0 : (wb_aged ? wb_age : wb_age + 1'b1);

			// read data: taken CAP_H half periods after the READ was loaded
			// into the pin registers: one edge after its decision with the
			// command stage, at the edge that ends it without
			rd_q     <= rd_now;
			rdp_q    <= op_port;
			rdq_v    <= {rdq_v[RQD-2:0], (CMD_STAGE != 0) ? rd_q : rd_now};
			rdq_p[0] <= (CMD_STAGE != 0) ? rdp_q : op_port;
			for(int k = 1; k < RQD; k++) rdq_p[k] <= rdq_p[k - 1];
			for(int g = 0; g < N_PORTS; g++)
				rvalid[g] <= RD_PORTS[g] && rdq_v[CAP_M - 1] && rdq_p[CAP_M - 1] == PW'(g);
			if(rdq_v[CAP_M - 1]) rdata <= (CAP_FALL != 0) ? dq_fall : sdram_dq;
		end
	end
endmodule
