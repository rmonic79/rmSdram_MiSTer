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

`timescale 1ns/1ps
//============================================================================
// Bench of sdrc_job (rtl/sdrc_job.sv) on sdrc (rtl/sdrc.sv) and the model of
// a MiSTer SDRAM board (sdr_board.sv), with the pin delays of the compiled
// netlist (the same glue as tb_sdrc.sv). Two job ports:
//   J0 on sdrc port 0 reads and writes (FIXED1 = FIX);
//   J1 on sdrc port 1 only reads (HAS_WR = 0).
// The bench keeps a model of each job port (the running job and the one
// pending, as the header of rtl/sdrc_job.sv describes them) and a shadow of
// the memory, and checks at every falling edge:
//   - write jobs: every word the controller takes is the next word of the
//     job: halfword address adr + bnum, its wdat and bsel (on the even dword
//     address, in the half of that halfword); done in the cycle after the
//     last word is taken, and in no other cycle;
//   - read jobs: blen words back (2^BL_BITS for blen = 0, one with FIXED1),
//     rval with bnum 0, 1, 2 ... in order, rdat the shadow of adr + bnum;
//     done in the cycle after the last rval, and in no other cycle; rdat keeps
//     the last word until the next one;
//   - a pulse during a job starts the next job when the running one ends; a
//     newer pulse replaces the pending one; rd wins over wr in the same cycle;
//     busy is high exactly while a job runs;
//   - a read taken after the done of a write, from the other port, sees the
//     words written.
// Phases: fill (J0 writes regions A and B), random jobs on J0 over region A
// (direct, pending, replaced, rd+wr) with J1 reading region B at the same
// time (direct and pending), cross-port (J0 writes in A, J1 reads it back
// right after done).
//============================================================================
module tb_sdrc_job;
	parameter int CLK_KHZ  = 48000;
	parameter int CORNER   = 0;
	parameter int N_CHIPS  = 2;      // board (sdr_board.sv) and controller
	parameter int COL_BITS = 10;
	parameter int WB2      = 0;      // 1: the W9825G6KH: tRRD, tWR, tMRD of two clocks and eight power-up refreshes
	parameter int BLB      = 9;      // BL_BITS of the job ports: jobs of up to 2^BLB halfwords
	parameter int LNB      = 4;      // LEN_BITS of sdrc and of the job ports (longer reads go out in pieces)
	parameter int FIX      = 0;      // FIXED1 of J0
	parameter int N_JOBS   = (BLB >= 6) ? 600 : 3000;   // random jobs of J0 (fewer when they are long)
	parameter int N_CROSS  = 200;    // cross-port checks

	localparam int PER_PS    = 1000000000 / CLK_KHZ;    // the period as sdrc computes it
	localparam int T2_PS     = WB2 ? ((2 * PER_PS > 14000) ? 2 * PER_PS : 14000) : 14000;
	localparam int INIT_REFS = WB2 ? 8 : 2;
	localparam int MAXJ      = 1 << BLB;                 // longest job, halfwords
	localparam int RSZ       = 4096;                     // halfwords of each region
	localparam [23:0] RA     = 24'h010000;               // region A: J0 random jobs, cross-port
	localparam [23:0] RB     = 24'h024000;               // region B: J1 random reads
	localparam int NP        = 2;

	localparam real HALF_PERIOD = 500000.0 / CLK_KHZ;
	reg clk = 0;
	always #(HALF_PERIOD) clk = ~clk;
	reg rst_n = 0;

	//----------------------------------------------------------------- DUT
	wire        ncs, nras, ncas, nwe, ready;
	wire  [1:0] ba;
	wire [12:0] a;
	wire [15:0] dq;
	wire [NP-1:0]     p_req, p_we, p_ready, p_rvalid;
	wire [NP*26-1:0]  p_addr;
	wire [NP*LNB-1:0] p_len;
	wire [NP*32-1:0]  p_wdata;
	wire [NP*4-1:0]   p_wbe;
	wire [15:0]       p_rdata;

	sdrc #(.CLK_KHZ(CLK_KHZ), .N_PORTS(NP), .LEN_BITS(LNB), .MAP(1), .N_CHIPS(N_CHIPS), .COL_BITS(COL_BITS),
	       .T_RRD_PS(T2_PS), .T_WR_PS(T2_PS), .T_MRD_PS(T2_PS), .INIT_REFS(INIT_REFS), .WR_PORTS(2'b01)) dut (
		.clk(clk), .rst_n(rst_n), .ready(ready),
		.req(p_req), .we(p_we), .urgent(2'b00), .addr(p_addr), .len(p_len), .wdata(p_wdata), .wbe(p_wbe),
		.port_ready(p_ready), .rdata(p_rdata), .rvalid(p_rvalid),
		.sdram_ncs(ncs), .sdram_nras(nras), .sdram_ncas(ncas), .sdram_nwe(nwe),
		.sdram_ba(ba), .sdram_a(a), .sdram_dq(dq)
	);

	// the job ports
	reg  [23:0]    j_adr  [NP];
	reg            j_rd   [NP];
	reg            j_wr   [NP];
	reg  [BLB-1:0] j_blen [NP];
	wire [15:0]    j_rdat [NP];
	wire           j_rval [NP];
	wire [BLB-1:0] j_bnum [NP];
	wire           j_done [NP];
	wire           j_busy [NP];
	wire [1:0]     bsel0;
	wire [15:0]    wdat0;
	initial for(int p = 0; p < NP; p++) begin j_adr[p] = 0; j_rd[p] = 0; j_wr[p] = 0; j_blen[p] = 0; end

	sdrc_job #(.ADR_BITS(24), .BL_BITS(BLB), .LEN_BITS(LNB), .FIXED1(FIX)) j0 (
		.clk(clk), .rst_n(rst_n),
		.adr(j_adr[0]), .rd(j_rd[0]), .wr(j_wr[0]), .blen(j_blen[0]), .bsel(bsel0), .wdat(wdat0),
		.rdat(j_rdat[0]), .rval(j_rval[0]), .bnum(j_bnum[0]), .done(j_done[0]), .busy(j_busy[0]),
		.p_req(p_req[0]), .p_we(p_we[0]), .p_addr(p_addr[0 +: 26]), .p_len(p_len[0 +: LNB]),
		.p_wdata(p_wdata[0 +: 32]), .p_wbe(p_wbe[0 +: 4]), .p_ready(p_ready[0]), .p_rdata(p_rdata), .p_rvalid(p_rvalid[0])
	);
	sdrc_job #(.ADR_BITS(24), .BL_BITS(BLB), .LEN_BITS(LNB), .HAS_WR(0)) j1 (
		.clk(clk), .rst_n(rst_n),
		.adr(j_adr[1]), .rd(j_rd[1]), .wr(1'b0), .blen(j_blen[1]), .bsel(2'b00), .wdat(16'h0),
		.rdat(j_rdat[1]), .rval(j_rval[1]), .bnum(j_bnum[1]), .done(j_done[1]), .busy(j_busy[1]),
		.p_req(p_req[1]), .p_we(p_we[1]), .p_addr(p_addr[26 +: 26]), .p_len(p_len[LNB +: LNB]),
		.p_wdata(p_wdata[32 +: 32]), .p_wbe(p_wbe[4 +: 4]), .p_ready(p_ready[1]), .p_rdata(p_rdata), .p_rvalid(p_rvalid[1])
	);

	//---------------------------------------------------------- the pins
	// The delays of the compiled netlist (bench/pin_datasheet.tcl) on each pin,
	// board side moved back by X.
	localparam real T_A    = CORNER ? 3.088 : 9.050;
	localparam real T_BA   = CORNER ? 3.040 : 8.573;
	localparam real T_CS   = CORNER ? 3.041 : 8.297;
	localparam real T_RAS  = CORNER ? 3.268 : 8.852;
	localparam real T_CAS  = CORNER ? 3.150 : 8.616;
	localparam real T_WE   = CORNER ? 3.217 : 8.768;
	localparam real T_DQO  = CORNER ? 3.174 : 9.194;
	localparam real T_CLKO = CORNER ? 3.121 : 8.601;
	localparam real T_SU   = CORNER ? 0.248 : -0.584;
	localparam real T_H    = CORNER ? 1.716 : 3.737;
	localparam real X      = (T_H - T_SU) / 2.0;

	reg        p_ncs = 1'b1, p_nras = 1'b1, p_ncas = 1'b1, p_nwe = 1'b1, p_clk = 1'b0, p_oe = 1'b0;
	reg  [1:0] p_ba = 2'd0;
	reg [12:0] p_a = 13'h1800;
	reg [15:0] p_dqo = 16'd0;
	wire [15:0] p_dq;
	initial begin
		#0.001;
		p_ncs = ncs; p_nras = nras; p_ncas = ncas; p_nwe = nwe;
		p_ba = ba; p_a = a; p_oe = dut.dq_oe; p_dqo = dut.dq_out;
	end
	always @(ncs)         p_ncs  <= #(T_CS  - X) ncs;
	always @(nras)        p_nras <= #(T_RAS - X) nras;
	always @(ncas)        p_ncas <= #(T_CAS - X) ncas;
	always @(nwe)         p_nwe  <= #(T_WE  - X) nwe;
	always @(ba)          p_ba   <= #(T_BA  - X) ba;
	always @(a)           p_a    <= #(T_A   - X) a;
	always @(dut.dq_oe)   p_oe   <= #(T_DQO - X) dut.dq_oe;
	always @(dut.dq_out)  p_dqo  <= #(T_DQO - X) dut.dq_out;
	localparam real D_CLK = T_CLKO - X;
	generate
		if(D_CLK < HALF_PERIOD) begin : g_clk_inv
			always @(clk) p_clk <= #(D_CLK) ~clk;
		end
		else begin : g_clk_fwd
			always @(clk) p_clk <= #(D_CLK - HALF_PERIOD) clk;
		end
	endgenerate
	assign p_dq = p_oe ? p_dqo : 16'hzzzz;
	assign dq   = dut.dq_oe ? 16'hzzzz : p_dq;

	longint ncs_high = 0;
	always @(posedge clk) if(N_CHIPS == 1 && ncs !== 1'b0) ncs_high++;

	sdr_board #(.N_CHIPS(N_CHIPS), .COL_BITS(COL_BITS), .INIT_REFS_MIN(WB2 ? 8 : 2),
	            .T_RRD(T2_PS / 1000.0), .T_WR(T2_PS / 1000.0), .T_MRD(T2_PS / 1000.0)) board (
		.sdram_clk(p_clk), .ncs(p_ncs), .nras(p_nras), .ncas(p_ncas), .nwe(p_nwe),
		.ba(p_ba), .a(p_a), .dq(p_dq), .ctl_oe(p_oe)
	);

	//------------------------------------------------------- shadow memory
	bit [15:0] shA [0:RSZ-1];
	bit [15:0] shB [0:RSZ-1];
	int unsigned data_err = 0, proto_err = 0, bench_err = 0;
	function automatic bit in_a(input [23:0] ad); return ad >= RA && ad < RA + 24'(RSZ); endfunction
	function automatic bit in_b(input [23:0] ad); return ad >= RB && ad < RB + 24'(RSZ); endfunction
	function automatic [15:0] sh_get(input [23:0] ad);
		if(in_a(ad)) return shA[ad - RA];
		if(in_b(ad)) return shB[ad - RB];
		bench_err++; $display("BENCH address outside the regions %06x", ad); return 16'h0;
	endfunction
	function automatic void sh_put(input [23:0] ad, input [15:0] v, input [1:0] s);
		bit [15:0] o;
		o = sh_get(ad);
		if(s[1]) o[15:8] = v[15:8];
		if(s[0]) o[7:0]  = v[7:0];
		if(in_a(ad)) shA[ad - RA] = o;
		else if(in_b(ad)) shB[ad - RB] = o;
	endfunction

	//------------------------------------------------- model of the job ports
	// The job J0 presents (adr, blen on its inputs, data here); the data of
	// the write job that runs; per port: a job runs, writes, address, words,
	// words done (taken / back), a job pending and whether it writes.
	bit [15:0] pre_d [0:MAXJ-1];
	bit [1:0]  pre_s [0:MAXJ-1];
	bit [15:0] cur_d [0:MAXJ-1];
	bit [1:0]  cur_s [0:MAXJ-1];
	assign wdat0 = cur_d[j_bnum[0]];
	assign bsel0 = cur_s[j_bnum[0]];

	bit        m_act [NP], m_w [NP], m_pv [NP], m_pw [NP];
	bit [23:0] m_a   [NP];
	int        m_n   [NP], m_t [NP];
	bit        m_ed  [NP];          // done expected at the next falling edge
	bit        m_end [NP];          // the job ends at the next rising edge
	bit [15:0] m_last [NP];         // the last word given back (rdat holds it)
	longint    n_start [NP], n_from_pend [NP], n_wjob, n_rjob [NP], n_wword, n_rword [NP];
	initial for(int p = 0; p < NP; p++) begin
		m_act[p] = 0; m_w[p] = 0; m_pv[p] = 0; m_pw[p] = 0; m_a[p] = 0; m_n[p] = 0; m_t[p] = 0;
		m_ed[p] = 0; m_end[p] = 0; m_last[p] = 0; n_start[p] = 0; n_from_pend[p] = 0; n_rjob[p] = 0; n_rword[p] = 0;
	end
	initial begin n_wjob = 0; n_wword = 0; end

	// progress: the cycle of the last job started or ended or word moved, in
	// all (prog_*) and per port (pp_*), one variable per edge. With nothing for
	// STUCK cycles, or a port with a job running and nothing of its own for
	// STUCK cycles (the other port may go on), the run ends with an error
	// instead of waiting for ever (a defect that loses a job, a word or a done)
	localparam longint STUCK = 16 * MAXJ + 5000;
	longint cyc = 0, prog_pos = 0, prog_neg = 0;
	longint pp_pos [NP], pp_neg [NP];
	initial for(int p = 0; p < NP; p++) begin pp_pos[p] = 0; pp_neg[p] = 0; end
	always @(posedge clk) cyc <= cyc + 1;

	function automatic int len_of(input int p, input [BLB-1:0] bl);
		if(p == 0 && FIX != 0) return 1;
		return (bl == '0) ? MAXJ : int'(bl);
	endfunction

	// rising edge: what sdrc_job does with the pulses of this edge, from the
	// state before it; then the job that ended at this edge
	always @(posedge clk) if(rst_n) begin
		for(int p = 0; p < NP; p++) begin
			bit pulse, pw;
			pulse = j_rd[p] || (p == 0 && j_wr[p]);
			pw    = (p == 0) && !j_rd[p] && j_wr[p];
			if(!m_act[p]) begin
				if(pulse || m_pv[p]) begin
					if(!pulse) n_from_pend[p]++;
					m_act[p] = 1; m_w[p] = pulse ? pw : m_pw[p]; m_pv[p] = 0;
					m_a[p] = j_adr[p]; m_n[p] = len_of(p, j_blen[p]); m_t[p] = 0; n_start[p]++; prog_pos = cyc; pp_pos[p] = cyc;
					if(p == 0 && m_w[p]) for(int i = 0; i < MAXJ; i++) begin cur_d[i] = pre_d[i]; cur_s[i] = pre_s[i]; end
					if(m_w[p]) n_wjob++; else n_rjob[p]++;
				end
			end
			else if(pulse) begin
				m_pv[p] = 1; m_pw[p] = pw;
			end
			if(m_end[p]) begin m_act[p] = 0; m_end[p] = 0; prog_pos = cyc; pp_pos[p] = cyc; end
		end
	end

	always @(posedge clk) if(rst_n && ready) begin
		if(cyc - ((prog_pos > prog_neg) ? prog_pos : prog_neg) > STUCK) begin
			bench_err++;
			$display("  BENCH nothing moved for %0d cycles (time %0t)", STUCK, $realtime);
			report_end();
		end
		for(int p = 0; p < NP; p++)
			if(m_act[p] && cyc - ((pp_pos[p] > pp_neg[p]) ? pp_pos[p] : pp_neg[p]) > STUCK) begin
				bench_err++;
				$display("  BENCH J%0d: a job running and nothing of its own for %0d cycles (time %0t)", p, STUCK, $realtime);
				report_end();
			end
	end

	// falling edge: the outputs of the last rising edge and the request of the
	// next one are stable
	always @(negedge clk) if(rst_n) begin
		for(int p = 0; p < NP; p++) begin
			bit ed;
			if(j_done[p] !== m_ed[p]) begin
				proto_err++;
				if(proto_err <= 10) $display("  J%0d done=%b, expected %b (time %0t)", p, j_done[p], m_ed[p], $realtime);
			end
			// busy: the job of the model runs (a job ending at the next edge still runs)
			if(j_busy[p] !== m_act[p]) begin
				proto_err++;
				if(proto_err <= 10) $display("  J%0d busy=%b, model %b (time %0t)", p, j_busy[p], m_act[p], $realtime);
			end
			ed = 0;
			// a word the controller takes at the next edge
			if(p_req[p] && p_ready[p] && p_we[p]) begin
				if(p != 0 || !m_act[p] || !m_w[p] || m_end[p] || m_t[p] >= m_n[p]) begin
					proto_err++;
					if(proto_err <= 10) $display("  J%0d write taken outside a write job (time %0t)", p, $realtime);
				end
				else begin
					bit [23:0] ha; bit [25:0] ea; bit [3:0] eb;
					ha = m_a[p] + 24'(m_t[p]);
					ea = {2'b00, ha[23:1], 1'b0};
					eb = ha[0] ? {cur_s[m_t[p]], 2'b00} : {2'b00, cur_s[m_t[p]]};
					if(p_addr[p*26 +: 26] !== ea || p_wdata[p*32 +: 32] !== {cur_d[m_t[p]], cur_d[m_t[p]]} || p_wbe[p*4 +: 4] !== eb) begin
						proto_err++;
						if(proto_err <= 10) $display("  J0 word %0d of a write at %06x: addr %07x wdata %08x wbe %b, expected %07x %08x %b",
						                            m_t[p], m_a[p], p_addr[p*26 +: 26], p_wdata[p*32 +: 32], p_wbe[p*4 +: 4],
						                            ea, {cur_d[m_t[p]], cur_d[m_t[p]]}, eb);
					end
					sh_put(ha, cur_d[m_t[p]], cur_s[m_t[p]]);
					m_t[p]++; n_wword++; prog_neg = cyc; pp_neg[p] = cyc;
					if(m_t[p] == m_n[p]) begin ed = 1; m_end[p] = 1; end
				end
			end
			// a word given back
			if(j_rval[p]) begin
				if(!m_act[p] || m_w[p] || m_end[p] || m_t[p] >= m_n[p]) begin
					proto_err++;
					if(proto_err <= 10) $display("  J%0d rval outside a read job (time %0t)", p, $realtime);
				end
				else begin
					bit [15:0] ev;
					ev = sh_get(m_a[p] + 24'(m_t[p]));
					if(j_bnum[p] !== BLB'(m_t[p])) begin
						proto_err++;
						if(proto_err <= 10) $display("  J%0d bnum %0d, expected %0d", p, j_bnum[p], m_t[p]);
					end
					if(j_rdat[p] !== ev) begin
						data_err++;
						if(data_err <= 10) $display("  J%0d read at %06x word %0d: %04x, expected %04x (time %0t)",
						                           p, m_a[p], m_t[p], j_rdat[p], ev, $realtime);
					end
					m_last[p] = j_rdat[p];
					m_t[p]++; n_rword[p]++; prog_neg = cyc; pp_neg[p] = cyc;
					if(m_t[p] == m_n[p]) begin ed = 1; m_end[p] = 1; end
				end
			end
			else if(j_rdat[p] !== m_last[p]) begin
				proto_err++;
				if(proto_err <= 10) $display("  J%0d rdat changed without rval", p);
			end
			m_ed[p] = ed;
		end
	end

	//------------------------------------------------------------ drivers
	// Called 1 ns after a rising edge: present a job (address, length, data)
	// and pulse rd and/or wr for one edge; returns 1 ns after that edge.
	longint n_issued [NP], n_replaced, n_both;
	initial begin n_issued[0] = 0; n_issued[1] = 0; n_replaced = 0; n_both = 0; end
	function automatic [1:0] rnd_bsel();
		int r; r = $urandom % 20;
		return (r < 14) ? 2'b11 : (r < 17) ? 2'b01 : (r < 19) ? 2'b10 : 2'b00;
	endfunction
	task automatic pulse0(input bit rd, input bit wr, input [23:0] ad, input [BLB-1:0] bl, input bit full);
		j_adr[0] = ad; j_blen[0] = bl;
		for(int i = 0; i < MAXJ; i++) begin pre_d[i] = 16'($urandom); pre_s[i] = full ? 2'b11 : rnd_bsel(); end
		j_rd[0] = rd; j_wr[0] = wr;
		@(posedge clk); #1;
		j_rd[0] = 0; j_wr[0] = 0;
		n_issued[0]++;
		if(rd && wr) n_both++;
	endtask
	task automatic pulse1(input [23:0] ad, input [BLB-1:0] bl);
		j_adr[1] = ad; j_blen[1] = bl;
		j_rd[1] = 1;
		@(posedge clk); #1;
		j_rd[1] = 0;
		n_issued[1]++;
	endtask
	task automatic idle(input int n);
		repeat(n) @(posedge clk);
		#1;
	endtask
	task automatic wait_free(input int p);    // no job running nor pending
		while(m_act[p] || m_pv[p] || m_end[p]) idle(1);
	endtask
	function automatic [BLB-1:0] rnd_blen();
		return ($urandom % 16 == 0) ? '0 : BLB'(1 + $urandom % (MAXJ - 1));
	endfunction
	function automatic [23:0] rnd_adr(input [23:0] base);
		return base + 24'($urandom % (RSZ - MAXJ + 1));
	endfunction

	bit ph2 = 0, j1_done_loop = 0;

	// J1 during phase 2: random reads of region B, some pending
	initial begin
		wait(ph2);
		while(ph2) begin
			if(!m_act[1] && !m_pv[1] && !m_end[1]) begin
				pulse1(rnd_adr(RB), rnd_blen());
				if($urandom % 5 == 0) begin
					idle(1 + $urandom % 3);
					if(m_act[1] && !m_end[1] && m_n[1] - m_t[1] > 4) pulse1(rnd_adr(RB), rnd_blen());
				end
			end
			else idle(1);
		end
		wait_free(1);
		j1_done_loop = 1;
	end

	//---------------------------------------------------------- the run
	initial begin
		realtime t0;
		#100;
		@(negedge clk) rst_n = 1;
		wait(ready);
		t0 = $realtime;
		$display("READY at %0.1f us (CLK %0d kHz, corner %0d, chips %0d, column bits %0d, W9825G6KH timings %0d, BL_BITS %0d, LEN_BITS %0d, FIXED1 %0d)",
		         t0/1000.0, CLK_KHZ, CORNER, N_CHIPS, COL_BITS, WB2, BLB, LNB, FIX);
		prog_neg = cyc;                          // the reference of the progress check
		@(posedge clk); #1;

		// phase 1: fill regions A and B with whole jobs (blen = 0: 2^BL_BITS words)
		for(int r = 0; r < 2; r++)
			for(int o = 0; o < RSZ; o += ((FIX != 0) ? 1 : MAXJ)) begin
				wait_free(0);
				pulse0(1'b0, 1'b1, (r ? RB : RA) + 24'(o), '0, 1'b1);
			end
		wait_free(0);

		// phase 2: random jobs of J0 over region A, J1 reading region B
		ph2 = 1;
		for(int i = 0; i < N_JOBS; i++) begin
			int mode; bit w, both;
			mode = $urandom % 10;
			w    = $urandom % 2;
			both = ($urandom % 8 == 0);
			if(mode < 7) begin                       // a job when the port is free
				wait_free(0);
				pulse0(!w || both, w || both, rnd_adr(RA), rnd_blen(), 1'b0);
				idle($urandom % 4);
			end
			else begin                               // a job pending behind the running one
				// (with FIXED1 every job is one word: any running job will do; the
				// model follows whatever happens at the edge of the pulse)
				while(!(m_act[0] && !m_pv[0] && !m_end[0] && m_n[0] - m_t[0] > ((FIX != 0) ? 0 : 4))) begin
					if(!m_act[0] && !m_pv[0] && !m_end[0]) pulse0(1'b1, 1'b0, rnd_adr(RA), rnd_blen(), 1'b0);
					else idle(1);
				end
				pulse0(!w || both, w || both, rnd_adr(RA), rnd_blen(), 1'b0);
				if(mode == 9 && m_pv[0] && m_act[0] && !m_end[0]) begin   // replaced by a newer pulse
					pulse0(1'b0, 1'b1, rnd_adr(RA), rnd_blen(), 1'b0);
					n_replaced++;
				end
				while(m_pv[0]) idle(1);              // the pending job keeps its inputs until it starts
			end
		end
		wait_free(0);
		ph2 = 0;
		wait(j1_done_loop);

		// phase 3: J0 writes in region A, J1 reads the same words right after done
		for(int i = 0; i < N_CROSS; i++) begin
			bit [23:0] ad; bit [BLB-1:0] bl;
			ad = rnd_adr(RA); bl = rnd_blen();
			wait_free(0);
			pulse0(1'b0, 1'b1, ad, bl, 1'b0);
			while(!j_done[0]) idle(1);
			pulse1(ad, (FIX != 0) ? BLB'(1) : bl);
			wait_free(1);
		end
		wait_free(0); wait_free(1);
		idle(20);
		report_end();
	end

	// the end of the run (also when nothing moves any more)
	task automatic report_end();
		// every job issued started, but the replaced ones
		if(n_start[0] != n_issued[0] - n_replaced) begin
			proto_err++; $display("  J0 jobs started %0d, issued %0d, replaced %0d", n_start[0], n_issued[0], n_replaced);
		end
		if(n_start[1] != n_issued[1]) begin
			proto_err++; $display("  J1 jobs started %0d, issued %0d", n_start[1], n_issued[1]);
		end
		$display("JOBS      J0 write jobs=%0d read jobs=%0d (from pending %0d, replaced %0d, rd+wr %0d)  J1 read jobs=%0d (from pending %0d)",
		         n_wjob, n_rjob[0], n_from_pend[0], n_replaced, n_both, n_rjob[1], n_from_pend[1]);
		$display("WORDS     written=%0d  read J0=%0d J1=%0d", n_wword, n_rword[0], n_rword[1]);
		board.u1.report();
		if(N_CHIPS == 2) board.u2.report();
		$display("CHECK     data_errors=%0d  protocol_errors=%0d  chip_errors=%0d  bench_errors=%0d",
		         data_err, proto_err, board.u1.err + board.u2.err, bench_err + ncs_high);
		$finish;
	endtask
endmodule
