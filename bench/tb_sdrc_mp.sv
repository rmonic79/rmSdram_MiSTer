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
// Multi-port bench of sdrc (rtl/sdrc.sv) on the model of a MiSTer SDRAM
// board (sdr_board.sv: the 128 MB board, or one chip with COL_BITS), with
// the pin delays of the compiled netlist (the same glue as tb_sdrc.sv).
//
// Four ports, four traffic generators (SPEC 6), each region with one writer:
//   P0 CPU    best effort: single dword writes with random byte enables,
//             cache line reads (8 halfwords) and reads of 2 or 4 halfwords,
//             blocking (the next request only after the data); frame blits
//             (the frame written from its end back to its start) and posts
//             to the mailbox.
//   P1 video  real time: reads the front frame in bursts of VB halfwords into
//             a FIFO of FIFO_CAP halfwords emptied at one halfword every
//             VID_DIV cycles; urgent while the FIFO is under URG_LVL. The FIFO
//             must never run dry.
//   P2 DMA    best effort: bursts of 16 dword writes, then the same block
//             read back in pieces of random length (odd addresses too),
//             pipelined, and reads of 63 halfwords anywhere (across rows).
//   P3 audio  real time: every AUD_PERIOD cycles a read of 4 halfwords and a
//             write of one dword; urgent after AUD_URG cycles of waiting; and
//             the mailbox reads. Its reads must be back within AUD_PERIOD.
// Order checks (SPEC 4.2): P2 reads right after its own writes; P1 reads the
// frame P0 has just published (the last writes of the blit are the first
// halfwords of the frame, and may still be in the write buffer); P3 reads
// the mailbox right after P0 wrote it.
// Scoreboard: every region is preloaded through the cells of the chip models
// (with the map of sdrc, MAP) and shadowed; a write updates the shadow in the
// cycle it is taken, a read takes its expected halfwords in the cycle it is
// taken. Each region has one writer, and a port reads what another port
// writes only after the writer is done with it (frame and mailbox
// handshakes), so every expected value is exact.
//============================================================================
// Directed tests of the ordering (DIR = 1), before the traffic: D1 (one
// chip) a write to the address above the memory of A, the engine kept busy
// by three writes before it, then a read of A from another port, which must
// return the new data; D2 (LB > COL_BITS) a write in the third block of a
// read, then that read, which must return the new data.
module tb_sdrc_mp;
	parameter int CLK_KHZ    = 66667;
	parameter int CORNER     = 0;
	parameter int MAP        = 1;
	parameter int RUN_CYCLES = 300000;
	parameter int N_CHIPS    = 2;      // board (sdr_board.sv) and controller
	parameter int COL_BITS   = 10;
	parameter int CMD_STAGE  = 1;
	parameter int WB2        = 0;      // 1: the W9825G6KH: tRRD, tWR, tMRD of two clocks and eight power-up refreshes
	parameter int LB         = 6;      // reads up to 2^LB-1 halfwords
	parameter int DIR        = 0;      // 1: the directed tests D1 (one chip) and D2 (LB > COL_BITS) before the traffic

	localparam int PER_PS   = 1000000000 / CLK_KHZ;    // the period as sdrc computes it
	localparam int T_RRD_PS = WB2 ? ((2 * PER_PS > 14000) ? 2 * PER_PS : 14000) : 14000;
	localparam int T_WR_PS  = T_RRD_PS;                 // the -7 default of all three is 14 ns
	localparam int T_MRD_PS = T_RRD_PS;
	parameter int INIT_REFS = WB2 ? 8 : 2;              // AUTO REFRESH of the power-up (W9825G6KH: 8)

	localparam int NP = 4;

	// traffic
	localparam int VB         = 24;         // video burst, halfwords
	localparam int FIFO_CAP   = 128;
	localparam int URG_LVL    = 64;
	localparam int VID_DIV    = 3;          // one halfword out of the FIFO every VID_DIV cycles
	localparam int FRAME      = 1200;       // halfwords per frame
	localparam int AUD_PERIOD = 256;
	localparam int AUD_URG    = 16;

	// regions (halfword addresses): with MAP 0 each one sits in its own bank
	// one chip: the regions moved inside the memory, each still in its own bank
	// with MAP 0 (bank h[C+14:C+13]): h[25:24] goes to the bank bits, h[21:0] stays
	function automatic [25:0] reloc(input [25:0] h);
		if(N_CHIPS == 2)  return h;
		if(COL_BITS == 9) return {2'b00, h[25:24], h[21:0]};
		return {1'b0, h[25:24], 1'b0, h[21:0]};
	endfunction
	localparam [25:0] R0 = reloc(26'h0100000), R0_SZ = 26'h8000;     // CPU
	localparam [25:0] FB = reloc(26'h1200300);                       // two frames of FRAME
	localparam [25:0] R2 = reloc(26'h2A00156), R2_SZ = 26'h4000;     // DMA
	localparam [25:0] R3 = reloc(26'h3C00000), R3_SZ = 26'h0800;     // audio: reads below 0x400, writes above
	localparam [25:0] MB = reloc(26'h3C01000);                       // mailbox, one dword

	localparam real HALF_PERIOD = 500000.0 / CLK_KHZ;
	reg clk = 0;
	always #(HALF_PERIOD) clk = ~clk;
	reg rst_n = 0;

	//----------------------------------------------------------------- DUT
	wire        ncs, nras, ncas, nwe, ready;
	wire  [1:0] ba;
	wire [12:0] a;
	wire [15:0] dq;

	reg  [NP-1:0]         rq = '0, rwe = '0, rurg = '0;
	reg  [NP-1:0][25:0]   raddr = '0;
	reg  [NP-1:0][LB-1:0] rlen = '0;
	reg  [NP-1:0][31:0]   rwd = '0;
	reg  [NP-1:0][3:0]    rbe = '0;
	wire [NP-1:0]         port_ready, rvalid;
	wire [15:0]           rdata;

	sdrc #(.CLK_KHZ(CLK_KHZ), .N_PORTS(NP), .LEN_BITS(LB), .MAP(MAP), .N_CHIPS(N_CHIPS), .COL_BITS(COL_BITS),
	       .CMD_STAGE(CMD_STAGE), .T_RRD_PS(T_RRD_PS), .T_WR_PS(T_WR_PS), .T_MRD_PS(T_MRD_PS), .INIT_REFS(INIT_REFS)) dut (
		.clk(clk), .rst_n(rst_n), .ready(ready),
		.req(rq), .we(rwe), .urgent(rurg), .addr(raddr), .len(rlen), .wdata(rwd), .wbe(rbe),
		.port_ready(port_ready), .rdata(rdata), .rvalid(rvalid),
		.sdram_ncs(ncs), .sdram_nras(nras), .sdram_ncas(ncas), .sdram_nwe(nwe),
		.sdram_ba(ba), .sdram_a(a), .sdram_dq(dq)
	);

	//---------------------------------------------------------- the pins
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

	sdr_board #(.N_CHIPS(N_CHIPS), .COL_BITS(COL_BITS), .INIT_REFS_MIN(WB2 ? 8 : 2),
	               .T_RRD(T_RRD_PS / 1000.0), .T_WR(T_WR_PS / 1000.0), .T_MRD(T_MRD_PS / 1000.0)) board (
		.sdram_clk(p_clk), .ncs(p_ncs), .nras(p_nras), .ncas(p_ncas), .nwe(p_nwe),
		.ba(p_ba), .a(p_a), .dq(p_dq), .ctl_oe(p_oe)
	);

	// one chip: ncs never high (on the 128 MB board ncs high selects U2)
	longint ncs_high = 0;
	always @(posedge clk) if(N_CHIPS == 1 && ncs !== 1'b0) ncs_high++;

	//-------------------------------------------------------- scoreboard
	// the map of sdrc as its header documents it (MAP, N_CHIPS, COL_BITS), used
	// here only to preload the cells of the chips
	function automatic void bd_write(input [25:0] h, input [15:0] v);
		bit [1:0] b; bit [12:0] r; bit [9:0] c; bit ch; bit [24:0] ix;
		if(N_CHIPS == 2) begin
			if(MAP == 1) begin b = h[12:11]; ch = h[10]; r = h[25:13]; end
			else         begin b = h[25:24]; ch = h[23]; r = h[22:10]; end
			c = h[9:0];
		end
		else begin
			ch = 1'b0;
			c  = 10'(h & ((26'd1 << COL_BITS) - 26'd1));
			if(MAP == 1) begin b = 2'(h >> COL_BITS);        r = 13'(h >> (COL_BITS + 2)); end
			else         begin b = 2'(h >> (COL_BITS + 13)); r = 13'(h >> COL_BITS);       end
		end
		ix = {b, r, c};
		if(ch) board.u2.mem[ix] = v;
		else   board.u1.mem[ix] = v;
	endfunction
	function automatic [31:0] pat(input int unsigned x, input int unsigned k);
		pat = (x * 32'h9E3779B1) ^ (k * 32'h85EBCA77);
	endfunction

	bit [15:0] shadow [bit [25:0]];
	int unsigned errors = 0, bench_err = 0;

	task automatic preload(input [25:0] base, input int unsigned n);
		for(int unsigned i = 0; i < n; i++) begin
			bit [31:0] p; p = pat(base + i, 7);
			shadow[base + 26'(i)] = p[15:0];
			bd_write(base + 26'(i), p[15:0]);
		end
	endtask

	// one chip: an address above the memory is the cell it folds onto (the bits
	// above 2^(COL_BITS+15) halfwords are not used)
	function automatic [25:0] canon(input [25:0] h);
		if(N_CHIPS == 2) return h;
		return h & ((26'd1 << (COL_BITS + 15)) - 26'd1);
	endfunction
	function automatic void sb_wr(input [25:0] ad_in, input [31:0] d, input [3:0] be);
		bit [15:0] lo, hi; bit [25:0] ad;
		ad = canon(ad_in);
		if(!shadow.exists(ad) || !shadow.exists(ad + 1)) begin
			bench_err++; $display("BENCH write outside the regions a=%07x", ad); return;
		end
		lo = shadow[ad]; hi = shadow[ad + 1];
		if(be[0]) lo[7:0]  = d[7:0];
		if(be[1]) lo[15:8] = d[15:8];
		if(be[2]) hi[7:0]  = d[23:16];
		if(be[3]) hi[15:8] = d[31:24];
		shadow[ad] = lo; shadow[ad + 1] = hi;
	endfunction

	// one expected halfword: the cycle of its request, last of its read,
	// a tag (video: frame; audio: 3 = mailbox), address, value
	typedef struct packed {
		bit [31:0] t_req;
		bit        last;
		bit  [1:0] tag;
		bit [25:0] a;
		bit [15:0] v;
	} ex_t;
	ex_t exq [NP][$];

	longint cyc = 0;
	always @(posedge clk) cyc <= cyc + 1;

	// per port statistics
	longint n_wr [NP], n_rd [NP], wr_wait [NP], lat_sum [NP], lat_n [NP], p_out [NP];
	longint lat_max [NP];
	initial for(int p = 0; p < NP; p++) begin
		n_wr[p] = 0; n_rd[p] = 0; wr_wait[p] = 0; lat_sum[p] = 0; lat_n[p] = 0; lat_max[p] = 0; p_out[p] = 0;
	end

	// cycles a read stays in the read slot of its port (from taken to given
	// to the engine): what the aging of sdrc bounds
	longint slot_run [NP], slot_max [NP];
	initial for(int p = 0; p < NP; p++) begin slot_run[p] = 0; slot_max[p] = 0; end
	always @(posedge clk)
		for(int p = 0; p < NP; p++) begin
			if(dut.rd_v[p]) begin
				slot_run[p]++;
				if(slot_run[p] > slot_max[p]) slot_max[p] = slot_run[p];
			end
			else slot_run[p] = 0;
		end

	//------------------------------------------------------ port driver
	// Called 1 ns after a rising edge; puts the request up and returns 1 ns
	// after the edge that took it, the request still up (the caller puts up
	// the next one or goes idle). port_ready is sampled at the falling edge,
	// the value the next rising edge sees (requests change only after edges).
	task automatic issue(input int p, input bit w, input [25:0] ad, input int n,
	                     input [31:0] d, input [3:0] be, input [1:0] tag);
		bit acc; longint t0;
		t0 = cyc;
		rq[p] = 1'b1; rwe[p] = w; raddr[p] = ad; rlen[p] = LB'(n); rwd[p] = d; rbe[p] = be;
		do begin
			@(negedge clk);
			acc = port_ready[p];
			if(!acc && w) wr_wait[p]++;
			@(posedge clk);
		end while(!acc);
		// taken at this edge
		if(w) begin
			sb_wr(ad, d, be);
			n_wr[p]++;
		end
		else begin
			for(int i = 0; i < n; i++) begin
				ex_t e;
				if(!shadow.exists(canon(ad + 26'(i)))) begin bench_err++; $display("BENCH read outside the regions a=%07x", ad + 26'(i)); end
				e.t_req = 32'(t0); e.last = (i == n - 1); e.tag = tag; e.a = ad + 26'(i);
				e.v = shadow.exists(canon(ad + 26'(i))) ? shadow[canon(ad + 26'(i))] : 16'h0;
				exq[p].push_back(e);
			end
			p_out[p] += n;
			n_rd[p]++;
		end
		#1;
		rq[p] = 1'b0;
	endtask

	task automatic idle(input int n);
		repeat(n) @(posedge clk);
		#1;
	endtask

	bit go = 0, stop = 0;
	bit [NP-1:0] gen_done = '0;

	//------------------------------------------------ shared handshakes
	int  front = -1, pub = -1;           // frame the video reads, last frame published
	int  vid_out [2] = '{0, 0};          // halfwords of each frame requested and not back
	int  fifo_lvl = 0, vid_infl = 0, fifo_min = FIFO_CAP;
	bit  vid_on = 0;
	int  vdiv = 0;
	longint underflow = 0, urg_cyc = 0, vid_switch = 0;
	bit  mb_post = 0, mb_busy = 0;
	bit  aud_pend = 0; longint aud_t = 0, aud_late = 0;
	longint n_mbox = 0, n_blit = 0;

	function automatic [25:0] fb_base(input int b);
		return FB + 26'(b * FRAME);
	endfunction
	function automatic bit fb_free(input int b);
		return (b != front) && (b != pub) && (vid_out[b] == 0);
	endfunction

	//----------------------------------------- data back, FIFO of the video
	// At the falling edge: the halfwords back (rvalid and rdata are registers
	// of the rising edge), then the FIFO of the video gives out its halfword.
	always @(negedge clk) begin
		int nv;
		nv = 0;
		for(int p = 0; p < NP; p++) if(rvalid[p]) begin
			nv++;
			if(exq[p].size() == 0) begin
				errors++; if(errors < 10) $display("  UNEXPECTED data on port %0d", p);
			end
			else begin
				ex_t e;
				e = exq[p].pop_front();
				if(rdata !== e.v) begin
					errors++;
					if(errors < 10) $display("  MISMATCH P%0d a=%07x got %04x want %04x (cycle %0d)", p, e.a, rdata, e.v, cyc);
				end
				p_out[p]--;
				if(e.last) begin
					longint l; l = cyc - longint'(e.t_req);
					if(!(p == 3 && e.tag == 2'd3)) begin
						lat_sum[p] += l; lat_n[p]++; if(l > lat_max[p]) lat_max[p] = l;
					end
					if(p == 3 && e.tag == 2'd3) mb_busy = 0;
					if(p == 3 && e.tag == 2'd0) begin
						aud_pend = 0;
						if(l >= AUD_PERIOD) aud_late++;
					end
				end
				if(p == 1) begin
					fifo_lvl++; vid_infl--; vid_out[e.tag]--;
				end
			end
		end
		if(nv > 1) begin errors++; $display("  TWO rvalid in one cycle"); end

		if(!vid_on && !stop && fifo_lvl >= FIFO_CAP / 2) vid_on = 1;
		if(stop) vid_on = 0;
		if(vid_on) begin
			vdiv++;
			if(vdiv == VID_DIV) begin
				vdiv = 0;
				if(fifo_lvl == 0) begin
					underflow++;
					if(underflow <= 5) $display("  UNDERFLOW video FIFO at cycle %0d", cyc);
				end
				else fifo_lvl--;
			end
			if(fifo_lvl < fifo_min) fifo_min = fifo_lvl;
		end
		rurg[1] = vid_on && (fifo_lvl < URG_LVL);
		rurg[3] = aud_pend && (cyc - aud_t >= AUD_URG);
		if(rurg[1]) urg_cyc++;
	end

	//--------------------------------------------------------- P0 CPU
	initial begin : p_cpu
		int unsigned r;
		wait(go); @(posedge clk); #1;
		while(!stop) begin
			r = $urandom % 100;
			if(r < 50) begin
				bit [25:0] ad; bit [3:0] be;
				ad = R0 + 26'(2 * ($urandom % (R0_SZ / 2)));
				be = ($urandom % 4 == 0) ? 4'($urandom % 15 + 1) : 4'hF;
				issue(0, 1'b1, ad, 0, pat(ad, cyc), be, 2'd0);
			end
			else if(r < 70) begin
				issue(0, 1'b0, R0 + 26'(8 * ($urandom % (R0_SZ / 8))), 8, 0, 0, 2'd0);
				while(p_out[0] != 0) idle(1);
			end
			else if(r < 85) begin
				issue(0, 1'b0, R0 + 26'(2 * ($urandom % (R0_SZ / 2 - 2))), ($urandom % 2) ? 4 : 2, 0, 0, 2'd0);
				while(p_out[0] != 0) idle(1);
			end
			else if(r < 95) begin
				int w; w = -1;
				if(fb_free(0)) w = 0; else if(fb_free(1)) w = 1;
				if(w >= 0) begin
					for(int i = FRAME / 2 - 1; i >= 0; i--)
						issue(0, 1'b1, fb_base(w) + 26'(2 * i), 0, pat(i, 1000 + n_blit), 4'hF, 2'd0);
					pub = w; n_blit++;
				end
			end
			else if(!mb_busy) begin
				issue(0, 1'b1, MB, 0, pat(n_mbox, 77), 4'hF, 2'd0);
				mb_busy = 1; mb_post = 1;
			end
			idle($urandom % 4);
		end
		gen_done[0] = 1;
	end

	//------------------------------------------------------- P1 video
	initial begin : p_video
		int pos; pos = 0;
		wait(go); @(posedge clk); #1;
		while(!stop) begin
			if(pos == 0 && pub >= 0 && pub != front) begin
				front = pub; vid_switch++;
			end
			if(front >= 0 && fifo_lvl + vid_infl + VB <= FIFO_CAP) begin
				int n; n = (FRAME - pos < VB) ? FRAME - pos : VB;
				vid_infl += n; vid_out[front] += n;
				issue(1, 1'b0, fb_base(front) + 26'(pos), n, 0, 0, 2'(front));
				pos += n; if(pos == FRAME) pos = 0;
			end
			else idle(1);
		end
		gen_done[1] = 1;
	end

	//--------------------------------------------------------- P2 DMA
	initial begin : p_dma
		wait(go); @(posedge clk); #1;
		while(!stop) begin
			bit [25:0] base; int off;
			base = R2 + 26'(2 * ($urandom % ((R2_SZ - 32) / 2)));
			for(int i = 0; i < 16; i++)
				issue(2, 1'b1, base + 26'(2 * i), 0, pat(base + i, cyc), ($urandom % 4 == 0) ? 4'($urandom % 15 + 1) : 4'hF, 2'd0);
			off = 0;
			while(off < 32) begin
				int n; n = 1 + $urandom % (32 - off);
				issue(2, 1'b0, base + 26'(off), n, 0, 0, 2'd0);
				off += n;
			end
			if($urandom % 4 == 0)
				issue(2, 1'b0, R2 + 26'($urandom % (R2_SZ - 63)), 63, 0, 0, 2'd0);
			idle($urandom % 21);
		end
		gen_done[2] = 1;
	end

	//------------------------------------------------------- P3 audio
	initial begin : p_audio
		longint next_t; int ard, awr;
		ard = 0; awr = 0;
		wait(go); @(posedge clk); #1;
		next_t = cyc + AUD_PERIOD;
		while(!stop) begin
			if(cyc >= next_t && !aud_pend) begin
				next_t += AUD_PERIOD;
				aud_pend = 1; aud_t = cyc;
				issue(3, 1'b0, R3 + 26'(ard), 4, 0, 0, 2'd0);
				ard = (ard + 4) % 26'h400;
				issue(3, 1'b1, R3 + 26'h400 + 26'(awr), 0, pat(awr, cyc), 4'hF, 2'd0);
				awr = (awr + 2) % 26'h400;
			end
			else if(mb_post) begin
				mb_post = 0;
				issue(3, 1'b0, MB, 2, 0, 0, 2'd3);
				n_mbox++;
			end
			else idle(1);
		end
		gen_done[3] = 1;
	end

	//---------------------------------------------------------- the run
	longint ticks = 0;
	always @(posedge clk) if(dut.refi_tick) ticks <= ticks + 1;

	initial begin
		longint c0, c1;
		int unsigned r1, r2, rd0, wr0, act0, pre0, rd1, wr1, act1, pre1;
		bit ref_ok; int inprog [2];
		preload(R0, R0_SZ);
		preload(FB, 2 * FRAME);
		preload(R2, R2_SZ);
		preload(R3, R3_SZ);
		preload(MB, 2);
		#100;
		@(negedge clk) rst_n = 1;
		wait(ready);
		r1 = board.u1.n_ref_ready; r2 = board.u2.n_ref_ready;
		$display("READY at %0.1f us (CLK %0d kHz, corner %0d, MAP %0d, chips %0d, column bits %0d, command stage %0d, two clock tRRD/tWR/tMRD %0d, LB %0d, directed %0d)",
		         $realtime/1000.0, CLK_KHZ, CORNER, MAP, N_CHIPS, COL_BITS, CMD_STAGE, WB2, LB, DIR);
		@(posedge clk); #1;
		if(DIR != 0) begin
			int e0, n1, n2;
			bit [25:0] a, f, s0, t; int bl, len;
			e0 = errors; n1 = 0; n2 = 0;
			bl = 1 << COL_BITS;
			// D1: the write to the address above the memory, then the read of the cell it folds onto
			if(N_CHIPS == 1)
				for(int k = 0; k < 32; k++) begin
					a = R0 + 26'((k * 1000) & ~1);
					for(int i = 0; i < 3; i++) begin
						f = R2 + 26'(2 * (((k * 3 + i) * 517) % (R2_SZ / 2 - 2)));
						issue(0, 1'b1, f, 0, pat(f, 500 + k), 4'hF, 2'd0);
					end
					issue(0, 1'b1, a | (26'd1 << (COL_BITS + 15)), 0, pat(a, 900 + k), 4'hF, 2'd0);
					issue(2, 1'b0, a, 2, 0, 0, 2'd0);
					while(p_out[2] != 0) idle(1);
					idle(40); n1++;
				end
			// D2: the write in the third block of a read, then the read from its first block
			if(LB > COL_BITS)
				for(int k = 0; k < 4; k++) begin
					s0  = ((R2 + 26'(bl)) & ~26'(bl - 1)) + 26'(k * 3 * bl);   // a block start inside R2
					t   = s0 + 26'(2 * bl + 4);                                // column 4 of the third block
					len = bl + 20;                                             // from column bl-10: three blocks
					for(int i = 0; i < 3; i++) begin
						f = R0 + 26'(2 * (((k * 3 + i) * 733) % (R0_SZ / 2 - 2)));
						issue(0, 1'b1, f, 0, pat(f, 700 + k), 4'hF, 2'd0);
					end
					issue(0, 1'b1, t, 0, pat(t, 800 + k), 4'hF, 2'd0);
					issue(2, 1'b0, s0 + 26'(bl - 10), len, 0, 0, 2'd0);
					while(p_out[2] != 0) idle(1);
					idle(40); n2++;
				end
			$display("DIRECTED  D1 (address above the memory) %0d  D2 (three blocks) %0d  data errors in them %0d", n1, n2, errors - e0);
		end
		@(posedge clk);
		c0 = cyc;
		rd0 = board.u1.n_rd + board.u2.n_rd; wr0 = board.u1.n_wr + board.u2.n_wr;
		act0 = board.u1.n_act + board.u2.n_act; pre0 = board.u1.n_pre + board.u2.n_pre;
		go = 1;
		repeat(RUN_CYCLES) @(posedge clk);
		stop = 1;
		c1 = cyc;
		rd1 = board.u1.n_rd + board.u2.n_rd; wr1 = board.u1.n_wr + board.u2.n_wr;
		act1 = board.u1.n_act + board.u2.n_act; pre1 = board.u1.n_pre + board.u2.n_pre;
		// drain: every generator out of its loop and every read back
		begin
			int t; t = 0;
			while(!(gen_done == '1 && p_out[0] == 0 && p_out[1] == 0 && p_out[2] == 0 && p_out[3] == 0) && t < 200000) begin
				@(posedge clk); t++;
			end
			if(t >= 200000) begin
				bench_err++;
				$display("BENCH drain timeout: done=%b out %0d %0d %0d %0d", gen_done, p_out[0], p_out[1], p_out[2], p_out[3]);
			end
		end
		// the refresh count is read where no AUTO REFRESH can be on its way to
		// the chips: 60 cycles after a tREFI tick (both chips refreshed, idle),
		// far from the next tick
		@(posedge clk iff dut.refi_tick);
		repeat(60) @(negedge clk);

		$display("P0 CPU    writes=%0d reads=%0d  read latency avg=%0.2f max=%0d  write wait cycles=%0d",
		         n_wr[0], n_rd[0], real'(lat_sum[0]) / (lat_n[0] ? lat_n[0] : 1), lat_max[0], wr_wait[0]);
		$display("P1 video  reads=%0d  latency avg=%0.2f max=%0d  underflows=%0d  fifo min=%0d  urgent cycles=%0d  frames published=%0d switches=%0d",
		         n_rd[1], real'(lat_sum[1]) / (lat_n[1] ? lat_n[1] : 1), lat_max[1], underflow, fifo_min, urg_cyc, n_blit, vid_switch);
		$display("P2 DMA    writes=%0d reads=%0d  read latency avg=%0.2f max=%0d  write wait cycles=%0d",
		         n_wr[2], n_rd[2], real'(lat_sum[2]) / (lat_n[2] ? lat_n[2] : 1), lat_max[2], wr_wait[2]);
		$display("P3 audio  reads=%0d  latency avg=%0.2f max=%0d (deadline %0d, late %0d)  writes=%0d  mailbox reads=%0d",
		         lat_n[3], real'(lat_sum[3]) / (lat_n[3] ? lat_n[3] : 1), lat_max[3], AUD_PERIOD, aud_late, n_wr[3], n_mbox);
		$display("SLOT      max cycles a read waited in its slot: P0 %0d  P1 %0d  P2 %0d  P3 %0d  (AGE_MAX %0d)",
		         slot_max[0], slot_max[1], slot_max[2], slot_max[3], dut.AGE_MAX);
		$display("BUS       run cycles=%0d  READ=%0d WRITE=%0d  ACT=%0d PRE=%0d  column commands %0.1f%% of the cycles",
		         c1 - c0, rd1 - rd0, wr1 - wr0, act1 - act0, pre1 - pre0,
		         100.0 * real'(rd1 - rd0 + wr1 - wr0) / real'(c1 - c0));
		// refresh: done + debt + the one in the engine = tREFI ticks, per chip
		for(int c = 0; c < 2; c++)
			inprog[c] = (dut.op_v && dut.op_k == 2'd2 && dut.ref_chip == c) ? 1 : 0;
		// one chip: U2 has no debt and no refresh
		ref_ok = (board.u1.n_ref_ready - r1 + dut.debt[0] + inprog[0] == ticks) &&
		         ((N_CHIPS == 2) ? (board.u2.n_ref_ready - r2 + dut.debt[1] + inprog[1] == ticks)
		                         : (dut.debt[1] == 0 && inprog[1] == 0 && board.u2.n_ref == 0));
		$display("REFRESH   tREFI ticks=%0d  U1 done=%0d debt=%0d  U2 done=%0d debt=%0d  exact=%0b",
		         ticks, board.u1.n_ref_ready - r1, dut.debt[0], board.u2.n_ref_ready - r2, dut.debt[1], ref_ok);
		board.u1.report();
		if(N_CHIPS == 2) board.u2.report();
		else $display("[U2] no chip (one chip board): commands seen %0d", board.u2.n_act + board.u2.n_rd + board.u2.n_wr + board.u2.n_pre + board.u2.n_ref);
		if(N_CHIPS == 1) $display("NCS       cycles with ncs high=%0d", ncs_high);
		$display("CHECK     data_errors=%0d  chip_errors=%0d  bench_errors=%0d  lost=%0d  underflows=%0d  audio_late=%0d  refresh_exact=%0b",
		         errors, board.u1.err + board.u2.err, bench_err + ncs_high,
		         exq[0].size() + exq[1].size() + exq[2].size() + exq[3].size(), underflow, aud_late, ref_ok);
		$finish;
	end
endmodule
