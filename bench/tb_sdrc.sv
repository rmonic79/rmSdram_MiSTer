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
// One-port bench of sdrc (rtl/sdrc.sv) on the model of a MiSTer SDRAM board
// (sdr_board.sv: the 128 MB board, or one chip with COL_BITS), with the pin
// delays of the compiled netlist (bench/pin_datasheet.tcl), in the corner
// given by CORNER. MAP 0. After 'ready': back-to-back writes over one row
// (W_BTB), spaced writes (W_GAP), cache-line reads (R_LINE) and single reads
// (R_ONE) of what was written, then random mixed traffic over the four banks
// (MIXED); every read checked against a shadow of the memory. Then, per
// chip: no error of the model, the power-up sequence complete, the AUTO
// REFRESH done against those due; with one chip, ncs never high.
//============================================================================
module tb_sdrc;
	parameter int CLK_KHZ = 66667;
	parameter int CORNER  = 0;
	parameter int N_CHIPS  = 2;     // board (sdr_board.sv) and controller
	parameter int COL_BITS = 10;
	parameter int WB2      = 0;     // 1: the W9825G6KH: tRRD, tWR, tMRD of two clocks and eight power-up refreshes
	localparam int PER_PS   = 1000000000 / CLK_KHZ;    // the period as sdrc computes it
	localparam int T2_PS    = WB2 ? ((2 * PER_PS > 14000) ? 2 * PER_PS : 14000) : 14000;
	parameter int INIT_REFS = WB2 ? 8 : 2;             // AUTO REFRESH of the power-up (W9825G6KH: 8)
	localparam int DW_ROW   = 1 << (COL_BITS - 1);     // dwords in a row
	localparam int N_GAP    = (DW_ROW < 256) ? DW_ROW : 256;

	localparam real HALF_PERIOD = 500000.0 / CLK_KHZ;
	reg clk = 0;
	always #(HALF_PERIOD) clk = ~clk;
	reg rst_n = 0;

	//----------------------------------------------------------------- DUT
	wire        ncs, nras, ncas, nwe, ready;
	wire  [1:0] ba;
	wire [12:0] a;
	wire [15:0] dq;             // the data pins as the controller sees them

	parameter int CAP_H_FORCE = -1;
	reg         req = 0, we = 0;
	reg  [25:0] addr = 0;
	reg   [3:0] len = 4'd2;              // halfwords
	reg  [31:0] wdata = 0;
	reg   [3:0] wbe = 4'hF;
	wire        port_ready, rvalid;
	wire [15:0] rdata;

	sdrc #(.CLK_KHZ(CLK_KHZ), .N_PORTS(1), .LEN_BITS(4), .MAP(0), .CAP_H_FORCE(CAP_H_FORCE),
	       .N_CHIPS(N_CHIPS), .COL_BITS(COL_BITS), .T_RRD_PS(T2_PS), .T_WR_PS(T2_PS), .T_MRD_PS(T2_PS), .INIT_REFS(INIT_REFS)) dut (
		.clk(clk), .rst_n(rst_n), .ready(ready),
		.req(req), .we(we), .urgent(1'b0), .addr(addr), .len(len), .wdata(wdata), .wbe(wbe),
		.port_ready(port_ready), .rdata(rdata), .rvalid(rvalid),
		.sdram_ncs(ncs), .sdram_nras(nras), .sdram_ncas(ncas), .sdram_nwe(nwe),
		.sdram_ba(ba), .sdram_a(a), .sdram_dq(dq)
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
	localparam real HALF   = (T_H + T_SU) / 2.0;

	reg        p_ncs = 1'b1, p_nras = 1'b1, p_ncas = 1'b1, p_nwe = 1'b1, p_clk = 1'b0, p_oe = 1'b0;
	reg  [1:0] p_ba = 2'd0;
	reg [12:0] p_a = 13'h1800;
	reg [15:0] p_dqo = 16'd0;
	wire [15:0] p_dq;

	// at time zero the delayed pins take the start values of the controller's
	// pins (1 ps after its initial blocks), then follow them with the delays
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

	// one chip: ncs never high (on the 128 MB board ncs high selects U2)
	longint ncs_high = 0;
	always @(posedge clk) if(N_CHIPS == 1 && ncs !== 1'b0) ncs_high++;

	sdr_board #(.N_CHIPS(N_CHIPS), .COL_BITS(COL_BITS), .INIT_REFS_MIN(WB2 ? 8 : 2),
	            .T_RRD(T2_PS / 1000.0), .T_WR(T2_PS / 1000.0), .T_MRD(T2_PS / 1000.0)) board (
		.sdram_clk(p_clk), .ncs(p_ncs), .nras(p_nras), .ncas(p_ncas), .nwe(p_nwe),
		.ba(p_ba), .a(p_a), .dq(p_dq), .ctl_oe(p_oe)
	);

	//------------------------------------------------------ port master
	// requests change 1 ns after a rising edge; port_ready and rvalid are
	// sampled at the falling edge (the value the next rising edge sees)
	longint cyc = 0;
	always @(posedge clk) cyc <= cyc + 1;
	bit [15:0] rbuf [0:7];

	task automatic do_write(input [25:0] ad, input [31:0] d, input [3:0] be, output int unsigned cycles);
		bit acc;
		@(posedge clk); #1;
		req = 1; we = 1; addr = ad; wdata = d; wbe = be; len = 4'd2;
		cycles = 0;
		do begin @(negedge clk); cycles++; acc = port_ready; @(posedge clk); end while(!acc);
		#1 req = 0;
	endtask

	task automatic do_read(input [25:0] ad, input [2:0] n, output int unsigned cycles);
		bit acc, accepted; int k;
		@(posedge clk); #1;
		req = 1; we = 0; addr = ad; len = {n, 1'b0};
		cycles = 0; k = 0; accepted = 0;
		while(!(accepted && k >= 2 * n)) begin
			@(negedge clk);
			cycles++;
			if(rvalid && k < 8) begin rbuf[k] = rdata; k++; end
			acc = !accepted && port_ready;
			@(posedge clk);
			if(acc) begin accepted = 1; #1 req = 0; end
		end
	endtask

	//-------------------------------------------------------- scoreboard
	bit [15:0] shadow [bit [25:0]];      // halfword address -> value
	int unsigned errors = 0;
	function automatic [31:0] pat(input int unsigned x, input int unsigned k);
		pat = (x * 32'h9E3779B1) ^ (k * 32'h85EBCA77);
	endfunction
	task automatic sb_write(input [25:0] ad, input [31:0] d, input [3:0] be);
		bit [15:0] lo, hi;
		lo = shadow.exists(ad)     ? shadow[ad]     : 16'h0;
		hi = shadow.exists(ad + 1) ? shadow[ad + 1] : 16'h0;
		shadow[ad]     = {be[1] ? d[15:8]  : lo[15:8], be[0] ? d[7:0]   : lo[7:0]};
		shadow[ad + 1] = {be[3] ? d[31:24] : hi[15:8], be[2] ? d[23:16] : hi[7:0]};
	endtask
	task automatic sb_check(input string tag, input [25:0] ad, input int n);
		for(int i = 0; i < 2 * n; i++)
			if(shadow.exists(ad + i) && rbuf[i] != shadow[ad + i]) begin
				errors++;
				if(errors < 6) $display("  MISMATCH %s a=%07x got %04x want %04x", tag, ad + i, rbuf[i], shadow[ad + i]);
			end
	endtask

	// halfword address of dword dw of a row, MAP 0: {bank, chip, row, col};
	// one chip: {bank, row, col} (no chip bit), col of COL_BITS
	function automatic [25:0] hw(input [1:0] b, input bit c, input [12:0] r, input [8:0] dw);
		if(N_CHIPS == 2) return {b, c, r, dw, 1'b0};
		return (26'(b) << (COL_BITS + 13)) | (26'(r) << COL_BITS) | (26'({dw, 1'b0}) & ((26'd1 << COL_BITS) - 26'd1));
	endfunction

	//---------------------------------------------------------- the run
	initial begin
		realtime t0;
		int unsigned r1, r2, c, mx, i, k;
		longint total, ts;
		real due;
		#100;
		@(negedge clk) rst_n = 1;
		wait(ready);
		t0 = $realtime;
		r1 = board.u1.n_ref; r2 = board.u2.n_ref;
		$display("READY at %0.1f us (CLK %0d kHz, corner %0d, chips %0d, column bits %0d, two clock tRRD/tWR/tMRD %0d)",
		         t0/1000.0, CLK_KHZ, CORNER, N_CHIPS, COL_BITS, WB2);

		// W_BTB: DW_ROW dword writes, one row (512 with 10 column bits)
		ts = cyc;
		for(i = 0; i < DW_ROW; i++) begin
			do_write(hw(2'd0, 1'b0, 13'd4, 9'(i)), pat(i, 1), 4'hF, c);
			sb_write(hw(2'd0, 1'b0, 13'd4, 9'(i)), pat(i, 1), 4'hF);
		end
		$display("W_BTB   n=%0d  cycles/op=%0.2f", DW_ROW, real'(cyc - ts) / real'(DW_ROW));

		// W_GAP: N_GAP writes with 12 idle cycles between them
		total = 0; mx = 0;
		for(i = 0; i < N_GAP; i++) begin
			repeat(12) @(posedge clk);
			do_write(hw(2'd1, 1'b1, 13'd7, 9'(i)), pat(i, 2), 4'hF, c);
			sb_write(hw(2'd1, 1'b1, 13'd7, 9'(i)), pat(i, 2), 4'hF);
			total += c; if(c > mx) mx = c;
		end
		$display("W_GAP   n=%0d  gap=12  wait-to-accept/op=%0.2f  max=%0d", N_GAP, real'(total)/real'(N_GAP), mx);

		// R_LINE: DW_ROW/4 reads of 4 dwords over what W_BTB wrote
		total = 0; mx = 0; ts = cyc;
		for(i = 0; i < DW_ROW / 4; i++) begin
			do_read(hw(2'd0, 1'b0, 13'd4, 9'(4*i)), 3'd4, c);
			total += c; if(c > mx) mx = c;
			sb_check("R_LINE", hw(2'd0, 1'b0, 13'd4, 9'(4*i)), 4);
		end
		$display("R_LINE  n=%0d  request-to-last-beat/op=%0.2f  max=%0d  period/op=%0.2f",
		         DW_ROW / 4, real'(total)/real'(DW_ROW / 4), mx, real'(cyc - ts)/real'(DW_ROW / 4));

		// R_ONE: N_GAP single dword reads over what W_GAP wrote
		total = 0; mx = 0; ts = cyc;
		for(i = 0; i < N_GAP; i++) begin
			do_read(hw(2'd1, 1'b1, 13'd7, 9'(i)), 3'd1, c);
			total += c; if(c > mx) mx = c;
			sb_check("R_ONE", hw(2'd1, 1'b1, 13'd7, 9'(i)), 1);
		end
		$display("R_ONE   n=%0d  request-to-last-beat/op=%0.2f  max=%0d  period/op=%0.2f",
		         N_GAP, real'(total)/real'(N_GAP), mx, real'(cyc - ts)/real'(N_GAP));

		// MIXED: 4000 random operations over the 4 banks (and the 2 chips), a
		// small set of rows so that reads meet written data; random byte enables
		ts = cyc;
		for(i = 0; i < 4000; i++) begin
			bit [25:0] ad; bit [3:0] be; bit [31:0] d; int unsigned r;
			r  = $urandom;
			ad = hw(r[1:0], r[2], 13'(r[5:3]), 9'($urandom % DW_ROW));
			if(shadow.exists(ad) && r[6]) begin
				do_read(ad, 3'd1, c);
				sb_check("MIXED", ad, 1);
			end
			else begin
				be = shadow.exists(ad) ? 4'($urandom | 1) : 4'hF;
				d  = pat(ad, i + 9);
				do_write(ad, d, be, c);
				sb_write(ad, d, be);
			end
		end
		$display("MIXED   n=4000  period/op=%0.2f", real'(cyc - ts)/4000.0);

		due = ($realtime - t0) / 7800.0;
		board.u1.report();
		if(N_CHIPS == 2) board.u2.report();
		else $display("[U2] no chip (one chip board): commands seen %0d  ncs high %0d cycles",
		              board.u2.n_act + board.u2.n_rd + board.u2.n_wr + board.u2.n_pre + board.u2.n_ref, ncs_high);
		$display("REFRESH after ready: U1 %0d  U2 %0d  due %0.1f per chip", board.u1.n_ref - r1, board.u2.n_ref - r2, due);
		$display("CHECK   data_errors=%0d  chip_errors=%0d  ready U1=%0b U2=%0b  bench_errors=%0d",
		         errors, board.u1.err + board.u2.err, board.u1.ready, board.u2.ready, ncs_high);
		$finish;
	end
endmodule
