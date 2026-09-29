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
// sdr_chip -- one AS4C32M16SB (Alliance Memory, B die), behavioural, with
// every timing of the datasheet checked in nanoseconds.
//
// Source: Alliance Memory AS4C32M16SB datasheet, B die, Rev 1.4, June 2024 (not in the
//         repository; see SPEC_controller.md §2.1)
//   geometry          p.2 Overview, p.6 Table 3 (row A0-A12, column A0-A9)
//   commands          p.8 Table 4 and the command descriptions p.9-19
//   mode register     p.16 Tables 7-9 (burst 1/2/4/8/full page, CL 2/3)
//   AC timings        p.22 Table 16 (defaults below: the -7 grade)
//   refresh           p.18 item 12 (8192 AUTO REFRESH per 64 ms, all banks
//                     idle, tRP between two refreshes, duration tRC)
//   power up          p.23 Note 11 (200 us of stable clock, all banks
//                     precharged, MODE REGISTER SET, at least 2 AUTO REFRESH
//                     before or after it)
//   DQM               p.10 (read: two-clock latency) and p.39-40, Figures
//                     31.1-31.2 (write: zero clock latency): DQM masks write
//                     data in the same cycle and read data two cycles later
//
// What the model does NOT do (reported as "not modelled" errors, never
// silently): auto precharge (A10 on READ/WRITE), interleaved bursts, test
// modes, power down / self refresh (CKE is tied high on the MiSTer board).
//
// Output timing of reads: the word the controller must sample at chip edge k
// is held from edge k-1 + tAC(max) until edge k + tOH; between edge k-1 + tOH
// and edge k-1 + tAC the pins carry a POISON value, so a capture at the wrong
// time shows up as a data error in the scoreboard of the bench.
//
// Every check increments err and prints the first MAX_PRINT errors.
//============================================================================
module sdr_chip #(
	parameter string NAME         = "U1",
	parameter real   T_RC         = 63.0,
	parameter real   T_RFC        = 63.0,
	parameter real   T_RCD        = 21.0,
	parameter real   T_RP         = 21.0,
	parameter real   T_RRD        = 14.0,
	parameter real   T_MRD        = 14.0,
	parameter real   T_RAS        = 42.0,
	parameter real   T_RAS_MAX    = 120000.0,
	parameter real   T_WR         = 14.0,
	parameter real   T_CK_CL2     = 10.0,
	parameter real   T_CK_CL3     = 7.0,
	parameter real   T_AC_CL2     = 6.0,
	parameter real   T_AC_CL3     = 5.4,
	parameter real   T_OH         = 2.5,
	parameter real   T_HZ         = 5.4,
	parameter real   T_IS         = 1.5,
	parameter real   T_IH         = 0.8,
	parameter real   T_REFI       = 7800.0,
	parameter real   T_INIT       = 200000.0,
	parameter int    MAX_POSTPONE = 8,       // refresh debt allowed (spec, 4.5)
	parameter int    COL_BITS     = 10,      // column A0..A(COL_BITS-1): 10 AS4C32M16SB, 9 AS4C16M16SA / W9825G6KH
	parameter int    INIT_REFS_MIN = 2,      // AUTO REFRESH the power-up wants: 2 AS4C32M16SB / AS4C16M16SA (Note 11),
	                                         // 8 W9825G6KH (Rev. A04, 7.1 p.7)
	parameter int    MAX_PRINT    = 20
)(
	input         clk,
	input         cke,
	input         cs_n,
	input         ras_n,
	input         cas_n,
	input         we_n,
	input   [1:0] ba,
	input  [12:0] a,
	input         dqml,
	input         dqmh,
	inout  [15:0] dq,
	input         ctl_oe          // bench side band: the controller drives dq
);
	localparam [15:0] POISON = 16'hDEAD;

	// +cmdtrace=N: print the first N commands this chip decodes (other than NOP)
	int trace_left = 0;
	int max_print = MAX_PRINT;
	initial if($value$plusargs("maxprint=%d", max_print)) ;
	initial if($value$plusargs("cmdtrace=%d", trace_left)) ;

	//------------------------------------------------------------ counters
	int unsigned err = 0, n_act = 0, n_rd = 0, n_wr = 0, n_ref = 0, n_pre = 0;
	int unsigned n_rdword = 0, n_wrword = 0;

	function automatic void fail(input string what);
		err++;
		if(err <= max_print) $display("[%s %0t] ERROR %s", NAME, $realtime, what);
	endfunction

	//------------------------------------------------------------- memory
	bit [15:0] mem [bit [24:0]];
	function automatic [15:0] rd_word(input bit [24:0] idx);
		bit [31:0] t;
		if(mem.exists(idx)) return mem[idx];
		t = {7'd0, idx} * 32'd2654435761;   // unwritten cells are never zero
		return t[28:13];
	endfunction

	//------------------------------------------------------------- state
	realtime t_first = -1.0;          // first rising edge (clock started)
	realtime t_edge = -1.0;           // last rising edge
	realtime t_mrs = -1.0e12, t_ref = -1.0e12, t_act_any = -1.0e12, t_ready = -1.0;
	bit      pre_all_seen = 0, mrs_seen = 0, ready = 0;
	int      n_ref_init = 0;
	int unsigned n_ref_ready = 0;

	bit      open_   [4];
	int      row_    [4];
	realtime t_act   [4];
	realtime t_pre   [4];
	realtime t_wlast [4];

	int      cl = 2, bl = 1;
	bit      single_write = 0;

	// edge counter of this chip
	longint  e = 0;

	// write burst in progress
	bit      w_on = 0;
	int      w_bank, w_row, w_col0, w_n;
	// read beats: each is due at a chip edge, with its bank/row/column
	typedef struct { longint due; int bank; int row; int col; } beat_t;
	beat_t   rq [$];
	int      r_bank;                  // bank of the last read burst (for PRE/BST)
	longint  r_last_due = -1;
	// DQM history (sampled at each edge) for the read latency of 2
	bit [1:0] dqm_h1 = 2'b11, dqm_h2 = 2'b11;

	//-------------------------------------------------- setup / hold (tIS/tIH)
	realtime t_chg_ctl = -1.0e12, t_chg_dq = -1.0e12;
	bit      last_edge_relevant = 0;
	always @(cs_n or ras_n or cas_n or we_n or ba or a or dqml or dqmh) begin
		t_chg_ctl = $realtime;
		if(last_edge_relevant && t_edge >= 0 && ($realtime - t_edge) < T_IH && $realtime != t_edge)
			fail($sformatf("hold of command/address/DQM: changed %0.3f ns after the edge (tIH %0.1f)", $realtime - t_edge, T_IH));
	end

	//---------------------------------------------------------- data pins
	bit        drv = 0;
	bit [15:0] drv_val = 16'h0;
	assign dq = drv ? drv_val : 16'hzzzz;

	always @(dq) begin
		if(!drv) t_chg_dq = $realtime;
	end
	always @(drv or ctl_oe) begin
		if(drv && ctl_oe) fail("bus contention: chip and controller drive DQ together");
	end

	function automatic int burst_col(input int col0, input int i);
		int base, off;
		if(bl == (1 << COL_BITS)) return (col0 + i) % (1 << COL_BITS);
		base = col0 & ~(bl - 1);
		off  = (col0 + i) & (bl - 1);
		return base | off;
	endfunction

	//------------------------------------------------------ command decode
	always @(posedge clk) begin
		bit [3:0] cmd;
		bit sel, is_nop;
		realtime now;
		now = $realtime;
		if(t_first < 0) t_first = now;
		e++;

		cmd    = {cs_n, ras_n, cas_n, we_n};
		sel    = !cs_n;
		is_nop = !sel || cmd == 4'b0111;

		// relevance of this edge for setup/hold: a command to this chip, or
		// data/DQM that this chip uses at this edge
		last_edge_relevant = sel || w_on || (rq.size() != 0);
		if(last_edge_relevant && (now - t_chg_ctl) < T_IS)
			fail($sformatf("setup of command/address/DQM: %0.3f ns before the edge (tIS %0.1f)", now - t_chg_ctl, T_IS));
		t_edge = now;

		if(!cke) fail("CKE low: power down / clock suspend not modelled (CKE is tied high on the board)");

		// clock period vs CAS latency (Table 16, tCK)
		// (checked on the second edge onward)

		if(sel && cmd != 4'b0111 && trace_left > 0) begin
			$display("[%s %0t] CMD %b ba=%0d a=%04x dqm=%b%b", NAME, now, cmd, ba, a, dqmh, dqml);
			trace_left--;
		end

		//------------------------------------------------ power up (Note 11)
		if(!is_nop && now - t_first < T_INIT)
			fail($sformatf("command %b %0.1f us after the clock started (power up wants 200 us of NOP)", cmd, (now - t_first)/1000.0));

		//-------------------------------------------------- write data beat
		if(w_on) begin
			// a new command other than NOP/deselect ends the burst at this edge
			if(!is_nop) w_on = 0;
		end
		if(w_on) begin
			int col; bit [24:0] idx; bit [15:0] w;
			col = burst_col(w_col0, w_n);
			if((now - t_chg_dq) < T_IS) fail($sformatf("setup of write data: %0.3f ns (tIS %0.1f)", now - t_chg_dq, T_IS));
			idx = {2'(w_bank), 13'(w_row), 10'(col)};
			w = rd_word(idx);
			if(!dqml) w[7:0]  = dq[7:0];
			if(!dqmh) w[15:8] = dq[15:8];
			if(!(dqml && dqmh)) begin mem[idx] = w; n_wrword++; t_wlast[w_bank] = now; end
			w_n++;
			if(single_write || w_n >= bl) w_on = 0;
		end

		//--------------------------------------------------------- commands
		if(sel && cmd != 4'b0111) begin
			case(cmd[2:0])
			3'b011: begin	// ACTIVE
				int b; b = ba;
				if(!ready) fail("ACTIVE before the power-up sequence is complete");
				if(open_[b]) fail($sformatf("ACTIVE on bank %0d that is already open", b));
				if(now - t_pre[b] < T_RP)   fail($sformatf("tRP: ACTIVE %0.2f ns after PRECHARGE of bank %0d", now - t_pre[b], b));
				if(now - t_act[b] < T_RC)   fail($sformatf("tRC: ACTIVE %0.2f ns after ACTIVE of bank %0d", now - t_act[b], b));
				if(now - t_act_any < T_RRD) fail($sformatf("tRRD: ACTIVE %0.2f ns after another ACTIVE", now - t_act_any));
				if(now - t_ref < T_RFC)     fail($sformatf("tRFC: ACTIVE %0.2f ns after AUTO REFRESH", now - t_ref));
				if(now - t_mrs < T_MRD)     fail($sformatf("tMRD: ACTIVE %0.2f ns after MODE REGISTER SET", now - t_mrs));
				if(!(pre_all_seen && mrs_seen && n_ref_init >= INIT_REFS_MIN))
					fail($sformatf("ACTIVE before the power-up sequence is complete (%0d of %0d AUTO REFRESH)", n_ref_init, INIT_REFS_MIN));
				open_[b] = 1; row_[b] = a; t_act[b] = now; t_act_any = now; n_act++;
			end
			3'b101, 3'b100: begin	// READ / WRITE
				int b, col; b = ba; col = int'(a[9:0]) % (1 << COL_BITS);   // the chip ignores the column bits it has not
				if(a[10]) fail("auto precharge (A10 on READ/WRITE) not modelled");
				if(!open_[b]) fail($sformatf("%s on bank %0d that is not open", cmd[0] ? "READ" : "WRITE", b));
				else if(now - t_act[b] < T_RCD) fail($sformatf("tRCD: %s %0.2f ns after ACTIVE of bank %0d", cmd[0] ? "READ" : "WRITE", now - t_act[b], b));
				if(now - t_mrs < T_MRD) fail("tMRD: column command too soon after MODE REGISTER SET");
				if(!(pre_all_seen && mrs_seen && n_ref_init >= INIT_REFS_MIN))
					fail("READ/WRITE before the power-up sequence is complete");
				if(cmd[0]) begin
					// READ: a read in progress keeps its beats up to this edge + CL - 1
					while(rq.size() && rq[$].due >= e + cl) void'(rq.pop_back());
					for(int i = 0; i < bl; i++) begin
						beat_t bt; bt.due = e + cl + i; bt.bank = b; bt.row = row_[b]; bt.col = burst_col(col, i);
						rq.push_back(bt);
					end
					r_bank = b; n_rd++;
				end
				else begin
					// WRITE: the reads still to come would collide on the pins;
					// they are dropped (the controller must have masked them)
					while(rq.size() && rq[$].due >= e) void'(rq.pop_back());
					w_on = 1; w_bank = b; w_row = row_[b]; w_col0 = col; w_n = 0; n_wr++;
					begin	// first word at this edge
						bit [24:0] idx; bit [15:0] w;
						if((now - t_chg_dq) < T_IS) fail($sformatf("setup of write data: %0.3f ns (tIS %0.1f)", now - t_chg_dq, T_IS));
						idx = {2'(b), 13'(row_[b]), 10'(col)};
						w = rd_word(idx);
						if(!dqml) w[7:0]  = dq[7:0];
						if(!dqmh) w[15:8] = dq[15:8];
						if(!(dqml && dqmh)) begin mem[idx] = w; n_wrword++; t_wlast[b] = now; end
						w_n = 1;
						if(single_write || bl == 1) w_on = 0;
					end
				end
			end
			3'b010: begin	// PRECHARGE (A10 = all banks)
				for(int b = 0; b < 4; b++) if(a[10] || b == ba) begin
					if(open_[b]) begin
						if(now - t_act[b] < T_RAS)  fail($sformatf("tRAS: PRECHARGE %0.2f ns after ACTIVE of bank %0d", now - t_act[b], b));
						if(now - t_wlast[b] < T_WR) fail($sformatf("tWR: PRECHARGE %0.2f ns after the last write data of bank %0d", now - t_wlast[b], b));
					end
					open_[b] = 0; t_pre[b] = now;
				end
				// a read of a precharged bank keeps its beats up to this edge + CL - 1
				if(a[10] || r_bank == ba)
					while(rq.size() && rq[$].due >= e + cl) void'(rq.pop_back());
				if(a[10]) pre_all_seen = 1;
				n_pre++;
			end
			3'b001: begin	// AUTO REFRESH
				for(int b = 0; b < 4; b++) begin
					if(open_[b]) fail($sformatf("AUTO REFRESH with bank %0d open", b));
					else if(now - t_pre[b] < T_RP) fail($sformatf("tRP: AUTO REFRESH %0.2f ns after PRECHARGE of bank %0d", now - t_pre[b], b));
				end
				if(!pre_all_seen) fail("AUTO REFRESH before the first PRECHARGE ALL of the power-up sequence");
				if(now - t_ref < T_RFC) fail($sformatf("tRFC: AUTO REFRESH %0.2f ns after AUTO REFRESH", now - t_ref));
				if(now - t_mrs < T_MRD) fail("tMRD: AUTO REFRESH too soon after MODE REGISTER SET");
				t_ref = now; n_ref++;
				if(!ready) n_ref_init++; else n_ref_ready++;
			end
			3'b000: begin	// MODE REGISTER SET
				for(int b = 0; b < 4; b++) if(open_[b]) fail("MODE REGISTER SET with a bank open");
				if(!pre_all_seen) fail("MODE REGISTER SET before the first PRECHARGE ALL");
				if(now - t_ref < T_RFC) fail("tRFC: MODE REGISTER SET too soon after AUTO REFRESH");
				case(a[2:0])
					3'b000: bl = 1; 3'b001: bl = 2; 3'b010: bl = 4; 3'b011: bl = 8; 3'b111: bl = 1 << COL_BITS;
					default: fail($sformatf("burst length field %b reserved", a[2:0]));
				endcase
				if(a[3]) fail("interleaved burst not modelled");
				case(a[6:4])
					3'b010: cl = 2; 3'b011: cl = 3;
					default: fail($sformatf("CAS latency field %b reserved", a[6:4]));
				endcase
				if(a[8:7] != 2'b00) fail("test mode (A8:A7) not modelled");
				single_write = a[9];
				t_mrs = now; mrs_seen = 1;
			end
			3'b110: begin	// BURST STOP
				w_on = 0;
				while(rq.size() && rq[$].due >= e + cl) void'(rq.pop_back());
			end
			default: ;
			endcase
		end

		// power-up sequence complete
		if(!ready && pre_all_seen && mrs_seen && n_ref_init >= INIT_REFS_MIN && now - t_mrs >= T_MRD) begin
			ready = 1; t_ready = now;
		end

		// open row longer than tRAS(max)
		for(int b = 0; b < 4; b++)
			if(open_[b] && now - t_act[b] > T_RAS_MAX) begin
				fail($sformatf("tRAS(max): bank %0d open for %0.1f us", b, (now - t_act[b])/1000.0));
				t_act[b] = now;   // report once per period
			end

		// refresh rate (8192 per 64 ms = one per tREFI on average), with the
		// debt allowed by the spec
		if(ready) begin
			int due; due = int'((now - t_ready) / T_REFI);
			if(int'(n_ref_ready) + MAX_POSTPONE < due) begin
				fail($sformatf("refresh late: %0d done, %0d due after %0.1f us", n_ref_ready, due, (now - t_ready)/1000.0));
				n_ref_ready = due;   // report once per missing batch
			end
		end

		//--------------------------------------------------------- read data
		// DQM at this edge masks the beat due two edges later
		dqm_h2 = dqm_h1;
		dqm_h1 = {dqmh, dqml};
		begin
			bit have_now, have_next;
			beat_t nb;
			have_now  = rq.size() && rq[0].due == e;
			if(have_now) begin void'(rq.pop_front()); n_rdword++; end
			have_next = rq.size() && rq[0].due == e + 1;
			if(have_next) nb = rq[0];
			// the beat of edge e+1 is masked by the DQM sampled at edge e-1 (dqm_h2)
			fork begin
				#(T_OH);
				if(have_next) begin
					drv = 1; drv_val = POISON;
				end
				else if(have_now) begin
					drv = 1; drv_val = POISON;
					#(T_HZ - T_OH);
					drv = 0;
				end
			end join_none
			if(have_next) begin
				bit [1:0] mk;
				bit [24:0] idx;
				mk  = dqm_h2;
				idx = {2'(nb.bank), 13'(nb.row), 10'(nb.col)};
				fork begin
					bit [15:0] v;
					v = rd_word(idx);
					#((cl == 2) ? T_AC_CL2 : T_AC_CL3);
					if(mk == 2'b11) drv = 0;
					else begin
						drv = 1;
						drv_val = {mk[1] ? POISON[15:8] : v[15:8], mk[0] ? POISON[7:0] : v[7:0]};
					end
				end join_none
			end
		end
	end

	//----------------------------------------------------- clock period check
	realtime t_prev_edge = -1.0;
	always @(posedge clk) begin
		if(t_prev_edge >= 0) begin
			if(cl == 2 && ($realtime - t_prev_edge) < T_CK_CL2 - 0.001)
				fail($sformatf("tCK %0.3f ns shorter than the %0.1f ns of CL=2", $realtime - t_prev_edge, T_CK_CL2));
			if(cl == 3 && ($realtime - t_prev_edge) < T_CK_CL3 - 0.001)
				fail($sformatf("tCK %0.3f ns shorter than the %0.1f ns of CL=3", $realtime - t_prev_edge, T_CK_CL3));
		end
		t_prev_edge = $realtime;
	end

	function automatic void report();
		$display("[%s] errors=%0d ACT=%0d READ=%0d (words %0d) WRITE=%0d (words %0d) PRE=%0d REF=%0d ready=%0b CL=%0d BL=%0d",
		         NAME, err, n_act, n_rd, n_rdword, n_wr, n_wrword, n_pre, n_ref, ready, cl, bl);
	endfunction
endmodule
