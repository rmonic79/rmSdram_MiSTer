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
// sdr_board -- the MiSTer SDRAM boards for the bench, from their schematics:
//   N_CHIPS 2: the 128 MB board (sdram_xsds 2.9 / 3.0): U1 CS# = ncs,
//              U2 CS# = !ncs;
//   N_CHIPS 1: one chip, CS# = ncs: the 32 MB board (sdram_xs 2.2) with
//              COL_BITS 9, a 64 MB one-chip board with COL_BITS 10. U2 is
//              here only so that the bench names the same counters: its CS#
//              is tied high, it never sees a command. That the controller
//              never raises ncs (so that it also runs on the 128 MB board,
//              where ncs high is U2) is checked by the bench.
// Both: DQML = A11, DQMH = A12, CKE tied high. COL_BITS: column bits of the
// chips (10: AS4C32M16SB; 9: AS4C16M16SA, W9825G6KH).
//============================================================================
module sdr_board #(
	parameter int  N_CHIPS  = 2,
	parameter int  COL_BITS = 10,
	parameter int  INIT_REFS_MIN = 2,   // AUTO REFRESH the power-up of the chips wants (8: W9825G6KH)
	parameter real T_RC  = 63.0,  parameter real T_RFC = 63.0,
	parameter real T_RCD = 21.0,  parameter real T_RP  = 21.0,
	parameter real T_RRD = 14.0,  parameter real T_MRD = 14.0,
	parameter real T_RAS = 42.0,  parameter real T_WR  = 14.0,
	parameter real T_AC_CL2 = 6.0, parameter real T_AC_CL3 = 5.4,
	parameter real T_OH  = 2.5,   parameter real T_HZ  = 5.4,
	parameter real T_IS  = 1.5,   parameter real T_IH  = 0.8
)(
	input         sdram_clk,
	input         ncs,
	input         nras,
	input         ncas,
	input         nwe,
	input   [1:0] ba,
	input  [12:0] a,
	inout  [15:0] dq,
	input         ctl_oe          // bench side band: the FPGA drives dq at the pins
);
	sdr_chip #(.NAME("U1"), .COL_BITS(COL_BITS), .INIT_REFS_MIN(INIT_REFS_MIN), .T_RC(T_RC), .T_RFC(T_RFC), .T_RCD(T_RCD), .T_RP(T_RP),
	           .T_RRD(T_RRD), .T_MRD(T_MRD), .T_RAS(T_RAS), .T_WR(T_WR),
	           .T_AC_CL2(T_AC_CL2), .T_AC_CL3(T_AC_CL3), .T_OH(T_OH), .T_HZ(T_HZ),
	           .T_IS(T_IS), .T_IH(T_IH)) u1 (
		.clk(sdram_clk), .cke(1'b1), .cs_n(ncs), .ras_n(nras), .cas_n(ncas), .we_n(nwe),
		.ba(ba), .a(a), .dqml(a[11]), .dqmh(a[12]), .dq(dq), .ctl_oe(ctl_oe)
	);
	sdr_chip #(.NAME("U2"), .COL_BITS(COL_BITS), .INIT_REFS_MIN(INIT_REFS_MIN), .T_RC(T_RC), .T_RFC(T_RFC), .T_RCD(T_RCD), .T_RP(T_RP),
	           .T_RRD(T_RRD), .T_MRD(T_MRD), .T_RAS(T_RAS), .T_WR(T_WR),
	           .T_AC_CL2(T_AC_CL2), .T_AC_CL3(T_AC_CL3), .T_OH(T_OH), .T_HZ(T_HZ),
	           .T_IS(T_IS), .T_IH(T_IH)) u2 (
		.clk(sdram_clk), .cke(1'b1), .cs_n((N_CHIPS == 2) ? !ncs : 1'b1), .ras_n(nras), .cas_n(ncas), .we_n(nwe),
		.ba(ba), .a(a), .dqml(a[11]), .dqmh(a[12]), .dq(dq), .ctl_oe(ctl_oe)
	);
endmodule
