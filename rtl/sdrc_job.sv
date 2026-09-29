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
// sdrc_job -- a "job" port on one port of sdrc.
//
// The job port:
//   - a pulse on rd or wr starts a job of blen halfwords (0 = 2^BL_BITS; one
//     halfword with FIXED1) at the halfword address adr; rd wins over wr in
//     the same cycle. adr and blen are taken in the cycle the job starts
//     (the clients hold them until done);
//   - write: bnum is the index of the word; wdat and bsel (bsel[1] = high
//     byte) are that word and its byte selects, given by the client from
//     bnum in the same cycle; bsel = 00 writes nothing;
//   - read: each word comes back on rdat with rval high, bnum its index;
//     rdat keeps the last word of this port until the next one;
//   - done: a one-cycle pulse, the cycle after the last rval (read) or after
//     the cycle sdrc takes the last word (write);
//   - a pulse that comes while a job runs is kept and starts the next job
//     (one job pending: a newer pulse replaces it).
// Writes are posted: done comes when sdrc has taken the words into its write
// buffer, and a read taken after that, from any port, sees them (sdrc order).
// Reads longer than the sdrc port takes (2^LEN_BITS-1) go out in pieces.
// HAS_RD = 0 / HAS_WR = 0: the port never reads / never writes; rd / wr are
// ignored and that side is not built (give sdrc the same in RD_PORTS/WR_PORTS).
//============================================================================
module sdrc_job #(
	parameter int ADR_BITS = 24,
	parameter int BL_BITS  = 9,
	parameter int LEN_BITS = 10,
	parameter int FIXED1   = 0,
	parameter int HAS_RD   = 1,
	parameter int HAS_WR   = 1
)(
	input                     clk,
	input                     rst_n,

	// job port
	input      [ADR_BITS-1:0] adr,
	input                     rd,
	input                     wr,
	input      [BL_BITS-1:0]  blen,
	input      [1:0]          bsel,
	input      [15:0]         wdat,
	output reg [15:0]         rdat,
	output reg                rval,
	output     [BL_BITS-1:0]  bnum,
	output reg                done,
	output                    busy,       // a job runs

	// one port of sdrc
	output                    p_req,
	output                    p_we,
	output     [25:0]         p_addr,
	output     [LEN_BITS-1:0] p_len,
	output     [31:0]         p_wdata,
	output     [3:0]          p_wbe,
	input                     p_ready,
	input      [15:0]         p_rdata,
	input                     p_rvalid
);
	localparam int NW   = BL_BITS + 1;                 // width of a word count
	localparam int MAXR = (1 << LEN_BITS) - 1;         // longest read of the sdrc port
	localparam int CW   = (NW > LEN_BITS) ? NW : LEN_BITS;   // width to compare the two counts

	localparam [1:0] S_IDLE = 2'd0, S_WR = 2'd1, S_RD = 2'd2;
	reg  [1:0]         st = S_IDLE;
	reg                pend_v = 1'b0, pend_w = 1'b0;   // a job waiting, and whether it writes
	reg [ADR_BITS-1:0] a0;
	reg [NW-1:0]       n;              // words of the job
	reg [NW-1:0]       k;              // write: words taken by sdrc; read: words asked
	reg [NW-1:0]       rv;             // read: words given back

	wire [NW-1:0] len_now = (FIXED1 != 0) ? NW'(1) : ((blen == '0) ? NW'(1 << BL_BITS) : NW'(blen));
	wire [NW-1:0] left    = n - k;
	wire [CW-1:0] left_w  = CW'(left);
	wire [25:0]   wa      = 26'(a0) + 26'(k);

	wire rd_i = (HAS_RD != 0) && rd;           // the pulses this port can take
	wire wr_i = (HAS_WR != 0) && wr;

	assign busy    = (st != S_IDLE);
	assign bnum    = (st == S_WR) ? k[BL_BITS-1:0] : rv[BL_BITS-1:0];
	assign p_req   = (st == S_WR) || (st == S_RD && k != n);
	assign p_we    = (st == S_WR);
	assign p_addr  = (st == S_WR) ? {wa[25:1], 1'b0} : wa;
	// a read in pieces only if a job can be longer than the sdrc port takes
	generate
		if((1 << BL_BITS) > MAXR) begin : g_split
			assign p_len = (left_w > CW'(MAXR)) ? LEN_BITS'(MAXR) : LEN_BITS'(left);
		end
		else begin : g_whole
			assign p_len = LEN_BITS'(left);
		end
	endgenerate
	assign p_wdata = {wdat, wdat};
	assign p_wbe   = wa[0] ? {bsel, 2'b00} : {2'b00, bsel};

	always @(posedge clk) begin
		if(!rst_n) begin
			st <= S_IDLE; pend_v <= 1'b0; pend_w <= 1'b0;
			done <= 1'b0; rval <= 1'b0; rdat <= 16'h0;
			n <= '0; k <= '0; rv <= '0;
		end
		else begin
			done <= 1'b0;
			rval <= (HAS_RD != 0) && p_rvalid;
			if((HAS_RD != 0) && p_rvalid) rdat <= p_rdata;

			// a pulse during a job waits for the next one
			if(st == S_IDLE) begin
				if(rd_i || wr_i || pend_v) pend_v <= 1'b0;
			end
			else if(rd_i || wr_i) begin
				pend_v <= 1'b1; pend_w <= !rd_i;
			end

			case(st)
			S_IDLE:
				if(rd_i || wr_i || pend_v) begin
					a0 <= adr; n <= len_now; k <= '0; rv <= '0;
					st <= (HAS_RD == 0) ? S_WR : (HAS_WR == 0) ? S_RD :
					      (rd_i ? S_RD : (wr_i ? S_WR : (pend_w ? S_WR : S_RD)));
				end
			S_WR:
				if(p_ready) begin
					k <= k + 1'b1;
					if(k + 1'b1 == n) begin st <= S_IDLE; done <= 1'b1; end
				end
			S_RD: begin
				if(p_req && p_ready) k <= k + NW'(p_len);
				if(rval) begin
					rv <= rv + 1'b1;
					if(rv + 1'b1 == n) begin st <= S_IDLE; done <= 1'b1; end
				end
			end
			default: st <= S_IDLE;
			endcase
		end
	end
endmodule
