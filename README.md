# rmSdram — a synchronous SDRAM controller for the MiSTer SDRAM boards

A single-clock SDRAM controller: the controller, its access ports and the write
buffer all run on the core's clock. There is no second clock domain inside it,
no extra PLL for the logic and nothing to add to the SDC.

It keeps one row open per bank, serves several ports through one arbiter,
buffers writes so they do not block readers, and overlaps banks — while one
operation holds the data bus, the bank of the next candidate is being activated
or precharged on the free command cycles, so when its turn comes only the column
commands are left.

Every chip timing is a parameter in picoseconds, turned into cycles of your
clock at elaboration. The edge at which read data is captured is chosen the
same way, and the CAS latency is 2 at every clock up to 100 MHz (§7). Nothing is hard-wired to one frequency or to
one board: the same source drives the 128 MB board with both chips, or a single
chip with 9 column bits, which runs on every MiSTer SDRAM board (§2).

`SPEC_controller.md` is the design specification, with the datasheet values and
the board facts it was written from; this file is how to use it.

---

## Contents

1. [What you need](#1-what-you-need)
2. [Boards](#2-boards)
3. [Quick start](#3-quick-start)
4. [The port protocol](#4-the-port-protocol)
5. [Address map](#5-address-map)
6. [The job port: `sdrc_job`](#6-the-job-port-sdrc_job)
7. [Parameters](#7-parameters)
8. [The clock to the SDRAM — read this first](#8-the-clock-to-the-sdram--read-this-first)
9. [`CMD_STAGE`](#9-cmd_stage)
10. [What it does not do](#10-what-it-does-not-do)
11. [Verification](#11-verification)
12. [How it was written](#12-how-it-was-written)
13. [License](#13-license)

---

## 1. What you need

- A Cyclone V (or comparable) FPGA and a MiSTer SDRAM board (§2).
- One clock for the core, up to 100 MHz (§7).
- A clock for the SDRAM pin. **The controller does not generate it** — see §8,
  which is the part to read before anything else.

Files: `rtl/sdrc.sv` is the controller; `rtl/sdrc_job.sv` is an optional adapter
that puts a "job" interface on one of its ports (§6).

---

## 2. Boards

All the boards have chips with 4 banks of 8192 rows, a 16-bit data bus, `DQML`
and `DQMH` wired to `A11` and `A12`, and `CKE` tied high. Two parameters describe
what the controller drives:

| `N_CHIPS` | `COL_BITS` | memory | chips | runs on |
|---|---|---|---|---|
| 2 | 10 | 128 MB | two AS4C32M16SB; `sdram_ncs` selects one (U2 through an inverter) | the 128 MB board (sdram_xsds 2.9 / 3.0) |
| 1 | 10 | 64 MB | one AS4C32M16SB-class chip (10 column bits) | a board with such a chip, and the 128 MB board |
| 1 | 9 | 32 MB | one AS4C16M16SA or W9825G6KH (9 column bits) | **every board**: the 32 MB board (sdram_xs 2.2) and the 128 MB board |

With one chip, `sdram_ncs` stays low: on the 32 MB board it is the chip select of
the only chip, on the 128 MB board it selects U1 and U2 is never touched. The
power-up sequence and the refresh are those of that chip alone.

**Which to choose.** Two chips give eight open rows instead of four, and 128 MB
instead of 32 MB: if the core needs the bandwidth or the memory, build for the
128 MB board; if it does not, build for one chip with 9 column bits and it runs on any
board. Measured on the Darius core at 48 MHz (three tile layers and the sprites,
each on its own port, plus two CPUs, all reading at the same time): the worst
line of the tile layers took 2449 of 3052 cycles with two chips and 2501 with one;
the sprite port, the most random stream, completed 18% fewer reads in the same
time on one chip (average latency 25 cycles against 19).

**The W9825G6KH.** The 32 MB board may carry either chip. The Winbond part gives
`tRRD`, `tWR` and `tMRD` as **two clocks** rather than in nanoseconds, and wants
**eight** AUTO REFRESH at power-up where the Alliance parts want two. A one-chip
build meant for every board should pass `T_RRD_PS`, `T_WR_PS` and `T_MRD_PS` of
at least two periods (41666 at 48 MHz) and `INIT_REFS = 8`. Every other timing of
both 32 MB chips is within the defaults (§7).

The board is a build-time choice: the controller does not probe the memory.

---

## 3. Quick start

```systemverilog
sdrc #(
    .CLK_KHZ  (48000),   // your core clock, in kHz
    .N_PORTS  (4),
    .LEN_BITS (4),       // reads of 1..15 halfwords
    .MAP      (1),
    .N_CHIPS  (1),       // one chip, 9 column bits: every board (§2)
    .COL_BITS (9),
    .T_RRD_PS (41666),   // two clocks at 48 MHz, for the W9825G6KH (§2)
    .T_WR_PS  (41666),
    .T_MRD_PS (41666),
    .INIT_REFS(8),       // eight power-up refreshes, for the W9825G6KH (§2)
    .CMD_STAGE(0),       // §9
    .TCKO_SLOW_PS (13864), // §8: these describe YOUR clock path and board
    .TCKO_FAST_PS (6809),  //     (here: the Darius core, SDRAM clock at half a period)
    .TSU_SLOW_PS  (-5743),
    .TH_SLOW_PS   (8036),
    .TSU_FAST_PS  (-2384),
    .TH_FAST_PS   (3934)
) u_sdrc (
    .clk        (clk),
    .rst_n      (~reset),
    .ready      (sdram_ready),

    .req        (p_req),        // [N_PORTS-1:0]
    .we         (p_we),
    .urgent     ('0),
    .addr       (p_addr),       // port g: p_addr[g*26 +: 26]
    .len        (p_len),        // port g: p_len[g*LEN_BITS +: LEN_BITS]
    .wdata      (p_wdata),      // port g: p_wdata[g*32 +: 32]
    .wbe        (p_wbe),        // port g: p_wbe[g*4 +: 4]
    .port_ready (p_ready),
    .rdata      (p_rdata),      // 16 bits, shared
    .rvalid     (p_rvalid),     // [N_PORTS-1:0]

    .sdram_ncs  (SDRAM_nCS),
    .sdram_nras (SDRAM_nRAS),
    .sdram_ncas (SDRAM_nCAS),
    .sdram_nwe  (SDRAM_nWE),
    .sdram_ba   (SDRAM_BA),
    .sdram_a    (SDRAM_A),
    .sdram_dqm  ({SDRAM_DQMH, SDRAM_DQML}),
    .sdram_dq   (SDRAM_DQ)
);

assign SDRAM_CLK = clk_sdram;   // from a PLL output, see §8
assign SDRAM_CKE = 1'b1;
```

`rst_n` is active low. `ready` rises when the power-up sequence has finished on
every chip; hold your clients off until then.

The port signals are packed vectors, one slice per port. If you prefer a
per-port bundle, a thin adapter around these slices is a few lines of glue and
costs nothing — the protocol below is what it has to speak.

---

## 4. The port protocol

**A request is taken in the cycle in which `req[g]` and `port_ready[g]` are both
high.** Hold the request until that happens. All port signals are sampled on
`clk`, on that same edge.

### Write

One dword per request, at an **even halfword address**, with `wdata[g]` and the
four byte enables `wbe[g]`. It goes into the shared write buffer and the port is
free again immediately. A write with `wbe = 0` is taken and dropped.

Writes from several ports in the same cycle are taken one at a time, in turn:
that is why `port_ready` can fall for a cycle even when the buffer is not full.

### Read

`len[g]` halfwords, from **any** halfword address, `len` in `1 .. 2^LEN_BITS-1`.
`LEN_BITS` goes up to `COL_BITS + 1`: reads of up to 1023 halfwords with either
column width, up to 2047 with 10 column bits. The data comes back on the shared
`rdata` bus with `rvalid[g]` high for one cycle per halfword, in the order that
port asked for them. A read with `len = 0` is taken and dropped.

**One read slot per port**: a port's next read is taken as soon as the
controller has started its previous one (the slot frees when the read becomes
the current operation, before its data is back); the data still comes back in
order. Two ports read independently of each other.

### `urgent`

A real-time port (video line fetch, audio) raises `urgent[g]` while it must be
served before everything else; only a refresh forced by `REF_MAX` (§7) goes
before it. Leave it at zero if you have no such port.

### Ordering between ports

Memory is ordered in **blocks** of `2^COL_BITS` halfwords (the halfwords of one
row of one bank).

- A read waits for the writes taken before it **when one of those writes touches
  a block the read touches**.
- A port's write is not taken while a read of that same port is still in its
  slot.
- A read of one port and a write of another have **no** order between them if
  the write is taken in the same cycle as the read or later. If you need one,
  enforce it outside the controller.

---

## 5. Address map

Every port uses the same 26-bit **linear halfword address** and the same map.
One map for all ports is deliberate: with a per-port map, two ports sharing
memory — a CPU writing VRAM and the video reading it — would see the same
address in different cells.

Two chips (`COL_BITS` 10):

| `MAP` | bank | chip | row | column |
|---|---|---|---|---|
| `0` | h[25:24] | h[23] | h[22:10] | h[9:0] |
| `1` | h[12:11] | h[10] | h[25:13] | h[9:0] |

One chip, with C = `COL_BITS`:

| `MAP` | bank | row | column |
|---|---|---|---|
| `0` | h[C+14:C+13] | h[C+12:C] | h[C-1:0] |
| `1` | h[C+1:C] | h[C+14:C+2] | h[C-1:0] |

With one chip the address bits above the memory (h[25] with 10 column bits,
h[25:24] with 9) are not used: those addresses fold onto the memory, and the
ordering of §4 treats them as the cells they fold onto.

The column is always the low `COL_BITS`: a burst that runs past the last column
continues in the next block, mapped again.

Choose `MAP 0` when you want to place different streams in different banks by
address region — the high address bits are the bank. Choose `MAP 1` when you
want consecutive blocks to fall in different banks by themselves, so that
unrelated streams interleave without you arranging anything.

---

## 6. The job port: `sdrc_job`

`sdrc_job` sits on one port of `sdrc` and turns it into a job interface, for
clients that think in bursts of words rather than in requests:

- a one-cycle pulse on `rd` or `wr` starts a job of `blen` halfwords
  (`0` = 2^`BL_BITS`) at the halfword address `adr`; `rd` wins over `wr` in the
  same cycle. The client holds `adr` and `blen` until `done`;
- write: `bnum` is the index of the word wanted now; the client answers in the
  same cycle with `wdat` and `bsel` (`bsel[1]` = high byte; `00` writes nothing);
- read: each word comes back on `rdat` with `rval` high and `bnum` its index;
  `rdat` keeps the last word until the next one;
- `done` is a one-cycle pulse, the cycle after the last `rval` (read) or after
  the controller has taken the last word into its write buffer (write). Writes
  are posted: a read taken after `done`, from any port, sees them;
- a pulse that arrives while a job runs is kept and starts the next job (a newer
  pulse replaces it);
- reads longer than the port takes (`2^LEN_BITS-1`) go out in pieces.

`HAS_RD = 0` or `HAS_WR = 0` leaves that side unbuilt; give `sdrc` the same in
`RD_PORTS` / `WR_PORTS`.

Parameters: `ADR_BITS` (24) is the width of `adr`: 24 bits reach 32 MB, use 25
or 26 for the 64 MB and 128 MB builds; `BL_BITS` (9) is the width of `blen` and
`bnum`; `LEN_BITS` (10) must equal the `LEN_BITS` of `sdrc`; `FIXED1 = 1` makes
every job one halfword (`blen` ignored); `HAS_RD`, `HAS_WR` (1). `busy` is high
while a job runs.

---

## 7. Parameters

### You set these

| Parameter | Default | Meaning |
|---|---|---|
| `CLK_KHZ` | 66667 | Core clock in kHz. Every timing below is converted with it. |
| `N_PORTS` | 1 | Number of access ports. |
| `LEN_BITS` | 4 | Width of `len`: reads of `1 .. 2^LEN_BITS-1` halfwords, `LEN_BITS` ≤ `COL_BITS + 1`. |
| `MAP` | 1 | Address map, §5. |
| `N_CHIPS` | 2 | Chips on the board, 2 or 1 (§2). |
| `COL_BITS` | 10 | Column bits of the chip, 10 or 9 (§2). Two chips only with 10. |
| `CMD_STAGE` | 1 | Command stage, §9. |
| `RD_PORTS`, `WR_PORTS` | all | Ports that read and that write. What a port never does is not built. |
| `URG_PORTS` | none | Ports that are always urgent: no age counter and no row-hit order are built for them. Tie their `urgent[g]` high: with it low their reads never age and are taken only in the pass for "the rest" (`SPEC_controller.md` §4.4, item 6), after every aged, full-buffer and row-hit choice. |

### Tuning

| Parameter | Default | Meaning |
|---|---|---|
| `WB_DEPTH` | 4 | Write buffer, in dwords. A power of two, ≥ 2. Deeper absorbs longer write bursts; it is built from registers, not block RAM. |
| `REF_MAX` | 4 | Refresh debt at which a refresh goes before everything else. Higher lets refreshes bunch up and get out of the way of bursts; the average interval is still honoured. Range 1 to 8. |
| `AGE_MAX` | 64 | Cycles a request may wait before it is served ahead of requests that would hit an already open row. This is the fairness limit: lower is fairer, higher is faster on average. It counts cycles, so at half the clock half the value gives the same time. Range 1 to 255. |

### Chip timings, in picoseconds

Defaults are the AS4C32M16SB, speed grade −7 (datasheet p.22, Table 16). The
AS4C16M16SA −7 has the same values; the W9825G6KH −6 is within them except for
the three timings it gives in clocks and the eight power-up refreshes (§2). Each
timing becomes `ceil(ps / period)` cycles, except `T_REFI_PS`, which is rounded
down so that the refresh is never late.

`T_RC_PS`, `T_RFC_PS`, `T_RCD_PS`, `T_RP_PS`, `T_RRD_PS`, `T_MRD_PS`,
`T_RAS_PS`, `T_WR_PS`, `T_REFI_PS`, `T_INIT_PS`, `INIT_REFS`.

`T_AC2_PS` (data access time with CL 2) and `T_OH_PS` (output hold) are used to
place the read capture edge, §8; `T_AC3_PS` (CL 3) is never used by a build that
runs (§7).

The CAS latency is **not** a parameter: it is 2 when the period allows it
(≥ 10 ns, so clocks up to 100 MHz). CL 3 is not supported (the read mask on
A11/A12 would fall in the wrong cycle, `SPEC_controller.md` §2.6 rule 4): a
simulation of a build whose clock would need it stops with an error.

### Pin delays — the ones you must measure

| Parameter | Meaning |
|---|---|
| `TCKO_SLOW_PS` | Rise of the SDRAM clock pin after the falling edge of `clk`, slow corner. |
| `TCKO_FAST_PS` | The same, fast corner. |
| `TSU_SLOW_PS`, `TH_SLOW_PS` | Input setup and hold of `sdram_dq` at the FPGA, slow corner. |
| `TSU_FAST_PS`, `TH_FAST_PS` | The same, fast corner. |
| `CAP_H_FORCE` | `-1` (computed). A value of 1 or more picks the capture half period by hand — for measurements only. |

These six delays describe **your board and your clock path**, not the chip.
Read them from your own compiled netlist (`bench/pin_datasheet.tcl` reads them
with the Quartus timing analyzer). The defaults are a measurement of one
particular path and are almost certainly wrong for yours.

---

## 8. The clock to the SDRAM — read this first

The controller does not produce `SDRAM_CLK`. You do, and **how you produce it
decides whether the design works at all** above roughly 70 MHz.

At elaboration the controller computes, from the datasheet (`tAC`, `tOH`) and
from `TCKO_*`, `TSU_*`, `TH_*`, the half period at which the read data is valid
**in both corners**. If no such half period exists, a simulation of the
controller stops with an error rather than let you build something that works
only at one temperature. The check is simulation code (Quartus does not see it),
so **run one simulation with your parameters**: the controller prints the
capture edge it chose (`sdrc: read capture half period ...`), or stops.

The trap: if you take `SDRAM_CLK` out through an ordinary output path, its delay
varies with the corner on its own — on one measured Cyclone V path, 3121 ps fast
against 8601 ps slow (the defaults of the pin delays). With those delays the
window stays open only up to about 70 MHz; from about 36 MHz up the capture is
on a falling edge, below that on a rising one. At 96 MHz the window closes and
the simulation stops.

**Generate the SDRAM clock from a dedicated PLL output**, with a phase shift.
Measured at the pins against the board clock input, its delay still changes
with the corner, but the setup and hold of the data pins change with it, and
the controller checks each corner with its own values. The controller itself
stops at 100 MHz, the fastest clock with CAS latency 2 (§7).

**How to read the six delays.** `bench/pin_datasheet.tcl` runs the Quartus
timing analyzer on your compiled netlist and prints the datasheet report of each
corner: the clock-to-output time of the `SDRAM_CLK` pin and the setup and hold
times of the `sdram_dq` pins, all referred to the same board clock input.
`TCKO_*` is the clock-to-output time of the rising edge of `SDRAM_CLK` minus half
a period of `clk` (the parameter counts from the falling edge of `clk`); `TSU_*`
and `TH_*` are the worst of the rise and fall values reported.

Worked example, measured on the compiled netlist of the Darius core (Cyclone V,
48 MHz): second PLL output at 48 MHz, phase shift **10417 ps** (half a period).
The analyzer reports a clock-to-output time of `SDRAM_CLK` of 24281 ps in the
slow corner and 17226 ps in the fast one, so `TCKO_SLOW_PS = 13864` and
`TCKO_FAST_PS = 6809`; setup and hold of `sdram_dq` give `TSU_SLOW_PS = -5743`,
`TH_SLOW_PS = 8036`, `TSU_FAST_PS = -2384`, `TH_FAST_PS = 3934`. The controller
picks `CAP_H = 5`, a falling edge, and the core runs on the board.

If the simulation stops with no valid window, the answer is not `CAP_H_FORCE`. It
is that your clock path is too loose, and `CAP_H_FORCE` would only hide it until
a board runs warm.

---

## 9. `CMD_STAGE`

`CMD_STAGE = 1` (default): every command is decided one cycle before it appears
on the pins, so the pin registers are driven by registers and nothing else. The
distances between commands are unchanged — they are the distances between the
decisions — so no chip timing is affected. The cost is **one more cycle of read
latency**.

`CMD_STAGE = 0`: the decided command reaches the pins on the edge that ends its
decision. No extra latency cycle.

Which one you want depends on whether the decision logic plus the wire to the
I/O cell fits in one period of your clock. Measured on a Cyclone V at 96 MHz,
with the pin registers packed in the I/O cells at the die edge, **the wire from
the core logic to the I/O cell alone was 4.4 to 5.5 ns** of a 10.4 ns period —
so it does not fit, and `CMD_STAGE = 1` is required there. At 48 MHz it fits,
and the latency cycle is free to give back.

The way to decide is to build once with `CMD_STAGE = 0` and look at the worst
path ending at the pin registers. If its slack is negative, set `CMD_STAGE` to 1.

---

## 10. What it does not do

- **One operation on the data bus at a time.** Banks are overlapped so that
  activations and prechargings hide under another operation's data, but this is
  not a pipelined multi-bank controller: the data bus carries one transfer at a
  time, which is a property of the memory, not of the controller.
- **No burst writes.** One dword per write request (`sdrc_job` streams a
  longer write as consecutive write requests, one halfword each).
- **No reordering of reads within a port.** Reads come back in the order asked.
- **No probing of the memory.** The board is chosen when you build (§2).
- **No error checking.** The chip is assumed present and healthy.

---

## 11. Verification

`bench/` holds the benches, a model of the chip (`sdr_chip.sv`) and of the
boards (`sdr_board.sv`). The chip model checks at the pins, in nanoseconds,
`tRCD`, `tRP`, `tRAS`, `tRC`, `tWR`, `tRRD`, `tMRD`, `tRFC`, `tCK`, setup and
hold of command, address and DQM, setup of write data (not its hold, nor
`tCH`/`tCL`), the refresh rate, the power-up sequence, the mode register and
contention on the data bus — and puts a poison value on the pins
outside the window in which read data is valid, so that a capture at the wrong
edge shows up as a data error. It is what makes a run meaningful rather than
merely green.

The benches:

- `tb_sdrc.sv` — one port: back-to-back writes, spaced writes, cache-line
  reads, single reads, mixed traffic;
- `tb_sdrc_mp.sv` — four ports with four traffic generators (a CPU with
  scattered writes, cache-line reads and frame blits; a real-time video reader
  with a FIFO that must never run dry; a DMA with bursts, read-back and long
  reads across rows; an audio port with a deadline), every halfword read
  checked against a shadow of the memory, the refresh count checked against the
  ticks, and, before the traffic, two directed tests of the ordering (a write to
  an address above the memory then a read of the cell it folds onto; a write in
  the third block of a read then that read);
- `tb_sdrc_job.sv` — two job ports on the controller, one that reads and writes
  and one that only reads, against a model of the job port: every word written
  checked as the controller takes it (address, data, byte selects), every word
  read checked with its `bnum`, `done` checked in every cycle, jobs started while
  another runs, replaced by a newer pulse, `rd` and `wr` together, reads longer
  than the port (in pieces), one-word jobs (`FIXED1`), and reads from the other
  port right after the `done` of a write.

```sh
bench/regress_sdrc.sh
```

builds and runs every configuration with Verilator and compares the result lines
with the reference in `bench/ref/`. Every run must end with `data_errors=0`,
`chip_errors=0`, `bench_errors=0` (and `protocol_errors=0`, `refresh_exact=1`,
`lost=0`, `underflows=0`, `audio_late=0` where the bench reports them). `VERILATOR`, `MAKE` and `PYTHON3` name the tools
if they are not `verilator`, `make` and `python3`.

### Figures

From the regression of this version, in cycles of the core clock, with the
default command stage (`CMD_STAGE = 1`). One port, `MAP 0`:

| | two chips, 48 MHz | two chips, 66.67 MHz | one chip (9 column bits, W9825G6KH timings), 48 MHz |
|---|---|---|---|
| back-to-back writes, cycles per dword | 2.00 | 2.00 | 2.00 |
| a write after 12 idle cycles, cycles until taken | 1.00 | 1.00 | 1.00 |
| read of 4 dwords, request to last halfword (average / max) | 15.21 / 21 | 15.17 / 23 | 15.28 / 21 |
| read of 1 dword, request to last halfword (average / max) | 9.11 / 13 | 9.08 / 17 | 9.10 / 13 |
| random mixed traffic over all the banks, cycles per operation | 4.08 | 4.07 | 4.94 |

A dword is two halfwords, and the bus carries one halfword per cycle: two
cycles per dword is the bus full.

Four ports, 300,000 cycles of traffic that keeps every port asking: the data bus
carried a READ or a WRITE in 82.4–83.9% of the cycles with two chips, 78.1–80.9%
with one chip and 9 column bits, 79.7–82.3% with one chip and 10; the video FIFO
never ran dry and the audio port never missed its deadline, in every
configuration.

---

## 12. How it was written

Clean room. The controller was designed from `SPEC_controller.md`, the
datasheets of the chips, the schematics of the boards and pin delays measured on
a compiled netlist — nothing else. No code of any other SDRAM controller was
opened or copied. The datasheets are not in this repository (they are third
party material); `SPEC_controller.md` names each one with its revision and cites
the table and page of every value used.

The choices that depend on traffic were made on the bench: the order in which
the scheduler picks the next operation was chosen with the four-port bench
(`SPEC_controller.md` §4.4), and the overlap of the banks was kept because, when
it was introduced, it took the random mixed traffic of the one-port bench
(`MIXED` in `tb_sdrc.sv`, two chips) from 5.44–5.47 to 4.07–4.08 cycles per
operation.

---

## 13. License

rmSdram is Copyright (c) 2026 Umberto Parisi (rmonic79). It is released under
the **GNU General Public License, version 3 or (at your option) any later
version**: the full text is in [LICENSE](LICENSE), and every source file carries
the same notice.

For use without the obligations of the GPL — for example in a closed-source
product — a **commercial license** is available from the author, who can also be
contacted for other licensing needs: rmonic79 on GitHub.
