# rmSdram — controller specification (clean room)

A standalone controller, reusable in every core. It is designed **only** from
this specification, from the datasheets of the chips, from the schematics of the
boards and from measurements on the bench. No code of any other controller is
opened or copied.

The datasheets and schematics are not in the repository (third-party material).
Each is named below with its revision, and every value taken from it with its
table and page, so that it can be checked.

## 1. Principles

1. **One clock.** Controller, ports and writes run on the core's clock. No
   clock-domain crossing inside the controller, no extra PLL for the logic,
   nothing in the SDC.
2. **Any clock up to 100 MHz.** Every chip timing is a parameter in
   picoseconds and becomes cycles of the clock: `cycles = ceil(ps / period)`
   (`floor` for the refresh interval, so that it is never late). The CAS latency
   is 2, which the datasheet allows up to 100 MHz (§2.2).
3. **The board is a parameter.** The same source drives the 128 MB board with
   both chips, or one chip with 9 column bits, which runs on every board (§2.8).
4. **Every choice is decided with a measurement**, written down first together
   with what will be decided from its result.

## 2. Hardware

### 2.1 Sources

| Source | Document |
|---|---|
| AS4C32M16SB | Alliance Memory, "AS4C32M16SB" B die, Rev 1.4, June 2024 |
| AS4C16M16SA | Alliance Memory, "AS4C16M16SA-C&I", Rev. 3.0, March 2015 |
| W9825G6KH | Winbond, "W9825G6KH", Revision A04, March 20, 2017 |
| 128 MB board | "SDRAM board (dual-chip) for MiSTer (extra slim)", rev. 3.0 (3-04-2023) and rev. 2.9 (21-04-2021), MiSTer-devel/Hardware_MiSTer, releases (`sdram_xsds_3.0.pdf`, `sdram_xsds_2.9.pdf`) |
| 32 MB board | "SDRAM board for MiSTer (extra slim)", rev. 2.2 (06.09.2019), MiSTer-devel/Hardware_MiSTer, releases (`sdram_xs_2.2.pdf`); its chip from the bill of materials in the MiSTer wiki, "SDRAM Board Assembly (DIY)": AS4C16M16SA-6TCN or W9825G6KH-6 |
| pin delays | `report_datasheet` of a compiled netlist on the DE10-nano, read with `bench/pin_datasheet.tcl` |

### 2.2 AS4C32M16SB

**Geometry** (p. 2 "Overview", p. 6 Table 3): per chip 4 banks × 8M words ×
16 bits = 64 MB; row A0-A12 (8192 rows); column A0-A9 (1024 columns); A10 in a
column command = auto precharge. Table 8 (p. 16) writes "full page 0-511": that
contradicts A0-A9 and 8M words per bank; Table 3 holds.

**Timings** (p. 22, Table 16; minimum unless written):

| Symbol | -6 | -7 | Notes |
|---|---|---|---|
| tRC | 60 ns | 63 ns | |
| tRFC | 60 ns | 63 ns | duration of an AUTO REFRESH (p. 18: "tRC(min)") |
| tRCD | 18 ns | 21 ns | |
| tRP | 18 ns | 21 ns | also between two AUTO REFRESH (p. 18) |
| tRRD | 12 ns | 14 ns | |
| tMRD | 12 ns | 14 ns | |
| tRAS | 42 ns (max 120K) | 42 ns (max 120K) | |
| tWR | 12 ns | 14 ns | |
| tCK with CL=2 | 10 ns | 10 ns | **CL=2 holds up to 100 MHz** |
| tCK with CL=3 | 6 ns | 7 ns | |
| tAC (max), CL=2 / CL=3 | 6 / 5 ns | 6 / 5.4 ns | |
| tOH | 2.5 ns | 2.5 ns | |
| tHZ (max) | 5 ns | 5.4 ns | |
| tIS / tIH | 1.5 / 0.8 ns | 1.5 / 0.8 ns | |
| tREFI (max) | 7.8 µs | 7.8 µs | |

**Refresh** (p. 18): 8192 AUTO REFRESH every 64 ms, all banks closed, CKE high;
tRP before another AUTO REFRESH.

**Power-up** (p. 23, Note 11): power with CKE low, DQM high, NOP; stable clock
for at least 200 µs, then CKE high (DQM high recommended); PRECHARGE of all
banks; MODE REGISTER SET; **at least 2 AUTO REFRESH**, before or after the MODE
REGISTER SET.

**Mode register** (p. 16): sequential burst of 1, 2, 4, 8 or full page
(Tables 7-8); CAS latency 2 or 3 (Table 9).

The defaults of the controller are the -7 values (the slower of the two grades).

### 2.3 AS4C16M16SA

**Geometry** (p. 4, pin description): row address A0-A12 (8192 rows), column
address A0-A8 (512 columns, full page 512, p. 13), 4 banks, 16 bits: 32 MB.

**Timings** (p. 19, Table 16): the same values as the AS4C32M16SB above, for
both grades (-6: tRC = tRFC 60, tRCD 18, tRP 18, tRRD 12, tMRD 12, tRAS 42, tWR 12;
-7: 63, 21, 21, 14, 14, 42, 14 ns; tCK(CL=2) 10 ns; tAC(CL=2) 6 ns; tOH 2.5 ns;
tIS / tIH 1.5 / 0.8 ns).

**Refresh** (p. 16): 8192 AUTO REFRESH within 64 ms, duration tRC(min).

**Power-up** (p. 20, Note 11): the same sequence as the AS4C32M16SB, **at least
2 AUTO REFRESH**.

### 2.4 W9825G6KH

**Geometry** (p. 5, pin description; p. 6): row address A0-A12, column address
A0-A8; cell array 8192 × 512 × 16 per bank, 4 banks: 32 MB.

**Timings** (p. 15, 9.5 AC Characteristics), grade -6:

| Symbol | -6 | Notes |
|---|---|---|
| tRC | 60 ns | "Ref/Active to Ref/Active Command Period": also the AUTO REFRESH |
| tRAS | 42 ns (max 100000 ns) | |
| tRCD | 15 ns | |
| tRP | 15 ns | |
| tRRD | **2 tCK** | |
| tWR | **2 tCK** | CL 2 and 3 |
| tRSC (tMRD) | **2 tCK** | |
| tCK with CL=2 / CL=3 | 7.5 / 6 ns | |
| tAC (max), CL=2 / CL=3 | 6 / 5 ns | |
| tOH | 3 ns | |
| tREF | 64 ms, 8K refresh cycles | |

**Power-up** (p. 7, 7.1): pause of 200 µs with DQM and CKE high, PRECHARGE of
all banks, MODE REGISTER SET, and **eight AUTO REFRESH**, before or after it.

Every timing of the -6 in nanoseconds is within the -7 defaults of the
controller. Three are given in clocks (tRRD, tWR, tMRD: 2 tCK) and the power-up
wants eight refreshes: a build that may meet this chip passes `T_RRD_PS`,
`T_WR_PS` and `T_MRD_PS` of at least two periods and `INIT_REFS = 8` (§2.8).

### 2.5 Cycles at the bench clocks

What the controller derives from the -7 defaults (`ceil(ps / period)`, the
period in whole picoseconds; tREFI with `floor`):

| | 48 MHz (20833 ps) | 66.67 MHz (14999 ps) | 96 MHz (10416 ps) |
|---|---|---|---|
| CL | 2 | 2 | 2 |
| tRCD, tRP | 2 | 2 | 3 |
| tRAS | 3 | 3 | 5 |
| tRC = tRFC | 4 | 5 | 7 |
| tRRD, tMRD, tWR | 1 | 1 | 2 |
| the same, two clocks (W9825G6KH) | 2 | 2 | 2 |
| tREFI | 374 | 520 | 748 |
| power-up (200 µs) | 9601 | 13335 | 19202 |

### 2.6 The 128 MB board (sdram_xsds 2.9 / 3.0)

The two revisions have the same logical wiring (CS, DQM, CKE, connector); they
can differ only in the layout. From the schematic (p. 1):

- Two AS4C32M16SB (U1, U2) share A0-A12, BA0-BA1, DQ0-DQ15, CLK, RAS#, CAS#,
  WE#.
- **CS:** `CS1` (the FPGA's `nCS`) goes to U1; `CS2 = NOT CS1` (inverter U3,
  LVC1G04) goes to U2. One chip is always selected, and `nCS` acts as the address
  bit that picks the chip.
- **DQM:** DQML = **A11**, DQMH = **A12**, on both chips. The DQM pins of the
  FPGA are not connected.
- **CKE** tied to VCC on both chips.

The controller sees 4 banks of 32 MB, each made of the two chips' banks with the
same BA, chosen by the CS. Rules that follow:

1. **Eight independent open rows** (2 chips × 4 banks).
2. **Refresh per chip:** each chip wants 8192 AUTO REFRESH every 64 ms with its
   four banks closed. One chip can be refreshed while the other works.
3. **Every command goes to one chip.** "No command" is the NOP (RAS#, CAS#, WE#
   high), not the deselect: the CS cannot switch both chips off.
4. **Byte masks on A11/A12.** A write word is masked by DQM in its own cycle
   (zero clock latency, AS4C32M16SB p. 39-40, Figures 31.1-31.2; the two-clock
   read latency is on p. 10), so the WRITE command carries the mask of its
   word on A12/A11. A read word is masked by DQM sampled two cycles before it:
   with CL 2 and burst length 1 that is the cycle of its own READ, which carries
   A12/A11 = 00. So an ACTIVE, whose row uses A11/A12, never masks a word. (CL 3
   would move the read mask to the cycle after the READ: it is not supported, and
   a simulation of the controller stops if the clock would need it.)
5. **No power-down, no self refresh** (CKE tied high).
6. **Power-up:** CKE is already high. During the 200 µs the pins carry NOP with
   A11/A12 high (DQM high, data pins in high impedance, as Note 11 recommends);
   then PRECHARGE ALL, the AUTO REFRESH and the MODE REGISTER SET, **on each of
   the two chips**.

### 2.7 The 32 MB board (sdram_xs 2.2)

From the schematic (sheet 1): one chip (U1: AS4C16M16SA or W9825G6KH, as the
bill of materials of the board allows) on A0-A12, BA0-BA1, DQ0-DQ15, CLK, RAS#,
CAS#, WE#; **CS# directly from the FPGA's `nCS`**; DQML (pin 15) = **A11**,
DQMH (pin 39) = **A12**; **CKE** tied high. Rules 3 to 6 of §2.6 hold, for one
chip.

### 2.8 What the controller drives

| `N_CHIPS` | `COL_BITS` | memory | banks with an open row |
|---|---|---|---|
| 2 | 10 | 128 MB, the board of §2.6 | 8 |
| 1 | 10 | 64 MB, one AS4C32M16SB-class chip | 4 |
| 1 | 9 | 32 MB: the board of §2.7, and the board of §2.6 (U1) | 4 |

With one chip `nCS` stays low. On the 32 MB board that selects the chip; on the
128 MB board it selects U1, and U2, never selected, is never powered up nor
refreshed, which is allowed: a chip that sees only deselects does nothing. So
one chip with 9 column bits runs on every board. The power-up and the refresh
are those of that chip alone. Two chips exist only with 10 column bits.

A one-chip build meant for every board must also meet the W9825G6KH (§2.4):
`T_RRD_PS`, `T_WR_PS`, `T_MRD_PS` of at least two periods and `INIT_REFS = 8`.

### 2.9 The clock to the chips

`SDRAM_CLK` is not made by the controller: it comes from a dedicated output of
the PLL, with a phase shift. Its delay at the pin still changes with the corner
(on the Darius build at 48 MHz, 7 ns between slow and fast, measured against the
board clock input), and so do the setup and hold of the data pins. The
controller is told that delay (`TCKO_SLOW_PS`, `TCKO_FAST_PS`, counted from
the falling edge of `clk`) and the input setup and hold of the data pins
(`TSU_*`, `TH_*`), all read from the same netlist against the same reference,
and places the capture of the read data from them, checking each corner (§4.7).
Through an ordinary output path, with the delays measured on one Cyclone V path
(`SDRAM_CLK` 3121 ps fast, 8601 ps slow: the defaults), no capture edge is valid
in both corners above about 70 MHz.

## 3. Parameters

| Parameter | Meaning |
|---|---|
| `CLK_KHZ` | clock of the core |
| `N_PORTS`, `LEN_BITS` | number of ports; reads of 1 .. 2^`LEN_BITS`-1 halfwords, `LEN_BITS` ≤ `COL_BITS` + 1 |
| `MAP` | address map, one for all the ports (§4.1) |
| `N_CHIPS`, `COL_BITS` | the board (§2.8) |
| `WB_DEPTH` | write buffer, dwords (power of two ≥ 2) |
| `REF_MAX` | refresh debt at which the refresh goes first (§4.5) |
| `AGE_MAX` | cycles a request waits before it goes ahead of the open rows (§4.4) |
| `T_RC_PS` … `T_WR_PS`, `T_REFI_PS`, `T_INIT_PS`, `INIT_REFS` | chip timings in ps (-7 defaults), power-up wait and refreshes |
| `T_AC2_PS`, `T_OH_PS` (`T_AC3_PS` unused: CL 3 is not supported) | read data access and hold, for the capture edge |
| `TCKO_*`, `TSU_*`, `TH_*`, `CAP_H_FORCE` | pin delays of the build, capture edge by hand (measurement only) |
| `CMD_STAGE` | 1: commands decided one cycle before the pins (§4.7) |
| `RD_PORTS`, `WR_PORTS`, `URG_PORTS` | what each port does: what a port never does is not built |

The CAS latency is not a parameter: 2 when the period is at least 10 ns
(Table 16); a clock that would need CL 3 is not supported (§2.6 rule 4). The mode register is fixed: burst length 1, sequential.

## 4. Structure

```
 ports ──► port intake ──► write buffer ──────────────┐
 (N)       (taken in one   (ports that write)          │
            cycle)         read slots (ports that read)▼
                                              next operation (§4.4)
         refresh debt ───────────────────────►  current operation  ◄── bank machines
         power-up sequence ──────────────────►  + bank preparation      (open row, histories)
                                                       │ one command per cycle
                                              pin registers (NOP / DQM high at start)
                                                       │
                       read data ◄─────────────────────┘ (capture edge, §4.7)
```

### 4.1 Port intake and address map

- Handshake: a request is taken in the cycle in which `req` and `port_ready`
  are both high.
- Write: one dword at an even halfword address, with four byte enables, into
  the write buffer; the port is free at once. Read: `len` halfwords from any
  halfword address into the read slot of the port (one per port).
- **One map for all the ports** (`MAP`). With one map per port, two ports that
  share memory (the CPU writing VRAM, the video reading it) would see the same
  address in different cells. To put streams in different banks, choose the
  regions: with `MAP 0` the high address bits are the bank.
  - two chips: `MAP 0` bank h[25:24], chip h[23], row h[22:10], column h[9:0];
    `MAP 1` bank h[12:11], chip h[10], row h[25:13], column h[9:0];
  - one chip, C = `COL_BITS`: `MAP 0` bank h[C+14:C+13], row h[C+12:C],
    column h[C-1:0]; `MAP 1` bank h[C+1:C], row h[C+14:C+2], column h[C-1:0].
- A **block** is the 2^`COL_BITS` halfwords that share the address bits above the
  column (one row of one bank). The column is always the low `COL_BITS`: a burst that runs past
  the last column goes on in the next block, mapped again.
- With one chip the bits above the memory (h[25] with 10 column bits, h[25:24]
  with 9) are not used: those addresses fold onto the memory.
- Read data: `rdata` + `rvalid[port]`, always in the order of the requests of
  the same port.

### 4.2 Write buffer and ordering

- `WB_DEPTH` dwords, registers (not block RAM). Writes of several ports in the
  same cycle are taken one at a time, in turn.
- **A read never sees old data of a write taken before it.** A read waits for
  the writes taken before it when one of them touches a block the read touches;
  the others do not stop it. A read touches its first block, the next one when
  column + length runs past the last column, and (reads longer than a block,
  `LEN_BITS` = `COL_BITS` + 1) the one after when it runs past two: each buffer
  entry is compared as it is, minus one and minus two, on the block bits the map
  uses (with one chip the top bit of the block is above the memory and is left
  out, so an address above the memory meets the writes to the cell it folds
  onto).
- In the same port, a write is not taken while the read of that port is still
  in its slot.
- Between different ports there is no order when the write is taken in the same
  cycle as the read or after it.
- Decision: the read **waits**; the data is not forwarded from the buffer.

### 4.3 Bank machines

One per bank of each chip: 8 with two chips, 4 with one. State: closed, or row R
open. A row stays open until another row of the same bank is needed or the
refresh of its chip closes it. Each machine keeps the history of its last
ACTIVE (tRCD, tRAS, tRC), PRECHARGE (tRP) and WRITE (tWR); each chip the history
of its last ACTIVE (tRRD) and AUTO REFRESH (tRFC). A history of K-1 bits holds a
constraint of K cycles.

### 4.4 Scheduler

- **One operation on the data bus at a time**: a read of `len` halfwords, a
  write of one dword (one or two halfwords), or the refresh of one chip. The next
  operation is loaded in the cycle the current one issues its last command, with
  no idle cycle between.
- An operation issues, in turn: PRECHARGE (another row open), ACTIVE, then one
  READ or WRITE per halfword (burst length 1), each when the histories allow it.
  A WRITE waits for the data of the last READ to leave the pins.
- **Banks overlapped**: in every cycle in which the current operation leaves the
  command bus free, the candidate chosen in the cycle before gets its bank
  prepared (PRECHARGE of another row, ACTIVE of its row), or the banks of the
  chip of a pending refresh get closed. The current operation always goes first
  and is never delayed by a preparation; a preparation is never an ACTIVE that
  would delay the ACTIVE of the current operation (tRRD). The candidate is
  registered for one cycle, so that no path runs from the choice to the pins in
  one cycle; a stale choice costs nothing, because what the bank needs is checked
  again in the cycle the command goes out.
- **Order of the next operation**, chosen with the four-port bench:
  1. refresh with the debt at `REF_MAX`;
  2. reads of ports with `urgent`;
  3. aged reads (`AGE_MAX` cycles in the slot), then the aged head of the write
     buffer;
  4. full write buffer;
  5. reads that find their row open, then the head of the buffer if it does;
  6. the rest (reads first);
  7. refresh in the cycles with nothing else to do.

  With "full buffer" above the aged requests, during a CPU blit (600 writes in a
  row) a DMA read waited 2615 cycles. With the order above, at 66.67 MHz with two chips,
  the longest wait in a slot was 116–169 cycles for the best-effort ports (CPU,
  DMA) and 117–123 for the video (`bench/ref/tb_sdrc_mp.txt`). The bound is `AGE_MAX` + the operation in progress + the aged
  requests ahead: with 4 ports and reads of 63 halfwords, about 480 cycles.

### 4.5 Refresh

- A counter of `T_REFI_PS` (in cycles, rounded down) adds one to the debt of every chip; the debt goes down
  when the refresh is given to the engine, not when its command leaves, so the
  same debt is never paid twice.
- The refresh is done in the cycles with nothing else to do, and forced only at
  debt `REF_MAX` (4 by default; the datasheet gives the average, 8192 per
  64 ms).
- A refresh closes the open banks of its chip (one bank alone, more with
  PRECHARGE ALL, which touches every bank of the chip and so waits tRP for all),
  then issues AUTO REFRESH.
- **Never a filler refresh**: the refreshes done must equal those due (the
  four-port bench checks it: `refresh_exact`). With one chip there is no debt for
  the second.

### 4.6 Power-up

Wait `T_INIT_PS` (200 µs) with NOP and DQM high, also without a reset; PRECHARGE ALL
on each chip; `INIT_REFS` AUTO REFRESH on each chip, tRFC apart per chip; MODE
REGISTER SET on each chip (CL, burst length 1, sequential); tMRD; then `ready`.
With two chips each step goes to U1 then to U2 in the next cycle.

### 4.7 PHY

- Every output to the chips leaves from registers that **start as a NOP with DQM
  high**: with the CS that selects a chip, registers at zero at power-up would be
  a MODE REGISTER SET.
- **Command stage** (`CMD_STAGE = 1`): every command is decided one cycle before
  it is on the pins, so the pin registers are fed by registers only; the
  distances between commands, and so every chip timing, are those of the
  decisions; one more cycle of read latency. `CMD_STAGE = 0` puts the command on
  the pins at the edge that ends its decision, for clocks where the logic before
  the pins fits in one cycle (measured on a Cyclone V at 96 MHz: the wire from
  the core logic to the I/O cell alone was 4.4–5.5 ns of 10.4 ns, so the stage is
  needed there; at 48 MHz it is not).
- **Capture edge of the read data**, chosen at elaboration: the READ loaded at
  a rising edge reaches the chips at the next rise of `SDRAM_CLK` (half a period
  + `TCKO_*`); its word is valid at the pins from CL-1 edges later + tAC to CL edges
  later + tOH. The capture is the first half period, counted from the edge that
  loads the READ, inside the window valid in **both** corners after the setup and
  hold of the input pins. If there is none, a simulation of the controller stops
  (the check is simulation code: a design is simulated once with its
  parameters, and the controller prints the edge it chose). A falling edge
  is taken by a register on the falling edge and moved to the core on the next
  rising edge.
- Read latency = command stage + CL + capture: fixed for a given build.

### 4.8 Port classes

- **Real time** (video, audio): `urgent`, for example the FIFO of the reader
  below a threshold, puts its read first, after only a refresh forced at
  `REF_MAX` (§4.4). It must never run dry: the bench checks it.
- **Best effort** (CPU, DMA, ROM loading): served in the free cycles, with
  aging.

## 5. Decisions

1. **Burst length 1** in the mode register, one column command per halfword:
   any length from any address, no burst terminate, and the read DQM always
   falls in the cycle of its own READ (rule 4 of §2.6).
2. **Rows stay open** until another row of the bank or a refresh needs the bank.
3. **One write buffer for all the ports**, depth `WB_DEPTH`; a read that meets a
   write in it **waits**, nothing is forwarded.
4. **The order of §4.4**, from the measurement written there.
5. **Banks overlapped**: when it was introduced, the one-port bench (`MIXED`,
   two chips) measured random mixed accesses going from 5.44–5.47 to 4.07–4.08
   cycles per operation, write
   figures unchanged and single reads one cycle longer (the command stage).

## 6. Verification

- **Model of the chip in nanoseconds** (`bench/sdr_chip.sv`), from the
  datasheets: the timings tRCD, tRP, tRAS, tRC, tWR, tRRD, tMRD, tRFC, tRAS max
  and tCK, setup and hold of command, address and DQM, and setup of write data,
  at the pins (not the hold of write data, nor tCH/tCL), the refresh
  rate with the debt the specification allows, the power-up sequence (including
  the number of refreshes the chip wants, 2 or 8) and that no access comes before
  it, the mode register, contention on the data bus; the column width of the chip
  (`COL_BITS`: the column bits it does not have are ignored, as by the chip).
  Read data is held from tAC to tOH and a poison value is on the pins outside
  that window, so a capture at the wrong edge is a data error.
- **Model of the boards** (`bench/sdr_board.sv`): the wiring of §2.6 and §2.7.
- **Pin delays** of the compiled netlist on every pin, in both corners.
- **Scoreboard**: every halfword read is compared with a shadow of the memory,
  byte enables included; addresses above the memory are compared as the cells
  they fold onto.
- **Traffic**: one port (back-to-back writes, spaced writes, cache-line reads,
  single reads, random mixed traffic over all the banks); four ports (a CPU with
  scattered writes, cache-line reads, frame blits and a mailbox; a real-time video
  reader with a deadline; a DMA with bursts, read-back and long reads across
  rows; an audio port with a deadline).
- **Directed tests of the ordering**: a write to an address above the memory,
  the engine kept busy so that the write stays in the buffer, then a read of the
  cell it folds onto; a write in the third block of a long read, then that read.
  Each test was checked to fail on a controller without the corresponding
  comparison; the power-up check of the chip model was checked to fail with two
  refreshes on the W9825G6KH.
- **Job port** (`bench/tb_sdrc_job.sv`): two `sdrc_job` ports (one reads and
  writes, one only reads) against a model of the job port: every word written
  checked as the controller takes it, every word read with its `bnum`, `done` and
  `busy` in every cycle, pending and replaced jobs, `rd` and `wr` together, reads
  split into pieces, one-word jobs, reads from the other port right after the
  `done` of a write; a run in which nothing moves for too long ends with an error.
  The bench was checked to fail on three changed adapters (byte selects swapped,
  type of the pending job inverted, `done` of a read missing).
- **Regression** (`bench/regress_sdrc.sh`): every configuration (one and two
  chips, 9 and 10 column bits, both maps, 48 and 66.67 MHz, both corners, both
  command stages, the W9825G6KH timings, reads up to 2047 halfwords, the job port
  with jobs longer and shorter than the port and one-word jobs); the result lines
  must equal those in `bench/ref/`, and every run must end without errors.
