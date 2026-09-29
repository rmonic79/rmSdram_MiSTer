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

// Driver of the one-port bench: it only advances time, with the Verilator 5
// loop for --timing designs (the testbench makes its own clock).
#include "verilated.h"
#include "Vtb_sdrc.h"

// The mingw build of the Verilator runtime still links the legacy time hook.
double sc_time_stamp() { return 0; }

int main(int argc, char** argv) {
    VerilatedContext* ctx = new VerilatedContext;
    ctx->commandArgs(argc, argv);
    Vtb_sdrc* top = new Vtb_sdrc{ctx};
    while (!ctx->gotFinish()) {
        top->eval();
        if (!top->eventsPending()) break;
        ctx->time(top->nextTimeSlot());
    }
    top->final();
    delete top;
    delete ctx;
    return 0;
}
