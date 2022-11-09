/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2018  gatecat <gatecat@ds0.me>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "cells.h"
#include "design_utils.h"
#include "log.h"
#include "nextpnr.h"
#include "timing.h"
#include "util.h"
NEXTPNR_NAMESPACE_BEGIN

inline NetInfo *port_or_nullptr(const CellInfo *cell, IdString name)
{
    auto found = cell->ports.find(name);
    if (found == cell->ports.end())
        return nullptr;
    return found->second.net;
}

bool Arch::slices_compatible(LogicTileStatus *lts) const
{
    if (lts == nullptr)
        return true;
    for (int sl = 0; sl < 4; sl++) {
        if (!lts->slices[sl].dirty) {
            if (!lts->slices[sl].valid)
                return false;
            continue;
        }
        lts->slices[sl].dirty = false;
        lts->slices[sl].valid = false;
        bool found_ff = false;
        uint8_t last_ff_flags = 0;
        IdString last_ce_sig;
        bool ramw_used = false;
        if (sl == 2 && lts->cells[((sl * 2) << lc_idx_shift) | BEL_RAMW] != nullptr)
            ramw_used = true;
        for (int l = 0; l < 2; l++) {
            bool comb_m_used = false;
            CellInfo *comb = lts->cells[((sl * 2 + l) << lc_idx_shift) | BEL_COMB];
            if (comb != nullptr) {
                uint8_t flags = comb->combInfo.flags;
                if (ramw_used && !(flags & ArchCellInfo::COMB_RAMW_BLOCK))
                    return false;
                if (flags & ArchCellInfo::COMB_MUX5) {
                    // MUX5 uses M signal and must be in LC 0
                    comb_m_used = true;
                    if (l != 0)
                        return false;
                }
                if (flags & ArchCellInfo::COMB_MUX6) {
                    // MUX6+ uses M signal and must be in LC 1
                    comb_m_used = true;
                    if (l != 1)
                        return false;
                    if (comb->combInfo.mux_fxad != nullptr &&
                        (comb->combInfo.mux_fxad->combInfo.flags & ArchCellInfo::COMB_MUX5)) {
                        // LUT6 structure must be rooted at SLICE 0 or 2
                        if (sl != 0 && sl != 2)
                            return false;
                    }
                }
                // LUTRAM must be in bottom two SLICEs only
                if ((flags & ArchCellInfo::COMB_LUTRAM) && (sl > 1))
                    return false;
                if (l == 1) {
                    // Carry usage must be the same for LCs 0 and 1 in a SLICE
                    CellInfo *comb0 = lts->cells[((sl * 2 + 0) << lc_idx_shift) | BEL_COMB];
                    if (comb0 &&
                        ((comb0->combInfo.flags & ArchCellInfo::COMB_CARRY) != (flags & ArchCellInfo::COMB_CARRY)))
                        return false;
                }
            }

            CellInfo *ff = lts->cells[((sl * 2 + l) << lc_idx_shift) | BEL_FF];
            if (ff != nullptr) {
                uint8_t flags = ff->ffInfo.flags;
                if (comb_m_used && (flags & ArchCellInfo::FF_M_USED))
                    return false;
                if (found_ff) {
                    if ((flags & ArchCellInfo::FF_GSREN) != (last_ff_flags & ArchCellInfo::FF_GSREN))
                        return false;
                    if ((flags & ArchCellInfo::FF_CECONST) != (last_ff_flags & ArchCellInfo::FF_CECONST))
                        return false;
                    if ((flags & ArchCellInfo::FF_CEINV) != (last_ff_flags & ArchCellInfo::FF_CEINV))
                        return false;
                    if (ff->ffInfo.ce_sig != last_ce_sig)
                        return false;
                } else {
                    found_ff = true;
                    last_ff_flags = flags;
                    last_ce_sig = ff->ffInfo.ce_sig;
                }
            }
        }

        lts->slices[sl].valid = true;
    }
    if (lts->tile_dirty) {
        bool found_global_ff = false;
        bool found_global_dpram = false;
        bool global_lsrinv = false;
        bool global_clkinv = false;
        bool global_async = false;

        IdString clk_sig, lsr_sig;

        lts->tile_dirty = false;
        lts->tile_valid = false;

#define CHECK_EQUAL(x, y)                                                                                              \
    do {                                                                                                               \
        if ((x) != (y))                                                                                                \
            return false;                                                                                              \
    } while (0)
        for (int i = 0; i < 8; i++) {
            if (i < 4) {
                // DPRAM
                CellInfo *comb = lts->cells[(i << lc_idx_shift) | BEL_COMB];
                if (comb != nullptr && (comb->combInfo.flags & ArchCellInfo::COMB_LUTRAM)) {
                    if (found_global_dpram) {
                        CHECK_EQUAL(bool(comb->combInfo.flags & ArchCellInfo::COMB_RAM_WCKINV), global_clkinv);
                        CHECK_EQUAL(bool(comb->combInfo.flags & ArchCellInfo::COMB_RAM_WREINV), global_lsrinv);
                    } else {
                        global_clkinv = bool(comb->combInfo.flags & ArchCellInfo::COMB_RAM_WCKINV);
                        global_lsrinv = bool(comb->combInfo.flags & ArchCellInfo::COMB_RAM_WREINV);
                        found_global_dpram = true;
                    }
                }
            }
            // FF
            CellInfo *ff = lts->cells[(i << lc_idx_shift) | BEL_FF];
            if (ff != nullptr) {
                if (found_global_dpram) {
                    CHECK_EQUAL(bool(ff->ffInfo.flags & ArchCellInfo::FF_CLKINV), global_clkinv);
                    CHECK_EQUAL(bool(ff->ffInfo.flags & ArchCellInfo::FF_LSRINV), global_lsrinv);
                }
                if (found_global_ff) {
                    CHECK_EQUAL(ff->ffInfo.clk_sig, clk_sig);
                    CHECK_EQUAL(ff->ffInfo.lsr_sig, lsr_sig);
                    CHECK_EQUAL(bool(ff->ffInfo.flags & ArchCellInfo::FF_CLKINV), global_clkinv);
                    CHECK_EQUAL(bool(ff->ffInfo.flags & ArchCellInfo::FF_LSRINV), global_lsrinv);
                    CHECK_EQUAL(bool(ff->ffInfo.flags & ArchCellInfo::FF_ASYNC), global_async);

                } else {
                    clk_sig = ff->ffInfo.clk_sig;
                    lsr_sig = ff->ffInfo.lsr_sig;
                    global_clkinv = bool(ff->ffInfo.flags & ArchCellInfo::FF_CLKINV);
                    global_lsrinv = bool(ff->ffInfo.flags & ArchCellInfo::FF_LSRINV);
                    global_async = bool(ff->ffInfo.flags & ArchCellInfo::FF_ASYNC);
                    found_global_ff = true;
                }
            }
        }
#undef CHECK_EQUAL
        lts->tile_valid = true;
    } else {
        if (!lts->tile_valid)
            return false;
    }

    return true;
}

bool Arch::isBelLocationValid(BelId bel) const
{
    IdString bel_type = getBelType(bel);
    if (bel_type.in(id_TRELLIS_COMB, id_TRELLIS_FF, id_TRELLIS_RAMW)) {
        return slices_compatible(tile_status.at(tile_index(bel)).lts);
    } else {
        CellInfo *cell = getBoundBelCell(bel);
        if (cell == nullptr) {
            return true;
        } else if (cell->type.in(id_DCUA, id_EXTREFB, id_PCSCLKDIV)) {
            return args.type != ArchArgs::LFE5U_25F && args.type != ArchArgs::LFE5U_45F &&
                   args.type != ArchArgs::LFE5U_85F;
        } else if (cell->type.in(id_MULT18X18D, id_ALU54B)) {
            return is_dsp_location_valid(cell);
        } else {
            return true;
        }
    }
}

// Check if this DSP bel configuration would result in more than four
// distinct clock/ce/rst per block of two DSP slices.
bool Arch::is_dsp_location_valid(CellInfo* cell) const
{
    auto ctx = getCtx();
    BelId bel = cell->bel;

    int block_x = cell->getLocation().x - cell->getLocation().z;
    int y = bel.location.y;

    /*
    log_info("DSP bel: x=%d y=%d x0=%d\n", bel.location.x, bel.location.y, block_x);
    log_info("DSP cell: name=%s type=%s\n", cell->name.c_str(ctx), cell->type.c_str(ctx));
    */

    std::array<IdString, 12> ports = {
        id_CLK0, id_CLK1, id_CLK2, id_CLK3,
        id_CE0, id_CE1, id_CE2, id_CE3,
        id_RST0, id_RST1, id_RST2, id_RST3
    };

    // Store the first net found for each shared port,
    // and the cell that is connected to it.
    NetInfo *block_ports[ports.size()] = {};
    CellInfo *block_cells[ports.size()] = {};

    // Check what nets are connected to each MULT18X18D or ALU54B.
    for (auto dx : {0, 1, 3, 4, 5, 7}) {
        BelId bel_dsp = getBelByLocation(Loc(block_x + dx, y, dx));
        if(bel_dsp == BelId()) continue;
        CellInfo* cell_dsp = getBoundBelCell(bel_dsp);
        if(cell_dsp == nullptr) continue;

        for (size_t i = 0; i < ports.size(); i++) {
            IdString port = ports[i];
            NetInfo *net = cell_dsp->ports.at(port).net;
            if(net == nullptr) continue;
            if(block_ports[i] == nullptr) {
                block_ports[i] = net;
                block_cells[i] = cell_dsp;
                continue;
            } else if(net != block_ports[i]) {
                // If the conflicting cells are both locked or user-placed,
                // there's no way the placer can resolve this conflict,
                // so display a useful error.
                if(cell_dsp->belStrength >= STRENGTH_LOCKED &&
                   block_cells[i]->belStrength >= STRENGTH_LOCKED)
                {
                    log_error("DSP block signal %s cannot be used for net '%s'"
                              " in %s '%s', as it is already connected to net "
                              "'%s' in %s '%s'.\n",
                              port.c_str(ctx),
                              net->name.c_str(ctx),
                              cell_dsp->type.c_str(ctx),
                              cell_dsp->name.c_str(ctx),
                              block_ports[i]->name.c_str(ctx),
                              block_cells[i]->type.c_str(ctx),
                              block_cells[i]->name.c_str(ctx)
                    );
                }
                return false;
            }
        }
    }

    /*
    BelId bel_m0 = getBelByLocation(Loc(block_x + 0, y, 0));
    BelId bel_m1 = getBelByLocation(Loc(block_x + 1, y, 1));
    BelId bel_a3 = getBelByLocation(Loc(block_x + 3, y, 3));
    BelId bel_m4 = getBelByLocation(Loc(block_x + 4, y, 4));
    BelId bel_m5 = getBelByLocation(Loc(block_x + 5, y, 5));
    BelId bel_a7 = getBelByLocation(Loc(block_x + 7, y, 7));
    if(bel_m0 != BelId()) {
        CellInfo* cell_m0 = getBoundBelCell(bel_m0);
        if(cell_m0) log_info("    m0: x=%d idx=%d name=%s type=%s\n", bel_m0.location.x, bel_m0.index, cell_m0->name.c_str(ctx), cell_m0->type.c_str(ctx));
        else log_info("    m0: x=%d idx=%d\n", bel_m0.location.x, bel_m0.index);
    }
    if(bel_m1 != BelId()) {
        CellInfo* cell_m1 = getBoundBelCell(bel_m1);
        if(cell_m1) log_info("    m1: x=%d idx=%d name=%s type=%s\n", bel_m1.location.x, bel_m1.index, cell_m1->name.c_str(ctx), cell_m1->type.c_str(ctx));
        else log_info("    m1: x=%d idx=%d\n", bel_m1.location.x, bel_m1.index);
    }
    if(bel_a3 != BelId()) {
        CellInfo* cell_a3 = getBoundBelCell(bel_a3);
        if(cell_a3) log_info("    a3: x=%d idx=%d name=%s type=%s\n", bel_a3.location.x, bel_a3.index, cell_a3->name.c_str(ctx), cell_a3->type.c_str(ctx));
        else log_info("    a3: x=%d idx=%d\n", bel_a3.location.x, bel_a3.index);
    }
    if(bel_m4 != BelId()) {
        CellInfo* cell_m4 = getBoundBelCell(bel_m4);
        if(cell_m4) log_info("    m4: x=%d idx=%d name=%s type=%s\n", bel_m4.location.x, bel_m4.index, cell_m4->name.c_str(ctx), cell_m4->type.c_str(ctx));
        else log_info("    m4: x=%d idx=%d\n", bel_m4.location.x, bel_m4.index);
    }
    if(bel_m5 != BelId()) {
        CellInfo* cell_m5 = getBoundBelCell(bel_m5);
        if(cell_m5) log_info("    m5: x=%d idx=%d name=%s type=%s\n", bel_m5.location.x, bel_m5.index, cell_m5->name.c_str(ctx), cell_m5->type.c_str(ctx));
        else log_info("    m5: x=%d idx=%d\n", bel_m5.location.x, bel_m5.index);
    }
    if(bel_a7 != BelId()) {
        CellInfo* cell_a7 = getBoundBelCell(bel_a7);
        if(cell_a7) log_info("    a7: x=%d idx=%d name=%s type=%s\n", bel_a7.location.x, bel_a7.index, cell_a7->name.c_str(ctx), cell_a7->type.c_str(ctx));
        else log_info("    a7: x=%d idx=%d\n", bel_a7.location.x, bel_a7.index);
    }
    */

    return true;
}

void Arch::setup_wire_locations()
{
    wire_loc_overrides.clear();
    for (auto &cell : cells) {
        CellInfo *ci = cell.second.get();
        if (ci->bel == BelId())
            continue;
        if (ci->type.in(id_ALU54B, id_MULT18X18D, id_DCUA, id_DDRDLL, id_DQSBUFM, id_EHXPLLL)) {
            for (auto &port : ci->ports) {
                if (port.second.net == nullptr)
                    continue;
                WireId pw = getBelPinWire(ci->bel, port.first);
                if (pw == WireId())
                    continue;
                if (port.second.type == PORT_OUT) {
                    for (auto dh : getPipsDownhill(pw)) {
                        WireId pip_dst = getPipDstWire(dh);
                        wire_loc_overrides[pw] = std::make_pair(pip_dst.location.x, pip_dst.location.y);
                        break;
                    }
                } else {
                    for (auto uh : getPipsUphill(pw)) {
                        WireId pip_src = getPipSrcWire(uh);
                        wire_loc_overrides[pw] = std::make_pair(pip_src.location.x, pip_src.location.y);
                        break;
                    }
                }
            }
        }
    }
}

NEXTPNR_NAMESPACE_END
