/*
 * Chinese AVS video (AVS1-P16, GuangDian profile) AEC arithmetic decoder
 * Copyright (c) 2026 Rainbaby <rainbaby@outlook.jp>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Advanced entropy coding (AEC) arithmetic decoding engine used by the
 * GuangDian profile of Chinese AVS video (AVS1-P16, "AVS+").
 *
 * Specified by GY/T 257.1-2012 clause 8.4, implemented in the equivalent
 * Annex E formulation of GB/T 20090.16-2016; see cavs_aec.c. This header
 * exposes the engine only, the binarisations live in the caller.
 */

#ifndef AVCODEC_CAVS_AEC_H
#define AVCODEC_CAVS_AEC_H

#include <stdint.h>

#include "libavutil/error.h"

#include "get_bits.h"

/** Number of context models: Table 51's largest start index is 322
    (weighting_prediction), a single context. */
#define CAVS_AEC_NUM_CTX 323

/**
 * Form a context index from a Table 51 start index, the element's model
 * count and a ctxIdxInc, which is ultimately bitstream driven; call this
 * where the increment is derived, the per-bin functions only assert.
 * @return the index, AVERROR_INVALIDDATA if @p inc is outside @p span, or
 *         AVERROR_BUG if the range is not inside the context array
 */
static inline int ff_cavs_aec_ctx_idx(int start, int span, int inc)
{
    if (span <= 0 || start < 0 || start > CAVS_AEC_NUM_CTX - span)
        return AVERROR_BUG;
    if ((unsigned)inc >= (unsigned)span)
        return AVERROR_INVALIDDATA;

    return start + inc;
}

/** One adaptive context model, GY/T 257.1-2012 8.4.2.1; all three members
    are reset at the start of every slice. */
typedef struct CAVSAECModel {
    uint8_t  mps;      ///< mps:    more probable symbol, 1 bit, init 0
    uint8_t  cycno;    ///< cycno:  adaptation counter, 2 bits, init 0
    uint16_t lg_pmps;  ///< lgPmps: log domain P(MPS), 11 bits, init 1023
} CAVSAECModel;

/** AEC engine state: the registers of 8.4.2.2 plus the b_flag of GB E.2;
    range (rs1, rt1) and offset (value_s, value_t) are each an exponent
    and an 8 bit mantissa. */
typedef struct CAVSAECContext {
    GetBitContext *gb; ///< bitstream reader, shared with the slice parser
    int rs1;           ///< range exponent,  init 0    (rS1)
    int rt1;           ///< range mantissa,  init 0xFF (rT1)
    int value_s;       ///< offset exponent            (valueS)
    int value_t;       ///< offset mantissa, 9 bits inside a call (valueT)
    int b_flag;        ///< offset window exhausted, GB/T 20090.16-2016 E.2
    CAVSAECModel ctx[CAVS_AEC_NUM_CTX]; ///< context models, 8.4.2.1
} CAVSAECContext;

/**
 * Reset all context models and start the arithmetic decoder (8.4.1).
 * @param gb reader, byte aligned after the slice header's
 *           aec_byte_alignment_bit padding; at least 9 bits are consumed
 */
void ff_cavs_aec_init(CAVSAECContext *aec, GetBitContext *gb);

/**
 * Decode one bin against a single context model and adapt it (8.4.4.3.2).
 * @param ctx_idx formed with ff_cavs_aec_ctx_idx() when bitstream driven
 */
int ff_cavs_aec_decode_decision(CAVSAECContext *aec, int ctx_idx);

/** Decode one bin against two weighted context models and adapt both
    (8.4.4.3.2 with contextWeighting == 1, used by 8.4.4.2 j) only). */
int ff_cavs_aec_decode_decision_w(CAVSAECContext *aec, int ctx_idx,
                                  int ctx_idx_w);

/** Decode one bypass bin (8.4.4.3.3); unlike CABAC not a plain bit read,
    the engine range state is carried forward. */
int ff_cavs_aec_decode_bypass(CAVSAECContext *aec);

/** Decode one aec_mb_stuffing_bit (8.4.4.3.4): 1 on the last macroblock
    of the slice (7.2.4), the counterpart of CABAC's end_of_slice_flag. */
int ff_cavs_aec_decode_stuffing_bit(CAVSAECContext *aec);

#endif /* AVCODEC_CAVS_AEC_H */
