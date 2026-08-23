/*
 * Chinese AVS video (AVS1-P2, JiZhun profile) NEON optimisations
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

#include "libavutil/aarch64/asm.S"
#include "neon.S"

/*
 * One 8 point inverse transform pass over v16-v23, eight columns at a
 * time, in 16 bit lanes with wrapping arithmetic exactly like the SSE2
 * version; \bias is a vector added to a4/a5 before the >> \shift, which
 * matches the C code's rounding (its DC preload of 8 folds into the
 * second pass bias of 64 because (x + 64) >> 3 == (x >> 3) + 8).
 */
.macro cavs_idct8_1d bias, shift
        shl             v24.8h, v17.8h, #1      // 2*src1
        shl             v25.8h, v19.8h, #1      // 2*src3
        shl             v26.8h, v21.8h, #1      // 2*src5
        shl             v27.8h, v23.8h, #1      // 2*src7
        add             v28.8h, v24.8h, v17.8h  // 3*src1
        add             v29.8h, v25.8h, v19.8h  // 3*src3
        add             v30.8h, v26.8h, v21.8h  // 3*src5
        add             v31.8h, v27.8h, v23.8h  // 3*src7
        sub             v1.8h,  v28.8h, v27.8h  // a0 = 3*src1 - 2*src7
        add             v2.8h,  v29.8h, v26.8h  // a1 = 3*src3 + 2*src5
        sub             v3.8h,  v25.8h, v30.8h  // a2 = 2*src3 - 3*src5
        add             v4.8h,  v24.8h, v31.8h  // a3 = 2*src1 + 3*src7

        add             v5.8h,  v1.8h,  v2.8h
        add             v5.8h,  v5.8h,  v4.8h   // a0 + a1 + a3
        sub             v6.8h,  v1.8h,  v2.8h
        add             v6.8h,  v6.8h,  v3.8h   // a0 - a1 + a2
        sub             v24.8h, v4.8h,  v3.8h
        sub             v24.8h, v24.8h, v2.8h   // a3 - a2 - a1
        sub             v25.8h, v1.8h,  v3.8h
        sub             v25.8h, v25.8h, v4.8h   // a0 - a2 - a3
        shl             v5.8h,  v5.8h,  #1
        add             v5.8h,  v5.8h,  v2.8h   // b4
        shl             v6.8h,  v6.8h,  #1
        add             v6.8h,  v6.8h,  v1.8h   // b5
        shl             v24.8h, v24.8h, #1
        add             v24.8h, v24.8h, v4.8h   // b6
        shl             v25.8h, v25.8h, #1
        sub             v25.8h, v25.8h, v3.8h   // b7

        shl             v26.8h, v18.8h, #2      // 4*src2
        shl             v27.8h, v22.8h, #2      // 4*src6
        add             v28.8h, v26.8h, v18.8h  // 5*src2
        add             v29.8h, v27.8h, v22.8h  // 5*src6
        shl             v28.8h, v28.8h, #1      // 10*src2
        shl             v29.8h, v29.8h, #1      // 10*src6
        sub             v1.8h,  v26.8h, v29.8h  // a7 = 4*src2 - 10*src6
        add             v2.8h,  v27.8h, v28.8h  // a6 = 4*src6 + 10*src2
        add             v3.8h,  v16.8h, v20.8h
        sub             v4.8h,  v16.8h, v20.8h
        shl             v3.8h,  v3.8h,  #3
        shl             v4.8h,  v4.8h,  #3
        add             v3.8h,  v3.8h,  \bias   // a4 = 8*(src0 + src4) + bias
        add             v4.8h,  v4.8h,  \bias   // a5 = 8*(src0 - src4) + bias
        add             v26.8h, v3.8h,  v2.8h   // b0
        add             v27.8h, v4.8h,  v1.8h   // b1
        sub             v28.8h, v4.8h,  v1.8h   // b2
        sub             v29.8h, v3.8h,  v2.8h   // b3

        add             v16.8h, v26.8h, v5.8h   // b0 + b4
        add             v17.8h, v27.8h, v6.8h   // b1 + b5
        add             v18.8h, v28.8h, v24.8h  // b2 + b6
        add             v19.8h, v29.8h, v25.8h  // b3 + b7
        sub             v20.8h, v29.8h, v25.8h  // b3 - b7
        sub             v21.8h, v28.8h, v24.8h  // b2 - b6
        sub             v22.8h, v27.8h, v6.8h   // b1 - b5
        sub             v23.8h, v26.8h, v5.8h   // b0 - b4
        sshr            v16.8h, v16.8h, #\shift
        sshr            v17.8h, v17.8h, #\shift
        sshr            v18.8h, v18.8h, #\shift
        sshr            v19.8h, v19.8h, #\shift
        sshr            v20.8h, v20.8h, #\shift
        sshr            v21.8h, v21.8h, #\shift
        sshr            v22.8h, v22.8h, #\shift
        sshr            v23.8h, v23.8h, #\shift
.endm

/*
 * void ff_cavs_idct8_add_neon(uint8_t *dst, int16_t *block,
 *                             ptrdiff_t stride)
 *
 * Expects the coefficients transposed, like the SSE2 version: the caller
 * sets idct_perm = FF_IDCT_PERM_TRANSPOSE. The block is not cleared.
 */
function ff_cavs_idct8_add_neon, export=1
        ld1             {v16.8h, v17.8h, v18.8h, v19.8h}, [x1], #64
        ld1             {v20.8h, v21.8h, v22.8h, v23.8h}, [x1]
        movi            v0.8h,  #4
        cavs_idct8_1d   v0.8h, 3
        transpose_8x8H  v16, v17, v18, v19, v20, v21, v22, v23, v30, v31
        movi            v0.8h,  #64
        cavs_idct8_1d   v0.8h, 7

.irp    r, 16, 17, 18, 19, 20, 21, 22, 23
        ld1             {v1.8b}, [x0]
        uaddw           v2.8h,  v\r\().8h, v1.8b
        sqxtun          v1.8b,  v2.8h
        st1             {v1.8b}, [x0], x2
.endr
        ret
endfunc
