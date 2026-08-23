;*****************************************************************************
;* Chinese AVS video (AVS1-P2, JiZhun profile) SSE2 in-loop deblocking filter
;* Copyright (c) 2026 Rainbaby <rainbaby@outlook.jp>
;*
;* This file is part of FFmpeg.
;*
;* FFmpeg is free software; you can redistribute it and/or
;* modify it under the terms of the GNU Lesser General Public
;* License as published by the Free Software Foundation; either
;* version 2.1 of the License, or (at your option) any later version.
;*
;* FFmpeg is distributed in the hope that it will be useful,
;* but WITHOUT ANY WARRANTY; without even the implied warranty of
;* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;* Lesser General Public License for more details.
;*
;* You should have received a copy of the GNU Lesser General Public
;* License along with FFmpeg; if not, write to the Free Software
;* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
;*****************************************************************************

%include "libavutil/x86/x86util.asm"

%if ARCH_X86_64

SECTION_RODATA

pb_80: times 16 db 0x80
pw_4:  times 8 dw 4
pw_2:  times 8 dw 2

SECTION .text

; out = |a - b| for unsigned bytes; out and tmp must differ from a and b
%macro ABSDIFF_B 4 ; out, a, b, tmp
    mova       %1, %2
    mova       %4, %3
    psubusb    %1, %3
    psubusb    %4, %2
    por        %1, %4
%endmacro

; build the luma per-lane enable mask in m6 from bs1/bs2 (8 + 8 lanes)
%macro LUMA_BS_MASK 0
    xor        r7d, r7d
    test       bs1d, bs1d
    setnz      r7b
    neg        r7
    movq       m6, r7
    xor        r7d, r7d
    test       bs2d, bs2d
    setnz      r7b
    neg        r7
    movq       m7, r7
    punpcklqdq m6, m7
%endmacro

; build the chroma per-lane enable mask in m6 from bs1/bs2 (4 + 4 lanes)
%macro CHROMA_BS_MASK 0
    xor        r7d, r7d
    test       bs1d, bs1d
    setnz      r7b
    neg        r7d
    movd       m6, r7d
    xor        r7d, r7d
    test       bs2d, bs2d
    setnz      r7b
    neg        r7d
    movd       m7, r7d
    punpckldq  m6, m7
%endmacro

; one 8-lane half of the bs < 2 luma filter (loop_filter_l1)
; inputs: m0-m5 = p2..q2 bytes, m11 = tc words, m12 = -tc words
; writes the new p1/p0/q0/q1 bytes of this half to the stack
%macro CAVS_LF_L1_HALF 1 ; 0 = low half, 1 = high half
%if %1
    %xdefine %%unpck punpckhbw
%else
    %xdefine %%unpck punpcklbw
%endif
    pxor       m15, m15
    mova       m7, m1
    %%unpck    m7, m15                 ; p1
    mova       m8, m2
    %%unpck    m8, m15                 ; p0
    mova       m9, m3
    %%unpck    m9, m15                 ; q0
    mova       m10, m4
    %%unpck    m10, m15                ; q1
    mova       m13, m9
    psubw      m13, m8                 ; q0 - p0
    mova       m14, m13
    psllw      m14, 1
    paddw      m13, m14                ; 3 * (q0 - p0)
    paddw      m13, m7
    psubw      m13, m10                ; + p1 - q1
    paddw      m13, [pw_4]
    psraw      m13, 3
    pminsw     m13, m11
    pmaxsw     m13, m12                ; delta
    paddw      m8, m13                 ; p0 + delta
    packuswb   m8, m8
    punpcklbw  m8, m15                 ; new p0, clipped to u8
    psubw      m9, m13                 ; q0 - delta
    packuswb   m9, m9
    punpcklbw  m9, m15                 ; new q0, clipped to u8
    ; p1 delta: ((new_p0 - p1) * 3 + p2 - new_q0 + 4) >> 3
    mova       m13, m8
    psubw      m13, m7
    mova       m14, m13
    psllw      m14, 1
    paddw      m13, m14
    mova       m14, m0
    %%unpck    m14, m15                ; p2
    paddw      m13, m14
    psubw      m13, m9
    paddw      m13, [pw_4]
    psraw      m13, 3
    pminsw     m13, m11
    pmaxsw     m13, m12
    paddw      m13, m7                 ; p1 + delta
    packuswb   m13, m13
    movq       [rsp+0x30+8*%1], m13    ; new p1
    ; q1 delta: ((q1 - new_q0) * 3 + new_p0 - q2 + 4) >> 3
    mova       m13, m10
    psubw      m13, m9
    mova       m14, m13
    psllw      m14, 1
    paddw      m13, m14
    paddw      m13, m8
    mova       m14, m5
    %%unpck    m14, m15                ; q2
    psubw      m13, m14
    paddw      m13, [pw_4]
    psraw      m13, 3
    pminsw     m13, m11
    pmaxsw     m13, m12
    psubw      m10, m13                ; q1 - delta
    packuswb   m10, m10
    movq       [rsp+0x60+8*%1], m10    ; new q1
    packuswb   m8, m8
    movq       [rsp+0x40+8*%1], m8     ; new p0
    packuswb   m9, m9
    movq       [rsp+0x50+8*%1], m9     ; new q0
%endmacro

; non-normative luma edge filter, bs < 2 (loop_filter_l1), 16 lanes
; in: m0-m5 = p2..q2 bytes, m6 = bs enable mask
; out: m0/m1/m2/m3 = new p1/p0/q0/q1 bytes; clobbers alphad/betad
%macro CAVS_LF_L1 0
    xor        alphad, 0x80
    SPLATB_REG m7, alpha, m15          ; alpha ^ 0x80
    xor        betad, 0x80
    SPLATB_REG m8, beta, m15           ; beta ^ 0x80
    mova       m9, [pb_80]
    ABSDIFF_B  m10, m2, m3, m11        ; |p0 - q0|
    pxor       m10, m9
    mova       m12, m7
    pcmpgtb    m12, m10                ; < alpha
    ABSDIFF_B  m10, m1, m2, m11        ; |p1 - p0|
    pxor       m10, m9
    mova       m13, m8
    pcmpgtb    m13, m10
    pand       m12, m13
    ABSDIFF_B  m10, m4, m3, m11        ; |q1 - q0|
    pxor       m10, m9
    mova       m13, m8
    pcmpgtb    m13, m10
    pand       m12, m13
    pand       m12, m6                 ; filter this lane
    mova       [rsp+0x00], m12
    ABSDIFF_B  m10, m0, m2, m11        ; |p2 - p0|
    pxor       m10, m9
    mova       m13, m8
    pcmpgtb    m13, m10
    pand       m13, m12
    mova       [rsp+0x10], m13         ; p1 also filtered
    ABSDIFF_B  m10, m5, m3, m11        ; |q2 - q0|
    pxor       m10, m9
    pcmpgtb    m8, m10
    pand       m8, m12
    mova       [rsp+0x20], m8          ; q1 also filtered
    movd       m11, tcd
    SPLATW     m11, m11                ; tc
    pxor       m12, m12
    psubw      m12, m11                ; -tc
    CAVS_LF_L1_HALF 0
    CAVS_LF_L1_HALF 1
    mova       m6, [rsp+0x00]
    mova       m7, [rsp+0x10]
    mova       m8, [rsp+0x20]
    mova       m9, [rsp+0x30]
    pand       m9, m7
    pandn      m7, m1
    por        m9, m7                  ; final p1
    mova       m10, [rsp+0x40]
    mova       m11, m6
    pand       m10, m6
    pandn      m11, m2
    por        m10, m11                ; final p0
    mova       m11, [rsp+0x50]
    pand       m11, m6
    pandn      m6, m3
    por        m11, m6                 ; final q0
    mova       m12, [rsp+0x60]
    pand       m12, m8
    pandn      m8, m4
    por        m12, m8                 ; final q1
    mova       m0, m9
    mova       m1, m10
    mova       m2, m11
    mova       m3, m12
%endmacro

; one 8-lane half of the bs == 2 luma filter (loop_filter_l2)
; inputs: m0-m5 = p2..q2 bytes
; writes the strong/weak p0/q0 candidate bytes of this half to the stack
%macro CAVS_LF_L2_HALF 1 ; 0 = low half, 1 = high half
%if %1
    %xdefine %%unpck punpckhbw
%else
    %xdefine %%unpck punpcklbw
%endif
    pxor       m15, m15
    mova       m9, m1
    %%unpck    m9, m15                 ; p1
    mova       m10, m2
    %%unpck    m10, m15                ; p0
    mova       m11, m3
    %%unpck    m11, m15                ; q0
    mova       m12, m4
    %%unpck    m12, m15                ; q1
    mova       m14, m10
    paddw      m14, m11                ; p0 + q0
    mova       m15, m14
    paddw      m15, m9                 ; p1 + p0 + q0
    mova       m6, m15
    paddw      m6, m10                 ; p1 + 2*p0 + q0
    paddw      m15, m9                 ; 2*p1 + p0 + q0
    paddw      m6, [pw_2]
    psraw      m6, 2
    packuswb   m6, m6
    movq       [rsp+0x30+8*%1], m6     ; strong p0
    paddw      m15, [pw_2]
    psraw      m15, 2
    packuswb   m15, m15
    movq       [rsp+0x40+8*%1], m15    ; weak p0 / new p1
    mova       m15, m14
    paddw      m15, m12                ; q1 + q0 + p0
    mova       m6, m15
    paddw      m6, m11                 ; q1 + 2*q0 + p0
    paddw      m15, m12                ; 2*q1 + q0 + p0
    paddw      m6, [pw_2]
    psraw      m6, 2
    packuswb   m6, m6
    movq       [rsp+0x50+8*%1], m6     ; strong q0
    paddw      m15, [pw_2]
    psraw      m15, 2
    packuswb   m15, m15
    movq       [rsp+0x60+8*%1], m15    ; weak q0 / new q1
%endmacro

; normative luma edge filter, bs == 2 (loop_filter_l2), 16 lanes
; in: m0-m5 = p2..q2 bytes
; out: m0/m1/m2/m3 = new p1/p0/q0/q1 bytes; clobbers alphad/betad, r7
%macro CAVS_LF_L2 0
    mov        r7d, alphad
    shr        r7d, 2
    add        r7d, 2                  ; alpha2 = (alpha >> 2) + 2
    xor        r7d, 0x80
    SPLATB_REG m10, r7, m15
    xor        alphad, 0x80
    SPLATB_REG m7, alpha, m15
    xor        betad, 0x80
    SPLATB_REG m8, beta, m15
    mova       m9, [pb_80]
    ABSDIFF_B  m11, m2, m3, m12        ; |p0 - q0|
    pxor       m11, m9
    pcmpgtb    m7, m11                 ; < alpha
    pcmpgtb    m10, m11                ; < alpha2
    ABSDIFF_B  m11, m1, m2, m12        ; |p1 - p0|
    pxor       m11, m9
    mova       m13, m8
    pcmpgtb    m13, m11
    pand       m7, m13
    ABSDIFF_B  m11, m4, m3, m12        ; |q1 - q0|
    pxor       m11, m9
    mova       m13, m8
    pcmpgtb    m13, m11
    pand       m7, m13                 ; filter this lane
    ABSDIFF_B  m11, m0, m2, m12        ; |p2 - p0|
    pxor       m11, m9
    mova       m13, m8
    pcmpgtb    m13, m11
    pand       m13, m10
    pand       m13, m7                 ; strong p side
    ABSDIFF_B  m11, m5, m3, m12        ; |q2 - q0|
    pxor       m11, m9
    pcmpgtb    m8, m11
    pand       m8, m10
    pand       m8, m7                  ; strong q side
    CAVS_LF_L2_HALF 0
    CAVS_LF_L2_HALF 1
    mova       m9,  [rsp+0x30]         ; strong p0
    mova       m10, [rsp+0x40]         ; weak p0
    mova       m11, [rsp+0x50]         ; strong q0
    mova       m12, [rsp+0x60]         ; weak q0
    mova       m14, m13
    mova       m15, m10
    pand       m15, m13
    pandn      m14, m1
    por        m15, m14                ; final p1
    pand       m9, m13
    pandn      m13, m10
    por        m9, m13                 ; p0 when filtered
    mova       m14, m7
    pand       m9, m7
    pandn      m14, m2
    por        m9, m14                 ; final p0
    mova       m14, m8
    mova       m10, m12
    pand       m10, m8
    pandn      m14, m4
    por        m10, m14                ; final q1
    pand       m11, m8
    pandn      m8, m12
    por        m11, m8                 ; q0 when filtered
    mova       m14, m7
    pand       m11, m7
    pandn      m14, m3
    por        m11, m14                ; final q0
    mova       m0, m15
    mova       m1, m9
    mova       m2, m11
    mova       m3, m10
%endmacro

; chroma bs < 2 filter (loop_filter_c1), 8 lanes in the low halves
; in: m1-m4 = p1..q1 bytes, m6 = bs enable mask
; out: m0 = new p0, m1 = new q0; clobbers alphad/betad
%macro CAVS_LF_C1 0
    xor        alphad, 0x80
    SPLATB_REG m7, alpha, m15
    xor        betad, 0x80
    SPLATB_REG m8, beta, m15
    mova       m9, [pb_80]
    ABSDIFF_B  m10, m2, m3, m11        ; |p0 - q0|
    pxor       m10, m9
    mova       m12, m7
    pcmpgtb    m12, m10
    ABSDIFF_B  m10, m1, m2, m11        ; |p1 - p0|
    pxor       m10, m9
    mova       m13, m8
    pcmpgtb    m13, m10
    pand       m12, m13
    ABSDIFF_B  m10, m4, m3, m11        ; |q1 - q0|
    pxor       m10, m9
    pcmpgtb    m8, m10
    pand       m12, m8
    pand       m12, m6                 ; filter this lane
    movd       m11, tcd
    SPLATW     m11, m11                ; tc
    pxor       m13, m13
    psubw      m13, m11                ; -tc
    pxor       m15, m15
    mova       m7, m1
    punpcklbw  m7, m15                 ; p1
    mova       m8, m2
    punpcklbw  m8, m15                 ; p0
    mova       m9, m3
    punpcklbw  m9, m15                 ; q0
    mova       m10, m4
    punpcklbw  m10, m15                ; q1
    mova       m14, m9
    psubw      m14, m8                 ; q0 - p0
    mova       m15, m14
    psllw      m15, 1
    paddw      m14, m15                ; 3 * (q0 - p0)
    paddw      m14, m7
    psubw      m14, m10                ; + p1 - q1
    paddw      m14, [pw_4]
    psraw      m14, 3
    pminsw     m14, m11
    pmaxsw     m14, m13                ; delta
    paddw      m8, m14
    packuswb   m8, m8                  ; new p0
    psubw      m9, m14
    packuswb   m9, m9                  ; new q0
    mova       m0, m12
    pand       m8, m12
    pandn      m0, m2
    por        m0, m8                  ; final p0
    mova       m1, m12
    pandn      m1, m3
    pand       m9, m12
    por        m1, m9                  ; final q0
%endmacro

; chroma bs == 2 filter (loop_filter_c2), 8 lanes in the low halves
; in: m0-m5 = p2..q2 bytes
; out: m0 = new p0, m1 = new q0; clobbers alphad/betad, r7
%macro CAVS_LF_C2 0
    mov        r7d, alphad
    shr        r7d, 2
    add        r7d, 2                  ; alpha2 = (alpha >> 2) + 2
    xor        r7d, 0x80
    SPLATB_REG m10, r7, m15
    xor        alphad, 0x80
    SPLATB_REG m7, alpha, m15
    xor        betad, 0x80
    SPLATB_REG m8, beta, m15
    mova       m9, [pb_80]
    ABSDIFF_B  m11, m2, m3, m12        ; |p0 - q0|
    pxor       m11, m9
    pcmpgtb    m7, m11                 ; < alpha
    pcmpgtb    m10, m11                ; < alpha2
    ABSDIFF_B  m11, m1, m2, m12        ; |p1 - p0|
    pxor       m11, m9
    mova       m13, m8
    pcmpgtb    m13, m11
    pand       m7, m13
    ABSDIFF_B  m11, m4, m3, m12        ; |q1 - q0|
    pxor       m11, m9
    mova       m13, m8
    pcmpgtb    m13, m11
    pand       m7, m13                 ; filter this lane
    ABSDIFF_B  m11, m0, m2, m12        ; |p2 - p0|
    pxor       m11, m9
    mova       m13, m8
    pcmpgtb    m13, m11
    pand       m13, m10
    pand       m13, m7                 ; strong p side
    ABSDIFF_B  m11, m5, m3, m12        ; |q2 - q0|
    pxor       m11, m9
    pcmpgtb    m8, m11
    pand       m8, m10
    pand       m8, m7                  ; strong q side
    pxor       m15, m15
    mova       m9, m1
    punpcklbw  m9, m15                 ; p1
    mova       m10, m2
    punpcklbw  m10, m15                ; p0
    mova       m11, m3
    punpcklbw  m11, m15                ; q0
    mova       m12, m4
    punpcklbw  m12, m15                ; q1
    mova       m14, m10
    paddw      m14, m11                ; p0 + q0
    mova       m15, m14
    paddw      m15, m9                 ; p1 + p0 + q0
    mova       m6, m15
    paddw      m6, m10                 ; p1 + 2*p0 + q0
    paddw      m15, m9                 ; 2*p1 + p0 + q0
    paddw      m6, [pw_2]
    psraw      m6, 2
    packuswb   m6, m6                  ; strong p0
    paddw      m15, [pw_2]
    psraw      m15, 2
    packuswb   m15, m15                ; weak p0
    mova       m9, m13
    pand       m6, m13
    pandn      m9, m15
    por        m6, m9                  ; p0 when filtered
    mova       m9, m7
    pand       m6, m7
    pandn      m9, m2
    por        m6, m9                  ; final p0
    mova       m15, m14
    paddw      m15, m12                ; q1 + q0 + p0
    mova       m9, m15
    paddw      m9, m11                 ; q1 + 2*q0 + p0
    paddw      m15, m12                ; 2*q1 + q0 + p0
    paddw      m9, [pw_2]
    psraw      m9, 2
    packuswb   m9, m9                  ; strong q0
    paddw      m15, [pw_2]
    psraw      m15, 2
    packuswb   m15, m15                ; weak q0
    mova       m10, m8
    pand       m9, m8
    pandn      m10, m15
    por        m9, m10                 ; q0 when filtered
    mova       m10, m7
    pand       m9, m7
    pandn      m10, m3
    por        m9, m10                 ; final q0
    mova       m0, m6
    mova       m1, m9
%endmacro

; load p2..q2 rows into m0-m5 for the horizontal luma edge; leaves dstq at d
%macro LOAD_LH 0
    lea        r7, [strideq*2+strideq]
    sub        dstq, r7
    movu       m0, [dstq]              ; p2
    movu       m1, [dstq+strideq]      ; p1
    movu       m2, [dstq+strideq*2]    ; p0
    movu       m3, [dstq+r7]           ; q0
    movu       m4, [dstq+strideq*4]    ; q1
    add        dstq, r7
    movu       m5, [dstq+strideq*2]    ; q2
%endmacro

; load 16 rows of 8 at d - 4 and transpose; p2..q2 end up in m0-m5
%macro LOAD_TRANSPOSE_LV 0
    lea        r7, [dstq-4]
    lea        r8, [r7+strideq*8]
    movq       m0, [r7]
    movq       m1, [r7+strideq]
    movhps     m0, [r8]
    movhps     m1, [r8+strideq]
    lea        r7, [r7+strideq*2]
    lea        r8, [r8+strideq*2]
    movq       m2, [r7]
    movq       m3, [r7+strideq]
    movhps     m2, [r8]
    movhps     m3, [r8+strideq]
    lea        r7, [r7+strideq*2]
    lea        r8, [r8+strideq*2]
    movq       m4, [r7]
    movq       m5, [r7+strideq]
    movhps     m4, [r8]
    movhps     m5, [r8+strideq]
    lea        r7, [r7+strideq*2]
    lea        r8, [r8+strideq*2]
    movq       m6, [r7]
    movq       m7, [r7+strideq]
    movhps     m6, [r8]
    movhps     m7, [r8+strideq]
    ; mk = row k | row k + 8; transpose to one register per column
    mova       m8, m0
    punpcklbw  m0, m1                  ; rows 0,1
    punpckhbw  m8, m1                  ; rows 8,9
    mova       m9, m2
    punpcklbw  m2, m3                  ; rows 2,3
    punpckhbw  m9, m3                  ; rows 10,11
    mova       m10, m4
    punpcklbw  m4, m5                  ; rows 4,5
    punpckhbw  m10, m5                 ; rows 12,13
    mova       m11, m6
    punpcklbw  m6, m7                  ; rows 6,7
    punpckhbw  m11, m7                 ; rows 14,15
    mova       m1, m0
    punpcklwd  m0, m2                  ; cols 0-3, rows 0-3
    punpckhwd  m1, m2                  ; cols 4-7, rows 0-3
    mova       m3, m4
    punpcklwd  m4, m6                  ; cols 0-3, rows 4-7
    punpckhwd  m3, m6                  ; cols 4-7, rows 4-7
    mova       m5, m8
    punpcklwd  m8, m9                  ; cols 0-3, rows 8-11
    punpckhwd  m5, m9                  ; cols 4-7, rows 8-11
    mova       m7, m10
    punpcklwd  m10, m11                ; cols 0-3, rows 12-15
    punpckhwd  m7, m11                 ; cols 4-7, rows 12-15
    mova       m2, m0
    punpckldq  m0, m4                  ; col 0 | col 1, rows 0-7
    punpckhdq  m2, m4                  ; col 2 | col 3, rows 0-7
    mova       m6, m1
    punpckldq  m1, m3                  ; col 4 | col 5, rows 0-7
    punpckhdq  m6, m3                  ; col 6 | col 7, rows 0-7
    mova       m9, m8
    punpckldq  m8, m10                 ; col 0 | col 1, rows 8-15
    punpckhdq  m9, m10                 ; col 2 | col 3, rows 8-15
    mova       m11, m5
    punpckldq  m5, m7                  ; col 4 | col 5, rows 8-15
    punpckhdq  m11, m7                 ; col 6 | col 7, rows 8-15
    punpckhqdq m0, m8                  ; col 1 = p2
    mova       m3, m2
    punpcklqdq m2, m9                  ; col 2 = p1
    punpckhqdq m3, m9                  ; col 3 = p0
    mova       m4, m1
    punpcklqdq m1, m5                  ; col 4 = q0
    punpckhqdq m4, m5                  ; col 5 = q1
    punpcklqdq m6, m11                 ; col 6 = q2
    mova       m7, m1                  ; q0
    mova       m1, m2                  ; p1
    mova       m2, m3                  ; p0
    mova       m3, m7                  ; q0
    mova       m5, m6                  ; q2
%endmacro

; transpose m0-m3 = new p1/p0/q0/q1 back and store 4 bytes per row at d - 2
%macro TRANSPOSE_STORE_LV 0
    mova       m4, m0
    punpcklbw  m0, m1                  ; p1,p0 rows 0-7
    punpckhbw  m4, m1                  ; p1,p0 rows 8-15
    mova       m5, m2
    punpcklbw  m2, m3                  ; q0,q1 rows 0-7
    punpckhbw  m5, m3                  ; q0,q1 rows 8-15
    mova       m1, m0
    punpcklwd  m0, m2                  ; rows 0-3
    punpckhwd  m1, m2                  ; rows 4-7
    mova       m3, m4
    punpcklwd  m3, m5                  ; rows 8-11
    punpckhwd  m4, m5                  ; rows 12-15
    lea        r7, [dstq-2]
    lea        r8, [strideq*2+strideq]
%assign %%i 0
%rep 4
 %if %%i == 0
  %xdefine %%r m0
 %elif %%i == 1
  %xdefine %%r m1
 %elif %%i == 2
  %xdefine %%r m3
 %else
  %xdefine %%r m4
 %endif
    movd       [r7], %%r
    psrldq     %%r, 4
    movd       [r7+strideq], %%r
    psrldq     %%r, 4
    movd       [r7+strideq*2], %%r
    psrldq     %%r, 4
    movd       [r7+r8], %%r
 %if %%i < 3
    lea        r7, [r7+strideq*4]
 %endif
 %assign %%i %%i+1
%endrep
%endmacro

; load 8 rows of 8 at d - 4 and transpose; p2..q2 end up in m0-m5 (low halves)
%macro LOAD_TRANSPOSE_CV 0
    lea        r7, [dstq-4]
    movq       m0, [r7]
    movq       m1, [r7+strideq]
    lea        r7, [r7+strideq*2]
    movq       m2, [r7]
    movq       m3, [r7+strideq]
    lea        r7, [r7+strideq*2]
    movq       m4, [r7]
    movq       m5, [r7+strideq]
    lea        r7, [r7+strideq*2]
    movq       m6, [r7]
    movq       m7, [r7+strideq]
    punpcklbw  m0, m1                  ; rows 0,1
    punpcklbw  m2, m3                  ; rows 2,3
    punpcklbw  m4, m5                  ; rows 4,5
    punpcklbw  m6, m7                  ; rows 6,7
    mova       m1, m0
    punpcklwd  m0, m2                  ; cols 0-3, rows 0-3
    punpckhwd  m1, m2                  ; cols 4-7, rows 0-3
    mova       m3, m4
    punpcklwd  m3, m6                  ; cols 0-3, rows 4-7
    punpckhwd  m4, m6                  ; cols 4-7, rows 4-7
    mova       m2, m0
    punpckldq  m0, m3                  ; col 0 | col 1
    punpckhdq  m2, m3                  ; col 2 | col 3
    mova       m5, m1
    punpckldq  m1, m4                  ; col 4 | col 5
    punpckhdq  m5, m4                  ; col 6 | col 7
    mova       m7, m1                  ; q0 | q1
    psrldq     m0, 8                   ; col 1 = p2
    mova       m1, m2                  ; col 2 = p1
    psrldq     m2, 8                   ; col 3 = p0
    mova       m3, m7                  ; col 4 = q0
    mova       m4, m7
    psrldq     m4, 8                   ; col 5 = q1
    ; m5 low half = col 6 = q2
%endmacro

; interleave m0/m1 = new p0/q0 and store 2 bytes per row at d - 1
%macro TRANSPOSE_STORE_CV 0
    punpcklbw  m0, m1                  ; p0,q0 per row
    lea        r7, [dstq-1]
%assign %%i 0
%rep 4
    pextrw     r8d, m0, 2*%%i
    mov        [r7], r8w
    pextrw     r8d, m0, 2*%%i+1
    mov        [r7+strideq], r8w
 %if %%i < 3
    lea        r7, [r7+strideq*2]
 %endif
 %assign %%i %%i+1
%endrep
%endmacro

; load p2..q2 rows into m0-m5 for the horizontal chroma edge; leaves dstq at d
%macro LOAD_CH 0
    lea        r7, [strideq*2+strideq]
    sub        dstq, r7
    movq       m0, [dstq]              ; p2
    movq       m1, [dstq+strideq]      ; p1
    movq       m2, [dstq+strideq*2]    ; p0
    movq       m3, [dstq+r7]           ; q0
    movq       m4, [dstq+strideq*4]    ; q1
    add        dstq, r7
    movq       m5, [dstq+strideq*2]    ; q2
%endmacro

INIT_XMM sse2

cglobal cavs_filter_lv, 7, 9, 16, 0x70, dst, stride, alpha, beta, tc, bs1, bs2
    cmp        bs1d, 2
    je         .filter
    mov        r7d, bs1d
    or         r7d, bs2d
    jz         .end
.filter:
    LOAD_TRANSPOSE_LV
    cmp        bs1d, 2
    je         .bs2
    LUMA_BS_MASK
    CAVS_LF_L1
    jmp        .store
.bs2:
    CAVS_LF_L2
.store:
    TRANSPOSE_STORE_LV
.end:
    RET

cglobal cavs_filter_lh, 7, 8, 16, 0x70, dst, stride, alpha, beta, tc, bs1, bs2
    cmp        bs1d, 2
    je         .filter
    mov        r7d, bs1d
    or         r7d, bs2d
    jz         .end
.filter:
    LOAD_LH
    cmp        bs1d, 2
    je         .bs2
    LUMA_BS_MASK
    CAVS_LF_L1
    jmp        .store
.bs2:
    CAVS_LF_L2
.store:
    mov        r7, dstq
    sub        r7, strideq
    movu       [r7], m1                ; p0
    sub        r7, strideq
    movu       [r7], m0                ; p1
    movu       [dstq], m2              ; q0
    movu       [dstq+strideq], m3      ; q1
.end:
    RET

cglobal cavs_filter_cv, 7, 9, 16, 0, dst, stride, alpha, beta, tc, bs1, bs2
    cmp        bs1d, 2
    je         .filter
    mov        r7d, bs1d
    or         r7d, bs2d
    jz         .end
.filter:
    LOAD_TRANSPOSE_CV
    cmp        bs1d, 2
    je         .bs2
    CHROMA_BS_MASK
    CAVS_LF_C1
    jmp        .store
.bs2:
    CAVS_LF_C2
.store:
    TRANSPOSE_STORE_CV
.end:
    RET

cglobal cavs_filter_ch, 7, 8, 16, 0, dst, stride, alpha, beta, tc, bs1, bs2
    cmp        bs1d, 2
    je         .filter
    mov        r7d, bs1d
    or         r7d, bs2d
    jz         .end
.filter:
    LOAD_CH
    cmp        bs1d, 2
    je         .bs2
    CHROMA_BS_MASK
    CAVS_LF_C1
    jmp        .store
.bs2:
    CAVS_LF_C2
.store:
    mov        r7, dstq
    sub        r7, strideq
    movq       [r7], m0                ; p0
    movq       [dstq], m1              ; q0
.end:
    RET

%endif ; ARCH_X86_64
