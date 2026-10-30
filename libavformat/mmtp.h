/*
 * MPEG Media Transport Protocol (MMTP) parser, as defined in ISO/IEC 23008-1.
 * Copyright (c) 2025 SuperFashi G.K.
 * Copyright (c) 2026 Rainbaby
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

#ifndef AVFORMAT_MMTP_H
#define AVFORMAT_MMTP_H

#include <stdint.h>

#include "avformat.h"
#include "libavcodec/packet_internal.h"

/**
 * Parser state of one MMTP flow. It knows nothing of the carriage: the
 * caller hands it one MMTP packet (the payload of one UDP datagram) at a
 * time.
 */
typedef struct MMTPContext MMTPContext;

/**
 * Only parse signalling messages, skipping media.
 */
#define MMTP_PARSE_SIGNALLING_ONLY 1

/**
 * Create no streams: only those that exist already are updated.
 */
#define MMTP_PARSE_NO_NEW_STREAMS  2

/**
 * Open a parser for one MMTP flow.
 *
 * @param s       the demuxer context the flow's streams are created in
 * @param program the program the flow's streams are added to
 * @param queue   the list completed packets are appended to
 * @return a new context, or NULL on allocation failure
 */
MMTPContext *ff_mmtp_parse_open(AVFormatContext *s, AVProgram *program,
                                PacketList *queue);

/**
 * Parse one MMTP packet. Zero or more packets are appended to the queue.
 *
 * @param pos   byte position of the carrier packet holding buf; media that
 *              starts in it gets this position
 * @param flags MMTP_PARSE_* flags
 * @return 0 on success, AVERROR_INVALIDDATA if the packet was malformed and
 *         has been dropped, another negative error code on failure
 */
int ff_mmtp_parse_packet(MMTPContext *ctx, const uint8_t *buf, int size,
                         int64_t pos, int flags);

/**
 * Queue the access units still being assembled, at the end of the input.
 */
int ff_mmtp_flush(MMTPContext *ctx);

/**
 * Forget partial units and MPU state, e.g. after a seek.
 */
void ff_mmtp_reset_state(MMTPContext *ctx);

/**
 * Free a parser and set *ctx to NULL.
 */
void ff_mmtp_parse_close(MMTPContext **ctx);

#endif /* AVFORMAT_MMTP_H */
