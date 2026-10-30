/*
 * MPEG Media Transport Protocol (MMTP) parser, shared internals
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

#ifndef AVFORMAT_MMTP_INTERNAL_H
#define AVFORMAT_MMTP_INTERNAL_H

#include <limits.h>
#include <stdint.h>

#include "libavcodec/bytestream.h"
#include "avformat.h"
#include "mmtp.h"
#include "libavcodec/packet_internal.h"

/**
 * The signalling profile of an MMTP flow. The ISO/IEC 23008-1 core is the
 * same for all of them; they differ in the private ranges of message ids,
 * table ids and descriptor tags, and in a few carriage details.
 */
enum MMTPFlavor {
    MMTP_FLAVOR_UNKNOWN, ///< only the ISO/IEC 23008-1 subset is interpreted
    MMTP_FLAVOR_ARIB,    ///< ARIB STD-B60
};

#define MMTP_MAX_TIMESTAMP_DESCRIPTORS 32
/** AUs one MPU extended timestamp descriptor can time (num_of_au is 8 bits) */
#define MMTP_MAX_AU                    0x100
/** AU index for a packet timed with the MPU presentation time as such */
#define MMTP_AU_MPU_TIME               INT_MAX

typedef struct MMTPTimestamp {
    uint32_t seq_num;
    int64_t  presentation_time; ///< microseconds since the Unix epoch
} MMTPTimestamp;

typedef struct MMTPAUTiming {
    uint16_t dts_pts_offset;
    uint16_t pts_offset;
} MMTPAUTiming;

typedef struct MMTPExtTimestamp {
    uint32_t      seq_num;
    uint16_t      decoding_time_offset;
    uint8_t       num_of_au;
    uint8_t       au_timing;    ///< 0 if only the first AU can be timed
    MMTPAUTiming *au;           ///< num_of_au entries
    unsigned      au_alloc;     ///< allocated size of au, for av_fast_malloc()
} MMTPExtTimestamp;

typedef struct MMTPStream {
    AVStream *st;
    uint16_t  packet_id;
    int       supported;        ///< media is emitted for this asset

    int            nb_timestamps;
    MMTPTimestamp *timestamps;
    int               nb_ext_timestamps;
    MMTPExtTimestamp *ext_timestamps;

    uint32_t mpu_seq;
    int      au_count;
    int      started_mpu;      ///< a unit of the current MPU has been read
    int      in_sample;        ///< HEVC: a picture of the current AU was read

    /* HEVC access unit being assembled from its NAL units, straight from
     * the MFUs; the buffer becomes the packet's */
    uint8_t *au;               ///< with AV_INPUT_BUFFER_PADDING_SIZE more allocated
    int      au_size, au_cap;
    int      nal_open;         ///< a fragmented NAL unit is being received
    int      nal_start;        ///< its offset in au
    uint32_t nal_size;         ///< its size
    int      nal_prev_vcl;     ///< au_vcl before it
    int      au_open;
    int      au_vcl;           ///< the AU holds a slice
    int      au_index;         ///< decoding order index of the AU in its MPU
    int      au_index_lost;    ///< packets were lost in this MPU, au_index may be off
    int      au_untimed;       ///< the AU being assembled started after a loss
    int      was_timed;        ///< a timed AU was queued since the start or a seek
    PacketList held;           ///< AUs of the current MPU waiting for its timestamp,
                               ///< each with its AU index in pts
    int      nb_held;          ///< at most MMTP_MAX_AU
    int64_t  au_pos;
    int64_t  pos;

    int      ttml_tmd;          ///< ARIB time control mode, -1 if unknown

    struct MMTPStream *next;
} MMTPStream;

typedef struct FragmentAssembler FragmentAssembler;

struct MMTPContext {
    AVFormatContext *s;
    AVProgram       *program;
    PacketList      *queue;
    AVPacket        *pkt;

    enum MMTPFlavor  flavor;
    FragmentAssembler *assembler;
    MMTPStream      *streams;
    int              nb_signalling_assemblers;
    int              warned_scrambled;
    int              warned_version1;
    int              warned_assemblers;

    /* valid for the duration of one ff_mmtp_parse_packet() call */
    uint16_t current_pid;
    uint8_t  is_rap;
    int64_t  pos;
    int      parse_flags;      ///< MMTP_PARSE_*
};

enum {
    MMTP_MPU_TIMESTAMP_DESCRIPTOR = 0x0001,
};

/**
 * Latch the flavor of a flow on its first decisive evidence.
 */
void ff_mmtp_set_flavor(MMTPContext *ctx, enum MMTPFlavor flavor);

/**
 * Skip one descriptor using the length width of its tag. A private tag
 * outside an ARIB flow has no known width and ends the descriptor loop:
 * the rest of gbc is skipped.
 */
int ff_mmtp_skip_descriptor(MMTPContext *ctx, GetByteContext *gbc);

/**
 * Find a stream by packet_id, creating it if it does not exist yet.
 */
MMTPStream *ff_mmtp_get_stream(MMTPContext *ctx, uint16_t packet_id);

/**
 * Set the codec of an asset and how its packets are parsed.
 */
void ff_mmtp_set_codec(MMTPStream *ms, enum AVMediaType type, enum AVCodecID id);

/**
 * Find the cache slot for an MPU in an array of timestamp descriptor
 * entries, each starting with its uint32_t MPU sequence number: the entry
 * holding mpu_seq, else one for an MPU before current, else a new zeroed
 * one, else the entry farthest from mpu_seq. NULL on allocation failure.
 */
void *ff_mmtp_timestamp_slot(void **array, int *nb, size_t elem_size,
                             uint32_t mpu_seq, uint32_t current);

/**
 * Presentation time of an MPU from its MPU timestamp descriptor, in the
 * stream time base, or AV_NOPTS_VALUE if it has not been signalled.
 */
int64_t ff_mmtp_mpu_presentation_time(const MMTPStream *ms, uint32_t mpu_seq);

/**
 * Queue a packet holding size bytes of gbc, timestamped with pts/dts.
 */
int ff_mmtp_queue_packet(MMTPContext *ctx, MMTPStream *ms,
                         GetByteContext *gbc, int size,
                         int64_t pts, int64_t dts);

/**
 * Queue a packet holding size bytes of gbc as access unit au of the current
 * MPU, held until the MPU timestamps are known.
 *
 * @param au decoding order index, -1 for no timestamps, or MMTP_AU_MPU_TIME
 */
int ff_mmtp_queue_au(MMTPContext *ctx, MMTPStream *ms,
                     GetByteContext *gbc, int size, int au);

/* ARIB STD-B60, implemented in mmtp_arib.c */

/**
 * Parse an MPT asset descriptor in the ARIB private range (0x8000 and up).
 */
int ff_mmtp_arib_parse_asset_descriptor(MMTPContext *ctx, MMTPStream *ms,
                                        GetByteContext *gbc);

/**
 * Parse a signalling message in the ARIB private range (message_id 0x8000
 * and up).
 */
int ff_mmtp_arib_parse_message(MMTPContext *ctx, GetByteContext *gbc);

/**
 * Whether a multi-type header extension (extension_type 0x0000) marks the
 * payload as scrambled.
 */
int ff_mmtp_arib_is_scrambled(GetByteContext *gbc);

/**
 * Parse the ARIB subtitle MFU header and queue the TTML document it carries.
 */
int ff_mmtp_arib_parse_subtitle_mfu(MMTPContext *ctx, MMTPStream *ms,
                                    GetByteContext *gbc);

#endif /* AVFORMAT_MMTP_INTERNAL_H */
