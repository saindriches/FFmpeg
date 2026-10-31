/*
 * MPEG Media Transport Protocol (MMTP) parser: ARIB STD-B60 signalling
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

#include "libavcodec/bytestream.h"
#include "libavutil/crc.h"
#include "libavutil/dict.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "internal.h"
#include "mmtp_internal.h"

enum {
    M2_SECTION_MESSAGE_ID = 0x8000,
};

enum {
    VIDEO_COMPONENT_DESCRIPTOR        = 0x8010,
    MH_AUDIO_COMPONENT_DESCRIPTOR     = 0x8014,
    MH_DATA_COMPONENT_DESCRIPTOR      = 0x8020,
    MPU_EXTENDED_TIMESTAMP_DESCRIPTOR = 0x8026,
};

/* time control modes of ARIB STD-B60 Table 9-7 timed by the MPU */
enum {
    TMD_MPU_TIMESTAMP_ORIGIN = 0x3,
    TMD_MPU_TIMESTAMP        = 0x8,
};

/* Split off the body of an 8-bit length descriptor. */
static int descriptor_body(GetByteContext *gbc, GetByteContext *body)
{
    unsigned length;

    bytestream2_skipu(gbc, 2);
    length = bytestream2_get_byteu(gbc);
    if (bytestream2_get_bytes_left(gbc) < length)
        return AVERROR_INVALIDDATA;
    bytestream2_init(body, gbc->buffer, length);
    bytestream2_skipu(gbc, length);
    return 0;
}

static int set_language(AVStream *st, GetByteContext *gbc)
{
    char language[4];

    if (bytestream2_get_bytes_left(gbc) < 3)
        return AVERROR_INVALIDDATA;
    bytestream2_get_bufferu(gbc, language, 3);
    language[3] = '\0';
    return av_dict_set(&st->metadata, "language", language, 0);
}

static int parse_video_component_descriptor(MMTPStream *ms, GetByteContext *d)
{
    /* video_resolution .. video_transfer_characteristics, reserved */
    if (bytestream2_get_bytes_left(d) < 5)
        return AVERROR_INVALIDDATA;
    bytestream2_skipu(d, 5);
    return set_language(ms->st, d);
}

static int parse_mh_audio_component_descriptor(MMTPStream *ms, GetByteContext *d)
{
    AVStream *st = ms->st;
    unsigned  stream_content, stream_type, flags;
    int       ret;

    if (bytestream2_get_bytes_left(d) < 10)
        return AVERROR_INVALIDDATA;
    stream_content = bytestream2_get_byteu(d) & 0xF;
    bytestream2_skipu(d, 3);                          // component_type, component_tag
    stream_type = bytestream2_get_byteu(d);
    bytestream2_skipu(d, 1);                          // simulcast_group_tag
    flags = bytestream2_get_byteu(d);
    if ((ret = set_language(st, d)) < 0)
        return ret;
    if (flags & 0x80)                                 // ES_multi_lingual_flag
        bytestream2_skip(d, 3);

    if (stream_content == 0x3 && stream_type == 0x11)
        ff_mmtp_set_codec(ms, AVMEDIA_TYPE_AUDIO, AV_CODEC_ID_AAC_LATM);
    else if (stream_content == 0x3 && stream_type == 0x1C)
        ff_mmtp_set_codec(ms, AVMEDIA_TYPE_AUDIO, AV_CODEC_ID_AAC);
    else if (stream_content == 0x4)
        ff_mmtp_set_codec(ms, AVMEDIA_TYPE_AUDIO, AV_CODEC_ID_MP4ALS);
    return 0;
}

/* Additional_Arib_Subtitle_Info(), ARIB STD-B60 Table 9-3 */
static int parse_additional_arib_subtitle_info(MMTPStream *ms, GetByteContext *d)
{
    unsigned flags, format;
    int      ret;

    if (bytestream2_get_bytes_left(d) < 8)
        return AVERROR_INVALIDDATA;
    bytestream2_skipu(d, 1);                          // subtitle_tag
    flags = bytestream2_get_byteu(d);                 // version, start_mpu_sequence_number_flag
    if ((ret = set_language(ms->st, d)) < 0)
        return ret;
    format       = bytestream2_get_byteu(d) >> 2 & 0xF;
    ms->ttml_tmd = bytestream2_get_byteu(d) >> 4;     // TMD, DMF
    bytestream2_skipu(d, 1);                          // resolution, compression_type
    if (flags & 0x08) {
        if (bytestream2_get_bytes_left(d) < 4)
            return AVERROR_INVALIDDATA;
        bytestream2_skipu(d, 4);                      // start_mpu_sequence_number
    }

    if (format == 0)
        ff_mmtp_set_codec(ms, AVMEDIA_TYPE_SUBTITLE, AV_CODEC_ID_TTML);
    return 0;
}

static int parse_mh_data_component_descriptor(MMTPStream *ms, GetByteContext *d)
{
    if (bytestream2_get_bytes_left(d) < 2)
        return AVERROR_INVALIDDATA;
    if (bytestream2_get_be16u(d) == 0x0020)           // data_component_id: subtitles
        return parse_additional_arib_subtitle_info(ms, d);
    return 0;
}

/* MPU_Extended_Timestamp_Descriptor(), ARIB STD-B60 Table 7-82 */
static int parse_mpu_extended_timestamp_descriptor(MMTPStream *ms, GetByteContext *d)
{
    unsigned pts_offset_type, default_pts_offset = 0;
    unsigned flags;

    if (bytestream2_get_bytes_left(d) < 1)
        return AVERROR_INVALIDDATA;
    flags           = bytestream2_get_byteu(d);
    pts_offset_type = flags >> 1 & 3;
    if (flags & 1) {
        uint32_t timescale;

        if (bytestream2_get_bytes_left(d) < 4)
            return AVERROR_INVALIDDATA;
        timescale = bytestream2_get_be32u(d);
        if (!timescale || timescale > INT_MAX)
            return AVERROR_INVALIDDATA;
        if (ms->st->time_base.num != 1 || ms->st->time_base.den != timescale)
            avpriv_set_pts_info(ms->st, 64, 1, timescale);
    }
    if (pts_offset_type == 1) {
        if (bytestream2_get_bytes_left(d) < 2)
            return AVERROR_INVALIDDATA;
        default_pts_offset = bytestream2_get_be16u(d);
    }

    while (bytestream2_get_bytes_left(d) > 0) {
        MMTPExtTimestamp *ext = NULL;
        uint32_t          mpu_seq;
        uint16_t          decoding_time_offset;
        unsigned          num_of_au;

        if (bytestream2_get_bytes_left(d) < 8)
            return AVERROR_INVALIDDATA;
        mpu_seq = bytestream2_get_be32u(d);
        bytestream2_skipu(d, 1);                      // leap indicator
        decoding_time_offset = bytestream2_get_be16u(d);
        num_of_au            = bytestream2_get_byteu(d);
        if (bytestream2_get_bytes_left(d) < num_of_au * (pts_offset_type == 2 ? 4 : 2))
            return AVERROR_INVALIDDATA;

        if (mpu_seq >= ms->mpu_seq) {
            ext = ff_mmtp_timestamp_slot((void **)&ms->ext_timestamps,
                                         &ms->nb_ext_timestamps, sizeof(*ext),
                                         mpu_seq, ms->mpu_seq);
            if (!ext)
                return AVERROR(ENOMEM);
            av_fast_malloc(&ext->au, &ext->au_alloc, num_of_au * sizeof(*ext->au));
            if (!ext->au && num_of_au) {
                ext->num_of_au = 0;
                return AVERROR(ENOMEM);
            }
            ext->seq_num              = mpu_seq;
            ext->decoding_time_offset = decoding_time_offset;
            ext->num_of_au            = num_of_au;
            /* type 0 means a fixed interval set by operation, not by
             * STD-B60, and type 3 is not defined */
            ext->au_timing            = pts_offset_type == 1 || pts_offset_type == 2;
        }
        for (unsigned i = 0; i < num_of_au; i++) {
            uint16_t dts_pts_offset = bytestream2_get_be16u(d);
            uint16_t pts_offset     = pts_offset_type == 2 ? bytestream2_get_be16u(d)
                                                           : default_pts_offset;
            if (ext) {
                ext->au[i].dts_pts_offset = dts_pts_offset;
                ext->au[i].pts_offset     = pts_offset;
            }
        }
    }
    return 0;
}

int ff_mmtp_arib_parse_asset_descriptor(MMTPContext *ctx, MMTPStream *ms,
                                        GetByteContext *gbc)
{
    GetByteContext d;
    unsigned       tag = bytestream2_peek_be16u(gbc);
    int            ret;

    switch (tag) {
    case VIDEO_COMPONENT_DESCRIPTOR:
    case MH_AUDIO_COMPONENT_DESCRIPTOR:
    case MH_DATA_COMPONENT_DESCRIPTOR:
    case MPU_EXTENDED_TIMESTAMP_DESCRIPTOR:
        break;
    default:
        return ff_mmtp_skip_descriptor(ctx, gbc);
    }
    if ((ret = descriptor_body(gbc, &d)) < 0)
        return ret;

    switch (tag) {
    case VIDEO_COMPONENT_DESCRIPTOR:
        return parse_video_component_descriptor(ms, &d);
    case MH_AUDIO_COMPONENT_DESCRIPTOR:
        return parse_mh_audio_component_descriptor(ms, &d);
    case MH_DATA_COMPONENT_DESCRIPTOR:
        return parse_mh_data_component_descriptor(ms, &d);
    default:
        return parse_mpu_extended_timestamp_descriptor(ms, &d);
    }
}

/* STD-B60 Table 6-8: a list of hdr_ext_end_flag, hdr_ext_type and
 * hdr_ext_length. Type 0x0001 holds the scramble information of STD-B61,
 * whose encryption_flag is 10 (even key) or 11 (odd key) when the payload
 * is scrambled. */
int ff_mmtp_arib_is_scrambled(GetByteContext *gbc)
{
    while (bytestream2_get_bytes_left(gbc) >= 4) {
        unsigned type   = bytestream2_get_be16u(gbc);
        unsigned length = bytestream2_get_be16u(gbc);

        if (bytestream2_get_bytes_left(gbc) < length)
            return 0;
        if ((type & 0x7FFF) == 0x0001 && length >= 1)
            return (bytestream2_peek_byteu(gbc) >> 3 & 3) >= 2;
        bytestream2_skipu(gbc, length);
        if (type & 0x8000)
            break;
    }
    return 0;
}

/* M2 section message: one MPEG-2 style section with a CRC_32 */
int ff_mmtp_arib_parse_message(MMTPContext *ctx, GetByteContext *gbc)
{
    const uint8_t *section;
    unsigned       length, section_length;

    if (bytestream2_peek_be16u(gbc) != M2_SECTION_MESSAGE_ID)
        return 0;
    if (bytestream2_get_bytes_left(gbc) < 5)
        return AVERROR_INVALIDDATA;
    bytestream2_skipu(gbc, 3);                        // message_id, version
    length = bytestream2_get_be16u(gbc);
    if (length < 3 || bytestream2_get_bytes_left(gbc) < length)
        return AVERROR_INVALIDDATA;
    section        = gbc->buffer;
    section_length = AV_RB16(section + 1) & 0xFFF;
    if (section_length < 4 || section_length + 3 > length)
        return AVERROR_INVALIDDATA;
    if (av_crc(av_crc_get_table(AV_CRC_32_IEEE), UINT32_MAX,
               section, section_length + 3))
        return AVERROR_INVALIDDATA;
    ff_mmtp_set_flavor(ctx, MMTP_FLAVOR_ARIB);
    return 0;
}

/* Subtitle MFU, ARIB STD-B60 Table 9-1 */
int ff_mmtp_arib_parse_subtitle_mfu(MMTPContext *ctx, MMTPStream *ms,
                                    GetByteContext *gbc)
{
    unsigned subsample_number, last_subsample_number, flags;
    uint32_t data_size;
    int      length_ext;

    if (bytestream2_get_bytes_left(gbc) < 7)
        return AVERROR_INVALIDDATA;
    bytestream2_skipu(gbc, 2);                        // subtitle_tag, subtitle_sequence_number
    subsample_number      = bytestream2_get_byteu(gbc);
    last_subsample_number = bytestream2_get_byteu(gbc);
    flags                 = bytestream2_get_byteu(gbc);
    length_ext            = flags >> 3 & 1;
    if (flags >> 4)                                   // data_type: only TTML documents
        return 0;
    if (bytestream2_get_bytes_left(gbc) < (length_ext ? 4 : 2))
        return AVERROR_INVALIDDATA;
    data_size = length_ext ? bytestream2_get_be32u(gbc) : bytestream2_get_be16u(gbc);
    if (subsample_number == 0 && last_subsample_number > 0 && flags & 0x04)
        bytestream2_skip(gbc, last_subsample_number * (length_ext ? 5 : 3));
    if (bytestream2_get_bytes_left(gbc) < data_size)
        return AVERROR_INVALIDDATA;

    if (ms->ttml_tmd == TMD_MPU_TIMESTAMP_ORIGIN || ms->ttml_tmd == TMD_MPU_TIMESTAMP)
        return ff_mmtp_queue_au(ctx, ms, gbc, data_size, MMTP_AU_MPU_TIME);
    return ff_mmtp_queue_packet(ctx, ms, gbc, data_size,
                                AV_NOPTS_VALUE, AV_NOPTS_VALUE);
}
