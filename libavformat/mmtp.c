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

#include "libavcodec/bytestream.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "mmtp.h"
#include "mmtp_internal.h"
#include "packet_internal.h"

enum {
    PA_MESSAGE_ID        = 0x0000,
    ARIB_MESSAGE_ID_BASE = 0x8000,
};

enum {
    MMT_PACKAGE_TABLE_ID = 0x20,
};

/* Real flows carry a few fragmented signalling packet_ids, and messages of a
 * few kilobytes; STD-B60 tables have 16-bit lengths. */
#define MAX_SIGNALLING_ASSEMBLERS   32
#define MAX_SIGNALLING_MESSAGE_SIZE (1 << 20)

enum FragmentationIndicator {
    NOT_FRAGMENTED  = 0,
    FIRST_FRAGMENT  = 1,
    MIDDLE_FRAGMENT = 2,
    LAST_FRAGMENT   = 3,
};

enum {
    MPU_FRAGMENT_MFU = 2,
};

struct FragmentAssembler {
    uint16_t           packet_id;
    FragmentAssembler *next;

    uint8_t *data;
    size_t   size, cap;
    int64_t  pos;

    uint32_t last_seq;
    int      max_size;

    enum {
        FRAG_INIT = 0,
        FRAG_NOT_STARTED,
        FRAG_IN_FRAGMENT,
        FRAG_SKIP,
    } state;
};

void ff_mmtp_set_flavor(MMTPContext *ctx, enum MMTPFlavor flavor)
{
    static const char *const names[] = {
        [MMTP_FLAVOR_UNKNOWN] = "ISO/IEC 23008-1",
        [MMTP_FLAVOR_ARIB]    = "ARIB STD-B60",
    };

    if (ctx->flavor != MMTP_FLAVOR_UNKNOWN || flavor == MMTP_FLAVOR_UNKNOWN)
        return;
    ctx->flavor = flavor;
    av_log(ctx->s, AV_LOG_VERBOSE, "program %d: %s signalling\n",
           ctx->program->id, names[flavor]);
}

int ff_mmtp_skip_descriptor(MMTPContext *ctx, GetByteContext *gbc)
{
    /* bytes of descriptor_length for tags 0x0000 to 0x000C, ISO/IEC
     * 23008-1:2023 10.5 and Table 103; 0x0005 has an XML syntax only */
    static const uint8_t length_size[] = { 2, 1, 2, 4, 2, 0, 2, 2, 1, 1, 2, 2, 4 };
    unsigned tag, size, length;

    if (bytestream2_get_bytes_left(gbc) < 2)
        return AVERROR_INVALIDDATA;
    tag = bytestream2_get_be16u(gbc);

    if (tag < FF_ARRAY_ELEMS(length_size))
        size = length_size[tag];
    else if (tag <= 0x3FFF)
        size = 1;
    else if (tag <= 0x6FFF)
        size = 2;
    else if (tag <= 0x7FFF)
        size = 4;
    else if (ctx->flavor == MMTP_FLAVOR_ARIB)        // private: STD-B60 Table 4-10
        size = tag <= 0xEFFF ? 1 : 2;
    else
        size = 0;
    if (!size) {
        av_log(ctx->s, AV_LOG_TRACE, "descriptor 0x%04x has no known length, "
               "skipping the rest of the loop\n", tag);
        bytestream2_skipu(gbc, bytestream2_get_bytes_left(gbc));
        return 0;
    }
    if (bytestream2_get_bytes_left(gbc) < size)
        return AVERROR_INVALIDDATA;
    length = size == 1 ? bytestream2_get_byteu(gbc) :
             size == 2 ? bytestream2_get_be16u(gbc) : bytestream2_get_be32u(gbc);
    av_log(ctx->s, AV_LOG_TRACE, "skipping descriptor 0x%04x\n", tag);

    if (bytestream2_get_bytes_left(gbc) < length)
        return AVERROR_INVALIDDATA;
    bytestream2_skipu(gbc, length);
    return 0;
}

static MMTPStream *find_stream(MMTPContext *ctx, uint16_t packet_id)
{
    for (MMTPStream *ms = ctx->streams; ms; ms = ms->next)
        if (ms->packet_id == packet_id)
            return ms;
    return NULL;
}

MMTPStream *ff_mmtp_get_stream(MMTPContext *ctx, uint16_t packet_id)
{
    MMTPStream *ms = find_stream(ctx, packet_id);
    AVStream   *st;

    if (ms || ctx->parse_flags & MMTP_PARSE_NO_NEW_STREAMS)
        return ms;

    ms = av_mallocz(sizeof(*ms));
    if (!ms)
        return NULL;
    st = avformat_new_stream(ctx->s, NULL);
    if (!st) {
        av_free(ms);
        return NULL;
    }
    if (av_program_add_stream_index2(ctx->s, ctx->program->id, st->index) < 0) {
        ff_remove_stream(ctx->s, st);
        av_free(ms);
        return NULL;
    }
    st->id = packet_id;
    avpriv_set_pts_info(st, 64, 1, 90000);

    ms->st        = st;
    ms->packet_id = packet_id;
    ms->pos       = -1;
    ms->ttml_tmd  = -1;
    ms->next      = ctx->streams;
    ctx->streams  = ms;
    return ms;
}

void *ff_mmtp_timestamp_slot(void **array, int *nb, size_t elem_size,
                             uint32_t mpu_seq, uint32_t current)
{
    uint8_t *base = *array;
    uint32_t far_dist = 0;
    int      i, far = 0;

    /* every cache element starts with its uint32_t MPU sequence number */
#define SEQ(i) (*(const uint32_t *)(base + (i) * elem_size))
    for (i = 0; i < *nb; i++)
        if (SEQ(i) == mpu_seq)
            return base + i * elem_size;
    for (i = 0; i < *nb; i++)
        if (SEQ(i) < current)
            return base + i * elem_size;
    if (*nb < MMTP_MAX_TIMESTAMP_DESCRIPTORS) {
        /* not av_dynarray2_add(): it frees the array on failure, and the
         * elements may own memory; a new element is zeroed */
        uint8_t *tmp = av_realloc_array(base, *nb + 1, elem_size);
        if (!tmp)
            return NULL;
        *array = tmp;
        memset(tmp + *nb * elem_size, 0, elem_size);
        return tmp + (*nb)++ * elem_size;
    }
    for (i = 0; i < *nb; i++) {
        uint32_t dist = SEQ(i) > mpu_seq ? SEQ(i) - mpu_seq : mpu_seq - SEQ(i);
        if (dist > far_dist) {
            far_dist = dist;
            far      = i;
        }
    }
#undef SEQ
    return base + far * elem_size;
}

static int parse_mpu_timestamp_descriptor(MMTPStream *ms, GetByteContext *gbc)
{
    GetByteContext d;
    unsigned       length;

    bytestream2_skipu(gbc, 2);
    length = bytestream2_get_byteu(gbc);
    if (bytestream2_get_bytes_left(gbc) < length)
        return AVERROR_INVALIDDATA;
    bytestream2_init(&d, gbc->buffer, length);
    bytestream2_skipu(gbc, length);

    while (bytestream2_get_bytes_left(&d) >= 12) {
        uint32_t       mpu_seq = bytestream2_get_be32u(&d);
        int64_t        time    = ff_parse_ntp_time(bytestream2_get_be64u(&d)) -
                                 NTP_OFFSET_US;
        MMTPTimestamp *slot;

        if (mpu_seq < ms->mpu_seq)
            continue;
        slot = ff_mmtp_timestamp_slot((void **)&ms->timestamps, &ms->nb_timestamps,
                                      sizeof(*slot), mpu_seq, ms->mpu_seq);
        if (!slot)
            return AVERROR(ENOMEM);
        slot->seq_num           = mpu_seq;
        slot->presentation_time = time;
    }
    return 0;
}

int64_t ff_mmtp_mpu_presentation_time(const MMTPStream *ms, uint32_t mpu_seq)
{
    const AVRational tb = ms->st->time_base;

    for (int i = 0; i < ms->nb_timestamps; i++)
        if (ms->timestamps[i].seq_num == mpu_seq)
            return av_rescale(ms->timestamps[i].presentation_time,
                              tb.den, 1000000LL * tb.num);
    return AV_NOPTS_VALUE;
}

static const MMTPExtTimestamp *find_ext_timestamp(const MMTPStream *ms)
{
    for (int i = 0; i < ms->nb_ext_timestamps; i++)
        if (ms->ext_timestamps[i].seq_num == ms->mpu_seq)
            return &ms->ext_timestamps[i];
    return NULL;
}

/* Whether the timestamps of the current MPU are known. The presentation
 * time is that of the first AU in presentation order, which video, as it
 * may reorder, can only place with the MPU extended timestamp descriptor. */
static int mpu_timed(const MMTPStream *ms)
{
    if (ff_mmtp_mpu_presentation_time(ms, ms->mpu_seq) == AV_NOPTS_VALUE)
        return 0;
    return ms->st->codecpar->codec_type != AVMEDIA_TYPE_VIDEO ||
           find_ext_timestamp(ms);
}

/* Timestamps of access unit au of the current MPU, in decoding order, from
 * the MPU timestamp descriptor (the first AU in presentation order) and the
 * MPU extended timestamp descriptor (ARIB STD-B60 Table 7-82). */
static void au_timestamps(MMTPStream *ms, int au, int64_t *pts, int64_t *dts)
{
    const MMTPExtTimestamp *ext;
    int64_t                 time, t;

    *pts = *dts = AV_NOPTS_VALUE;
    time = ff_mmtp_mpu_presentation_time(ms, ms->mpu_seq);
    if (time == AV_NOPTS_VALUE)
        return;
    if (au == MMTP_AU_MPU_TIME) {
        *pts = *dts = time;
        return;
    }
    if (!(ext = find_ext_timestamp(ms))) {
        if (!au && ms->st->codecpar->codec_type != AVMEDIA_TYPE_VIDEO)
            *pts = time;
        return;
    }
    if (au >= ext->num_of_au || (au > 0 && !ext->au_timing))
        return;

    t = time - ext->decoding_time_offset;
    for (int j = 0; j < au; j++)
        t += ext->au[j].pts_offset;
    *dts = t;
    *pts = t + ext->au[au].dts_pts_offset;
}

/* Time the held AUs of the current MPU, as far as its timestamps are known,
 * and queue them. */
static int release_held(MMTPContext *ctx, MMTPStream *ms)
{
    AVPacket *pkt = ctx->pkt;
    int       ret;

    while (ff_packet_list_get(&ms->held, pkt) >= 0) {
        int au = pkt->pts;

        ms->nb_held--;
        if (au < 0 && ms->was_timed) {
            av_packet_unref(pkt);
            continue;
        }
        pkt->pts = pkt->dts = AV_NOPTS_VALUE;
        if (au >= 0)
            au_timestamps(ms, au, &pkt->pts, &pkt->dts);
        ms->was_timed |= pkt->dts != AV_NOPTS_VALUE || pkt->pts != AV_NOPTS_VALUE;
        if ((ret = ff_packet_list_put(ctx->queue, pkt, NULL, 0)) < 0) {
            av_packet_unref(pkt);
            return ret;
        }
    }
    return 0;
}

static int parse_asset_descriptor(MMTPContext *ctx, MMTPStream *ms,
                                  GetByteContext *gbc)
{
    unsigned tag;

    if (bytestream2_get_bytes_left(gbc) < 3)
        return AVERROR_INVALIDDATA;
    tag = bytestream2_peek_be16u(gbc);
    if (tag == MMTP_MPU_TIMESTAMP_DESCRIPTOR)
        return parse_mpu_timestamp_descriptor(ms, gbc);
    if (tag >= 0x8000 && ctx->flavor == MMTP_FLAVOR_ARIB)
        return ff_mmtp_arib_parse_asset_descriptor(ctx, ms, gbc);
    return ff_mmtp_skip_descriptor(ctx, gbc);
}

/* MMT_general_location_info(): the packet_id of a location in this flow
 * (type 0), -1 for a location elsewhere, or a negative error code. */
static int parse_location(GetByteContext *gbc)
{
    /* bytes before the packet_id (types 0 to 2) or in total (3 and 4) */
    static const uint8_t skip[] = { 0, 4 + 4 + 2, 16 + 16 + 2, 2 + 2 + 2,
                                    16 + 16 + 2 + 2 };
    unsigned type;

    if (bytestream2_get_bytes_left(gbc) < 1)
        return AVERROR_INVALIDDATA;
    type = bytestream2_get_byteu(gbc);
    if (type == 5 || type == 6) {                     // a URL, or private data
        int      size = type == 5 ? 1 : 2;
        unsigned length;

        if (bytestream2_get_bytes_left(gbc) < size)
            return AVERROR_INVALIDDATA;
        length = size == 1 ? bytestream2_get_byteu(gbc) : bytestream2_get_be16u(gbc);
        if (bytestream2_get_bytes_left(gbc) < length)
            return AVERROR_INVALIDDATA;
        bytestream2_skipu(gbc, length);
        return -1;
    }
    if (type >= FF_ARRAY_ELEMS(skip))
        return AVERROR_INVALIDDATA;
    if (bytestream2_get_bytes_left(gbc) < skip[type] + (type <= 2 ? 2 : 0))
        return AVERROR_INVALIDDATA;
    bytestream2_skipu(gbc, skip[type]);
    if (type == 0)
        return bytestream2_get_be16u(gbc);
    if (type <= 2)
        bytestream2_skipu(gbc, 2);
    return -1;
}

/* Skip n bytes only if they are all there. */
static int skip_exact(GetByteContext *gbc, unsigned n)
{
    if (bytestream2_get_bytes_left(gbc) < n)
        return 0;
    bytestream2_skipu(gbc, n);
    return 1;
}

/* Walk an MPT body with ARIB STD-B60's 8-bit asset_id_length (ISO/IEC
 * 23008-1 has 32 bits) and report whether it ends exactly at its length.
 * parse_mmt_package_table() relies on this walk for every length in it. */
static int mpt_has_arib_layout(GetByteContext gbc)
{
    unsigned n;

    if (!skip_exact(&gbc, 1) || bytestream2_get_bytes_left(&gbc) < 1 ||
        !skip_exact(&gbc, bytestream2_get_byteu(&gbc)) ||          // MMT_package_id
        bytestream2_get_bytes_left(&gbc) < 2 ||
        !skip_exact(&gbc, bytestream2_get_be16u(&gbc)) ||          // MPT_descriptors
        bytestream2_get_bytes_left(&gbc) < 1)
        return 0;
    for (n = bytestream2_get_byteu(&gbc); n > 0; n--) {
        unsigned locations;

        if (!skip_exact(&gbc, 5) || bytestream2_get_bytes_left(&gbc) < 1 ||
            !skip_exact(&gbc, bytestream2_get_byteu(&gbc)) ||      // asset_id
            !skip_exact(&gbc, 5) || bytestream2_get_bytes_left(&gbc) < 1)
            return 0;
        for (locations = bytestream2_get_byteu(&gbc); locations > 0; locations--)
            if (parse_location(&gbc) == AVERROR_INVALIDDATA)
                return 0;
        if (bytestream2_get_bytes_left(&gbc) < 2 ||
            !skip_exact(&gbc, bytestream2_get_be16u(&gbc)))        // asset descriptors
            return 0;
    }
    return bytestream2_get_bytes_left(&gbc) == 0;
}

void ff_mmtp_set_codec(MMTPStream *ms, enum AVMediaType type, enum AVCodecID id)
{
    AVCodecParameters *par = ms->st->codecpar;

    par->codec_type = type;
    par->codec_id   = id;
    /* packets are whole access units; the parser only reads headers */
    ffstream(ms->st)->need_parsing = id == AV_CODEC_ID_HEVC ||
                                     id == AV_CODEC_ID_AAC_LATM ?
                                     AVSTREAM_PARSE_HEADERS : AVSTREAM_PARSE_NONE;
    ms->supported = id == AV_CODEC_ID_HEVC || id == AV_CODEC_ID_AAC_LATM ||
                    id == AV_CODEC_ID_TTML;
}

/* MMT package table, ARIB STD-B60 Table 7-6: a stream for each supported
 * asset carried in this flow, with its descriptors. */
static int parse_mmt_package_table(MMTPContext *ctx, GetByteContext *gbc)
{
    GetByteContext t;
    unsigned       length, n;
    int            arib_layout;

    if (bytestream2_get_bytes_left(gbc) < 4)
        return AVERROR_INVALIDDATA;
    bytestream2_skipu(gbc, 2);
    length = bytestream2_get_be16u(gbc);
    if (bytestream2_get_bytes_left(gbc) < length)
        return AVERROR_INVALIDDATA;
    bytestream2_init(&t, gbc->buffer, length);
    bytestream2_skipu(gbc, length);

    arib_layout = mpt_has_arib_layout(t);
    if (arib_layout)
        ff_mmtp_set_flavor(ctx, MMTP_FLAVOR_ARIB);
    if (ctx->flavor != MMTP_FLAVOR_ARIB || !arib_layout) {
        /* MPTs with ISO/IEC 23008-1 32-bit asset ids are not read yet */
        av_log(ctx->s, AV_LOG_VERBOSE, "skipping an MPT without the ARIB layout\n");
        return 0;
    }

    bytestream2_skipu(&t, 1);                         // reserved, MPT_mode
    bytestream2_skipu(&t, bytestream2_get_byteu(&t)); // MMT_package_id
    bytestream2_skipu(&t, bytestream2_get_be16u(&t)); // MPT_descriptors

    for (n = bytestream2_get_byteu(&t); n > 0; n--) {
        GetByteContext d;
        MMTPStream    *ms = NULL;
        uint32_t       asset_type;
        unsigned       locations;
        int            packet_id = -1;

        bytestream2_skipu(&t, 5);                     // identifier_type, asset_id_scheme
        bytestream2_skipu(&t, bytestream2_get_byteu(&t));
        asset_type = bytestream2_get_le32u(&t);
        bytestream2_skipu(&t, 1);                     // asset_clock_relation_flag

        /* keep the first location in this flow; others may follow it */
        for (locations = bytestream2_get_byteu(&t); locations > 0; locations--) {
            int ret = parse_location(&t);
            if (ret >= 0 && packet_id < 0)
                packet_id = ret;
        }

        length = bytestream2_get_be16u(&t);
        bytestream2_init(&d, t.buffer, length);
        bytestream2_skipu(&t, length);

        if (packet_id < 0)
            continue;

        switch (asset_type) {
        /* not 'hvc1': its parameter sets are in the MPU metadata, not read */
        case MKTAG('h', 'e', 'v', '1'):
            if (!(ms = ff_mmtp_get_stream(ctx, packet_id)))
                break;
            if (ms->st->codecpar->codec_id != AV_CODEC_ID_HEVC)
                ff_mmtp_set_codec(ms, AVMEDIA_TYPE_VIDEO, AV_CODEC_ID_HEVC);
            break;
        case MKTAG('m', 'p', '4', 'a'):
            if (!(ms = ff_mmtp_get_stream(ctx, packet_id)))
                break;
            /* STD-B60 sends an AudioMuxElement; the audio component
             * descriptor can still name another coding */
            if (ms->st->codecpar->codec_id == AV_CODEC_ID_NONE)
                ff_mmtp_set_codec(ms, AVMEDIA_TYPE_AUDIO, AV_CODEC_ID_AAC_LATM);
            break;
        case MKTAG('s', 't', 'p', 'p'):
            if (!(ms = ff_mmtp_get_stream(ctx, packet_id)))
                break;
            ms->st->codecpar->codec_type = AVMEDIA_TYPE_SUBTITLE;
            break;
        default:
            continue;
        }
        if (!ms) {
            if (ctx->parse_flags & MMTP_PARSE_NO_NEW_STREAMS)
                continue;
            return AVERROR(ENOMEM);
        }
        ms->st->codecpar->codec_tag = asset_type;

        while (bytestream2_get_bytes_left(&d) > 0) {
            int ret = parse_asset_descriptor(ctx, ms, &d);
            if (ret < 0)
                return ret;
        }
        if (ms->held.head && mpu_timed(ms)) {
            int ret = release_held(ctx, ms);
            if (ret < 0)
                return ret;
        }
    }
    return 0;
}

static int parse_table(MMTPContext *ctx, GetByteContext *gbc)
{
    unsigned table_id, length;

    if (bytestream2_get_bytes_left(gbc) < 4)
        return AVERROR_INVALIDDATA;
    table_id = bytestream2_peek_byteu(gbc);
    if (table_id == MMT_PACKAGE_TABLE_ID)
        return parse_mmt_package_table(ctx, gbc);

    /* table_id, version, length: the header every PA table starts with;
     * nothing else carried in PA messages is needed for demuxing */
    bytestream2_skipu(gbc, 2);
    length = bytestream2_get_be16u(gbc);
    if (bytestream2_get_bytes_left(gbc) < length)
        return AVERROR_INVALIDDATA;
    bytestream2_skipu(gbc, length);
    return 0;
}

static int parse_pa_message(MMTPContext *ctx, GetByteContext *gbc)
{
    GetByteContext m;
    uint32_t       length;
    unsigned       nb_tables;

    if (bytestream2_get_bytes_left(gbc) < 7)
        return AVERROR_INVALIDDATA;
    bytestream2_skipu(gbc, 3);                        // message_id, version
    length = bytestream2_get_be32u(gbc);
    if (bytestream2_get_bytes_left(gbc) < length)
        return AVERROR_INVALIDDATA;
    bytestream2_init(&m, gbc->buffer, length);
    bytestream2_skipu(gbc, length);

    if (bytestream2_get_bytes_left(&m) < 1)
        return AVERROR_INVALIDDATA;
    nb_tables = bytestream2_get_byteu(&m);
    if (bytestream2_get_bytes_left(&m) < nb_tables * 4)
        return AVERROR_INVALIDDATA;
    bytestream2_skipu(&m, nb_tables * 4);             // table_id, version, length

    while (bytestream2_get_bytes_left(&m) > 0) {
        int ret = parse_table(ctx, &m);
        if (ret < 0)
            return ret;
    }
    return 0;
}

static int parse_signalling_message(MMTPContext *ctx, GetByteContext *gbc,
                                    int64_t pos)
{
    unsigned message_id;

    if (bytestream2_get_bytes_left(gbc) < 2)
        return AVERROR_INVALIDDATA;
    message_id = bytestream2_peek_be16u(gbc);
    if (message_id == PA_MESSAGE_ID)
        return parse_pa_message(ctx, gbc);
    if (message_id >= ARIB_MESSAGE_ID_BASE)
        return ff_mmtp_arib_parse_message(ctx, gbc);
    return 0;
}

static int append_data(FragmentAssembler *frag, const uint8_t *data, size_t size)
{
    if (frag->size + size > frag->max_size) {
        frag->size  = 0;
        frag->state = FRAG_SKIP;
        return AVERROR_INVALIDDATA;
    }
    if (frag->cap < frag->size + size) {
        size_t   cap = FFMAX(frag->size + size, frag->cap ? frag->cap * 2 : 1024);
        uint8_t *buf;

        cap = FFMIN(cap, frag->max_size);
        buf = av_realloc(frag->data, cap);
        if (!buf)
            return AVERROR(ENOMEM);
        frag->data = buf;
        frag->cap  = cap;
    }
    memcpy(frag->data + frag->size, data, size);
    frag->size += size;
    return 0;
}

static int check_sequence(MMTPContext *ctx, FragmentAssembler *frag,
                          uint32_t seq_num)
{
    int lost = 0;

    if (frag->state == FRAG_INIT) {
        frag->state = FRAG_SKIP;
    } else if (seq_num != frag->last_seq + 1) {
        lost = 1;
        av_log(ctx->s, AV_LOG_WARNING,
               "packet_id 0x%04x: packet sequence number jump %u -> %u%s\n",
               frag->packet_id, frag->last_seq, seq_num,
               frag->size ? ", dropping a partial unit" : "");
        frag->size  = 0;
        frag->state = FRAG_SKIP;
    }
    frag->last_seq = seq_num;
    return lost;
}

typedef int (*UnitParser)(MMTPContext *ctx, GetByteContext *gbc, int64_t pos);

static int assemble(MMTPContext *ctx, FragmentAssembler *frag,
                    enum FragmentationIndicator indicator,
                    const uint8_t *data, int size, UnitParser parse)
{
    GetByteContext gbc;
    int            ret;

    switch (indicator) {
    case NOT_FRAGMENTED:
        if (frag) {
            frag->size  = 0;
            frag->state = FRAG_NOT_STARTED;
        }
        bytestream2_init(&gbc, data, size);
        return parse(ctx, &gbc, ctx->pos);
    case FIRST_FRAGMENT:
        if (!frag)
            return 0;
        frag->size  = 0;
        frag->pos   = ctx->pos;
        frag->state = FRAG_IN_FRAGMENT;
        return append_data(frag, data, size);
    case MIDDLE_FRAGMENT:
    case LAST_FRAGMENT:
        if (!frag || frag->state != FRAG_IN_FRAGMENT)
            return 0;
        if ((ret = append_data(frag, data, size)) < 0 || indicator == MIDDLE_FRAGMENT)
            return ret;
        bytestream2_init(&gbc, frag->data, frag->size);
        ret = parse(ctx, &gbc, frag->pos);
        frag->size  = 0;
        frag->state = FRAG_NOT_STARTED;
        return ret;
    }
    return AVERROR_INVALIDDATA;
}

static FragmentAssembler *find_assembler(MMTPContext *ctx, uint16_t packet_id)
{
    for (FragmentAssembler *frag = ctx->assembler; frag; frag = frag->next)
        if (frag->packet_id == packet_id)
            return frag;
    return NULL;
}

static FragmentAssembler *new_assembler(MMTPContext *ctx, uint16_t packet_id,
                                        int max_size)
{
    FragmentAssembler *frag = av_mallocz(sizeof(*frag));

    if (!frag)
        return NULL;
    frag->packet_id = packet_id;
    frag->max_size  = max_size;
    frag->next      = ctx->assembler;
    ctx->assembler  = frag;
    return frag;
}

/* Only the first fragment of a message the parser reads gets an assembler,
 * so a flood of packet_ids or of other messages costs no memory. */
static int signalling_assembler(MMTPContext *ctx, FragmentAssembler **frag,
                                enum FragmentationIndicator indicator,
                                GetByteContext *gbc)
{
    unsigned message_id;

    if ((*frag = find_assembler(ctx, ctx->current_pid)) ||
        indicator != FIRST_FRAGMENT || bytestream2_get_bytes_left(gbc) < 2)
        return 0;
    message_id = bytestream2_peek_be16u(gbc);
    if (message_id != PA_MESSAGE_ID && message_id < ARIB_MESSAGE_ID_BASE)
        return 0;
    if (ctx->nb_signalling_assemblers >= MAX_SIGNALLING_ASSEMBLERS) {
        if (!ctx->warned_assemblers) {
            av_log(ctx->s, AV_LOG_WARNING, "too many fragmented signalling "
                   "packet_ids, ignoring 0x%04x\n", ctx->current_pid);
            ctx->warned_assemblers = 1;
        }
        return 0;
    }
    if (!(*frag = new_assembler(ctx, ctx->current_pid, MAX_SIGNALLING_MESSAGE_SIZE)))
        return AVERROR(ENOMEM);
    ctx->nb_signalling_assemblers++;
    return 0;
}

static int parse_signalling_messages(MMTPContext *ctx, uint32_t seq_num,
                                     GetByteContext *gbc)
{
    FragmentAssembler          *frag;
    enum FragmentationIndicator indicator;
    int                         length_extension, aggregation, ret;

    if (bytestream2_get_bytes_left(gbc) < 2)
        return AVERROR_INVALIDDATA;
    indicator        = bytestream2_peek_byteu(gbc) >> 6;
    length_extension = bytestream2_peek_byteu(gbc) >> 1 & 1;
    aggregation      = bytestream2_peek_byteu(gbc) & 1;
    bytestream2_skipu(gbc, 2);                        // flags, fragment_counter

    if ((ret = signalling_assembler(ctx, &frag, indicator, gbc)) < 0)
        return ret;
    if (frag)
        check_sequence(ctx, frag, seq_num);

    if (!aggregation)
        return assemble(ctx, frag, indicator, gbc->buffer,
                        bytestream2_get_bytes_left(gbc), parse_signalling_message);
    if (indicator != NOT_FRAGMENTED)
        return AVERROR_INVALIDDATA;

    while (bytestream2_get_bytes_left(gbc) > 0) {
        uint32_t length;

        if (bytestream2_get_bytes_left(gbc) < (length_extension ? 4 : 2))
            return AVERROR_INVALIDDATA;
        length = length_extension ? bytestream2_get_be32u(gbc)
                                  : bytestream2_get_be16u(gbc);
        if (bytestream2_get_bytes_left(gbc) < length)
            return AVERROR_INVALIDDATA;
        ret = assemble(ctx, frag, NOT_FRAGMENTED, gbc->buffer, length,
                       parse_signalling_message);
        if (ret < 0)
            return ret;
        bytestream2_skipu(gbc, length);
    }
    return 0;
}

/* Fill pkt with prefix and size bytes of gbc. */
static int new_packet(AVPacket *pkt, const uint8_t *prefix, int prefix_size,
                      GetByteContext *gbc, int size)
{
    int ret = av_new_packet(pkt, prefix_size + size);

    if (ret < 0)
        return ret;
    if (prefix_size)
        memcpy(pkt->data, prefix, prefix_size);
    bytestream2_get_bufferu(gbc, pkt->data + prefix_size, size);
    return 0;
}

/* Move pkt to list, or unreference it on failure. */
static int put_packet(MMTPStream *ms, PacketList *list, AVPacket *pkt,
                      int64_t pts, int64_t dts)
{
    int ret;

    pkt->stream_index = ms->st->index;
    pkt->pts          = pts;
    pkt->dts          = dts;
    pkt->pos          = ms->pos;
    ret = ff_packet_list_put(list, pkt, NULL, 0);
    if (ret < 0)
        av_packet_unref(pkt);
    return ret;
}

int ff_mmtp_queue_packet(MMTPContext *ctx, MMTPStream *ms,
                         GetByteContext *gbc, int size,
                         int64_t pts, int64_t dts)
{
    int ret = new_packet(ctx->pkt, NULL, 0, gbc, size);

    if (ret < 0)
        return ret;
    return put_packet(ms, ctx->queue, ctx->pkt, pts, dts);
}

/* Queue pkt as AU au of the current MPU, -1 if its index is unreliable.
 * ARIB STD-B60 sends the MPU timestamps before the MPU, so after a seek the
 * first AUs arrive before them; those are held until the MPT does. An
 * untimed AU joins held ones to keep its place, and at most MMTP_MAX_AU
 * are held. Once the stream has been timed, an AU that cannot be is
 * dropped: libavformat would extrapolate its timestamps from the last
 * packet, which may be minutes older when the stream paused. */
static int queue_au(MMTPContext *ctx, MMTPStream *ms, AVPacket *pkt, int au)
{
    int64_t pts = AV_NOPTS_VALUE, dts = AV_NOPTS_VALUE;
    int     timed = mpu_timed(ms), ret;

    if (au < 0 && ms->was_timed) {
        av_packet_unref(pkt);
        return 0;
    }
    if (ms->held.head || (au >= 0 && !timed)) {
        if ((ret = put_packet(ms, &ms->held, pkt, au, au)) < 0)
            return ret;
        if (++ms->nb_held <= MMTP_MAX_AU && !timed)
            return 0;
        return release_held(ctx, ms);
    }
    if (au >= 0)
        au_timestamps(ms, au, &pts, &dts);
    ms->was_timed |= pts != AV_NOPTS_VALUE || dts != AV_NOPTS_VALUE;
    return put_packet(ms, ctx->queue, pkt, pts, dts);
}

int ff_mmtp_queue_au(MMTPContext *ctx, MMTPStream *ms,
                     GetByteContext *gbc, int size, int au)
{
    int ret = new_packet(ctx->pkt, NULL, 0, gbc, size);

    if (ret < 0)
        return ret;
    return queue_au(ctx, ms, ctx->pkt, au);
}

static int append_au(MMTPStream *ms, const uint8_t *data, int size)
{
    if (size > INT_MAX - AV_INPUT_BUFFER_PADDING_SIZE - ms->au_size)
        return AVERROR_INVALIDDATA;
    if (ms->au_cap < ms->au_size + size) {
        /* a new buffer starts small: sizing it after the previous AU keeps
         * a key picture's size allocated while that picture is queued */
        int      cap = FFMIN(FFMAX3(ms->au_size + size, 2LL * ms->au_cap, 16384),
                             INT_MAX - AV_INPUT_BUFFER_PADDING_SIZE);
        uint8_t *buf = av_realloc(ms->au, cap + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!buf)
            return AVERROR(ENOMEM);
        ms->au     = buf;
        ms->au_cap = cap;
    }
    memcpy(ms->au + ms->au_size, data, size);
    ms->au_size += size;
    return 0;
}

/* Drop a NAL unit whose fragments did not all arrive. */
static void drop_nal(MMTPStream *ms)
{
    if (!ms->nal_open)
        return;
    ms->au_size  = ms->nal_start;
    ms->au_vcl   = ms->nal_prev_vcl;
    ms->nal_open = 0;
}

/* Queue the access unit being assembled, if any; its buffer becomes the
 * packet's. Its timestamps are looked up only now: the MPT that carries
 * them may arrive while a large picture is still being received. */
static int flush_au(MMTPContext *ctx, MMTPStream *ms)
{
    AVPacket *pkt = ctx->pkt;
    int64_t   pos = ms->pos;
    int       ret;

    if (!ms->au_open)
        return 0;
    ms->au_open = 0;
    drop_nal(ms);
    if (!ms->au_vcl) {                                // e.g. cut off at the end
        ms->au_size = 0;
        return 0;
    }
    if (ms->au_cap - ms->au_size > ms->au_size / 4) { // no slack in the packet
        uint8_t *buf = av_realloc(ms->au, ms->au_size + AV_INPUT_BUFFER_PADDING_SIZE);
        if (buf) {
            ms->au     = buf;
            ms->au_cap = ms->au_size;
        }
    }
    memset(ms->au + ms->au_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
    if ((ret = av_packet_from_data(pkt, ms->au, ms->au_size)) < 0) {
        ms->au_size = 0;
        return ret;
    }
    ms->au      = NULL;
    ms->au_size = ms->au_cap = 0;

    ms->pos = ms->au_pos;
    /* after a packet loss the index may be off: no timestamps then */
    ret = queue_au(ctx, ms, pkt, ms->au_untimed ? -1 : ms->au_index);
    ms->pos = pos;
    return ret;
}

int ff_mmtp_flush(MMTPContext *ctx)
{
    for (MMTPStream *ms = ctx->streams; ms; ms = ms->next) {
        int ret = flush_au(ctx, ms);
        if (ret < 0 || (ret = release_held(ctx, ms)) < 0)
            return ret;
    }
    return 0;
}

/* Whether a NAL unit starts an access unit, as in H.265 7.4.2.4.4.
 * Broadcasts leave the sample_number and offset fields of the MFU header at
 * 0, so they cannot mark the boundaries. */
static int hevc_starts_au(MMTPStream *ms, const uint8_t *nal, int size)
{
    int type, vcl;

    if (size < 3)
        return 0;
    type = nal[0] >> 1 & 0x3F;
    vcl  = type < 32;
    if (vcl) {
        /* first_slice_segment_in_pic_flag */
        int first = ms->in_sample && (nal[2] & 0x80);
        ms->in_sample = 1;
        return first;
    }
    if (type == 35 /* AUD */ ||
        (ms->in_sample && (type <= 34 || type == 39 /* prefix SEI */ ||
                           (type >= 41 && type <= 44) || (type >= 48 && type <= 55)))) {
        int start = type == 35 || ms->in_sample;
        ms->in_sample = 0;
        return start;
    }
    return 0;
}

/* An HEVC MFU, or a fragment of one, appended straight to the access unit
 * with its 4-byte NAL unit length turned into a start code. The first
 * fragment must hold the bytes the access unit checks read. */
static int hevc_data_unit(MMTPContext *ctx, MMTPStream *ms, FragmentAssembler *frag,
                          enum FragmentationIndicator indicator,
                          const uint8_t *data, int size)
{
    static const uint8_t start_code[3] = { 0, 0, 1 };

    uint32_t nal_size;
    int      ret;

    switch (indicator) {
    case NOT_FRAGMENTED:
    case FIRST_FRAGMENT:
        drop_nal(ms);
        frag->state = FRAG_NOT_STARTED;
        if (size < 4)
            return AVERROR_INVALIDDATA;
        nal_size = AV_RB32(data);
        data += 4;
        size -= 4;
        if (indicator == NOT_FRAGMENTED ? nal_size != size
                                        : nal_size <= size || size < 3)
            return AVERROR_INVALIDDATA;
        ms->pos = ctx->pos;
        /* the first unit of an MPU starts its first AU; the detector still
         * has to see it */
        if (hevc_starts_au(ms, data, size) || !ms->started_mpu) {
            if ((ret = flush_au(ctx, ms)) < 0)
                return ret;
            ms->au_index   = ms->au_count++;
            ms->au_untimed = ms->au_index_lost;
            ms->au_vcl     = 0;
            ms->au_pos     = ctx->pos;
            ms->au_open    = 1;
        }
        ms->started_mpu = 1;
        if (!ms->au_open)
            return 0;                                 // the start of this AU was lost
        ms->nal_start    = ms->au_size;
        ms->nal_size     = nal_size;
        ms->nal_prev_vcl = ms->au_vcl;
        if ((ret = append_au(ms, start_code, sizeof(start_code))) < 0 ||
            (ret = append_au(ms, data, size)) < 0)
            return ret;
        ms->au_vcl |= size > 0 && (data[0] >> 1 & 0x3F) < 32;
        if (indicator == FIRST_FRAGMENT) {
            ms->nal_open = 1;
            frag->state  = FRAG_IN_FRAGMENT;
        }
        return 0;
    case MIDDLE_FRAGMENT:
    case LAST_FRAGMENT: {
        uint32_t left;

        if (frag->state != FRAG_IN_FRAGMENT || !ms->nal_open)
            return 0;
        left = ms->nal_size - (ms->au_size - ms->nal_start - sizeof(start_code));
        if (indicator == LAST_FRAGMENT ? size != left : size > left) {
            drop_nal(ms);
            frag->state = FRAG_NOT_STARTED;
            return AVERROR_INVALIDDATA;
        }
        if ((ret = append_au(ms, data, size)) < 0)
            return ret;
        if (indicator == LAST_FRAGMENT) {
            ms->nal_open = 0;
            frag->state  = FRAG_NOT_STARTED;
        }
        return 0;
    }
    }
    return AVERROR_INVALIDDATA;
}

/* One MFU: each AAC frame gets a LOAS header, and TTML goes to the ARIB
 * part. */
static int consume_mfu(MMTPContext *ctx, GetByteContext *gbc, int64_t pos)
{
    MMTPStream *ms = find_stream(ctx, ctx->current_pid);
    int         size, au, ret;

    if (!ms)
        return 0;
    ms->pos = pos;

    switch (ms->st->codecpar->codec_id) {
    case AV_CODEC_ID_AAC_LATM: {
        uint8_t loas[3];

        size = bytestream2_get_bytes_left(gbc);
        if (size >= 1 << 13)
            return AVERROR_INVALIDDATA;
        loas[0] = 0x56;
        loas[1] = 0xE0 | size >> 8;
        loas[2] = size;
        au = ms->au_count++;
        if ((ret = new_packet(ctx->pkt, loas, sizeof(loas), gbc, size)) < 0)
            return ret;
        return queue_au(ctx, ms, ctx->pkt, ms->au_index_lost ? -1 : au);
    }
    case AV_CODEC_ID_TTML:
        return ff_mmtp_arib_parse_subtitle_mfu(ctx, ms, gbc);
    }
    return 0;
}

/* Every data unit, also every fragment of one, starts with a header: for
 * timed media movie_fragment_sequence_number, sample_number, offset,
 * priority and dependency_counter; for non-timed media item_id. */
static int parse_data_unit(MMTPContext *ctx, MMTPStream *ms, FragmentAssembler *frag,
                           enum FragmentationIndicator indicator, int timed,
                           const uint8_t *data, int size)
{
    int header = timed ? 14 : 4;

    if (size < header)
        return AVERROR_INVALIDDATA;
    if (!timed)
        return 0;                                     // no non-timed media is read
    if (ms->st->codecpar->codec_id == AV_CODEC_ID_HEVC)
        return hevc_data_unit(ctx, ms, frag, indicator, data + header, size - header);
    return assemble(ctx, frag, indicator, data + header, size - header, consume_mfu);
}

/* MPU payload, ARIB STD-B60 Table 6-1: its header, then one data unit or
 * an aggregation of them. */
static int parse_mpu(MMTPContext *ctx, uint32_t seq_num, GetByteContext *gbc)
{
    FragmentAssembler          *frag;
    MMTPStream                 *ms = find_stream(ctx, ctx->current_pid);
    enum FragmentationIndicator indicator;
    uint32_t                    mpu_seq;
    unsigned                    length;
    int                         timed, aggregation, fragment_type, ret;

    if (!ms || !ms->supported || ms->st->discard >= AVDISCARD_ALL)
        return 0;
    if (!(frag = find_assembler(ctx, ctx->current_pid)) &&
        !(frag = new_assembler(ctx, ctx->current_pid,
                               INT_MAX - AV_INPUT_BUFFER_PADDING_SIZE)))
        return AVERROR(ENOMEM);

    if (bytestream2_get_bytes_left(gbc) < 8)
        return AVERROR_INVALIDDATA;
    length = bytestream2_get_be16u(gbc);
    if (length != bytestream2_get_bytes_left(gbc))
        return AVERROR_INVALIDDATA;
    fragment_type = bytestream2_peek_byteu(gbc) >> 4;
    timed         = bytestream2_peek_byteu(gbc) >> 3 & 1;
    indicator     = bytestream2_peek_byteu(gbc) >> 1 & 3;
    aggregation   = bytestream2_peek_byteu(gbc) & 1;
    bytestream2_skipu(gbc, 2);                        // flags, fragment_counter
    mpu_seq = bytestream2_get_be32u(gbc);

    if (aggregation && indicator != NOT_FRAGMENTED)
        return AVERROR_INVALIDDATA;
    if (fragment_type != MPU_FRAGMENT_MFU)
        return 0;
    if (frag->state == FRAG_INIT && !ctx->is_rap)
        return 0;                                     // wait for a random access point

    if (frag->state == FRAG_INIT || mpu_seq != ms->mpu_seq) {
        if (frag->state != FRAG_INIT && mpu_seq != ms->mpu_seq + 1)
            av_log(ctx->s, AV_LOG_WARNING, "packet_id 0x%04x: MPU sequence "
                   "number jump %u -> %u\n", ms->packet_id, ms->mpu_seq, mpu_seq);
        /* an AU never spans MPUs, and the timestamps of the old one may be
         * evicted once mpu_seq moves on */
        if ((ret = flush_au(ctx, ms)) < 0 || (ret = release_held(ctx, ms)) < 0)
            return ret;
        ms->mpu_seq     = mpu_seq;
        ms->au_count    = 0;
        ms->au_index_lost = 0;
        ms->started_mpu = 0;
        ms->in_sample   = 0;
    }

    if (check_sequence(ctx, frag, seq_num)) {
        ms->au_index_lost = 1;
        drop_nal(ms);
    }

    if (!aggregation)
        return parse_data_unit(ctx, ms, frag, indicator, timed, gbc->buffer,
                               bytestream2_get_bytes_left(gbc));

    while (bytestream2_get_bytes_left(gbc) > 0) {
        if (bytestream2_get_bytes_left(gbc) < 2)
            return AVERROR_INVALIDDATA;
        length = bytestream2_get_be16u(gbc);
        if (bytestream2_get_bytes_left(gbc) < length)
            return AVERROR_INVALIDDATA;
        ret = parse_data_unit(ctx, ms, frag, NOT_FRAGMENTED, timed, gbc->buffer, length);
        if (ret < 0)
            return ret;
        bytestream2_skipu(gbc, length);
    }
    return 0;
}

MMTPContext *ff_mmtp_parse_open(AVFormatContext *s, AVProgram *program,
                                PacketList *queue)
{
    MMTPContext *ctx = av_mallocz(sizeof(*ctx));

    if (!ctx)
        return NULL;
    ctx->pkt = av_packet_alloc();
    if (!ctx->pkt) {
        av_free(ctx);
        return NULL;
    }
    ctx->s       = s;
    ctx->program = program;
    ctx->queue   = queue;
    return ctx;
}

int ff_mmtp_parse_packet(MMTPContext *ctx, const uint8_t *buf, int size,
                         int64_t pos, int flags)
{
    GetByteContext gbc;
    uint32_t       seq_num;
    int            version, packet_counter, fec_type, extension, payload_type;

    bytestream2_init(&gbc, buf, size);
    if (bytestream2_get_bytes_left(&gbc) < 12)
        return AVERROR_INVALIDDATA;

    version        = bytestream2_peek_byteu(&gbc) >> 6;
    packet_counter = bytestream2_peek_byteu(&gbc) >> 5 & 1;
    fec_type       = bytestream2_peek_byteu(&gbc) >> 3 & 3;
    extension      = bytestream2_peek_byteu(&gbc) >> 1 & 1;
    ctx->is_rap    = bytestream2_peek_byteu(&gbc) & 1;
    /* version 1, the QoS header of ISO/IEC 23008-1 9.2, is not supported
     * yet. Packets are dropped one by one: a single damaged header must not
     * disable the whole flow. */
    if (version) {
        if (version == 1 && !ctx->warned_version1) {
            avpriv_report_missing_feature(ctx->s, "MMTP version 1 (QoS header)");
            ctx->warned_version1 = 1;
        }
        return AVERROR_INVALIDDATA;
    }
    /* FEC repair packets carry no media, and a source packet ends with a
     * 32-bit source_FEC_payload_ID (ISO/IEC 23008-1 Table 8, Figure 8) */
    if (fec_type >= 2)
        return 0;
    if (fec_type == 1) {
        if (size < 12 + 4)
            return AVERROR_INVALIDDATA;
        bytestream2_init(&gbc, buf, size - 4);
    }
    bytestream2_skipu(&gbc, 1);
    payload_type     = bytestream2_get_byteu(&gbc) & 0x3F;
    ctx->current_pid = bytestream2_get_be16u(&gbc);
    bytestream2_skipu(&gbc, 4);                       // distribute_timestamp
    seq_num  = bytestream2_get_be32u(&gbc);
    ctx->pos = pos;
    ctx->parse_flags = flags;

    if (packet_counter)
        bytestream2_skip(&gbc, 4);
    if (extension) {
        GetByteContext ext;
        unsigned       type, length;

        if (bytestream2_get_bytes_left(&gbc) < 4)
            return AVERROR_INVALIDDATA;
        type   = bytestream2_get_be16u(&gbc);
        length = bytestream2_get_be16u(&gbc);
        if (bytestream2_get_bytes_left(&gbc) < length)
            return AVERROR_INVALIDDATA;
        bytestream2_init(&ext, gbc.buffer, length);
        bytestream2_skipu(&gbc, length);
        if (type == 0x0000 && payload_type == 0x00 &&
            ff_mmtp_arib_is_scrambled(&ext)) {
            if (!ctx->warned_scrambled) {
                av_log(ctx->s, AV_LOG_WARNING, "packet_id 0x%04x is scrambled, "
                       "its payload is skipped\n", ctx->current_pid);
                ctx->warned_scrambled = 1;
            }
            return 0;
        }
    }

    switch (payload_type) {
    case 0x00:
        if (flags & MMTP_PARSE_SIGNALLING_ONLY)
            return 0;
        return parse_mpu(ctx, seq_num, &gbc);
    case 0x02:
        return parse_signalling_messages(ctx, seq_num, &gbc);
    }
    return 0;
}

void ff_mmtp_reset_state(MMTPContext *ctx)
{
    for (FragmentAssembler *frag = ctx->assembler; frag; frag = frag->next) {
        frag->state = FRAG_INIT;
        frag->size  = 0;
    }
    /* The timestamp caches are kept: they are keyed by MPU sequence number,
     * and the timestamp searches of a seek fill them with the MPUs around the
     * target, which the MPT may only time after their first access units. */
    for (MMTPStream *ms = ctx->streams; ms; ms = ms->next) {
        ms->mpu_seq           = 0;
        ms->au_count          = 0;
        ms->au_index_lost     = 0;
        ms->was_timed         = 0;
        ff_packet_list_free(&ms->held);
        ms->nb_held           = 0;
        ms->started_mpu       = 0;
        ms->in_sample         = 0;
        ms->pos               = -1;
        ms->au_open           = 0;
        ms->au_size           = 0;
        ms->nal_open          = 0;
    }
}

void ff_mmtp_parse_close(MMTPContext **pctx)
{
    MMTPContext *ctx = *pctx;

    if (!ctx)
        return;
    for (FragmentAssembler *frag = ctx->assembler, *next; frag; frag = next) {
        next = frag->next;
        av_free(frag->data);
        av_free(frag);
    }
    for (MMTPStream *ms = ctx->streams, *next; ms; ms = next) {
        next = ms->next;
        av_free(ms->timestamps);
        for (int i = 0; i < ms->nb_ext_timestamps; i++)
            av_free(ms->ext_timestamps[i].au);
        av_free(ms->ext_timestamps);
        av_free(ms->au);
        ff_packet_list_free(&ms->held);
        av_free(ms);
    }
    av_packet_free(&ctx->pkt);
    av_freep(pctx);
}
