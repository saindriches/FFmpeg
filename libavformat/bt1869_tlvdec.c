/*
 * ITU-R BT.1869 TLV multiplexing demuxer
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

/**
 * @file
 * TLV packets (ITU-R BT.1869, first specified as ARIB STD-B32) carry IP
 * packets, optionally with compressed headers. Each UDP flow found in them
 * is handed to an MMTP parser as one program.
 */

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"
#include "mmtp.h"
#include "libavcodec/packet_internal.h"

#define TLV_SYNC 0x7F

enum {
    TLV_IPV4_PACKET               = 0x01,
    TLV_IPV6_PACKET               = 0x02,
    TLV_COMPRESSED_IP_PACKET      = 0x03,
    TLV_TRANSMISSION_CONTROL      = 0xFE,
    TLV_NULL_PACKET               = 0xFF,
};

enum {
    CID_PARTIAL_IPV4_UDP = 0x20,
    CID_IPV4             = 0x21,
    CID_PARTIAL_IPV6_UDP = 0x60,
    CID_NO_COMPRESSION   = 0x61,
};

#define UDP_PROTOCOL 17
#define NTP_PORT     123
#define MAX_RESYNC   (65536 + 4096)
#define MAX_HELD     32
#define MAX_FLOWS    256

typedef struct TLVFlow {
    int            context_id;   ///< compressed IP context, -1 for full headers
    uint8_t        key[36];      ///< full headers: source and destination address and port
    int            key_size;
    MMTPContext   *mmtp;
    struct TLVFlow *next;
} TLVFlow;

typedef struct TLVContext {
    TLVFlow   *flows;
    int        nb_flows;
    int        next_flow_id;
    PacketList queue;
    int64_t    queue_end;        ///< input position after the queued packets
    PacketList held;             ///< timed subtitles read before first_pts was known
    int        nb_held;

    int64_t  last_pos;
    int      resync_size;
    int      resync_fails;
    int      warned_ipv4_cid;
    int      warned_flows;
    int      parse_flags;        ///< MMTP_PARSE_* for the media reads
    int      flushed;            ///< end of input reached and flows flushed
    int64_t  read_limit;         ///< if nonzero, read_queued() stops at this position

    uint8_t *buf;
    int      buf_size;

    int64_t  first_pts;          ///< in AV_TIME_BASE units
} TLVContext;

static int tlv_valid_type(uint8_t type)
{
    return type == TLV_IPV4_PACKET || type == TLV_IPV6_PACKET ||
           type == TLV_COMPRESSED_IP_PACKET ||
           type == TLV_TRANSMISSION_CONTROL || type == TLV_NULL_PACKET;
}

static int bt1869_tlv_probe(const AVProbeData *p)
{
    int processed = 0, recognized = 0;

    for (int i = 0; i + 4 < p->buf_size && processed < 100;) {
        int type, length;

        if (p->buf[i] != TLV_SYNC) {
            i++;
            continue;
        }
        processed++;
        type   = p->buf[i + 1];
        length = AV_RB16(p->buf + i + 2);
        i += 4;

        if (type == TLV_COMPRESSED_IP_PACKET) {
            if (length < 3 || i + 2 >= p->buf_size)
                continue;
            switch (p->buf[i + 2]) {
            case CID_PARTIAL_IPV4_UDP:
            case CID_IPV4:
            case CID_PARTIAL_IPV6_UDP:
            case CID_NO_COMPRESSION:
                recognized++;
                i += length;
            }
        } else if (type == TLV_IPV4_PACKET) {
            /* version and total_length must match the TLV packet */
            if (length < 20 || i + 4 > p->buf_size || p->buf[i] >> 4 != 4 ||
                AV_RB16(p->buf + i + 2) != length)
                continue;
            recognized++;
            i += length;
        } else if (type == TLV_IPV6_PACKET) {
            /* version and payload_length must match the TLV packet */
            if (length < 40 || i + 6 > p->buf_size || p->buf[i] >> 4 != 6 ||
                AV_RB16(p->buf + i + 4) != length - 40)
                continue;
            recognized++;
            i += length;
        } else if (type == TLV_NULL_PACKET) {
            int j;
            for (j = i; j < i + length && j < p->buf_size; j++)
                if (p->buf[j] != 0xFF)
                    break;
            if (j < i + length && j < p->buf_size)
                continue;
            recognized++;
            i += length;
        }
    }
    return recognized * AVPROBE_SCORE_MAX / FFMAX(processed, 10);
}

/* Find the flow of a packet or create it. *pflow is left NULL when no new
 * flow is wanted: during the duration scan, or past MAX_FLOWS, which keeps
 * the linear lookup cheap on hostile input. */
static int get_flow(AVFormatContext *s, int context_id, const uint8_t *key,
                    int key_size, int flags, TLVFlow **pflow)
{
    TLVContext *tlv = s->priv_data;
    TLVFlow    *flow;
    AVProgram  *program;
    int         id;

    *pflow = NULL;
    for (flow = tlv->flows; flow; flow = flow->next) {
        if (context_id >= 0 ? flow->context_id == context_id :
            flow->context_id < 0 && flow->key_size == key_size &&
            !memcmp(flow->key, key, key_size)) {
            *pflow = flow;
            return 0;
        }
    }

    if (flags & MMTP_PARSE_NO_NEW_STREAMS)
        return 0;
    if (tlv->nb_flows >= MAX_FLOWS) {
        if (!tlv->warned_flows)
            av_log(s, AV_LOG_WARNING, "more than %d flows, ignoring new ones\n",
                   MAX_FLOWS);
        tlv->warned_flows = 1;
        return 0;
    }
    flow = av_mallocz(sizeof(*flow));
    if (!flow)
        return AVERROR(ENOMEM);
    /* compressed flows keep their context_id as program id */
    id      = context_id >= 0 ? context_id : 0x1000 + tlv->next_flow_id++;
    program = av_new_program(s, id);
    if (!program || !(flow->mmtp = ff_mmtp_parse_open(s, program, &tlv->queue))) {
        av_free(flow);
        return AVERROR(ENOMEM);
    }
    flow->context_id = context_id;
    if (key)
        memcpy(flow->key, key, key_size);
    flow->key_size = key_size;
    flow->next = tlv->flows;
    tlv->flows = flow;
    tlv->nb_flows++;
    *pflow = flow;
    return 0;
}

/* addr points to the source and the destination address, each addr_size
 * bytes, as both IPv4 and IPv6 headers store them. */
static int feed_udp(AVFormatContext *s, const uint8_t *addr, int addr_size,
                    const uint8_t *udp, int size, int64_t pos, int flags)
{
    TLVFlow    *flow;
    uint8_t     key[36];
    int         length, ret;

    if (size < 8)
        return AVERROR_INVALIDDATA;
    length = AV_RB16(udp + 4);
    if (length < 8 || length > size)
        return AVERROR_INVALIDDATA;
    if (AV_RB16(udp + 2) == NTP_PORT)
        return 0;
    /* a flow is identified by both endpoints */
    memcpy(key, addr, 2 * addr_size);
    memcpy(key + 2 * addr_size, udp, 4);
    if ((ret = get_flow(s, -1, key, 2 * addr_size + 4, flags, &flow)) < 0 || !flow)
        return ret;
    return ff_mmtp_parse_packet(flow->mmtp, udp + 8, length - 8, pos, flags);
}

static int read_ip_packet(AVFormatContext *s, const uint8_t *buf, int size,
                          int version, int64_t pos, int flags)
{
    int header;

    if (version == 4) {
        if (size < 20 || buf[0] >> 4 != 4)
            return AVERROR_INVALIDDATA;
        header = (buf[0] & 0xF) * 4;
        if (header < 20 || header > size)
            return AVERROR_INVALIDDATA;
        if (buf[9] != UDP_PROTOCOL || AV_RB16(buf + 6) & 0x3FFF)  // or a fragment
            return 0;
        return feed_udp(s, buf + 12, 4, buf + header, size - header, pos, flags);
    }
    if (size < 40 || buf[0] >> 4 != 6)
        return AVERROR_INVALIDDATA;
    if (buf[6] != UDP_PROTOCOL)                       // extension headers are not followed
        return 0;
    return feed_udp(s, buf + 8, 16, buf + 40, size - 40, pos, flags);
}

static int read_compressed_ip_packet(AVFormatContext *s, const uint8_t *buf,
                                     int size, int64_t pos, int flags)
{
    TLVContext *tlv = s->priv_data;
    TLVFlow    *flow;
    int         context_id, ret;

    if (size < 3)
        return AVERROR_INVALIDDATA;
    context_id = AV_RB16(buf) >> 4;

    switch (buf[2]) {
    case CID_PARTIAL_IPV6_UDP:
        /* IPv6 header without payload_length, UDP header without length
         * and checksum */
        if (size < 3 + 38 + 4)
            return AVERROR_INVALIDDATA;
        buf  += 3 + 38 + 4;
        size -= 3 + 38 + 4;
        break;
    case CID_NO_COMPRESSION:
        buf  += 3;
        size -= 3;
        break;
    case CID_PARTIAL_IPV4_UDP:
    case CID_IPV4:
        if (!tlv->warned_ipv4_cid) {
            avpriv_report_missing_feature(s, "Compressed IPv4 headers");
            tlv->warned_ipv4_cid = 1;
        }
        return 0;
    default:
        return AVERROR_INVALIDDATA;
    }

    if ((ret = get_flow(s, context_id, NULL, 0, flags, &flow)) < 0 || !flow)
        return ret;
    return ff_mmtp_parse_packet(flow->mmtp, buf, size, pos, flags);
}

static void reset_flows(AVFormatContext *s)
{
    TLVContext *tlv = s->priv_data;

    for (TLVFlow *flow = tlv->flows; flow; flow = flow->next)
        ff_mmtp_reset_state(flow->mmtp);
    avpriv_packet_list_free(&tlv->queue);
    avpriv_packet_list_free(&tlv->held);
    tlv->nb_held = 0;
    tlv->flushed = 0;
}

/* Find the next TLV packet: a sync byte and a known packet_type, followed by
 * another sync byte where its length says, or by the end of the file. 0x7F
 * is common inside payloads, so the second header is what makes it safe. */
static int resync(AVFormatContext *s)
{
    TLVContext *tlv   = s->priv_data;
    AVIOContext *pb   = s->pb;
    int64_t      start = avio_tell(pb), pos;
    uint8_t      header[4];
    int          ret;

    for (pos = start; pos - start < tlv->resync_size; pos++) {
        int length;

        if (avio_seek(pb, pos, SEEK_SET) < 0)
            return AVERROR_EOF;
        if ((ret = ffio_ensure_seekback(pb, 4)) < 0)
            return ret;
        if ((ret = avio_read(pb, header, 4)) != 4)
            return ret < 0 && ret != AVERROR_EOF ? ret : AVERROR_EOF;
        if (header[0] != TLV_SYNC || !tlv_valid_type(header[1]))
            continue;

        length = AV_RB16(header + 2);
        if (avio_seek(pb, pos, SEEK_SET) < 0)
            return AVERROR_EOF;
        if ((ret = ffio_ensure_seekback(pb, 4 + length + 1)) < 0)
            return ret;
        if (avio_skip(pb, 4 + length) < 0)
            return AVERROR_EOF;
        /* the last packet of a file has no successor. A seek past the end
         * does not prove its payload exists, so check against the size; on
         * an unseekable input the skip has read the payload. */
        ret = avio_r8(pb);
        if (ret == TLV_SYNC ||
            (avio_feof(pb) && (!(pb->seekable & AVIO_SEEKABLE_NORMAL) ||
                               avio_size(pb) == pos + 4 + length))) {
            if (avio_seek(pb, pos, SEEK_SET) < 0)
                return AVERROR_EOF;
            tlv->last_pos     = pos;
            tlv->resync_fails = 0;
            reset_flows(s);
            return 0;
        }
    }
    if (avio_seek(pb, pos, SEEK_SET) < 0)
        return AVERROR_EOF;
    /* the position has advanced by the whole window, so retrying makes
     * progress; a long run of damaged data is skipped, not fatal */
    if (++tlv->resync_fails == 32)
        av_log(s, AV_LOG_WARNING, "no TLV packet found in %d bytes at %"PRId64"\n",
               32 * tlv->resync_size, start);
    return FFERROR_REDO;
}

/* Read one TLV packet and feed its payload to the flows. */
static int read_tlv_packet(AVFormatContext *s, int flags)
{
    TLVContext *tlv = s->priv_data;
    AVIOContext *pb = s->pb;
    uint8_t      header[4];
    int64_t      pos = avio_tell(pb);
    int          length, ret;

    if (pos < 0)
        return pos;
    if (pos != tlv->last_pos && (ret = resync(s)) < 0)
        return ret;

    pos = tlv->last_pos;
    if ((ret = ffio_read_size(pb, header, 4)) < 0)
        return avio_feof(pb) ? AVERROR_EOF : ret;
    if (header[0] != TLV_SYNC) {
        tlv->last_pos = -1;                           // resync on the next call
        return FFERROR_REDO;
    }
    length = AV_RB16(header + 2);
    if (length > tlv->buf_size) {
        uint8_t *buf = av_realloc(tlv->buf, length);
        if (!buf)
            return AVERROR(ENOMEM);
        tlv->buf      = buf;
        tlv->buf_size = length;
    }
    if ((ret = ffio_read_size(pb, tlv->buf, length)) < 0)
        return avio_feof(pb) ? AVERROR_EOF : ret;
    tlv->last_pos = pos + 4 + length;

    switch (header[1]) {
    case TLV_IPV4_PACKET:
        ret = read_ip_packet(s, tlv->buf, length, 4, pos, flags);
        break;
    case TLV_IPV6_PACKET:
        ret = read_ip_packet(s, tlv->buf, length, 6, pos, flags);
        break;
    case TLV_COMPRESSED_IP_PACKET:
        ret = read_compressed_ip_packet(s, tlv->buf, length, pos, flags);
        break;
    default:                                          // null and transmission control
        ret = 0;
    }

    /* a malformed payload costs only its own TLV packet */
    if (ret == AVERROR_INVALIDDATA) {
        av_log(s, AV_LOG_DEBUG, "dropping a malformed TLV packet at %"PRId64"\n", pos);
        ret = 0;
    }
    return ret;
}

static int read_queued(AVFormatContext *s, AVPacket *pkt)
{
    TLVContext *tlv = s->priv_data;

    /* the input was moved (a seek, a timestamp search): what is queued
     * belongs to the old position */
    if (tlv->queue.head && avio_tell(s->pb) != tlv->queue_end)
        reset_flows(s);

    while (!tlv->queue.head) {
        int ret;
        if (tlv->read_limit && avio_tell(s->pb) >= tlv->read_limit)
            return AVERROR_EOF;
        ret = read_tlv_packet(s, tlv->parse_flags);
        if (ret == AVERROR_EOF && !tlv->flushed) {
            tlv->flushed = 1;
            for (TLVFlow *flow = tlv->flows; flow; flow = flow->next)
                if ((ret = ff_mmtp_flush(flow->mmtp)) < 0)
                    return ret;
            continue;
        }
        if (ret < 0 && ret != FFERROR_REDO)
            return ret;
    }
    tlv->queue_end = avio_tell(s->pb);
    return avpriv_packet_list_get(&tlv->queue, pkt);
}

static void set_origin(AVFormatContext *s, const AVPacket *pkt)
{
    TLVContext *tlv = s->priv_data;

    tlv->first_pts = av_rescale_q(pkt->pts, s->streams[pkt->stream_index]->time_base,
                                  AV_TIME_BASE_Q);
}

/* MMT presentation times are absolute (NTP based), so the streams are made
 * to start near zero, also on inputs that cannot be scanned at open. Subtitle
 * documents can be timed long before the media, so the origin comes from
 * audio or video; timed subtitles read before it is known are held back. */
static int read_timed(AVFormatContext *s, AVPacket *pkt)
{
    TLVContext *tlv = s->priv_data;
    int         ret;

    for (;;) {
        if (tlv->held.head && tlv->first_pts != AV_NOPTS_VALUE) {
            tlv->nb_held--;
            return avpriv_packet_list_get(&tlv->held, pkt);
        }
        ret = read_queued(s, pkt);
        if (ret == AVERROR_EOF && tlv->held.head) {
            set_origin(s, &tlv->held.head->pkt);      // only subtitles are timed
            continue;
        }
        if (ret < 0 || tlv->first_pts != AV_NOPTS_VALUE || pkt->pts == AV_NOPTS_VALUE)
            return ret;
        if (s->streams[pkt->stream_index]->codecpar->codec_type !=
            AVMEDIA_TYPE_SUBTITLE) {
            set_origin(s, pkt);
            return 0;
        }
        if ((ret = avpriv_packet_list_put(&tlv->held, pkt, NULL, 0)) < 0) {
            av_packet_unref(pkt);
            return ret;
        }
        if (++tlv->nb_held >= MAX_HELD)
            set_origin(s, &tlv->held.head->pkt);
    }
}

static int bt1869_tlv_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    TLVContext *tlv = s->priv_data;
    int         ret = read_timed(s, pkt);

    if (ret < 0)
        return ret;
    if (tlv->first_pts != AV_NOPTS_VALUE) {
        AVRational tb     = s->streams[pkt->stream_index]->time_base;
        int64_t    offset = av_rescale_q(tlv->first_pts, AV_TIME_BASE_Q, tb);
        if (pkt->pts != AV_NOPTS_VALUE)
            pkt->pts -= offset;
        if (pkt->dts != AV_NOPTS_VALUE)
            pkt->dts -= offset;
    }
    return 0;
}

/* Last timestamp in [pos, pos_limit), for the duration estimate. Streams
 * first signalled near the end are not created: the stream set must not
 * depend on how the file ends. */
static int64_t scan_last_ts(AVFormatContext *s, int64_t pos, int64_t pos_limit,
                            int *stream_index)
{
    TLVContext *tlv = s->priv_data;
    AVPacket   *pkt = av_packet_alloc();
    int64_t     last_ts = AV_NOPTS_VALUE;

    *stream_index = -1;
    if (!pkt || avio_seek(s->pb, pos, SEEK_SET) < 0) {
        av_packet_free(&pkt);
        return AV_NOPTS_VALUE;
    }
    tlv->last_pos    = -1;
    tlv->resync_size = MAX_RESYNC;
    tlv->parse_flags = MMTP_PARSE_NO_NEW_STREAMS;
    reset_flows(s);

    while (avio_tell(s->pb) < pos_limit) {
        int ret = bt1869_tlv_read_packet(s, pkt);
        if (ret < 0) {
            if (ret == AVERROR_INVALIDDATA)
                continue;
            break;
        }
        if (pkt->pts != AV_NOPTS_VALUE) {
            last_ts       = pkt->pts;
            *stream_index = pkt->stream_index;
        }
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
    tlv->resync_size = 4096;
    tlv->parse_flags = 0;
    return last_ts;
}

static int bt1869_tlv_read_header(AVFormatContext *s)
{
    TLVContext *tlv   = s->priv_data;
    int64_t     allow = s->probesize, file_size, start = avio_tell(s->pb);
    AVPacket   *pkt;
    int         ret;

    tlv->last_pos     = -1;
    tlv->resync_size  = 4096;
    tlv->first_pts    = AV_NOPTS_VALUE;
    s->ctx_flags     |= AVFMTCTX_NOHEADER;

    if (!(s->pb->seekable & AVIO_SEEKABLE_NORMAL))
        return 0;

    /* find the streams from the signalling, of every flow that starts
     * within probesize */
    while (allow > 0) {
        int64_t cur = avio_tell(s->pb);
        ret = read_tlv_packet(s, MMTP_PARSE_SIGNALLING_ONLY);
        if (ret == AVERROR_EOF)
            break;
        if (ret < 0 && ret != FFERROR_REDO)
            return ret;
        allow -= avio_tell(s->pb) - cur;
    }
    if (!s->nb_streams)
        goto rewind;

    /* anchor first_pts on the first timestamped packet of the file, within
     * probesize: scrambled media yields no packets at all */
    if (avio_seek(s->pb, start, SEEK_SET) < 0)
        return AVERROR(EIO);
    tlv->last_pos = -1;
    reset_flows(s);
    if (!(pkt = av_packet_alloc()))
        return AVERROR(ENOMEM);
    tlv->read_limit = start + s->probesize;
    while (tlv->first_pts == AV_NOPTS_VALUE) {
        if ((ret = bt1869_tlv_read_packet(s, pkt)) < 0)
            break;
        av_packet_unref(pkt);
    }
    tlv->read_limit = 0;
    av_packet_free(&pkt);
    if (ret < 0 && ret != AVERROR_EOF && ret != AVERROR_INVALIDDATA)
        return ret;

    /* estimate the duration from the last 32 MiB */
    if (tlv->first_pts != AV_NOPTS_VALUE && (file_size = avio_size(s->pb)) > 0) {
        int     index;
        int64_t last_ts = scan_last_ts(s, FFMAX(0, file_size - (32 << 20)),
                                       file_size, &index);
        if (last_ts != AV_NOPTS_VALUE && index >= 0 && index < s->nb_streams)
            s->duration = av_rescale_q(last_ts, s->streams[index]->time_base,
                                       AV_TIME_BASE_Q);
    }

rewind:
    if (avio_seek(s->pb, start, SEEK_SET) < 0)
        return AVERROR(EIO);
    tlv->last_pos = -1;
    reset_flows(s);
    return 0;
}

static int bt1869_tlv_read_close(AVFormatContext *s)
{
    TLVContext *tlv = s->priv_data;

    for (TLVFlow *flow = tlv->flows, *next; flow; flow = next) {
        next = flow->next;
        ff_mmtp_parse_close(&flow->mmtp);
        av_free(flow);
    }
    tlv->flows = NULL;
    avpriv_packet_list_free(&tlv->queue);
    avpriv_packet_list_free(&tlv->held);
    av_freep(&tlv->buf);
    return 0;
}

/* DTS of the first access unit starting at or after *pos, for
 * ff_seek_frame_binary(). pos_limit is an absolute position. */
static int64_t bt1869_tlv_read_timestamp(AVFormatContext *s, int stream_index,
                                         int64_t *pos, int64_t pos_limit)
{
    TLVContext *tlv = s->priv_data;
    AVPacket   *pkt = av_packet_alloc();
    int64_t     ts  = AV_NOPTS_VALUE;

    if (!pkt)
        return AV_NOPTS_VALUE;
    if (avio_seek(s->pb, *pos, SEEK_SET) < 0) {
        av_packet_free(&pkt);
        return AV_NOPTS_VALUE;
    }
    tlv->last_pos    = -1;
    tlv->resync_size = MAX_RESYNC;
    reset_flows(s);

    while (avio_tell(s->pb) < pos_limit) {
        int ret = bt1869_tlv_read_packet(s, pkt);
        if (ret < 0) {
            if (ret == AVERROR_INVALIDDATA)
                continue;
            break;
        }
        if (pkt->dts != AV_NOPTS_VALUE &&
            (stream_index < 0 || pkt->stream_index == stream_index)) {
            *pos = pkt->pos;
            ts   = pkt->dts;
            av_packet_unref(pkt);
            break;
        }
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
    tlv->resync_size = 4096;
    return ts;
}

static int bt1869_tlv_read_seek(AVFormatContext *s, int stream_index,
                                int64_t target_ts, int flags)
{
    TLVContext *tlv = s->priv_data;
    int         ret = ff_seek_frame_binary(s, stream_index, target_ts, flags);

    if (ret < 0)
        return ret;
    /* the search lands blindly, possibly inside a large TLV packet */
    tlv->last_pos     = -1;
    tlv->resync_size  = MAX_RESYNC;
    tlv->resync_fails = 0;
    reset_flows(s);
    return 0;
}

const FFInputFormat ff_bt1869_tlv_demuxer = {
    .p.name         = "bt1869_tlv",
    .p.long_name    = NULL_IF_CONFIG_SMALL("ITU-R BT.1869 TLV multiplexing"),
    .p.extensions   = "mmts",
    .p.flags        = AVFMT_SHOW_IDS | AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(TLVContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = bt1869_tlv_probe,
    .read_header    = bt1869_tlv_read_header,
    .read_packet    = bt1869_tlv_read_packet,
    .read_close     = bt1869_tlv_read_close,
    .read_timestamp = bt1869_tlv_read_timestamp,
    .read_seek      = bt1869_tlv_read_seek,
};
