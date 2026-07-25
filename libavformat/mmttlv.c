/*
 * MMT protocol over TLV packets (MMT/TLV) demuxer, as defined in ARIB STD-B32.
 * Copyright (c) 2025 SuperFashi G.K.
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

#include "config_components.h"

#include "libavutil/mem.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"
#include "mmtp.h"

#define HEADER_BYTE 0b01111111

enum {
    UNDEFINED_PACKET            = 0x00,
    IPV4_PACKET                 = 0x01,
    IPV6_PACKET                 = 0x02,
    HEADER_COMPRESSED_IP_PACKET = 0x03,
    TRANSMISSION_CONTROL_PACKET = 0xFE,
    NULL_PACKET                 = 0xFF,
};

enum {
    CONTEXT_IDENTIFICATION_PARTIAL_IPV4_AND_PARTIAL_UDP_HEADER = 0x20,
    CONTEXT_IDENTIFICATION_IPV4_HEADER                         = 0x21,
    CONTEXT_IDENTIFICATION_PARTIAL_IPV6_AND_PARTIAL_UDP_HEADER = 0x60,
    CONTEXT_IDENTIFICATION_NO_COMPRESSED_HEADER                = 0x61,
};

static int mmttlv_probe(const AVProbeData *p)
{
    size_t   i, j;
    uint8_t  packet_type;
    uint16_t data_length;

    int processed  = 0;
    int recognized = 0;

    for (i = 0; i + 4 < p->buf_size && processed < 100;) {
        if (p->buf[i] != HEADER_BYTE) {
            ++i;
            continue;
        }
        ++processed;

        packet_type = p->buf[i + 1];
        data_length = AV_RB16(p->buf + i + 2);
        i += 4;

        if (packet_type == HEADER_COMPRESSED_IP_PACKET) {
            if (data_length < 3 || i + 2 >= p->buf_size) goto skip;
            switch (p->buf[i + 2]) {
            case CONTEXT_IDENTIFICATION_PARTIAL_IPV4_AND_PARTIAL_UDP_HEADER:
            case CONTEXT_IDENTIFICATION_IPV4_HEADER:
            case CONTEXT_IDENTIFICATION_PARTIAL_IPV6_AND_PARTIAL_UDP_HEADER:
            case CONTEXT_IDENTIFICATION_NO_COMPRESSED_HEADER:
                ++recognized;
                i += data_length;
            }
        } else if (packet_type == NULL_PACKET) {
            // null packets should contain all 0xFFs
            for (j = i; j < i + data_length && j < p->buf_size; ++j) {
                if (p->buf[j] != 0xFF) goto skip;
            }
            ++recognized;
            i += data_length;
        }

        skip:;
    }

    return recognized * AVPROBE_SCORE_MAX / FFMAX(processed, 10);
}

struct MMTTLVContext {
    struct Program {
        uint32_t       cid;
        MMTPContext    *mmtp;
        struct Program *next;
    } *programs;

    int64_t last_pos;
    size_t  resync_size;
    int     resync_fails;   // consecutive exhausted hunts; bounds the REDO retries

    size_t  cap;
    uint8_t *buf;

    int64_t first_pts;
};

static int mmttlv_read_ipv6_packet(
    struct MMTTLVContext *ctx, AVFormatContext *s, AVPacket *pkt,
    const uint8_t *buf, uint16_t size)
{
    uint8_t  next_header;
    uint16_t payload_length;
    uint16_t dst_port;

    // 1. parse ipv6 header
    if (size < 40) return AVERROR_INVALIDDATA;
    next_header = buf[6];
    size -= 40;
    buf += 40;

    if (next_header != 17) return AVERROR_PATCHWELCOME;
    // 2. parse udp header
    if (size < 8) return AVERROR_INVALIDDATA;
    dst_port       = AV_RB16(buf + 2);
    payload_length = AV_RB16(buf + 4);
    // TODO: checksum
    size -= 8;
    buf += 8;

    if (size != payload_length - 8) return AVERROR_INVALIDDATA;
    if (dst_port == 123) {
        // NTP
        return pkt == NULL ? 0 : FFERROR_REDO;
    }
    return AVERROR_PATCHWELCOME;
}

static int mmttlv_read_compressed_ip_packet(
    struct MMTTLVContext *ctx, AVFormatContext *s, AVPacket *pkt,
    const uint8_t *buf, uint16_t size)
{
    // partial udp header are udp header without data length (16 bits) and checksum (16 bits)
#define PARTIAL_UDP_HEADER_LENGTH (8 - 4)
    // partial ipv6 header are ipv6 header without payload length (16 bits)
#define PARTIAL_IPV6_HEADER_LENGTH (40 - 2)

    uint32_t       context_id;
    struct Program *program;

    if (size < 3)
        return AVERROR_INVALIDDATA;
    context_id = AV_RB16(buf) >> 4;
    buf += 3;
    size -= 3;

    for (program = ctx->programs; program != NULL; program = program->next)
        if (program->cid == context_id)
            break;

    if (program == NULL) {
        AVProgram *p = av_new_program(s, context_id);
        if (p == NULL) return AVERROR(ENOMEM);

        program = av_malloc(sizeof(struct Program));
        if (program == NULL) return AVERROR(ENOMEM);

        // av_malloc does not set errno; and an unchecked NULL mmtp would be dereferenced by
        // ff_mmtp_parse_packet below.
        program->mmtp = ff_mmtp_parse_open(p);
        if (program->mmtp == NULL) {
            av_free(program);
            return AVERROR(ENOMEM);
        }
        program->next = ctx->programs;
        ctx->programs = program;
        program->cid  = context_id;
    }

    switch (buf[-1]) {
    case CONTEXT_IDENTIFICATION_PARTIAL_IPV4_AND_PARTIAL_UDP_HEADER:
    case CONTEXT_IDENTIFICATION_IPV4_HEADER:
        return AVERROR_PATCHWELCOME;
    case CONTEXT_IDENTIFICATION_PARTIAL_IPV6_AND_PARTIAL_UDP_HEADER:
        if (size < PARTIAL_IPV6_HEADER_LENGTH + PARTIAL_UDP_HEADER_LENGTH)
            return AVERROR_INVALIDDATA;
        size -= PARTIAL_IPV6_HEADER_LENGTH + PARTIAL_UDP_HEADER_LENGTH;
        buf += PARTIAL_IPV6_HEADER_LENGTH + PARTIAL_UDP_HEADER_LENGTH;
    case CONTEXT_IDENTIFICATION_NO_COMPRESSED_HEADER:
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    return ff_mmtp_parse_packet(program->mmtp, s, pkt, buf, size);
}

// 0x7F occurs freely inside coded payloads (HEVC hits it often); a resync candidate must
// also carry a defined packet_type or the hunt latches onto false headers and the parser
// then reads garbage — which surfaces as nonsense timestamps and defeats the seek search.
static int mmttlv_valid_type(uint8_t t)
{
    return t == IPV4_PACKET || t == IPV6_PACKET ||
           t == HEADER_COMPRESSED_IP_PACKET ||
           t == TRANSMISSION_CONTROL_PACKET || t == NULL_PACKET;
}

static int mmttlv_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    uint8_t              header[4];
    uint16_t             size;
    int                  err;
    struct MMTTLVContext *ctx = s->priv_data;
    int64_t              pos;

    retry:
    pos = avio_tell(s->pb);

    if (pos < 0) return (int) pos;
    if (pos != ctx->last_pos) {
        ctx->last_pos = pos;

        while (pos - ctx->last_pos < ctx->resync_size) {
            if ((err = ffio_ensure_seekback(s->pb, 4)) < 0)
                return err;

            // A short read is not an error from avio_read, but there is no whole TLV header
            // left — report EOF instead of scanning on with stale `header` bytes.
            if ((err = avio_read(s->pb, header, 4)) != 4)
                return (err < 0 && !avio_feof(s->pb)) ? err : AVERROR_EOF;

            if (header[0] != HEADER_BYTE || !mmttlv_valid_type(header[1])) {
                if ((pos = avio_seek(s->pb, -3, SEEK_CUR)) < 0)
                    return (int) pos;
                continue;
            }

            size = AV_RB16(header + 2);

            if ((pos = avio_seek(s->pb, -4, SEEK_CUR)) < 0)
                return (int) pos;

            if ((err = ffio_ensure_seekback(s->pb, 4 + size + 1)) < 0)
                return err;

            if ((pos = avio_skip(s->pb, 4 + size)) < 0)
                return (int) pos;

            if ((err = avio_read(s->pb, header, 1)) != 1)
                return (err < 0 && !avio_feof(s->pb)) ? err : AVERROR_EOF;

            if (header[0] == HEADER_BYTE) {
                // found HEADER, [size], HEADER, should be good
                if ((pos = avio_seek(
                    s->pb, -(int64_t) (size) - 1 - 4, SEEK_CUR)) < 0)
                    return (int) pos;
                goto success;
            }

            if ((pos = avio_seek(
                s->pb, -(int64_t) (size) - 1 - 3, SEEK_CUR)) < 0)
                return (int) pos;
        }
        // Window exhausted. The position has advanced by the whole hunt, so retrying makes
        // progress; a hard error here lets a caller's error cap turn one bad landing into EOF
        // (mpv treats ~10 consecutive errors as fatal). Bounded by the give-up below.
        if (++ctx->resync_fails <= 32)
            return FFERROR_REDO;
        return AVERROR_INVALIDDATA;

        success:
        ctx->resync_fails = 0;
        ctx->last_pos = pos;

        for (struct Program *program = ctx->programs;
             program != NULL; program = program->next)
            ff_mmtp_reset_state(program->mmtp);
    }

    if (pkt != NULL) pkt->pos = ctx->last_pos;
    if ((err = ffio_read_size(s->pb, header, 4)) < 0)
        return avio_feof(s->pb) ? AVERROR_EOF : err;
    ctx->last_pos += 4;

    if (header[0] != HEADER_BYTE)
        return AVERROR_INVALIDDATA;

    size = AV_RB16(header + 2);
    if (header[1] == NULL_PACKET) {
        if ((ctx->last_pos = avio_skip(s->pb, size)) < 0)
            return (int) ctx->last_pos;
        return pkt == NULL ? 0 : FFERROR_REDO;
    }

    if (ctx->cap < size) {
        // Reset cap to 0 before allocating: on failure buf is NULL, and a stale cap == size would
        // make a later call skip the realloc and read into NULL.
        av_freep(&ctx->buf);
        ctx->cap = 0;
        if ((ctx->buf = av_malloc(size)) == NULL)
            return AVERROR(ENOMEM);
        ctx->cap = size;
    }
    if ((err = ffio_read_size(s->pb, ctx->buf, size)) < 0)
        return avio_feof(s->pb) ? AVERROR_EOF : err;
    ctx->last_pos += size;

    switch (header[1]) {
    case IPV6_PACKET:
        err = mmttlv_read_ipv6_packet(ctx, s, pkt, ctx->buf, size);
        break;
    case HEADER_COMPRESSED_IP_PACKET:
        err = mmttlv_read_compressed_ip_packet(ctx, s, pkt, ctx->buf, size);
        break;
    case TRANSMISSION_CONTROL_PACKET:
        err = FFERROR_REDO;
        break;
    default:
        av_log(s, AV_LOG_INFO, "Unsupported TLV packet type: 0x%02x\n", header[1]);
        err = FFERROR_REDO;
    }

    if (err == FFERROR_REDO)
        goto retry;

    // A malformed MMTP payload must not abort the whole demux. A real capture
    // splices/reconfigures mid-stream (here ~35 s in, at a packet_sequence_number
    // discontinuity), which can leave a truncated MPU, a stale timestamp
    // descriptor, or a broken fragment run — any of which surfaces as
    // AVERROR_INVALIDDATA from the MMTP parser. The offending TLV packet has
    // already been consumed (avio and last_pos point at the next packet), so drop
    // it and resync on the following one instead of returning fatally. Without
    // this, one bad packet ends playback at the discontinuity and defeats every
    // seek that must read across it.
    if (err == AVERROR_INVALIDDATA) {
        if (pkt)
            av_packet_unref(pkt);
        goto retry;
    }

    if (err >= 0 && pkt) {
        // On a non-seekable input read_header cannot scan for first_pts, so anchor
        // it on the first timestamped packet; MMT presentation times are absolute
        // (NTP-derived) and would otherwise reach the decoder unnormalized.
        if (ctx->first_pts == AV_NOPTS_VALUE && pkt->pts != AV_NOPTS_VALUE)
            ctx->first_pts = pkt->pts;
        if (ctx->first_pts != AV_NOPTS_VALUE) {
            if (pkt->pts != AV_NOPTS_VALUE)
                pkt->pts -= ctx->first_pts;
            if (pkt->dts != AV_NOPTS_VALUE)
                pkt->dts -= ctx->first_pts;
        }
    }

    return err;
}

static int64_t mmttlv_scan_last_ts(AVFormatContext *s, int64_t pos, int64_t pos_limit, int *stream_index_found)
{
    struct MMTTLVContext *ctx = s->priv_data;
    int64_t last_valid_ts = AV_NOPTS_VALUE;
    int last_valid_stream_index = -1;
    size_t saved_resync = ctx->resync_size;
    AVPacket *pkt;
    int ret;

    pkt = av_packet_alloc();
    if (!pkt)
        return AV_NOPTS_VALUE;

    if (avio_seek(s->pb, pos, SEEK_SET) < 0) {
        av_packet_free(&pkt);
        return AV_NOPTS_VALUE;
    }

    ctx->last_pos = -1; // force resync
    ctx->resync_size = 65536 + 4096; // Allow larger resync window for random access

    while (avio_tell(s->pb) < pos_limit) {
        av_packet_unref(pkt);
        ret = mmttlv_read_packet(s, pkt);
        if (ret >= 0) {
            if (pkt->pts != AV_NOPTS_VALUE) {
                last_valid_ts = pkt->pts;
                last_valid_stream_index = pkt->stream_index;
            }
            av_packet_unref(pkt);
        } else if (ret == AVERROR_EOF) {
            break;
        } else if (ret == FFERROR_REDO) {
            continue;
        } else {
            // Skip invalid packets during search
            continue;
        }
    }

    av_packet_free(&pkt);
    ctx->resync_size = saved_resync; // restore; otherwise normal playback runs with the wide window
    if (stream_index_found) *stream_index_found = last_valid_stream_index;
    return last_valid_ts;
}

static int mmttlv_read_header(AVFormatContext *s)
{
    int64_t              pos;
    int64_t              allow = s->probesize;
    struct MMTTLVContext *ctx  = s->priv_data;
    AVPacket             *pkt;
    int64_t              file_size;
    int                  ret;

    ctx->last_pos = avio_tell(s->pb);
    if (ctx->last_pos < 0)
        return (int) ctx->last_pos;
    ctx->last_pos -= 1; // force resync

    ctx->resync_size  = 4096;
    ctx->resync_fails = 0;
    ctx->first_pts   = AV_NOPTS_VALUE;
    s->ctx_flags |= AVFMTCTX_NOHEADER;

    if (!s->pb->seekable)
        return 0;

    if ((pos = avio_tell(s->pb)) < 0)
        return (int) pos;

    while (s->nb_streams <= 0 && allow > 0) {
        const int64_t cur = ctx->last_pos;
        const int     err = mmttlv_read_packet(s, NULL);
        if (err < 0) return err;
        allow -= ctx->last_pos - cur;
    }

    pkt = av_packet_alloc();
    if (!pkt)
        return AVERROR(ENOMEM);

    // Read until the first timestamped packet; mmttlv_read_packet() anchors
    // first_pts on it (it must not be re-assigned here to the already-normalized
    // value the read returns).
    while (ctx->first_pts == AV_NOPTS_VALUE) {
        av_packet_unref(pkt);
        ret = mmttlv_read_packet(s, pkt);
        if (ret < 0) {
            if (ret == AVERROR_EOF || ret == FFERROR_REDO)
                break;
            av_packet_free(&pkt);
            return ret;
        }
    }
    av_packet_free(&pkt);

    if (ctx->first_pts != AV_NOPTS_VALUE && (file_size = avio_size(s->pb)) > 0) {
        // Estimate duration from the last MPU timestamp, scanning only the trailing 32 MB so open()
        // stays cheap on multi-GB files.
        int64_t last_pos = FFMAX(0, file_size - 32 * 1024 * 1024);
        int stream_index_found = -1;
        int64_t last_ts = mmttlv_scan_last_ts(s, last_pos, file_size, &stream_index_found);
        if (last_ts != AV_NOPTS_VALUE && stream_index_found >= 0 && stream_index_found < s->nb_streams) {
            AVStream *st = s->streams[stream_index_found];
            if (st->time_base.den > 0) {
                s->duration = av_rescale_q(last_ts, st->time_base, AV_TIME_BASE_Q);
            } else {
                s->duration = last_ts;
            }
        }
    }

    // Reset to beginning and reset MMT state machine
    if (avio_seek(s->pb, 0, SEEK_SET) < 0)
        return (int) pos;

    ctx->last_pos = -1; // force resync after seek
    for (struct Program *program = ctx->programs;
         program != NULL; program = program->next)
        ff_mmtp_reset_state(program->mmtp);

    return 0;
}

static int mmttlv_read_close(AVFormatContext *ctx)
{
    struct Program       *program;
    struct MMTTLVContext *priv = ctx->priv_data;
    for (program = priv->programs; program != NULL;) {
        struct Program *next = program->next;
        ff_mmtp_parse_close(program->mmtp);
        av_free(program);
        program = next;
    }
    priv->programs = NULL;
    priv->cap      = 0;
    av_freep(&priv->buf);
    return 0;
}

// Timestamp of the next video access unit at or after *pos, for ff_seek_frame_binary().
// A probe may land inside a large HEVC access unit, so widen the resync window (a
// video TLV fragment can exceed the default), reset state on entry so probes are
// independent, and skip past a failed resync rather than failing the whole seek.
//
// pos_limit is an ABSOLUTE file position, not a byte budget: ff_find_last_ts() probes
// the tail with a small window ([filesize-1-step, filesize-1], step doubling until a
// timestamp turns up) and ff_gen_search() passes INT64_MAX. Stop as soon as the scan
// reaches it and report "nothing here", so the caller can widen its own window.
static int64_t mmttlv_read_timestamp(
    struct AVFormatContext *s, int stream_index,
    int64_t *pos, int64_t pos_limit)
{
    struct MMTTLVContext *ctx = s->priv_data;
    size_t  saved_resync = ctx->resync_size;
    int64_t result = AV_NOPTS_VALUE;

    if ((*pos = avio_seek(s->pb, *pos, SEEK_SET)) < 0)
        return *pos;
    ctx->resync_size = 65536 + 4096;
    ctx->last_pos    = -1; // force a clean resync + state reset at the probe

    while (*pos < pos_limit) {
        AVPacket      packet = {0};
        const int64_t prev   = *pos;
        const int     err    = mmttlv_read_packet(s, &packet);
        const int64_t ts     = packet.dts;
        const int64_t off    = packet.pos;
        const int     sid    = packet.stream_index;
        av_packet_unref(&packet);

        if (err == AVERROR_EOF)
            break;

        if (err >= 0 && ts != AV_NOPTS_VALUE &&
            (stream_index < 0 || sid == stream_index)) {
            *pos   = off;
            result = ts;
            break;
        }

        if (err < 0 && err != FFERROR_REDO) {
            // bad sync — advance one byte and resync again
            const int64_t here = avio_tell(s->pb);
            if (here < 0)
                break;
            *pos = here + 1;
            if (avio_seek(s->pb, *pos, SEEK_SET) < 0)
                break;
            ctx->last_pos = -1;
            continue;
        }

        // Keep reading sequentially from here: the MMTP parser needs consecutive packets to
        // finish assembling an MPU, so re-seeking (and resetting its state) per iteration
        // would keep discarding the access unit we are waiting for.
        *pos = ctx->last_pos;

        // Never stall: a packet that consumed nothing would otherwise spin here forever.
        if (*pos <= prev) {
            *pos = prev + 1;
            if (avio_seek(s->pb, *pos, SEEK_SET) < 0)
                break;
            ctx->last_pos = -1;
        }
    }

    ctx->resync_size = saved_resync;
    return result;
}

// The binary search lands blind: the chosen byte offset can sit inside a max-size video TLV
// packet, so reset the parser and widen the resync hunt enough to reach the next packet
// boundary (the default 4 KB window cannot escape a 64 KB packet).
static int mmttlv_read_seek(AVFormatContext *s, int stream_index,
                           int64_t target_ts, int flags)
{
    struct MMTTLVContext *ctx = s->priv_data;
    const int ret = ff_seek_frame_binary(s, stream_index, target_ts, flags);

    if (ret < 0)
        return ret;

    ctx->last_pos     = -1; // force resync + state reset at the new position
    ctx->resync_size  = 65536 + 4096;
    ctx->resync_fails = 0;
    return 0;
}

#if CONFIG_MMTTLV_DEMUXER
const FFInputFormat ff_mmttlv_demuxer = {
    .p.name         = "mmttlv",
    .p.long_name    = NULL_IF_CONFIG_SMALL(
        "MMT protocol over TLV packets (ARIB STD-B32)"),
    .p.flags        = AVFMT_SHOW_IDS | AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(struct MMTTLVContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = mmttlv_probe,
    .read_header    = mmttlv_read_header,
    .read_packet    = mmttlv_read_packet,
    .read_close     = mmttlv_read_close,
    .read_timestamp = mmttlv_read_timestamp,
    .read_seek      = mmttlv_read_seek,
};
#endif
