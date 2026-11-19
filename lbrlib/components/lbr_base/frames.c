#include "lbr/frames.h"

#include <string.h>

#include "lbr/bytes.h"
#include "lbr/crc32.h"
#include "lbr/nodes.h"

/*
 * The CAN data frame table, from the car's catalog (lbr/catalog_frames.def,
 * generated from catalog.toml by lbrlib/tools/lbrgen.py, which checks the
 * layout). Sorted by id (also checked by the host tests). Changing a frame is
 * a catalog change: the hash moves and the dashboard app updates in step.
 */

#define LBR_FRAME(id_, name_, sender_, rate_, dlc_, counter_, n_fields_)                           \
    {                                                                                              \
        .can_id = (id_), .name = (name_), .sender = LBR_NODE_##sender_, .rate_hz = (rate_),        \
        .dlc = (dlc_), .counter_offset = (counter_), .n_fields = (n_fields_), .fields = {
#define LBR_FIELD(sig_, off_, enc_, scale_)                                                        \
    {.signal = LBR_SIG_##sig_, .offset = (off_), .enc = LBR_ENC_##enc_, .scale = (scale_)},
#define LBR_FRAME_END                                                                              \
    }                                                                                              \
    }                                                                                              \
    ,

static const lbr_frame_def_t s_frames[] = {
#include "lbr/catalog_frames.def"
};

#undef LBR_FRAME
#undef LBR_FIELD
#undef LBR_FRAME_END

#define FRAME_COUNT (sizeof s_frames / sizeof s_frames[0])

const lbr_frame_def_t *lbr_frame_def(uint16_t can_id)
{
    size_t lo = 0;
    size_t hi = FRAME_COUNT;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        if (s_frames[mid].can_id == can_id) {
            return &s_frames[mid];
        }
        if (s_frames[mid].can_id < can_id) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return NULL;
}

size_t lbr_frame_count(void)
{
    return FRAME_COUNT;
}

const lbr_frame_def_t *lbr_frame_at(size_t index)
{
    return index < FRAME_COUNT ? &s_frames[index] : NULL;
}

const lbr_frame_def_t *lbr_frame_for_signal(lbr_signal_id_t signal, size_t *field_index)
{
    for (size_t i = 0; i < FRAME_COUNT; i++) {
        for (size_t f = 0; f < s_frames[i].n_fields; f++) {
            if (s_frames[i].fields[f].signal == signal) {
                if (field_index != NULL) {
                    *field_index = f;
                }
                return &s_frames[i];
            }
        }
    }
    return NULL;
}

unsigned lbr_field_enc_size(lbr_field_enc_t enc)
{
    switch (enc) {
        case LBR_ENC_I16:
        case LBR_ENC_U16:
            return 2;
        case LBR_ENC_U8:
            return 1;
        case LBR_ENC_I32:
        case LBR_ENC_U32:
            return 4;
        default:
            return 0;
    }
}

const char *lbr_field_enc_name(lbr_field_enc_t enc)
{
    static const char *const names[LBR_ENC_COUNT_] = {
        [LBR_ENC_I16] = "i16", [LBR_ENC_U16] = "u16", [LBR_ENC_U8] = "u8",
        [LBR_ENC_I32] = "i32", [LBR_ENC_U32] = "u32",
    };
    return ((unsigned)enc < LBR_ENC_COUNT_) ? names[enc] : "?";
}

uint32_t lbr_frames_hash(uint32_t crc)
{
    for (size_t i = 0; i < FRAME_COUNT; i++) {
        const lbr_frame_def_t *d = &s_frames[i];
        uint8_t                rec[8];
        lbr_put_u16(rec, d->can_id);
        rec[2] = d->sender;
        lbr_put_u16(rec + 3, d->rate_hz);
        rec[5] = d->dlc;
        rec[6] = d->counter_offset;
        rec[7] = d->n_fields;
        crc    = lbr_crc32_update(crc, rec, sizeof rec);
        for (size_t f = 0; f < d->n_fields; f++) {
            uint8_t fr[8];
            lbr_put_u16(fr, (uint16_t)d->fields[f].signal);
            fr[2] = d->fields[f].offset;
            fr[3] = d->fields[f].enc;
            lbr_put_u32(fr + 4, lbr_f32_bits(d->fields[f].scale));
            crc = lbr_crc32_update(crc, fr, sizeof fr);
        }
        crc = lbr_crc32_update(crc, d->name, strlen(d->name) + 1);
    }
    return crc;
}
