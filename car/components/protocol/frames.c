#include "lbr/frames.h"

#include <string.h>

#include "lbr/bytes.h"
#include "lbr/crc32.h"
#include "lbr/nodes.h"

/*
 * The CAN data frame table (DAQ-WIRING.md section 7). Sorted by id (checked
 * by the host tests). Scales marked TBD are provisional until the parts are
 * chosen; changing one is a catalog change (the hash moves) and must be
 * mirrored in lbr-dashboard.
 */

#define F(sig_, off_, enc_, scale_)                                                                \
    {.signal = LBR_SIG_##sig_, .offset = (off_), .enc = LBR_ENC_##enc_, .scale = (scale_)}

#define NO_CNT LBR_FRAME_NO_COUNTER

/* Corner/chassis IMU pair: accel X/Y/Z then gyro X/Y/Z, counter in bytes 6..7. */
#define IMU_ACCEL(id_, name_, origin_, p_)                                                         \
    {                                                                                              \
        .can_id = (id_), .name = (name_), .sender = LBR_NODE_IMU, .origin = (origin_),             \
        .rate_hz = 100, .dlc = 8, .counter_offset = 6, .n_fields = 3,                              \
        .fields = {F(p_##_ACCEL_X, 0, I16, 0.01f), F(p_##_ACCEL_Y, 2, I16, 0.01f),                 \
                   F(p_##_ACCEL_Z, 4, I16, 0.01f)}                                                 \
    }
#define IMU_GYRO(id_, name_, origin_, p_)                                                          \
    {                                                                                              \
        .can_id = (id_), .name = (name_), .sender = LBR_NODE_IMU, .origin = (origin_),             \
        .rate_hz = 100, .dlc = 8, .counter_offset = 6, .n_fields = 3,                              \
        .fields = {F(p_##_GYRO_X, 0, I16, 0.001f), F(p_##_GYRO_Y, 2, I16, 0.001f),                 \
                   F(p_##_GYRO_Z, 4, I16, 0.001f)}                                                 \
    }

static const lbr_frame_def_t s_frames[] = {
    /* Driver inputs: pedals 0.01 %, steering 0.1 deg (scaling TBD). */
    {.can_id         = 0x100,
     .name           = "DRIVER_INPUTS",
     .sender         = LBR_NODE_DRIVER_INPUT,
     .origin         = LBR_NODE_DRIVER_INPUT,
     .rate_hz        = 100,
     .dlc            = 8,
     .counter_offset = 6,
     .n_fields       = 3,
     .fields         = {F(BRAKE_PEDAL, 0, I16, 0.01f), F(GAS_PEDAL, 2, I16, 0.01f),
                        F(STEERING_ANGLE, 4, I16, 0.1f)}},

    /* Powertrain: speeds in whole rpm; fuel 0.1 % (scaling TBD). */
    {.can_id         = 0x200,
     .name           = "POWERTRAIN_SPEEDS",
     .sender         = LBR_NODE_POWERTRAIN,
     .origin         = LBR_NODE_POWERTRAIN,
     .rate_hz        = 100,
     .dlc            = 8,
     .counter_offset = 6,
     .n_fields       = 2,
     .fields         = {F(ENGINE_RPM, 0, U16, 1.0f), F(CVT_RPM, 2, U16, 1.0f)}},
    {.can_id         = 0x201,
     .name           = "FUEL_LEVEL",
     .sender         = LBR_NODE_POWERTRAIN,
     .origin         = LBR_NODE_POWERTRAIN,
     .rate_hz        = 1,
     .dlc            = 2,
     .counter_offset = NO_CNT,
     .n_fields       = 1,
     .fields         = {F(FUEL_LEVEL, 0, I16, 0.1f)}},

    /* IMUs: accel 0.01 m/s^2 (+/-327 m/s^2), gyro 0.001 rad/s (+/-32.7 rad/s). */
    IMU_ACCEL(0x300, "IMU_FL_ACCEL", LBR_NODE_CORNER_FL, IMU_FL),
    IMU_GYRO(0x301, "IMU_FL_GYRO", LBR_NODE_CORNER_FL, IMU_FL),
    IMU_ACCEL(0x302, "IMU_FR_ACCEL", LBR_NODE_CORNER_FR, IMU_FR),
    IMU_GYRO(0x303, "IMU_FR_GYRO", LBR_NODE_CORNER_FR, IMU_FR),
    IMU_ACCEL(0x304, "IMU_BL_ACCEL", LBR_NODE_CORNER_BL, IMU_BL),
    IMU_GYRO(0x305, "IMU_BL_GYRO", LBR_NODE_CORNER_BL, IMU_BL),
    IMU_ACCEL(0x306, "IMU_BR_ACCEL", LBR_NODE_CORNER_BR, IMU_BR),
    IMU_GYRO(0x307, "IMU_BR_GYRO", LBR_NODE_CORNER_BR, IMU_BR),
    IMU_ACCEL(0x308, "IMU_CH_ACCEL", LBR_NODE_IMU, IMU_CH),
    IMU_GYRO(0x309, "IMU_CH_GYRO", LBR_NODE_IMU, IMU_CH),
    {.can_id         = 0x30A,
     .name           = "CORNER_STATUS",
     .sender         = LBR_NODE_IMU,
     .origin         = LBR_NODE_IMU,
     .rate_hz        = 1,
     .dlc            = 6,
     .counter_offset = NO_CNT,
     .n_fields       = 6,
     .fields         = {F(CORNER_FL_BATTERY, 0, U8, 1.0f), F(CORNER_FR_BATTERY, 1, U8, 1.0f),
                        F(CORNER_BL_BATTERY, 2, U8, 1.0f), F(CORNER_BR_BATTERY, 3, U8, 1.0f),
                        F(CORNER_LINK, 4, U8, 1.0f), F(CORNER_SYNCED, 5, U8, 1.0f)}},

    /* Stress: whole microstrain (scaling TBD). */
    {.can_id         = 0x400,
     .name           = "STRESS_1_4",
     .sender         = LBR_NODE_STRESS,
     .origin         = LBR_NODE_STRESS,
     .rate_hz        = 100,
     .dlc            = 8,
     .counter_offset = NO_CNT,
     .n_fields       = 4,
     .fields = {F(STRESS_1, 0, I16, 1.0f), F(STRESS_2, 2, I16, 1.0f), F(STRESS_3, 4, I16, 1.0f),
                F(STRESS_4, 6, I16, 1.0f)}},
    {.can_id         = 0x401,
     .name           = "STRESS_5_8",
     .sender         = LBR_NODE_STRESS,
     .origin         = LBR_NODE_STRESS,
     .rate_hz        = 100,
     .dlc            = 8,
     .counter_offset = NO_CNT,
     .n_fields       = 4,
     .fields = {F(STRESS_5, 0, I16, 1.0f), F(STRESS_6, 2, I16, 1.0f), F(STRESS_7, 4, I16, 1.0f),
                F(STRESS_8, 6, I16, 1.0f)}},

    /* Temperatures: 0.1 degC. */
    {.can_id         = 0x500,
     .name           = "BRAKE_TEMPS",
     .sender         = LBR_NODE_TEMPERATURE,
     .origin         = LBR_NODE_TEMPERATURE,
     .rate_hz        = 10,
     .dlc            = 8,
     .counter_offset = NO_CNT,
     .n_fields       = 4,
     .fields         = {F(TEMP_BRAKE_FL, 0, I16, 0.1f), F(TEMP_BRAKE_FR, 2, I16, 0.1f),
                        F(TEMP_BRAKE_BL, 4, I16, 0.1f), F(TEMP_BRAKE_BR, 6, I16, 0.1f)}},
    {.can_id         = 0x501,
     .name           = "CVT_TEMP",
     .sender         = LBR_NODE_TEMPERATURE,
     .origin         = LBR_NODE_TEMPERATURE,
     .rate_hz        = 10,
     .dlc            = 2,
     .counter_offset = NO_CNT,
     .n_fields       = 1,
     .fields         = {F(TEMP_CVT, 0, I16, 0.1f)}},

    /* Central: GPS (any NMEA receiver) and the dashboard link. */
    {.can_id         = 0x600,
     .name           = "GPS_POSITION",
     .sender         = LBR_NODE_CENTRAL,
     .origin         = LBR_NODE_CENTRAL,
     .rate_hz        = 10,
     .dlc            = 8,
     .counter_offset = NO_CNT,
     .n_fields       = 2,
     .fields         = {F(GPS_LATITUDE, 0, I32, 1.0f), F(GPS_LONGITUDE, 4, I32, 1.0f)}},
    {.can_id         = 0x601,
     .name           = "GPS_MOTION",
     .sender         = LBR_NODE_CENTRAL,
     .origin         = LBR_NODE_CENTRAL,
     .rate_hz        = 10,
     .dlc            = 8,
     .counter_offset = 6,
     .n_fields       = 3,
     .fields         = {F(GPS_SPEED, 0, U16, 0.01f), F(GPS_COURSE, 2, U16, 0.01f),
                        F(GPS_ALTITUDE, 4, I16, 0.1f)}},
    {.can_id         = 0x602,
     .name           = "GPS_STATUS",
     .sender         = LBR_NODE_CENTRAL,
     .origin         = LBR_NODE_CENTRAL,
     .rate_hz        = 10,
     .dlc            = 8,
     .counter_offset = NO_CNT,
     .n_fields       = 4,
     .fields = {F(GPS_UTC, 0, U32, 1.0f), F(GPS_FIX, 4, U8, 1.0f), F(GPS_SATELLITES, 5, U8, 1.0f),
                F(GPS_HDOP, 6, U16, 0.01f)}},
    {.can_id         = 0x60A,
     .name           = "DASH_STATUS",
     .sender         = LBR_NODE_CENTRAL,
     .origin         = LBR_NODE_CENTRAL,
     .rate_hz        = 1,
     .dlc            = 2,
     .counter_offset = NO_CNT,
     .n_fields       = 2,
     .fields         = {F(DASH_BATTERY, 0, U8, 1.0f), F(DASH_LINK, 1, U8, 1.0f)}},
};

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
        uint8_t                rec[9];
        lbr_put_u16(rec, d->can_id);
        rec[2] = d->sender;
        rec[3] = d->origin;
        lbr_put_u16(rec + 4, d->rate_hz);
        rec[6] = d->dlc;
        rec[7] = d->counter_offset;
        rec[8] = d->n_fields;
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
