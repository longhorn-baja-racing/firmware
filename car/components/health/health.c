#include "lbr/health.h"

#include "lbr/bytes.h"

bool lbr_health_page_encode(const lbr_health_record_t *h, unsigned page, uint8_t out[8])
{
    switch (page) {
        case 0:
            lbr_put_u32(out, h->uptime_ms);
            out[4] = h->reset_reason;
            out[5] = h->state;
            lbr_put_u16(out + 6, h->flags);
            return true;
        case 1:
            lbr_put_u32(out, h->fw_version);
            lbr_put_u16(out + 4, h->protocol_version);
            lbr_put_u16(out + 6, h->boot_count);
            return true;
        case 2:
            lbr_put_u32(out, h->catalog_hash);
            lbr_put_u32(out + 4, h->config_crc);
            return true;
        case 3:
            out[0] = h->can_tec;
            out[1] = h->can_rec;
            lbr_put_u16(out + 2, 0);
            lbr_put_u32(out + 4, h->can_bus_off_count);
            return true;
        case 4:
            lbr_put_u32(out, h->can_tx_dropped);
            lbr_put_u32(out + 4, h->can_rx_dropped);
            return true;
        case 5:
            lbr_put_u32(out, h->overflow_count);
            lbr_put_u32(out + 4, h->queue_high_water);
            return true;
        case 6:
            lbr_put_u32(out, h->write_error_count);
            lbr_put_u32(out + 4, h->transport_error_count);
            return true;
        case 7:
            lbr_put_u32(out, h->sensor_status);
            lbr_put_u32(out + 4, h->calibration_state);
            return true;
        case 8:
            lbr_put_u32(out, h->last_sample_age_ms);
            lbr_put_u32(out + 4, h->last_sd_write_age_ms);
            return true;
        case 9:
            lbr_put_u16(out, h->acquire_hz);
            lbr_put_u16(out + 2, h->publish_hz);
            lbr_put_u16(out + 4, h->log_hz);
            lbr_put_u16(out + 6, 0);
            return true;
        case 10:
            lbr_put_u32(out, h->time_step_count);
            lbr_put_u32(out + 4, (uint32_t)h->last_sync_error_us);
            return true;
        case 11:
            lbr_put_u32(out, h->rejected_count);
            lbr_put_u32(out + 4, h->can_bus_errors);
            return true;
        default:
            return false;
    }
}

bool lbr_health_page_decode(lbr_health_record_t *h, unsigned page, const uint8_t in[8])
{
    switch (page) {
        case 0:
            h->uptime_ms    = lbr_get_u32(in);
            h->reset_reason = in[4];
            h->state        = in[5];
            h->flags        = lbr_get_u16(in + 6);
            return true;
        case 1:
            h->fw_version       = lbr_get_u32(in);
            h->protocol_version = lbr_get_u16(in + 4);
            h->boot_count       = lbr_get_u16(in + 6);
            return true;
        case 2:
            h->catalog_hash = lbr_get_u32(in);
            h->config_crc   = lbr_get_u32(in + 4);
            return true;
        case 3:
            if (lbr_get_u16(in + 2) != 0) {
                return false;
            }
            h->can_tec           = in[0];
            h->can_rec           = in[1];
            h->can_bus_off_count = lbr_get_u32(in + 4);
            return true;
        case 4:
            h->can_tx_dropped = lbr_get_u32(in);
            h->can_rx_dropped = lbr_get_u32(in + 4);
            return true;
        case 5:
            h->overflow_count   = lbr_get_u32(in);
            h->queue_high_water = lbr_get_u32(in + 4);
            return true;
        case 6:
            h->write_error_count     = lbr_get_u32(in);
            h->transport_error_count = lbr_get_u32(in + 4);
            return true;
        case 7:
            h->sensor_status     = lbr_get_u32(in);
            h->calibration_state = lbr_get_u32(in + 4);
            return true;
        case 8:
            h->last_sample_age_ms   = lbr_get_u32(in);
            h->last_sd_write_age_ms = lbr_get_u32(in + 4);
            return true;
        case 9:
            if (lbr_get_u16(in + 6) != 0) {
                return false;
            }
            h->acquire_hz = lbr_get_u16(in);
            h->publish_hz = lbr_get_u16(in + 2);
            h->log_hz     = lbr_get_u16(in + 4);
            return true;
        case 10:
            h->time_step_count    = lbr_get_u32(in);
            h->last_sync_error_us = (int32_t)lbr_get_u32(in + 4);
            return true;
        case 11:
            h->rejected_count = lbr_get_u32(in);
            h->can_bus_errors = lbr_get_u32(in + 4);
            return true;
        default:
            return false;
    }
}

size_t lbr_health_encode(const lbr_health_record_t *h, uint8_t *buf, size_t cap)
{
    if (h == NULL || buf == NULL || cap < LBR_HEALTH_WIRE_SIZE) {
        return 0;
    }
    buf[0] = h->node_id;
    for (unsigned p = 0; p < LBR_HEALTH_PAGE_COUNT; p++) {
        lbr_health_page_encode(h, p, buf + 1 + p * LBR_HEALTH_PAGE_SIZE);
    }
    return LBR_HEALTH_WIRE_SIZE;
}

size_t lbr_health_decode(const uint8_t *buf, size_t len, lbr_health_record_t *out)
{
    if (buf == NULL || out == NULL || len < LBR_HEALTH_WIRE_SIZE) {
        return 0;
    }
    lbr_health_record_t h = {.node_id = buf[0]};
    for (unsigned p = 0; p < LBR_HEALTH_PAGE_COUNT; p++) {
        if (!lbr_health_page_decode(&h, p, buf + 1 + p * LBR_HEALTH_PAGE_SIZE)) {
            return 0;
        }
    }
    *out = h;
    return LBR_HEALTH_WIRE_SIZE;
}
