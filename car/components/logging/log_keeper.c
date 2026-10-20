#include "lbr/log_keeper.h"

#include <string.h>

#include "lbr/protocol.h"

/* Pure SD-session keeper; see lbr/log_keeper.h. */

static void event(lbr_log_keeper_t *k, uint16_t code, bool fault)
{
    if (k->hal.on_event != NULL) {
        k->hal.on_event(k->hal.ctx, code, fault);
    }
}

void lbr_log_keeper_init(lbr_log_keeper_t *k, lbr_logger_t *log, const lbr_log_keeper_hal_t *hal,
                         const lbr_log_session_t *session)
{
    memset(k, 0, sizeof *k);
    k->hal     = *hal;
    k->log     = log;
    k->session = *session;
}

static bool open_session(lbr_log_keeper_t *k, uint64_t now_us)
{
    lbr_log_storage_t st;
    char              name[LBR_LOG_KEEPER_NAME_MAX] = "";
    if (!k->hal.open_file(k->hal.ctx, &st, name)) {
        return false;
    }
    k->session.start_time_us = now_us;
    if (lbr_log_open(k->log, &st, &k->session) != 0) {
        st.close(st.ctx);
        return false;
    }
    memcpy(k->name, name, sizeof k->name);
    k->name[sizeof k->name - 1] = '\0';
    return true;
}

void lbr_log_keeper_step(lbr_log_keeper_t *k, uint64_t mono_us, uint64_t now_us)
{
    if (!lbr_log_is_open(k->log)) {
        if (mono_us < k->next_mount_us) {
            return;
        }
        k->next_mount_us = mono_us + LBR_LOG_KEEPER_RETRY_US;
        if (!k->hal.mount(k->hal.ctx) || !open_session(k, now_us)) {
            k->hal.unmount(k->hal.ctx);
            k->sd_present = false;
            if (!k->reported) {
                event(k, LBR_EVT_SD_MISSING, true);
                k->reported = true;
            }
            return;
        }
        k->sd_present = true;
        k->reported   = false;
        lbr_log_stats_t ls;
        lbr_log_get_stats(k->log, &ls);
        k->last_errors = ls.write_errors;
        event(k, LBR_EVT_SD_MOUNTED, false);
        return;
    }

    lbr_log_pump(k->log, now_us);
    lbr_log_stats_t ls;
    lbr_log_get_stats(k->log, &ls);
    if (ls.write_errors != k->last_errors) {
        /* Card pulled or failing: close out, remount on a later pass. */
        event(k, LBR_EVT_SD_WRITE_ERROR, true);
        lbr_log_close(k->log, now_us);
        k->hal.unmount(k->hal.ctx);
        k->name[0]       = '\0';
        k->sd_present    = false;
        k->next_mount_us = mono_us + LBR_LOG_KEEPER_RETRY_US;
    }
}

void lbr_log_keeper_close(lbr_log_keeper_t *k, uint64_t now_us)
{
    if (lbr_log_is_open(k->log)) {
        lbr_log_close(k->log, now_us);
    }
    k->name[0] = '\0';
}
