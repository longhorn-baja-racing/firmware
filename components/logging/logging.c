#include "lbr/logging.h"

/*
 * Stub logger. The on-disk format (versioned, append-oriented, recoverable) is
 * not frozen yet. Implement buffered SD writes behind a bounded queue, with a
 * recoverable record framing, then make the host tests in tests/host cover
 * power-loss / truncated-tail recovery and queue pressure.
 */

struct lbr_logger {
    int placeholder;
};

int lbr_log_open(lbr_logger_t *lg, const lbr_log_session_t *session)
{
    (void)lg;
    (void)session;
    return -1; /* TODO */
}

int lbr_log_append_sample(lbr_logger_t *lg, const lbr_sample_t *s)
{
    (void)lg;
    (void)s;
    return -1; /* TODO */
}

int lbr_log_append_health(lbr_logger_t *lg, const lbr_health_record_t *h)
{
    (void)lg;
    (void)h;
    return -1; /* TODO */
}

int lbr_log_flush(lbr_logger_t *lg)
{
    (void)lg;
    return -1; /* TODO */
}

int lbr_log_close(lbr_logger_t *lg)
{
    (void)lg;
    return -1; /* TODO */
}
