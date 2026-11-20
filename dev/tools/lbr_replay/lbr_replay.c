/*
 * lbr_replay: re-run the car logic on a recorded log (AdvantageKit-style replay).
 *
 *     lbr_replay RUN.lbrlog [OUT.lbrlog] [--check] [--drop-task NAME]
 *
 * Feeds the log's CAN frames and logic/heartbeat ticks, in log order, through
 * the same decoder, node table, and logic runner the Central uses
 * (lbr/replay.h), with this build's car logic (car/logic). OUT gets every
 * record of RUN plus the replayed outputs as "Replay/<key>", so the dashboard
 * shows the original and the replay side by side. Change a task, rebuild, and
 * replay a run to see what the change would have done on it.
 *
 * It then compares the replayed outputs with the recorded ones, value by
 * value. --check exits 1 on any difference (or if the log does not start at
 * the Central's START, so it cannot be replayed from the beginning); with the
 * logic unchanged a replay is bit for bit identical. --drop-task leaves one
 * task out, to see what it contributed (and to test the comparison).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "car_logic.h"
#include "lbr/bytes.h"
#include "lbr/logging.h"
#include "lbr/replay.h"

#define REPLAY_PREFIX "Replay/"
#define REPLAY_ID_BIT 0x8000u
#define MAX_KEYS      256u
#define KEY_MAX       64u

typedef struct {
    uint64_t t_us;
    uint16_t key; /* index into the key names */
    uint8_t  status;
    uint32_t bits;
} value_t;

typedef struct {
    value_t *v;
    size_t   n, cap;
    char     names[MAX_KEYS][KEY_MAX]; /* by output id (1-based) */
} series_t;

static series_t s_original, s_replayed;
static FILE    *s_out;
static uint32_t s_out_records;
static uint32_t s_old_replay; /* Replay/ records already in RUN (left out) */
static bool     s_ended;      /* RUN has its END record (closed cleanly) */
static uint64_t s_last_logic; /* time of RUN's last LOGIC tick */

static void push(series_t *s, uint64_t t, uint16_t key, uint8_t status, uint32_t bits)
{
    if (s->n == s->cap) {
        s->cap     = s->cap ? s->cap * 2 : 4096;
        value_t *v = realloc(s->v, s->cap * sizeof *v);
        if (v == NULL) {
            fprintf(stderr, "lbr_replay: out of memory\n");
            exit(2);
        }
        s->v = v;
    }
    s->v[s->n++] = (value_t){t, key, status, bits};
}

static void note_key(series_t *s, const uint8_t *p, size_t len)
{
    if (len < 4) {
        return;
    }
    uint16_t id = lbr_get_u16(p) & (uint16_t)~REPLAY_ID_BIT;
    size_t   n  = len - 4 < KEY_MAX - 1 ? len - 4 : KEY_MAX - 1;
    if (id > 0 && id < MAX_KEYS) {
        memcpy(s->names[id], p + 4, n);
        s->names[id][n] = '\0';
    }
}

static void note_value(series_t *s, const uint8_t *p, size_t len)
{
    if (len < LBR_VALUE_REC_SIZE) {
        return;
    }
    uint16_t id = lbr_get_u16(p + 8) & (uint16_t)~REPLAY_ID_BIT;
    push(s, lbr_get_u64(p), id, p[10], lbr_get_u32(p + 11));
}

static void write_record(uint8_t type, const uint8_t *payload, size_t len)
{
    static uint8_t buf[LBR_LOG_MAX_RECORD];
    size_t         n = lbr_log_frame(type, payload, len, buf, sizeof buf);
    if (s_out != NULL && n > 0 && fwrite(buf, 1, n, s_out) == n) {
        s_out_records++;
    }
}

/* The replayed logic's outputs: renamed Replay/<key>, ids with the top bit set
 * so they never collide with the recorded ones. */
static void replay_sink(void *ctx, uint8_t type, const uint8_t *payload, size_t len)
{
    (void)ctx;
    uint8_t rec[4 + sizeof REPLAY_PREFIX + KEY_MAX];
    if (type == LBR_REC_KEY && len >= 4 && len - 4 < KEY_MAX) {
        note_key(&s_replayed, payload, len);
        memcpy(rec, payload, 4);
        lbr_put_u16(rec, (uint16_t)(lbr_get_u16(payload) | REPLAY_ID_BIT));
        memcpy(rec + 4, REPLAY_PREFIX, sizeof REPLAY_PREFIX - 1);
        memcpy(rec + 4 + sizeof REPLAY_PREFIX - 1, payload + 4, len - 4);
        write_record(type, rec, len + sizeof REPLAY_PREFIX - 1);
    } else if (type == LBR_REC_VALUE && len == LBR_VALUE_REC_SIZE) {
        note_value(&s_replayed, payload, len);
        memcpy(rec, payload, len);
        lbr_put_u16(rec + 8, (uint16_t)(lbr_get_u16(payload + 8) | REPLAY_ID_BIT));
        write_record(type, rec, len);
    }
}

/* A KEY / VALUE record of an earlier replay (RUN is itself a replay's OUT). */
static bool is_old_replay(const lbr_log_record_t *rec)
{
    if (rec->type == LBR_REC_KEY && rec->len >= 4) {
        return (lbr_get_u16(rec->payload) & REPLAY_ID_BIT) != 0;
    }
    if (rec->type == LBR_REC_VALUE && rec->len >= LBR_VALUE_REC_SIZE) {
        return (lbr_get_u16(rec->payload + 8) & REPLAY_ID_BIT) != 0;
    }
    return false;
}

static uint8_t *read_file(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = size > 0 ? malloc((size_t)size) : NULL;
    if (buf != NULL && fread(buf, 1, (size_t)size, f) != (size_t)size) {
        free(buf);
        buf = NULL;
    }
    fclose(f);
    *len = buf != NULL ? (size_t)size : 0;
    return buf;
}

static const char *name_of(const series_t *s, uint16_t id)
{
    return id < MAX_KEYS && s->names[id][0] != '\0' ? s->names[id] : "?";
}

/* Value by value, in order. Returns the number of differences. */
static size_t compare(void)
{
    size_t n    = s_original.n < s_replayed.n ? s_original.n : s_replayed.n;
    size_t diff = 0;
    for (size_t i = 0; i < n; i++) {
        const value_t *a = &s_original.v[i], *b = &s_replayed.v[i];
        bool same = a->t_us == b->t_us && a->status == b->status && a->bits == b->bits &&
                    strcmp(name_of(&s_original, a->key), name_of(&s_replayed, b->key)) == 0;
        if (!same && diff++ == 0) {
            printf("first difference, output %zu:\n  recorded %.6f s %s status %u bits 0x%08x\n"
                   "  replayed %.6f s %s status %u bits 0x%08x\n",
                   i, (double)a->t_us / 1e6, name_of(&s_original, a->key), a->status, a->bits,
                   (double)b->t_us / 1e6, name_of(&s_replayed, b->key), b->status, b->bits);
        }
    }
    size_t extra = s_original.n > s_replayed.n ? s_original.n - n : s_replayed.n - n;
    if (extra > 0 && s_replayed.n > s_original.n && !s_ended) {
        /* A log cut off (power loss) between a LOGIC tick and the last of
         * its outputs: the replay ran that tick in full. */
        bool final_tick = true;
        for (size_t i = n; i < s_replayed.n; i++) {
            final_tick = final_tick && s_replayed.v[i].t_us >= s_last_logic;
        }
        if (final_tick) {
            printf("%zu replayed output(s) of the final tick not compared (the log ends "
                   "mid-tick)\n",
                   extra);
            extra = 0;
        }
    }
    if (extra > 0) {
        printf("%zu %s output(s) with no counterpart\n", extra,
               s_original.n > s_replayed.n ? "recorded" : "replayed");
    }
    return diff + extra;
}

static int usage(void)
{
    fprintf(stderr, "usage: lbr_replay RUN.lbrlog [OUT.lbrlog] [--check] [--drop-task NAME]\n");
    return 2;
}

int main(int argc, char **argv)
{
    const char *in = NULL, *out = NULL, *drop = NULL;
    bool        check = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--check") == 0) {
            check = true;
        } else if (strcmp(argv[i], "--drop-task") == 0 && i + 1 < argc) {
            drop = argv[++i];
        } else if (argv[i][0] == '-') {
            return usage();
        } else if (in == NULL) {
            in = argv[i];
        } else if (out == NULL) {
            out = argv[i];
        } else {
            return usage();
        }
    }
    if (in == NULL) {
        return usage();
    }
    size_t   len;
    uint8_t *data = read_file(in, &len);
    if (data == NULL) {
        fprintf(stderr, "lbr_replay: cannot read %s\n", in);
        return 2;
    }

    /* The car's tasks, minus --drop-task. */
    const lbr_task *tasks[LBR_LOGIC_MAX_TASKS];
    size_t          n_tasks = 0;
    for (size_t i = 0; i < car_logic.n && n_tasks < LBR_LOGIC_MAX_TASKS; i++) {
        if (drop == NULL || strcmp(car_logic.tasks[i]->name, drop) != 0) {
            tasks[n_tasks++] = car_logic.tasks[i];
        }
    }
    if (car_logic.n > LBR_LOGIC_MAX_TASKS) {
        fprintf(stderr, "lbr_replay: %zu tasks, at most %u (the Central refuses this list)\n",
                car_logic.n, LBR_LOGIC_MAX_TASKS);
        return 2;
    }
    if (drop != NULL && n_tasks == car_logic.n) {
        fprintf(stderr, "lbr_replay: no task named %s\n", drop);
        return 2;
    }

    if (out != NULL && (s_out = fopen(out, "wb")) == NULL) {
        fprintf(stderr, "lbr_replay: cannot write %s\n", out);
        return 2;
    }
    static lbr_replay_t r;
    lbr_replay_init(&r, tasks, n_tasks, (lbr_logic_sink_t){.record = replay_sink});

    lbr_log_reader_t reader;
    lbr_log_record_t rec;
    bool             first = true;
    lbr_log_reader_init(&reader, data, len);
    while (lbr_log_reader_next(&reader, &rec)) {
        if (first) {
            first         = false;
            uint32_t hash = rec.len >= 12 ? lbr_get_u32(rec.payload + 8) : 0;
            if (rec.type != LBR_REC_SESSION || hash != lbr_catalog_hash()) {
                fprintf(stderr,
                        "lbr_replay: %s was written with a different catalog (or has no "
                        "session record); replay it with that firmware's catalog\n",
                        in);
                free(data);
                return 2;
            }
        }
        if (is_old_replay(&rec)) {
            s_old_replay++; /* an earlier replay's output: neither recorded nor copied */
            continue;
        }
        write_record(rec.type, rec.payload, rec.len);
        if (rec.type == LBR_REC_KEY) {
            note_key(&s_original, rec.payload, rec.len);
        } else if (rec.type == LBR_REC_VALUE) {
            note_value(&s_original, rec.payload, rec.len);
        } else if (rec.type == LBR_REC_END) {
            s_ended = true;
        } else if (rec.type == LBR_REC_TICK) {
            lbr_tick_t t;
            if (lbr_tick_decode(rec.payload, rec.len, &t) && t.kind == LBR_TICK_LOGIC) {
                s_last_logic = t.t_us;
            }
        }
        lbr_replay_record(&r, rec.type, rec.payload, rec.len);
    }
    if (s_out != NULL) {
        fclose(s_out);
    }

    printf("%s: %u records, %u frames, %u logic ticks, %u heartbeats%s%s\n", in, reader.records,
           r.frames, r.logic_ticks, r.heartbeats,
           reader.corrupt_regions ? ", corrupt regions skipped" : "",
           reader.truncated_tail ? ", truncated tail" : "");
    printf("outputs: %zu recorded, %zu replayed\n", s_original.n, s_replayed.n);
    if (s_old_replay > 0) {
        printf("%u records of an earlier replay left out\n", s_old_replay);
    }
    if (!r.started) {
        printf("the log does not start at the Central's START (opened mid-session): the "
               "replay starts cold and may differ until the logic warms up\n");
    }
    size_t diff = compare();
    if (diff == 0) {
        printf("replay matches the recording, bit for bit\n");
    }
    if (out != NULL) {
        printf("wrote %s (%u records, replayed outputs as %s<key>)\n", out, s_out_records,
               REPLAY_PREFIX);
    }
    free(data);
    return check && (diff != 0 || !r.started) ? 1 : 0;
}
