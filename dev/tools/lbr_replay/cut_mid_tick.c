/*
 * cut_mid_tick RUN.lbrlog OUT.lbrlog: OUT is RUN cut off a few bytes into the
 * first output (VALUE) after a LOGIC tick in the second half of the file, as
 * a power cut between a logic tick and its outputs would leave it. Test only:
 * lbr_replay must not count the outputs the cut took as differences.
 */
#include <stdio.h>
#include <stdlib.h>

#include "lbr/logging.h"
#include "lbr/replay.h"

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "usage: cut_mid_tick RUN.lbrlog OUT.lbrlog\n");
        return 2;
    }
    FILE *f = fopen(argv[1], "rb");
    if (f == NULL) {
        return 2;
    }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *b = n > 0 ? malloc((size_t)n) : NULL;
    if (b == NULL || fread(b, 1, (size_t)n, f) != (size_t)n) {
        fclose(f);
        free(b);
        return 2;
    }
    fclose(f);

    lbr_log_reader_t r;
    lbr_log_record_t rec;
    lbr_log_reader_init(&r, b, (size_t)n);
    bool   after_tick = false;
    size_t cut        = 0;
    while (cut == 0 && lbr_log_reader_next(&r, &rec)) {
        if (rec.offset < (size_t)n / 2) {
            continue;
        }
        lbr_tick_t t;
        if (rec.type == LBR_REC_TICK && lbr_tick_decode(rec.payload, rec.len, &t)) {
            after_tick = t.kind == LBR_TICK_LOGIC;
        } else if (after_tick && rec.type == LBR_REC_VALUE) {
            cut = rec.offset + 5;
        }
    }
    FILE *o  = cut > 0 ? fopen(argv[2], "wb") : NULL;
    int   rc = o != NULL && fwrite(b, 1, cut, o) == cut ? 0 : 2;
    if (o != NULL) {
        fclose(o);
    }
    free(b);
    return rc;
}
