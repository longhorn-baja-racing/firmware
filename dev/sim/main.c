/*
 * lbr_sim: run the whole car's firmware logic on a simulated car.
 *
 *   lbr_sim [options] SCENARIO.sim    run a scenario; exit status = failed checks
 *   lbr_sim [options] --interactive   read commands on stdin, JSON lines on stdout
 *                                     (what dev/sim/ui/server.py drives)
 *   lbr_sim [options] --duration S    run S seconds and print a summary
 *
 * Options: --seed N (default 1), --out DIR (default sim-out), --trace NODE.
 * A scenario is a list of "at <seconds> <command>" lines and one
 * "end <seconds>"; commands are the same as in interactive mode (see
 * dev/sim/README.md).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim.h"

static void usage(void)
{
    fprintf(stderr, "usage: lbr_sim [--seed N] [--out DIR] [--trace NODE] "
                    "(SCENARIO.sim | --interactive | --duration SECONDS)\n");
}

static int run_scenario(sim_t *s, const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "lbr_sim: cannot open %s\n", path);
        return 2;
    }
    char   line[512];
    int    lineno = 0;
    double end    = -1.0;
    printf("scenario %s (seed %llu)\n", path, (unsigned long long)s->seed);
    while (fgets(line, sizeof line, f) != NULL) {
        lineno++;
        char *p = line;
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (*p == '#' || *p == '\n' || *p == '\r' || *p == '\0') {
            continue;
        }
        double t;
        int    used = 0;
        if (sscanf(p, "end %lf", &t) == 1) {
            end = t;
            break;
        }
        if (sscanf(p, "at %lf %n", &t, &used) != 1 || used == 0) {
            fprintf(stderr, "%s:%d: expected 'at <seconds> <command>' or 'end <seconds>'\n", path,
                    lineno);
            fclose(f);
            return 2;
        }
        uint64_t at = (uint64_t)(t * 1e6);
        if (at < s->now) {
            fprintf(stderr, "%s:%d: times must not go backwards\n", path, lineno);
            fclose(f);
            return 2;
        }
        sim_run_until(s, at);
        p[strcspn(p, "\r\n")] = '\0';
        if (sim_command(s, p + used) < 0) {
            fprintf(stderr, "%s:%d: command failed: %s\n", path, lineno, p + used);
            fclose(f);
            return 2;
        }
    }
    fclose(f);
    if (end >= 0) {
        sim_run_until(s, (uint64_t)(end * 1e6));
    }
    printf("%u checks passed, %u failed\n", (unsigned)s->checks_passed, (unsigned)s->checks_failed);
    return s->checks_failed > 0 ? 1 : 0;
}

static int run_interactive(sim_t *s)
{
    char line[512];
    sim_snapshot(s);
    printf("{\"ev\":\"done\"}\n");
    fflush(stdout);
    while (fgets(line, sizeof line, stdin) != NULL) {
        line[strcspn(line, "\r\n")] = '\0';
        if (strcmp(line, "quit") == 0) {
            break;
        }
        sim_command(s, line);
        printf("{\"ev\":\"done\"}\n");
        fflush(stdout);
    }
    return 0;
}

int main(int argc, char **argv)
{
    unsigned long long seed        = 1;
    const char        *out         = "sim-out";
    const char        *trace       = NULL;
    const char        *scenario    = NULL;
    bool               interactive = false;
    double             duration    = -1.0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            seed = strtoull(argv[++i], NULL, 10);
        } else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out = argv[++i];
        } else if (strcmp(argv[i], "--trace") == 0 && i + 1 < argc) {
            trace = argv[++i];
        } else if (strcmp(argv[i], "--interactive") == 0) {
            interactive = true;
        } else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
            duration = atof(argv[++i]);
        } else if (argv[i][0] != '-' && scenario == NULL) {
            scenario = argv[i];
        } else {
            usage();
            return 2;
        }
    }
    if ((scenario != NULL) + interactive + (duration >= 0) != 1) {
        usage();
        return 2;
    }

    static sim_t s;
    if (sim_init(&s, seed, out, stdout, !interactive) != 0) {
        fprintf(stderr, "lbr_sim: init failed\n");
        return 2;
    }
    if (trace != NULL) {
        char cmd[96];
        snprintf(cmd, sizeof cmd, "trace %s", trace);
        s.quiet = false;
        sim_command(&s, cmd);
    }

    int rc;
    if (interactive) {
        rc = run_interactive(&s);
    } else if (scenario != NULL) {
        rc = run_scenario(&s, scenario);
    } else {
        sim_run_until(&s, (uint64_t)(duration * 1e6));
        s.quiet = false;
        sim_snapshot(&s);
        rc = 0;
    }
    sim_free(&s);
    return rc;
}
