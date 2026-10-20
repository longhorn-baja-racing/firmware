#pragma once

/*
 * Minimal host test harness: CHECK records a failure and keeps going so one
 * run reports every broken expectation; main returns the failure count.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static int lbr_test_failures;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond);               \
            lbr_test_failures++;                                                                   \
        }                                                                                          \
    } while (0)

#define CHECK_EQ_U(a, b)                                                                           \
    do {                                                                                           \
        uint64_t va_ = (uint64_t)(a), vb_ = (uint64_t)(b);                                         \
        if (va_ != vb_) {                                                                          \
            fprintf(stderr, "%s:%d: CHECK_EQ failed: %s == %s (%" PRIu64 " != %" PRIu64 ")\n",     \
                    __FILE__, __LINE__, #a, #b, va_, vb_);                                         \
            lbr_test_failures++;                                                                   \
        }                                                                                          \
    } while (0)

#define CHECK_EQ_I(a, b)                                                                           \
    do {                                                                                           \
        int64_t va_ = (int64_t)(a), vb_ = (int64_t)(b);                                            \
        if (va_ != vb_) {                                                                          \
            fprintf(stderr, "%s:%d: CHECK_EQ failed: %s == %s (%" PRId64 " != %" PRId64 ")\n",     \
                    __FILE__, __LINE__, #a, #b, va_, vb_);                                         \
            lbr_test_failures++;                                                                   \
        }                                                                                          \
    } while (0)

#define RUN(fn)                                                                                    \
    do {                                                                                           \
        int before_ = lbr_test_failures;                                                           \
        fn();                                                                                      \
        printf("%s %s\n", lbr_test_failures == before_ ? "PASS" : "FAIL", #fn);                    \
    } while (0)

#define TEST_MAIN_END() return lbr_test_failures == 0 ? 0 : 1
