#ifndef UTILS_H
#define UTILS_H

#include <assert.h>

#define ERR_DEBUG 0

/* To tell the user what's an IRQ handler */
#define __irq_handler

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*x))


#define DB_PRINT_L(level, ...) do { \
        if (ERR_DEBUG > (level)) { \
                    printf(": %s: ", __func__); \
                    printf(__VA_ARGS__); \
                } \
} while (0)

#define DB_PRINT(level, ...) do { \
        if (ERR_DEBUG > (level)) { \
                    printf(__VA_ARGS__); \
                } \
} while (0)

/*
 * Use instead of normal assertions.
 * we should catch all asserts before a user sees them, but we don't want the
 * off-chance that the user runs into an assert and it interrupts the user
 * experience.
 *
 * We have memory to spare, so print the function that would have asserted.
 */
#define DBG_ASSERT(x) do {                        \
    if (ERR_DEBUG) {                              \
        assert(x);                                \
    } else {                                      \
        printf("%s: ASSERT REACHED\n", __func__); \
    }                                             \
} while (0)
#endif
