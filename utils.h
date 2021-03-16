#ifndef UTILS_H
#define UTILS_H

#define ERR_DEBUG 5

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*x))


#define DB_PRINT_L(level, ...) do { \
        if (ERR_DEBUG > (level)) { \
                    printf(": %s: ", __func__); \
                    printf(__VA_ARGS__); \
                } \
} while (0)

#endif
