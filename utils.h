#include <stdbool.h>
#include <stdarg.h>

// Set a bit
#define SET_BIT(var, pos) ((var) |= (1UL << (pos)))

// Clear a bit
#define CLEAR_BIT(var, pos) ((var) &= ~(1UL << (pos)))

// Toggle a bit
#define TOGGLE_BIT(var, pos) ((var) ^= (1UL << (pos)))

// Read a bit
#define READ_BIT(var, pos) (((var) & (1UL << (pos))) >> (pos))

#define BIT(x) (1 << (x))

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define FIND_ENTRY(table, size, steer, field)                \
    ({                                                      \
        __typeof__(table[0]) *result = NULL;                \
        for (size_t i = 0; i < (size); i++) {               \
            if ((table)[i].field == (steer)) {              \
                result = &(table)[i];                       \
                break;                                      \
            }                                               \
        }                                                   \
        result;                                             \
    })