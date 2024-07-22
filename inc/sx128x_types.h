#ifndef SX128X_TYPES_H
#define SX128X_TYPES_H

#include <stdio.h>
#include <stdint.h>

/**
 * @brief main sx1280 context
 */
typedef struct sx128x_context_s
{
    uint32_t cs_gpio;
    uint32_t nrst_gpio;
} sx128x_context_t;

#endif