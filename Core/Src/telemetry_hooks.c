// Core/Src/telemetry_alloc.c
#include <stddef.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include <stdio.h>

/*
 * Rust expects these functions to exist for heap allocations:
 *
 *   void *telemetryMalloc(size_t);
 *   void telemetryFree(void *);
 *   void seds_error_msg(const char *, size_t);
 *
 */

void *telemetryMalloc(size_t xSize)
{
    /* Make sure pool is ready – safe to call multiple times */
     return pvPortMalloc(xSize);
}

void telemetryFree(void *pv)
{
    vPortFree(pv);
}

void seds_error_msg(const char *str, size_t len)
{
    (void)len;
    /* Simple implementation using standard printf */
    printf("%s\r\n", str);
}
