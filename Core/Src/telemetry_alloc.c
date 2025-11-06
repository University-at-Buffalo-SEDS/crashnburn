// Core/Src/telemetry_alloc.c
#include <stddef.h>
#include <stdlib.h>
#include "FreeRTOS.h"

/*
 * Rust expects these functions to exist for heap allocations:
 *
 *   void *telemetryMalloc(size_t);
 *   void telemetryFree(void *);
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
