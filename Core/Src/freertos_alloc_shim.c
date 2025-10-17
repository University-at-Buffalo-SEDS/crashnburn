#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>

// Minimal shims so code that calls pvPortMalloc/vPortFree links without FreeRTOS.
void *pvPortMalloc(size_t xSize) {
    return malloc(xSize);
}

void vPortFree(void *pv) {
    free(pv);
}