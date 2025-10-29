#include "FreeRTOS.h"
#include "task.h"

void vAssertCalled(const char *file, int line) {
    (void)file; (void)line;
    taskENTER_CRITICAL();
    __asm volatile("bkpt #0");
    for(;;){}
}

void vApplicationMallocFailedHook(void) {
    vAssertCalled(__FILE__, __LINE__);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    vAssertCalled(__FILE__, __LINE__);
}
