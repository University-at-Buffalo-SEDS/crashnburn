/*
 * Shared platform header
 *
 * The purpose of this file is to unify the Flight
 * Computer board API and helpers, providing a single
 * point of reference and modification. This file also
 * enables easier conditional compilation and poisoning.
 *
 * This header provides the following components for the
 * DMA, Distribution, Evaluation, and Recovery modules:
 *
 * - Type-specific Max, Min, and Abs helper macros;
 * - GPIO and EXTI port mappings;
 * - ThreadX, HAL, sedsprintf_rs, and driver includes;
 * - Variadic aliases for select sedsprintf_rs functions;
 * - Variadic aliases for substitute stdio functions.
 * - Aliases for select HAL and driver functions;
 * - Misc aliases and includes as required by the modules.
 */

#ifndef PLATFORM_H
#define PLATFORM_H


/* ------ Bundled std headers used ------ */

#include <stdio.h>        // IWYU pragma: export
#include <stddef.h>       // IWYU pragma: export
#include <stdarg.h>       // IWYU pragma: export
#include <stdint.h>       // IWYU pragma: export
#include <stdbool.h>      // IWYU pragma: export
#include <stdatomic.h>    // IWYU pragma: export
#include <string.h>       // IWYU pragma: export
#include <math.h>         // IWYU pragma: export

/* ------ Bundled std headers used ------ */


/* ------ Pre-compilation checks ------ */


/* ------ Platform integer aliases ----- */

/* Fast means the _fastest_ integer of minimum width. 
 * Determined by the bundled library. */

typedef uint_fast8_t  fu8;
typedef uint_fast16_t fu16;
typedef uint_fast32_t fu32;
typedef uint_fast64_t fu64;

typedef int_fast8_t  fi8;
typedef int_fast16_t fi16;
typedef int_fast32_t fi32;
typedef int_fast64_t fi64;

/* ------ Platform integer aliases ----- */


/* ------ Atomic ops and MO aliases ------ */

enum seds_atomic_mo {
  Rlx    = memory_order_relaxed,
  Con    = memory_order_consume,
  Acq    = memory_order_acquire,
  Rel    = memory_order_release,
  AcqRel = memory_order_acq_rel,
  SeqCst = memory_order_seq_cst
};

#define load        atomic_load_explicit
#define store       atomic_store_explicit
#define swap        atomic_exchange_explicit
#define fetch_add   atomic_fetch_add_explicit
#define fetch_sub   atomic_fetch_sub_explicit
#define fetch_and   atomic_fetch_and_explicit
#define fetch_or    atomic_fetch_or_explicit
#define fetch_xor   atomic_fetch_xor_explicit
#define cas_weak    atomic_compare_exchange_weak_explicit
#define cas_strong  atomic_compare_exchange_strong_explicit

/* ------ Atomic ops and MO aliases ------ */

/* ------ Task utilities ------ */

#define DO_NOT_EXIT 0

#define task_loop(exit_predicate) while (!(exit_predicate))

/* Data memory barrier */

#if defined(__ARMCC_VERSION) || defined(__GNUC__) || defined(__ICCARM__)
#include "cmsis_compiler.h"   // IWYU pragma: export

#else
#define __DMB() atomic_thread_fence(AcqRel)

#endif // DMB support

/* ------ Task utilities ------ */


/* ------ IREC 2026 GPIO port maps ------ */

#define PYRO_PORT GPIOB
#define CO2_PIN   GPIO_PIN_5
#define REEF_PIN  GPIO_PIN_6

/* ------ IREC 2026 GPIO port maps ------ */


/* ------ Type attributes ------ */

#define serial __attribute__((packed, aligned(4)))

#define tx_align __attribute__((aligned(sizeof(ULONG))))

#define IREC26_unused __attribute__((unused))

#define constexpr __attribute__((const))

#define blind_inline __attribute__((always_inline))

/* ------ Type attributes ------ */


/* ------ HAL Aliases ------ */

#include "stm32g4xx.h"                // IWYU pragma: export
#include "stm32g4xx_hal.h"            // IWYU pragma: export
#include "stm32g4xx_hal_def.h"        // IWYU pragma: export
#include "stm32g4xx_hal_spi.h"        // IWYU pragma: export
#include "stm32g4xx_hal_gpio.h"       // IWYU pragma: export

extern SPI_HandleTypeDef hspi1;

/* Get currect tick for custom timer */

#define now_ms() HAL_GetTick()

/* ------ HAL Aliases ------ */


/* ------ Sensor drivers and data collection ------ */

#include "barometer.h"        // IWYU pragma: export
#include "gyro.h"        // IWYU pragma: export
#include "accel.h"    // IWYU pragma: export

struct serial coords { float x, y, z; };

#define gpio_cs_low(sens)                               \
  HAL_GPIO_WritePin((GPIO_TypeDef *)gpio.port[sens],   \
                    gpio.pin[sens], GPIO_PIN_RESET)

#define gpio_cs_high(sens)                              \
  HAL_GPIO_WritePin((GPIO_TypeDef *)gpio.port[sens],   \
                    gpio.pin[sens], GPIO_PIN_SET)

#define terminate_transfers() \
  do {                        \
    baro_cs_high();           \
    gyro_cs_high();           \
    accl_cs_high();           \
  } while (0)

/* Identification bytes for DMA Tx buffers */

#define BARO_TX_BYTE ((uint8_t)(BARO_DATA_0 | BARO_SPI_READ_BIT))
#define GYRO_TX_BYTE ((uint8_t)(gyro_cmd_read(GYRO_RATE_X_LSB)))
#define ACCL_TX_BYTE ((uint8_t)(accl_cmd_read(ACCL_X_LSB)))

/* Peripheral sensor EXT interrupt pins */

#define ACCL_INT_PIN_1  GPIO_PIN_4
#define ACCL_INT_PIN_2  GPIO_PIN_5
#define GYRO_INT_PIN_1  GPIO_PIN_0
#define GYRO_INT_PIN_2  GPIO_PIN_1
#define BARO_INT_PIN    GPIO_PIN_7

/* Driver-specific data conversions */

#define U32(b0, b1, b2, b3)                                         \
  (((uint32_t)(b3) << 24) | ((uint32_t)(b2) << 16) |                \
   ((uint32_t)(b1) << 8 ) |  (uint32_t)(b0))

#define U24(b0, b1, b2)                                             \
  (((uint32_t)(b2) << 16) | ((uint32_t)(b1) << 8) | (uint32_t)(b0))

#define I16(b0, b1)                                                 \
  ((int16_t)(((uint16_t)(b1) << 8) | (uint16_t)(b0)))

#define F16(b0, b1) ((float)I16(b0, b1))

/* ------ Sensor drivers and data collection ------ */


/* ------ Master-side interrupt control ------ */

#define SPI1_GLOBAL_IRQ   SPI1_IRQn
#define DMA_RECEIVER_SPI1 GPDMA1_Channel0_IRQn

#define Baro_EXTI   EXTI7_IRQn
#define Gyro_EXTI_1 EXTI0_IRQn
#define Gyro_EXTI_2 EXTI1_IRQn
#define Accl_EXTI_1 EXTI4_IRQn
#define Accl_EXTI_2 EXTI5_IRQn

#define irq_off(irq) HAL_NVIC_DisableIRQ((irq))
#define irq_on(irq)  HAL_NVIC_EnableIRQ((irq))

/* ------ Master-side interrupt control ------ */


#include <sedsprintf.h>         // IWYU pragma: export
#include "telemetry.h"          // IWYU pragma: export

#define log_msg_sync(msg, size)                             \
  log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,           \
                            (msg), (size), sizeof(char))

#define log_msg(msg, size)                                  \
  log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA,          \
                             (msg), (size), sizeof(char))

#define log_measurement(type, buf)                          \
  log_telemetry_asynchronous((type), (buf), 3, sizeof(float));

#define log_filter_data(buf, size)                          \
  log_telemetry_asynchronous(SEDS_DT_KALMAN_FILTER_DATA,    \
                             (buf), (size), sizeof(float));

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L

#define log_err_sync(fmt, ...)                              \
  log_error_syncronous(fmt __VA_OPT__(,) __VA_ARGS__)
                           
#define log_err(fmt, ...)                                   \
  log_error_asyncronous(fmt __VA_OPT__(,) __VA_ARGS__)

#define log_die(fmt, ...) die(fmt __VA_OPT__(,) __VA_ARGS__)

#else 
#if defined (__GNUC__)

#define log_err_sync(fmt, ...)                              \
  log_error_syncronous(fmt, ##__VA_ARGS__)

#define log_err(fmt, ...)                                   \
  log_error_asyncronous(fmt, ##__VA_ARGS__)

#define log_die(fmt, ...) die(fmt, ##__VA_ARGS__)

#endif // GNUC
#endif // >= C23


#endif // PLATFORM_H