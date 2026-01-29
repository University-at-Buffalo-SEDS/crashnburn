// telemetry.c
#include "telemetry.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "sedsprintf.h"
#include "stm32g4xx_hal.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void vPrintHeapStats(const char *fmt) {
  size_t free_now = xPortGetFreeHeapSize();
  size_t min_ever_free = xPortGetMinimumEverFreeHeapSize();

  size_t total = configTOTAL_HEAP_SIZE; // bytes
  size_t used_now = total - free_now;
  size_t max_used = total - min_ever_free;

  printf("=========================\r\n%s\r\nHeap total: %u bytes\r\nHeap used "
         ": %u bytes\r\nHeap free : %u "
         "bytes\r\nHeap max used (high-water): %u "
         "bytes\r\n=========================\r\n",
         fmt ? fmt : "Heap Stats", (unsigned)total, (unsigned)used_now,
         (unsigned)free_now, (unsigned)max_used);
}

/* ---------------- Time helpers: 32->64 extender ---------------- */
static uint64_t stm_now_ms(void *user) {
  (void)user;
  static uint32_t last32 = 0;
  static uint64_t high = 0;
  uint32_t cur32 = HAL_GetTick();
  if (cur32 < last32) {
    high += (1ULL << 32);
  }
  last32 = cur32;
  return high | (uint64_t)cur32;
}

uint64_t node_now_since_bus_ms(void *user) {
  (void)user;
  const uint64_t now = stm_now_ms(NULL);
  const RouterState s = g_router; /* snapshot */
  return s.r ? (now - s.start_time) : 0;
}

/* ---------------- Global router state ---------------- */
RouterState g_router = {.r = NULL, .created = 0, .start_time = 0};

/* ---------------- TX path (stub/printf/USB) ---------------- */
SedsResult tx_send(const uint8_t *bytes, size_t len, void *user) {
  (void)user;
  (void)bytes;
  (void)len;

  /* Minimal stub: do nothing and report success. */
  return SEDS_OK;
}

/* ---------------- RX helpers ---------------- */
void rx_synchronous(const uint8_t *bytes, size_t len) {
  if (!bytes || !len) return;
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return;
  }
  (void)seds_router_receive_serialized(g_router.r, bytes, len);
}

void rx_asynchronous(const uint8_t *bytes, size_t len) {
  if (!bytes || !len) return;
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return;
  }
  (void)seds_router_rx_serialized_packet_to_queue(g_router.r, bytes, len);
}

/* ---------------- Local endpoint handler ---------------- */
SedsResult on_radio_packet(const SedsPacketView *pkt, void *user) {
  (void)user;

  int32_t need = seds_pkt_to_string_len(pkt);
  if (need <= 0) {
    printf("on_radio_packet: seds_pkt_to_string_len failed (%d)\r\n", (int)need);
    return (SedsResult)need;
  }

  /* VLA: if you want to avoid this entirely, replace with a fixed cap. */
  char buf[(size_t)need];
  SedsResult s = seds_pkt_to_string(pkt, buf, sizeof(buf));
  if (s == SEDS_OK) {
    printf("on_radio_packet: %s\r\n", buf);
  } else {
    printf("on_radio_packet: seds_pkt_to_string failed (%d)\r\n", (int)s);
  }
  return s;
}

/* ---------------- Router init (idempotent) ---------------- */
SedsResult init_telemetry_router(void) {
  if (g_router.created && g_router.r) return SEDS_OK;

  const SedsLocalEndpointDesc locals[] = {
      {
          .endpoint = SEDS_EP_SERIAL,
          .packet_handler = on_radio_packet,
          .serialized_handler = NULL,
          .user = NULL,
      },
  };

  /* NEW API signature */
  SedsRouter *r = seds_router_new(
      Seds_RM_Sink,
      node_now_since_bus_ms, /* monotonic ms clock callback */
      NULL,                  /* clock user */
      locals,
      (size_t)(sizeof(locals) / sizeof(locals[0])));

  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0;
    return SEDS_ERR;
  }

  /* Register outbound side (NEW API) */
  int32_t side_id = seds_router_add_side_serialized(
      r,
      "radio", 5,
      tx_send,
      NULL,
      false /* reliable_enabled */
  );

  if (side_id < 0) {
    printf("Error: failed to add TX side (%d)\r\n", (int)side_id);
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0;
    return (SedsResult)side_id;
  }

  g_router.r = r;
  g_router.created = 1;
  g_router.start_time = stm_now_ms(NULL);
  return SEDS_OK;
}

/* ---------------- Internal helper: best-effort elem kind ----------------
   telemetry.h's API does not expose elem-kind, so we infer it.
   - 4 bytes => float32
   - 8 bytes => float64
   - otherwise => unsigned integer
   If you need signed/int or want exact control, call the C11/C++ helpers from
   sedsprintf.h directly (seds_router_log_typed_ex / seds_router_log* macros).
*/
static SedsElemKind infer_elem_kind(size_t elem_size) {
  if (elem_size == 4) return SEDS_EK_FLOAT;
  if (elem_size == 8) return SEDS_EK_FLOAT;
  return SEDS_EK_UNSIGNED;
}

/* ---------------- Logging APIs (API preserved) ---------------- */
SedsResult log_telemetry_synchronous(SedsDataType data_type,
                                     const void *data,
                                     size_t element_count,
                                     size_t element_size) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  if (!data || element_count == 0 || element_size == 0) return SEDS_ERR;

  /* Optional: quick sanity check against schema fixed size. */
  int32_t expected = seds_dtype_expected_size(data_type);
  if (expected < 0) return (SedsResult)expected;

  size_t total_bytes = element_count * element_size;
  if ((int32_t)total_bytes != expected) {
    /* Keep old API behavior "best-effort": still return size mismatch rather than
       silently padding/truncating typed data. */
    return SEDS_SIZE_MISMATCH;
  }

  return seds_router_log_typed_ex(g_router.r,
                                 data_type,
                                 data,
                                 element_count,
                                 element_size,
                                 infer_elem_kind(element_size),
                                 /*timestamp*/NULL,
                                 /*queue*/0);
}

SedsResult log_telemetry_asynchronous(SedsDataType data_type,
                                      const void *data,
                                      size_t element_count,
                                      size_t element_size) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  if (!data || element_count == 0 || element_size == 0) return SEDS_ERR;

  int32_t expected = seds_dtype_expected_size(data_type);
  if (expected < 0) return (SedsResult)expected;

  size_t total_bytes = element_count * element_size;
  if ((int32_t)total_bytes != expected) {
    return SEDS_SIZE_MISMATCH;
  }

  return seds_router_log_typed_ex(g_router.r,
                                 data_type,
                                 data,
                                 element_count,
                                 element_size,
                                 infer_elem_kind(element_size),
                                 /*timestamp*/NULL,
                                 /*queue*/1);
}

/* ---------------- Queue processing (unchanged) ---------------- */
SedsResult dispatch_tx_queue(void) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_tx_queue(g_router.r);
}

SedsResult process_rx_queue(void) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_rx_queue(g_router.r);
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK) return SEDS_ERR;
  }
  return seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);
}

/* ---------------- Error printing ---------------- */
SedsResult print_telemetry_error(const int32_t error_code) {
  int32_t need = seds_error_to_string_len(error_code);
  if (need <= 0) {
    printf("Error: seds_error_to_string_len failed: %d\r\n", (int)need);
    return (SedsResult)need;
  }

  char buf[(size_t)need];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK) {
    printf("Error: %s\r\n", buf);
  } else {
    printf("Error: seds_error_to_string failed: %d\r\n", (int)res);
  }
  return res;
}

/* ---------------- Fatal helper ---------------- */
void die(const char *fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  while (1) {
    printf("FATAL: %s\r\n", buf);
    HAL_Delay(1000);
  }
}
