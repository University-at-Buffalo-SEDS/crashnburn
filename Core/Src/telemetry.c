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

  /* Minimal stub: do nothing and report success.
     For visibility during bring-up, you can uncomment the printf,
     but avoid heavy I/O inside the router’s critical path.

  printf("[TX %u]:", (unsigned)len);
  for (size_t i = 0; i < len; i++) printf(" %02X", bytes[i]);
  printf("\r\n");
  */

  return SEDS_OK;
}

/* ---------------- RX helpers (optional) ---------------- */
void rx_synchronous(const uint8_t *bytes, size_t len) {
  if (!bytes || !len)
    return;
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return;
  }

  seds_router_receive_serialized(g_router.r, bytes, len);
}

void rx_asynchronous(const uint8_t *bytes, size_t len) {
  if (!bytes || !len)
    return;
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return;
  }

  seds_router_rx_serialized_packet_to_queue(g_router.r, bytes, len);
}

/* ---------------- Local endpoint handler ---------------- */
SedsResult on_radio_packet(const SedsPacketView *pkt, void *user) {
  (void)user;

  /* Keep this lean; avoid big stack buffers. */
  /* If you need a formatted string, you can cap it. */
  char buf[seds_pkt_to_string_len(pkt)];
  SedsResult s = seds_pkt_to_string(pkt, buf, sizeof(buf));
  if (s == SEDS_OK) {
    printf("on_radio_packet: %s\r\n", buf);
  } else {
    printf("on_radio_packet: seds_pkt_to_string failed (%d)\r\n", s);
  }
  return s;
}

/* ---------------- Router init (idempotent) ---------------- */
SedsResult init_telemetry_router(void) {
  /* Fast check without lock to avoid needless acquire in the common case. */
  if (g_router.created && g_router.r)
    return SEDS_OK;

  if (g_router.created && g_router.r) {
    return SEDS_OK;
  }

  const SedsLocalEndpointDesc locals[] = {
      {.endpoint = SEDS_EP_SERIAL,
       .packet_handler = on_radio_packet,
       .user = NULL},
  };

  SedsRouter *r =
      seds_router_new(tx_send,               /* tx callback */
                      NULL,                  /* tx_user */
                      node_now_since_bus_ms, /* clock */
                      locals, (uint32_t)(sizeof(locals) / sizeof(locals[0])));

  if (!r) {
    printf("Error: failed to create router\r\n");
    g_router.r = NULL;
    g_router.created = 0;

    return SEDS_ERR;
  }

  g_router.r = r;
  g_router.created = 1;
  g_router.start_time = stm_now_ms(NULL);

  return SEDS_OK;
}

/* ---------------- Logging APIs (unchanged) ---------------- */
SedsResult log_telemetry_synchronous(SedsDataType data_type, const void *data,
                                     size_t element_count,
                                     size_t element_size) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  if (!data || element_count == 0 || element_size == 0)
    return SEDS_ERR;

  const size_t total_bytes = element_count * element_size;

  SedsResult res = seds_router_log(g_router.r, data_type, data, total_bytes);

  return res;
}

SedsResult log_telemetry_asynchronous(SedsDataType data_type, const void *data,
                                      size_t element_count,
                                      size_t element_size) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }
  if (!data || element_count == 0 || element_size == 0)
    return SEDS_ERR;

  const size_t total_bytes = element_count * element_size;

  SedsResult res =
      seds_router_log_queue(g_router.r, data_type, data, total_bytes);

  return res;
}

/* ---------------- Queue processing (unchanged) ---------------- */
SedsResult dispatch_tx_queue(void) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }

  SedsResult res = seds_router_process_send_queue(g_router.r);

  return res;
}

SedsResult process_rx_queue(void) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }

  SedsResult res = seds_router_process_received_queue(g_router.r);

  return res;
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }

  SedsResult res =
      seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);

  return res;
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }

  SedsResult res =
      seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);

  return res;
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms) {
  if (!g_router.r) {
    if (init_telemetry_router() != SEDS_OK)
      return SEDS_ERR;
  }

  SedsResult res =
      seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);

  return res;
}

/* ---------------- Error printing ---------------- */
SedsResult print_telemetry_error(const int32_t error_code) {
  /* Use a small fixed buffer to avoid big stack frames. */
  char buf[seds_error_to_string_len(error_code)];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK) {
    printf("Error: %s\r\n", buf);
  } else {
    printf("Error: seds_error_to_string failed: %d\r\n", res);
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
