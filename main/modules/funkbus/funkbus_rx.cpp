#include "funkbus_rx.h"

//
// funkbus_rx.cpp — ESP32 RMT-based Funkbus receiver
//
// Captures CC1101 GDO0 pulses, decodes 48-bit Funkbus frames, groups repeated
// presses, and publishes results via OMG pub().
//
// License: MIT
//

#include <Arduino.h>

#include "config_Funkbus.h"
#include "funkbus_cc1101_toolbox.h"
#include "funkbus_log.h"
#include "funkbus_tx.h"

#ifndef ARDUINO_ARCH_ESP32
#  error "Funkbus RX requires ESP32 (RMT). Build on ESP32 or disable Funkbus RX."
#endif

// ---- 1) User-facing: RX activity LED (override in config_Funkbus.h) --------
#ifndef FUNKBUS_LED_RX_ENABLE
#  define FUNKBUS_LED_RX_ENABLE 1 // 1: flash LED on RX activity
#endif

#if FUNKBUS_LED_RX_ENABLE
#  ifndef FUNKBUS_LED_RX_GPIO
#    ifdef LED_BUILTIN
#      define FUNKBUS_LED_RX_GPIO LED_BUILTIN // default: built-in LED
#    else
#      define FUNKBUS_LED_RX_GPIO 2 // fallback GPIO
#    endif
#  endif
#  ifndef FUNKBUS_LED_RX_MS
#    define FUNKBUS_LED_RX_MS 60 // LED on-time per RX event (ms)
#  endif
#  ifndef FUNKBUS_LED_ACTIVE_HIGH
#    define FUNKBUS_LED_ACTIVE_HIGH 1 // 1: LED active HIGH, 0: active LOW
#  endif
#endif

// ---- 2) RMT HW setup & low-level timers ------------------------------------
constexpr uint32_t FUNKBUS_RMT_CLKDIV = 80; // 80MHz/80 => 1 MHz tick
constexpr int FUNKBUS_RMT_RX_CHANNEL = RMT_CHANNEL_2;
constexpr uint8_t FUNKBUS_RMT_MEMBLOCKS = 4;

constexpr uint32_t FUNKBUS_PERF_RXRECV_BUDGET_US = 6000; // warn if RX loop > this
constexpr uint32_t FUNKBUS_PERF_RXRECV_EVERY_N = 0; // force log every N packets (0=off)

namespace {
inline constexpr uint32_t kRmtTickUs = 1; // 1 µs per tick
inline constexpr uint16_t kRmtGlitchUs = 130; // ignore pulses < this (µs)
inline constexpr uint32_t kRmtIdleThUs = 25000; // idle ends burst (µs)
inline constexpr uint32_t kRmtRecvTimeoutMs = 5; // ringbuffer read timeout
inline constexpr size_t kRmtRingbufBytes = 32768; // ringbuffer size
} // namespace

// ---- 3) Optional: gate RX by CC1101 GDO2 + worker task params --------------
#define FUNKBUS_RX_GATE_BY_CS 1 // 1: gate RX with GDO2; 0: always-on
constexpr bool FUNKBUS_CS_ACTIVE_HIGH = false; // true: GDO2 high=active

namespace {
inline constexpr uint32_t kCsMinHighMs = 2; // min HIGH time to arm (ms)
inline constexpr uint32_t kCsMinOnMs = 12; // min ON window (ms)
inline constexpr uint32_t kCsMinLowMs = 8; // min LOW time to re-arm (ms)

inline constexpr uint32_t kRxWorkerStack = 8192;
inline constexpr UBaseType_t kRxWorkerPrio = configMAX_PRIORITIES - 2;
#if defined(ARDUINO_RUNNING_CORE)
inline constexpr BaseType_t kRxWorkerCore = ARDUINO_RUNNING_CORE;
#else
inline constexpr BaseType_t kRxWorkerCore = 1;
#endif
} // namespace

// ---- 4) Optional periodic debug --------------------------------------------
#define FUNKBUS_RX_DEBUG 0
constexpr uint32_t FUNKBUS_RX_DEBUG_PERIOD_MS = 5000;

// ---- 5) Funkbus symbol thresholds & frame geometry -------------------------
constexpr uint32_t FUNKBUS_PRE_H_MIN_US = 3300;
constexpr uint32_t FUNKBUS_PRE_H_MAX_US = 4300;
constexpr uint32_t FUNKBUS_PRE_L_MIN_US = 900;
constexpr uint32_t FUNKBUS_PRE_L_MAX_US = 1500;

constexpr uint32_t FUNKBUS_SHORT_MAX_US = 750;
constexpr uint32_t FUNKBUS_LONG_MIN_US = FUNKBUS_SHORT_MAX_US + 1;

constexpr uint32_t FUNKBUS_MIN_PULSES = 10;
constexpr uint32_t FUNKBUS_STARTIDX_FALLBACK = 1;

constexpr int FUNKBUS_FRAME_BITS = 48;
constexpr int FUNKBUS_TRAILER_BITS = 8; // 3 scom + 1 parity + 4 checksum
constexpr int FUNKBUS_PAYLOAD_BITS = 40;
constexpr int FUNKBUS_PARITY_RANGE_BITS = 43;

// ---- 6) RX grouping/debounce (treat repeated presses as one) ---------------
namespace {
inline constexpr uint32_t kRxGroupGapMs = 350;
inline constexpr uint32_t kRxGroupMaxMs = 3000;
inline constexpr uint8_t kRxGroupMaxFrames = 24;
inline constexpr uint32_t kRxFirstFrameHoldMs = 450;

inline constexpr uint32_t kRxAdaptiveNum = 5;
inline constexpr uint32_t kRxAdaptiveDen = 2;
} // namespace

// ---- 7) SCOM semantics ------------------------------------------------------
constexpr uint8_t FUNKBUS_SCOM_RESTART_ON = 0;
constexpr uint8_t FUNKBUS_SCOM_RESTART_OFF = 1;
constexpr uint8_t FUNKBUS_SCOM_SHORT_MAX = 3; // ≤ this => SHORT press

// ---- 8) Local buffers & hotpath attributes ---------------------------------
namespace {
inline constexpr size_t kRxPulseCap = 256; // max pulses captured
} // namespace

#if defined(ESP32)
#  define FB_IRAM IRAM_ATTR
#else
#  define FB_IRAM
#endif

// ---- 9) Profiling & duration LUT -------------------------------------------
#define FUNKBUS_BUILD_EARLYCUT 1
#define FUNKBUS_RX_DECODE_PROF 1

constexpr uint32_t FUNKBUS_RX_DECODE_PROF_LOG_US = 10000; // log if total > this
constexpr uint32_t FUNKBUS_RX_DECODE_PROF_MIN_MS = 3000; // rate-limit logs

constexpr uint32_t FUNKBUS_LUT_BIN_US = 25;
constexpr uint32_t FUNKBUS_LUT_MAX_US = 5000;

// ---- 10) Optional: trace non-Funkbus bursts --------------------------------
#define FUNKBUS_FOREIGN_TRACE 1
constexpr uint32_t FUNKBUS_FOREIGN_TRACE_RATE_MS = 5000;

// ---- Symbols used by decoder & LUT -----------------------------------------
enum FbSym : uint8_t {
  FB_SYM_INV = 0,
  FB_SYM_ONE = 1, // short
  FB_SYM_ZERO = 2, // long
  FB_SYM_PREH = 3,
  FB_SYM_PREL = 4
};

// Duration→symbol LUT
static FbSym s_fb_durlut[(FUNKBUS_LUT_MAX_US / FUNKBUS_LUT_BIN_US) + 1];

static inline FB_IRAM uint16_t fb_lut_idx(uint32_t us) {
  const uint32_t cap = FUNKBUS_LUT_MAX_US;
  return (us >= cap) ? (uint16_t)(cap / FUNKBUS_LUT_BIN_US)
                     : (uint16_t)(us / FUNKBUS_LUT_BIN_US);
}

// Build classifier from thresholds
static void fb_init_duration_lut() {
  for (uint32_t us = 0; us <= FUNKBUS_LUT_MAX_US; us += FUNKBUS_LUT_BIN_US) {
    FbSym sym = FB_SYM_INV;
    if (us >= FUNKBUS_LONG_MIN_US)
      sym = FB_SYM_ZERO;
    else if (us <= FUNKBUS_SHORT_MAX_US)
      sym = FB_SYM_ONE;
    if (us >= FUNKBUS_PRE_H_MIN_US && us <= FUNKBUS_PRE_H_MAX_US) sym = FB_SYM_PREH;
    if (us >= FUNKBUS_PRE_L_MIN_US && us <= FUNKBUS_PRE_L_MAX_US) sym = FB_SYM_PREL;
    s_fb_durlut[fb_lut_idx(us)] = sym;
  }
}

static inline FB_IRAM FbSym fb_classify_us(uint32_t us) {
  return s_fb_durlut[fb_lut_idx(us)];
}

// ---- ISR-safe logging helpers ----------------------------------------------
#if FB_ISR_LOG_ENABLE
#  if __has_include(<rom/ets_sys.h>)
#    include <rom/ets_sys.h>
#  elif __has_include(<esp32/rom/ets_sys.h>)
#    include <esp32/rom/ets_sys.h>
#  else
extern "C" int ets_printf(const char*, ...);
#  endif

#  ifndef FB_ISR_LOG_BUDGET
#    define FB_ISR_LOG_BUDGET 4
#  endif
static volatile int s_fb_isr_log_left = FB_ISR_LOG_BUDGET;
#  define FB_ISR_LOGF(fmt, ...)                            \
    do {                                                   \
      if (FB_IN_ISR() && s_fb_isr_log_left > 0) {          \
        --s_fb_isr_log_left;                               \
        ets_printf("[fb ISR] " fmt "\r\n", ##__VA_ARGS__); \
      }                                                    \
    } while (0)
#else
#  define FB_ISR_LOGF(...) \
    do {                   \
    } while (0)
#endif

// Drop normal logs inside ISRs
#define FB_LOGF_IF_NOT_ISR(call) \
  do {                           \
    if (!FB_IN_ISR()) {          \
      call;                      \
    }                            \
  } while (0)

// ---- Derived constants for grouping ----------------------------------------
static constexpr uint32_t FB_GROUP_GAP_MS = kRxGroupGapMs;
static constexpr uint32_t FB_GROUP_MAX_MS = kRxGroupMaxMs;
static constexpr uint8_t FB_GROUP_MAX_FRAMES = kRxGroupMaxFrames;
static constexpr uint32_t FB_FIRST_FRAME_HOLD_MS = kRxFirstFrameHoldMs;

// ---- ESP32 RMT globals ------------------------------------------------------
static RingbufHandle_t s_rb = nullptr;
static rmt_channel_t s_rmt_chan = (rmt_channel_t)FUNKBUS_RMT_RX_CHANNEL;
static volatile bool s_rmt_running = false;
static volatile int s_cs_level = 0;

// ---- RMT helpers ------------------------------------------------------------
// Start the RMT receiver if not already running.
static void startRmtIfNeeded() {
  if (!s_rb) return;
  if (!s_rmt_running) {
    rmt_rx_start(s_rmt_chan, true);
    s_rmt_running = true;
  }
}

// Stop the RMT receiver if running.
static void stopRmtIfRunning() {
  if (s_rmt_running) {
    rmt_rx_stop(s_rmt_chan);
    s_rmt_running = false;
  }
}

// ---- ISR + Worker task (debounced GDO2 gating) ------------------------------
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

static volatile uint32_t g_isr_notify_count = 0;
static volatile uint32_t g_worker_wake_count = 0;
static size_t g_max_ring_bytes_seen = 0;

static volatile bool s_edge_flag = false;
static TaskHandle_t s_rxTask = nullptr;

// ISR for GDO2 edges; coalesces via task notification.
static void IRAM_ATTR isr_carrier_rise() {
  s_edge_flag = true;
  g_isr_notify_count++;
  BaseType_t hpw = pdFALSE;
  TaskHandle_t t = s_rxTask;
  if (t) {
    xTaskNotifyFromISR(t, 1u, eSetBits, &hpw);
    if (hpw == pdTRUE) portYIELD_FROM_ISR();
  }
}

// Worker task—debounced RMT on/off based on GDO2.
static void rxWorkerTask(void* /*arg*/) {
  static uint32_t t_last_high = 0, t_last_low = 0, t_started = 0;

  for (;;) {
    const uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
    if (!notified) continue;
    g_worker_wake_count += notified;

    if (!s_edge_flag) continue;
    s_edge_flag = false;

    const uint32_t now = millis();
    int raw = gpio_get_level((gpio_num_t)FUNKBUS_CC1101_GDO2_MCU);
    const bool cs_high = FUNKBUS_CS_ACTIVE_HIGH ? (raw != 0) : (raw == 0);

    // Debounce start/stop windows for RMT.
    if (cs_high) {
      t_last_high = now;
      if (!s_rmt_running) {
        const bool high_stable = (t_last_low == 0) || (now - t_last_low >= (uint32_t)kCsMinHighMs);
        if (high_stable) {
          startRmtIfNeeded();
          if (s_rmt_running) t_started = now;
        }
      }
    } else {
      t_last_low = now;
      if (s_rmt_running) {
        const bool min_on_met = (t_started != 0) && (now - t_started >= (uint32_t)kCsMinOnMs);
        const bool low_stable = (t_last_high == 0) || (now - t_last_high >= (uint32_t)kCsMinLowMs);
        if (min_on_met && low_stable) {
          stopRmtIfRunning();
          t_started = 0;
        }
      }
    }
  }
}

#if FUNKBUS_FOREIGN_TRACE && FUNK_LOG_VERBOSE
static void fb_trace_unknown(const uint32_t* pulses, const uint8_t* /*levels*/, size_t pc) {
  static uint32_t s_last = 0;
  const uint32_t now = millis();
  if (now - s_last < FUNKBUS_FOREIGN_TRACE_RATE_MS) return;
  s_last = now;

  const size_t show = (pc < 16) ? pc : 16;
  char buf[192];
  char* p = buf;
  size_t left = sizeof(buf);
  int w = snprintf(p, left, "[funkbus_rx] unknown burst pc=%u: ", (unsigned)pc);
  if (w < 0) return;
  p += w;
  left -= (size_t)w;
  for (size_t i = 0; i < show && left > 8; ++i) {
    w = snprintf(p, left, "%u%s", (unsigned)pulses[i], (i + 1 < show ? "," : ""));
    if (w < 0) break;
    p += w;
    left -= (size_t)w;
  }
  FB_VLOG("%s" CR, buf);
}
#endif

// ---- Bit helpers / pulse extraction ----------------------------------------

// Set bit at MSB-first index in a 48-bit accumulator.
static inline void pack_bit48(uint64_t& acc, int bit, int bit_index) {
  if (bit) acc |= (1ULL << (FUNKBUS_FRAME_BITS - 1 - bit_index));
}

// Parse RMT items into pulse durations (µs) and alternating levels.
static FB_IRAM size_t fb_build_pulses_levels(const rmt_item32_t* it, size_t n, uint32_t tick_us,
                                             uint32_t* pulses, uint8_t* levels, size_t cap) {
  size_t pc = 0;

#if FUNKBUS_BUILD_EARLYCUT
  const size_t needed_after_i = (FUNKBUS_FRAME_BITS * 2) + 4;
  size_t found_i = SIZE_MAX; // index of PRE_L once PRE_H→PRE_L seen
#endif

  for (size_t i = 0; i < n && pc + 2 <= cap; ++i) {
    // half 0
    if (it[i].duration0) {
      const uint8_t l0 = it[i].level0 ? 1 : 0;
      if (pc == 0)
        levels[0] = l0;
      else
        levels[pc] = l0;
      pulses[pc++] = it[i].duration0 * tick_us;

#if FUNKBUS_BUILD_EARLYCUT
      if (pc >= 2 && found_i == SIZE_MAX) {
        const size_t k0 = pc - 2, k1 = pc - 1;
        if (levels[k0] == 1 &&
            pulses[k0] >= FUNKBUS_PRE_H_MIN_US && pulses[k0] <= FUNKBUS_PRE_H_MAX_US &&
            levels[k1] == 0 &&
            pulses[k1] >= FUNKBUS_PRE_L_MIN_US && pulses[k1] <= FUNKBUS_PRE_L_MAX_US) {
          found_i = k1;
        }
      }
      if (found_i != SIZE_MAX && pc >= (found_i + needed_after_i)) {
        return pc;
      }
#endif
    }

    // half 1
    if (it[i].duration1) {
      const uint8_t l1 = it[i].level1 ? 1 : 0;
      if (pc == 0)
        levels[0] = l1;
      else
        levels[pc] = l1;
      pulses[pc++] = it[i].duration1 * tick_us;

#if FUNKBUS_BUILD_EARLYCUT
      if (pc >= 2 && found_i == SIZE_MAX) {
        const size_t k0 = pc - 2, k1 = pc - 1;
        if (levels[k0] == 1 &&
            pulses[k0] >= FUNKBUS_PRE_H_MIN_US && pulses[k0] <= FUNKBUS_PRE_H_MAX_US &&
            levels[k1] == 0 &&
            pulses[k1] >= FUNKBUS_PRE_L_MIN_US && pulses[k1] <= FUNKBUS_PRE_L_MAX_US) {
          found_i = k1;
        }
      }
      if (found_i != SIZE_MAX && pc >= (found_i + needed_after_i)) {
        return pc;
      }
#endif
    }
  }
  return pc;
}

// Find the LOW right after preamble HIGH (within thresholds).
static FB_IRAM size_t fb_find_preamble_low(const uint32_t* pulses, const uint8_t* levels, size_t pc) {
  for (size_t k = 0; k + 1 < pc; ++k) {
    const FbSym s0 = fb_classify_us(pulses[k]);
    const FbSym s1 = fb_classify_us(pulses[k + 1]);
    if (levels[k] == 1 && s0 == FB_SYM_PREH &&
        levels[k + 1] == 0 && s1 == FB_SYM_PREL) {
      return k + 1;
    }
  }
  return SIZE_MAX;
}

// Walk pulses and assemble 48 bits (MSB-first) using fixed thresholds.
static FB_IRAM bool fb_walk_bits(const uint32_t* pulses, size_t pc, size_t start_idx, uint64_t& acc) {
  uint64_t bits_acc = 0;
  int bits = 0;
  size_t i = start_idx;

  while (bits < FUNKBUS_FRAME_BITS && i < pc) {
    FbSym s0 = fb_classify_us(pulses[i]);
    if (s0 == FB_SYM_PREH || s0 == FB_SYM_PREL) s0 = FB_SYM_ZERO; // treat PRE* as long

    if (s0 == FB_SYM_ZERO) { // long => ZERO (single)
      pack_bit48(bits_acc, 0, bits++);
      i += 1;
      continue;
    }

    if (s0 == FB_SYM_ONE) { // ONE => short + short
      if (i + 1 >= pc) break;
      FbSym s1 = fb_classify_us(pulses[i + 1]);
      if (s1 == FB_SYM_ONE) {
        pack_bit48(bits_acc, 1, bits++);
        i += 2;
        continue;
      } else {
        break;
      }
    }
    break;
  }

  if (bits != FUNKBUS_FRAME_BITS) return false;
  acc = bits_acc;
  return true;
}

// ---- Frame/field mapping and reverse decode --------------------------------
struct FunkbusFrame48 {
  uint64_t left48 = 0;
  uint64_t payload40 = 0;
  uint8_t scom3 = 0;
  uint8_t parity1 = 0;
  uint8_t csum4 = 0;
  uint16_t t_half_us = 0;
  int bits_total = 0;
};

static void fb_fill_out_and_log(uint64_t acc, FunkbusFrame48& out) {
  out.left48 = acc;
  out.payload40 = (acc >> FUNKBUS_TRAILER_BITS) & ((1ULL << FUNKBUS_PAYLOAD_BITS) - 1);
  out.scom3 = (acc >> 5) & 0x7;
  out.parity1 = (acc >> 4) & 0x1;
  out.csum4 = acc & 0xF;
  out.bits_total = FUNKBUS_FRAME_BITS;
}

struct FunkbusDecoded {
  uint8_t scom = 0;
  bool parity_ok = false;
  bool checksum_ok = false;
  uint8_t rc_type = 0;
  uint8_t subtype = 0;
  uint32_t serial20 = 0;
  uint8_t unknown_29_30 = 0;
  bool battery_ok = false;
  uint8_t unknown_32_33 = 0;
  uint8_t button3 = 0;
  uint8_t channel2 = 0;
  uint8_t unknown_39 = 0;
  bool action_on = false;

  uint8_t button = 0;
  char channel_chr = '?';
  String serial_hex;
};

static inline uint8_t get_bit48(uint64_t v, int i) {
  return (uint8_t)((v >> (FUNKBUS_FRAME_BITS - 1 - i)) & 1ULL);
}

static bool parity_even_first43(uint64_t left48) {
  uint32_t ones = 0;
  for (int i = 0; i < FUNKBUS_PARITY_RANGE_BITS; ++i) ones += get_bit48(left48, i);
  const uint8_t expected = (ones & 1) ? 1 : 0;
  const uint8_t bit = (uint8_t)((left48 >> 4) & 0x1);
  return bit == expected;
}

static uint8_t checksum4_first43(uint64_t left48) {
  uint8_t xorv = 0, cur = 0;
  int cnt = 0;
  for (int i = 0; i < FUNKBUS_PARITY_RANGE_BITS; ++i) {
    cur |= (uint8_t)(get_bit48(left48, i) ? (1u << (7 - cnt)) : 0u);
    if (++cnt == 8) {
      xorv ^= cur;
      cur = 0;
      cnt = 0;
    }
  }
  if (cnt) xorv ^= cur;

  uint8_t nib = ((xorv >> 4) & 0x0F) ^ (xorv & 0x0F);
  uint8_t res = 0;
  if (nib & 0x8) res ^= 0x8C;
  if (nib & 0x4) res ^= 0x32;
  if (nib & 0x2) res ^= 0xC8;
  if (nib & 0x1) res ^= 0x23;
  return (uint8_t)(res & 0x0F);
}

static uint8_t decode_button_from_3bits(uint8_t b3) {
  switch (b3 & 0x7) {
    case 0b000:
      return 1;
    case 0b100:
      return 2;
    case 0b010:
      return 3;
    case 0b110:
      return 4;
    case 0b001:
      return 5;
    case 0b101:
      return 6;
    case 0b011:
      return 7;
    case 0b111:
      return 8;
  }
  return 0;
}

static char decode_channel2_to_char(uint8_t ch2) {
  switch (ch2 & 0x3) {
    case 0b00:
      return 'A';
    case 0b10:
      return 'B';
    case 0b01:
      return 'C';
    case 0b11:
      return 'L';
  }
  return '?';
}

static String serial20_to_hex5(uint32_t s20) {
  char buf[6];
  snprintf(buf, sizeof(buf), "%05X", (unsigned)(s20 & 0xFFFFF));
  return String(buf);
}

static uint8_t decode_scom_from_bits(uint8_t scom3) {
  switch (scom3 & 0x7) {
    case 0b000:
      return 0;
    case 0b100:
      return 1;
    case 0b010:
      return 2;
    case 0b110:
      return 3;
    case 0b001:
      return 4;
    case 0b101:
      return 5;
    case 0b011:
      return 6;
    case 0b111:
      return 7;
  }
  return 0xFF;
}

static void fb_reverse_decode(const FunkbusFrame48& in, FunkbusDecoded& d) {
  const uint64_t left48 = in.left48, p40 = in.payload40;
  uint8_t scom3 = (uint8_t)((left48 >> 5) & 0x7);
  d.scom = decode_scom_from_bits(scom3);
  d.parity_ok = parity_even_first43(left48);
  d.checksum_ok = (in.csum4 == checksum4_first43(left48));
  d.rc_type = (uint8_t)((p40 >> 36) & 0xF);
  d.subtype = (uint8_t)((p40 >> 32) & 0xF);
  d.serial20 = (uint32_t)((p40 >> 12) & 0xFFFFF);
  d.unknown_29_30 = (uint8_t)((p40 >> 10) & 0x3);
  d.battery_ok = ((p40 >> 9) & 0x1) != 0;
  d.unknown_32_33 = (uint8_t)((p40 >> 7) & 0x3);
  d.button3 = (uint8_t)((p40 >> 4) & 0x7);
  d.channel2 = (uint8_t)((p40 >> 2) & 0x3);
  d.unknown_39 = (uint8_t)((p40 >> 1) & 0x1);
  d.action_on = (p40 & 0x1) != 0;
  d.button = decode_button_from_3bits(d.button3);
  d.channel_chr = decode_channel2_to_char(d.channel2);
  d.serial_hex = serial20_to_hex5(d.serial20);
}

// Entry point for decoding one RMT burst into a 48-bit Funkbus frame.
static bool decode_funkbus_48(const rmt_item32_t* it, size_t n, uint32_t tick_us, FunkbusFrame48& out) {
#if FUNKBUS_RX_DECODE_PROF
  uint32_t t0 = micros();
#endif

  uint32_t pulses[kRxPulseCap];
  uint8_t levels[kRxPulseCap];

  const size_t pc = fb_build_pulses_levels(it, n, tick_us, pulses, levels, kRxPulseCap);
  if (pc < FUNKBUS_MIN_PULSES) {
#if FUNKBUS_FOREIGN_TRACE && FUNK_LOG_VERBOSE
    fb_trace_unknown(pulses, levels, pc);
#endif
    return false;
  }

#if FUNKBUS_RX_DECODE_PROF
  uint32_t t1 = micros();
#endif

  const size_t idx_low = fb_find_preamble_low(pulses, levels, pc);
  size_t i = (idx_low != SIZE_MAX) ? idx_low : ((pc > 0) ? FUNKBUS_STARTIDX_FALLBACK : 0);
  if (i >= pc) {
#if FUNKBUS_FOREIGN_TRACE && FUNK_LOG_VERBOSE
    fb_trace_unknown(pulses, levels, pc);
#endif
    return false;
  }

#if FUNKBUS_RX_DECODE_PROF
  uint32_t t2 = micros();
#endif

  // Clamp walk window (48 bits ⇒ need ≤ ~100 durations).
  size_t pc_needed = i + (FUNKBUS_FRAME_BITS * 2) + 4;
  if (pc_needed > pc) pc_needed = pc;

  uint64_t acc = 0;
  if (!fb_walk_bits(pulses, pc_needed, i, acc)) {
#if FUNKBUS_FOREIGN_TRACE && FUNK_LOG_VERBOSE
    fb_trace_unknown(pulses, levels, pc);
#endif
    return false;
  }

#if FUNKBUS_RX_DECODE_PROF && FUNK_LOG_VERBOSE
  uint32_t t3 = micros();
  static uint32_t s_last_prof_ms = 0;
  const uint32_t total_us = t3 - t0;
  const uint32_t now_ms = millis();
  if (total_us > FUNKBUS_RX_DECODE_PROF_LOG_US &&
      (now_ms - s_last_prof_ms) > FUNKBUS_RX_DECODE_PROF_MIN_MS) {
    s_last_prof_ms = now_ms;
    FB_VLOG("[RX.PROF] build=%u us, preamble=%u us, walk=%u us, total=%u us" CR,
            (unsigned)(t1 - t0), (unsigned)(t2 - t1),
            (unsigned)(t3 - t2), (unsigned)total_us);
  }
#endif

  fb_fill_out_and_log(acc, out);
  return true;
}

// ---- Grouping & MQTT publish -----------------------------------------------
struct FbPressGroup {
  uint32_t serial20 = 0;
  char channel = '?';
  uint8_t button = 0;
  bool action_on = false;
  uint32_t first_ms = 0;
  uint32_t last_ms = 0;
  uint32_t avg_gap_ms = 0;
  uint8_t scoms[FB_GROUP_MAX_FRAMES];
  uint8_t scom_count = 0;
  uint8_t max_scom = 0;
  uint8_t last_scom = 0xFF;
  bool battery_ok = true;
  bool seen2 = false, seen3 = false, seen4 = false, seen5 = false, seen6 = false, seen7 = false;
  char serial_hex[6] = {0};

  void reset() {
    serial20 = 0;
    channel = '?';
    button = 0;
    action_on = false;
    first_ms = last_ms = 0;
    avg_gap_ms = 0;
    scom_count = 0;
    max_scom = 0;
    last_scom = 0xFF;
    battery_ok = true;
    seen2 = seen3 = seen4 = seen5 = seen6 = seen7 = false;
    serial_hex[0] = '\0';
  }
  bool empty() const { return scom_count == 0; }
};

static FbPressGroup g_press;
static bool g_active = false;

#if FUNKBUS_LED_RX_ENABLE
static uint32_t s_led_off_at = 0;
static inline void rx_led_write(bool on) {
  digitalWrite(FUNKBUS_LED_RX_GPIO,
               (FUNKBUS_LED_ACTIVE_HIGH ? (on ? HIGH : LOW)
                                        : (on ? LOW : HIGH)));
}
static inline void rx_led_pulse(uint32_t now_ms) {
  rx_led_write(true);
  s_led_off_at = now_ms + (uint32_t)FUNKBUS_LED_RX_MS;
}
static inline void rx_led_maintain(uint32_t now_ms) {
  if (s_led_off_at && (int32_t)(now_ms - s_led_off_at) >= 0) {
    rx_led_write(false);
    s_led_off_at = 0;
  }
}
#endif // FUNKBUS_LED_RX_ENABLE

static inline bool fb_same_signature(const FbPressGroup& g, uint32_t serial20, char channel,
                                     uint8_t button, bool action_on) {
  return g.serial20 == serial20 && g.channel == channel && g.button == button && g.action_on == action_on;
}

static inline void fb_group_start(FbPressGroup& g, uint32_t now_ms, uint32_t serial20, const char* serial_hex5,
                                  char channel, uint8_t button, bool action_on, uint8_t scom, bool battery_ok) {
  g.reset();
  g.serial20 = serial20;
  g.channel = channel;
  g.button = button;
  g.action_on = action_on;
  strncpy(g.serial_hex, serial_hex5, sizeof(g.serial_hex) - 1);
  g.first_ms = g.last_ms = now_ms;
  g.avg_gap_ms = 0;
  g.scoms[0] = scom;
  g.scom_count = 1;
  g.max_scom = scom;
  g.last_scom = scom;
  g.battery_ok = battery_ok;
  g.seen2 |= (scom == 2);
  g.seen3 |= (scom == 3);
  g.seen4 |= (scom == 4);
  g.seen5 |= (scom == 5);
  g.seen6 |= (scom == 6);
  g.seen7 |= (scom == 7);
}

static inline void fb_group_append(FbPressGroup& g, uint32_t now_ms, uint8_t scom, bool battery_ok) {
  const uint32_t gap = now_ms - g.last_ms;
  g.avg_gap_ms = (g.avg_gap_ms == 0) ? gap : ((g.avg_gap_ms + gap) >> 1);
  g.last_ms = now_ms;
  if (g.scom_count < FB_GROUP_MAX_FRAMES) g.scoms[g.scom_count++] = scom;
  if (scom > g.max_scom) g.max_scom = scom;
  g.last_scom = scom;
  g.battery_ok = battery_ok;
  g.seen2 |= (scom == 2);
  g.seen3 |= (scom == 3);
  g.seen4 |= (scom == 4);
  g.seen5 |= (scom == 5);
  g.seen6 |= (scom == 6);
  g.seen7 |= (scom == 7);
}

static String fb_scom_list(const FbPressGroup& g) {
  char buf[FB_GROUP_MAX_FRAMES * 2 + 8] = {0};
  char* p = buf;
  for (uint8_t i = 0; i < g.scom_count; ++i) {
    if (i) *p++ = ',';
    p += snprintf(p, (size_t)(buf + sizeof(buf) - p), "%u", (unsigned)g.scoms[i]);
  }
  return String(buf);
}

// Publish grouped press as MQTT JSON on /RFtoMQTT/<serial>.
static void fb_publish_group_result(const FbPressGroup& g, char duration_code) {
  char serial_lc[6] = {0};
  for (int i = 0; i < 5 && g.serial_hex[i]; ++i) {
    char c = g.serial_hex[i];
    if (c >= 'A' && c <= 'F') c = (char)(c - 'A' + 'a');
    serial_lc[i] = c;
  }

  char topic[64];
  snprintf(topic, sizeof(topic), "/RFtoMQTT/%s", serial_lc);

  char chbuf[2] = {g.channel, '\0'};
  char dur[2] = {duration_code, '\0'};

  StaticJsonDocument<128> d;
  d["serial"] = serial_lc;
  d["channel"] = chbuf;
  d["button"] = g.button;
  d["action"] = g.action_on ? "on" : "off";
  d["duration"] = dur;

  String out;
  out.reserve(128);
  serializeJson(d, out);
  pub(topic, out.c_str());
}

static inline bool fb_should_force_split_on_restart(const FbPressGroup& g, uint8_t incoming_scom) {
  if (g.action_on) {
    if (incoming_scom == FUNKBUS_SCOM_RESTART_ON && (g.seen2 || g.seen6)) return true;
  } else {
    if (incoming_scom == FUNKBUS_SCOM_RESTART_OFF && (g.seen3 || g.seen7 || g.seen6)) return true;
  }
  return false;
}

static inline const char* fb_press_type(uint8_t max_scom) {
  return (max_scom <= FUNKBUS_SCOM_SHORT_MAX) ? "SHORT" : "LONG";
}

static void fb_group_flush_now() {
  if (!g_active || g_press.empty()) return;
  const uint32_t span_ms = g_press.last_ms - g_press.first_ms;
  String scom_seq = fb_scom_list(g_press);
  FB_LOG_N("[funkbus_rx] serial=%s batt=%s ch=%c btn=%u action=%s scom_max=%u type=%s frames=%u duration=%ums scoms=%s\n",
           g_press.serial_hex, g_press.battery_ok ? "LOW" : "OK", g_press.channel, (unsigned)g_press.button,
           g_press.action_on ? "ON" : "OFF", (unsigned)g_press.max_scom, fb_press_type(g_press.max_scom),
           (unsigned)g_press.scom_count, (unsigned)span_ms, scom_seq.c_str());
  const char duration_code = (g_press.max_scom <= FUNKBUS_SCOM_SHORT_MAX) ? 's' : 'l';
  fb_publish_group_result(g_press, duration_code);
  g_press.reset();
  g_active = false;
}

static void fb_group_flush_if_due(uint32_t now_ms) {
  if (!g_active || g_press.empty()) return;
  const uint32_t gap = now_ms - g_press.last_ms;
  const uint32_t span_ms = g_press.last_ms - g_press.first_ms;
  const uint32_t adaptive_gate =
      (g_press.avg_gap_ms ? (g_press.avg_gap_ms * kRxAdaptiveNum) / kRxAdaptiveDen : FB_GROUP_GAP_MS);
  uint32_t gate = max((uint32_t)FB_GROUP_GAP_MS, adaptive_gate);
  if (g_press.scom_count <= 1) gate = max(gate, FB_FIRST_FRAME_HOLD_MS);
  const bool gap_due = gap >= gate;
  const bool span_due = span_ms >= FB_GROUP_MAX_MS;
  if (gap_due || span_due) {
    String scom_seq = fb_scom_list(g_press);
    FB_LOG_N("[funkbus_rx] serial=%s batt=%s ch=%c btn=%u action=%s scom_max=%u type=%s frames=%u duration=%ums scoms=%s\n",
             g_press.serial_hex, g_press.battery_ok ? "LOW" : "OK", g_press.channel, (unsigned)g_press.button,
             g_press.action_on ? "ON" : "OFF", (unsigned)g_press.max_scom, fb_press_type(g_press.max_scom),
             (unsigned)g_press.scom_count, (unsigned)(span_ms), scom_seq.c_str());
    const char duration_code = (g_press.max_scom <= FUNKBUS_SCOM_SHORT_MAX) ? 's' : 'l';
    fb_publish_group_result(g_press, duration_code);
    g_press.reset();
    g_active = false;
  }
}

// ---- Public API implementations --------------------------------------------

// Temporarily disable RX during TX to avoid artifacts and buffer churn.
void FunkbusRx::pauseForTx() {
#if FUNKBUS_RX_GATE_BY_CS
  detachInterrupt(digitalPinToInterrupt(FUNKBUS_CC1101_GDO2_MCU));
#endif
  stopRmtIfRunning();
}

// Re-arm RX after TX; restore listen frequency and gating/LED state.
void FunkbusRx::resumeAfterTx() {
  using namespace FunkbusTB;

  FunkbusTB::setMHz(FunkbusTB::GetPersistedMhz());

  // Fast hop TX→RX; full apply only if fast fails.
  if (!FunkbusTB::switchTxToRxFast(20)) {
    (void)assureRxReady(true);
  }

#if FUNKBUS_RX_GATE_BY_CS
  attachInterrupt(digitalPinToInterrupt(FUNKBUS_CC1101_GDO2_MCU), isr_carrier_rise, CHANGE);
  s_cs_level = gpio_get_level((gpio_num_t)FUNKBUS_CC1101_GDO2_MCU);
  if (s_cs_level)
    startRmtIfNeeded();
  else
    stopRmtIfRunning();
#else
  startRmtIfNeeded();
#endif
}

// One-time init — listen freq, RMT, worker task, LED, LUT.
void FunkbusRx::begin() {
  using namespace FunkbusTB;
  FunkbusTB::EnsureListenFreqInitialized();
  (void)assureRxReady(20);

  if (s_rxTask == nullptr) {
    BaseType_t ok = xTaskCreatePinnedToCore(
        rxWorkerTask, "fb_rx", kRxWorkerStack, nullptr,
        kRxWorkerPrio, &s_rxTask, kRxWorkerCore);
    if (ok != pdPASS) s_rxTask = nullptr;
  }

  // RMT RX config
  pinMode(FUNKBUS_CC1101_GDO0_MCU, INPUT);
  rmt_config_t c = {};
  c.channel = s_rmt_chan;
  c.gpio_num = (gpio_num_t)FUNKBUS_CC1101_GDO0_MCU;
  c.clk_div = FUNKBUS_RMT_CLKDIV;
  c.mem_block_num = FUNKBUS_RMT_MEMBLOCKS;
  c.rmt_mode = RMT_MODE_RX;
  c.rx_config.filter_en = true;
  c.rx_config.filter_ticks_thresh = kRmtGlitchUs;
  c.rx_config.idle_threshold = kRmtIdleThUs;

  rmt_config(&c);
  rmt_driver_install(c.channel, kRmtRingbufBytes, 0);
  rmt_get_ringbuf_handle(c.channel, &s_rb);

#if FUNKBUS_RX_GATE_BY_CS
  pinMode(FUNKBUS_CC1101_GDO2_MCU, INPUT);
  attachInterrupt(digitalPinToInterrupt(FUNKBUS_CC1101_GDO2_MCU), isr_carrier_rise, CHANGE);
  s_cs_level = gpio_get_level((gpio_num_t)FUNKBUS_CC1101_GDO2_MCU);
  if (s_cs_level)
    startRmtIfNeeded();
  else
    stopRmtIfRunning();
#else
  startRmtIfNeeded();
#endif

#if FUNKBUS_LED_RX_ENABLE
  pinMode(FUNKBUS_LED_RX_GPIO, OUTPUT);
  // LED starts OFF
  digitalWrite(FUNKBUS_LED_RX_GPIO, (FUNKBUS_LED_ACTIVE_HIGH ? LOW : HIGH));
#endif

  fb_init_duration_lut();
}

// Main RX polling — drain RMT ringbuffer, decode, group, publish.
void FunkbusRx::loop() {
  if (!s_rb) return;

#if FUNKBUS_RX_DEBUG
  static uint32_t last_dbg = 0;
  const uint32_t now_dbg = millis();
  if (now_dbg - last_dbg >= FUNKBUS_RX_DEBUG_PERIOD_MS) {
    last_dbg = now_dbg;
    uint32_t isr = g_isr_notify_count;
    uint32_t wake = g_worker_wake_count;
    uint32_t coalesced = (isr >= wake) ? (isr - wake) : 0;
    int gdo2 = gpio_get_level((gpio_num_t)FUNKBUS_CC1101_GDO2_MCU);
    FB_LOGF_IF_NOT_ISR(FB_VLOG("[fb rx dbg] isr=%lu wake=%lu coalesced=%lu rmt_running=%d gdo2=%d max_ring=%uB" CR,
                               (unsigned)isr, (unsigned)wake, (unsigned)coalesced,
                               (int)s_rmt_running, gdo2, (unsigned)g_max_ring_bytes_seen));
  }
#endif

#if FUNKBUS_LED_RX_ENABLE
  rx_led_maintain(millis());
#endif

  size_t bytes = 0;
  rmt_item32_t* items = (rmt_item32_t*)xRingbufferReceive(
      s_rb, &bytes, kRmtRecvTimeoutMs / portTICK_PERIOD_MS);

  if (!items) {
    fb_group_flush_if_due(millis());
    return;
  }

#if FUNKBUS_LED_RX_ENABLE
  rx_led_pulse(millis());
#endif

  if (bytes > g_max_ring_bytes_seen) g_max_ring_bytes_seen = bytes;
  const size_t n = bytes / sizeof(rmt_item32_t);
  FunkbusFrame48 f{};
  if (decode_funkbus_48(items, n, kRmtTickUs, f)) {
    FunkbusDecoded d{};
    fb_reverse_decode(f, d);
    const uint32_t now_ms = millis();

    if (!g_active) {
      fb_group_start(g_press, now_ms, d.serial20, d.serial_hex.c_str(),
                     d.channel_chr, d.button, d.action_on, d.scom, d.battery_ok);
      g_active = true;
    } else if (fb_same_signature(g_press, d.serial20, d.channel_chr, d.button, d.action_on)) {
      if (fb_should_force_split_on_restart(g_press, d.scom)) {
        fb_group_flush_now();
        fb_group_start(g_press, now_ms, d.serial20, d.serial_hex.c_str(),
                       d.channel_chr, d.button, d.action_on, d.scom, d.battery_ok);
      } else {
        fb_group_append(g_press, now_ms, d.scom, d.battery_ok);
      }
    } else {
      fb_group_flush_now();
      fb_group_start(g_press, now_ms, d.serial20, d.serial_hex.c_str(),
                     d.channel_chr, d.button, d.action_on, d.scom, d.battery_ok);
      g_active = true;
    }
  }

  vRingbufferReturnItem(s_rb, (void*)items);
  fb_group_flush_if_due(millis());
}
