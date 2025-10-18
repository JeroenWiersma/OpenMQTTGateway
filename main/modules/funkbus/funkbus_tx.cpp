#include "funkbus_tx.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <string>
#include <vector>

#include "User_config.h"
#include "config_Funkbus.h"
#include "driver/rmt.h"
#include "esp_timer.h"
#include "funkbus_cc1101_toolbox.h"
#include "funkbus_log.h"
#include "funkbus_rx.h"

#ifndef ARDUINO_ARCH_ESP32
#  error "Funkbus TX requires ESP32 (RMT). Build on ESP32 or disable Funkbus TX."
#endif

// ============================================================================
// TU-LOCAL CONSTANTS (constexpr) — protocol geometry, sizes, indices
// ============================================================================
namespace {

// --- Bit geometry -----------------------------------------------------------
constexpr int FRAME_BITS = 48; // Total bits in a Funkbus frame
constexpr int FIRST40_BITS = 40; // Bits in the “first40” payload section
constexpr int PARITY_RANGE_BITS = 43; // Bits covered by parity (first40 + SCOM)
constexpr int SCOM_START_BIT = 40; // Bit index where 3-bit SCOM begins
constexpr int SCOM_MAX = 7; // Highest valid SCOM value (0..7)

// --- String buffer lengths (NUL-terminated) --------------------------------
constexpr int FRAME_STRLEN = FRAME_BITS + 1; // 48-bit string + NUL
constexpr int FIRST40_STRLEN = FIRST40_BITS + 1; // 40-bit string + NUL
static_assert(SCOM_START_BIT + 3 <= FRAME_BITS, "SCOM overflow");

// --- Field sizes & validation helpers --------------------------------------
constexpr int SERIAL_HEX_LEN = 5; // Serial printed as 5 hex chars
constexpr int SERIAL_BITS = 20; // Serial is 20 bits
constexpr int BUTTON_MIN = 1; // Smallest valid button id
constexpr int BUTTON_MAX = 8; // Largest valid button id

} // namespace

// ============================================================================
// FIXED BIT FIELDS (string literals) — observed constant fields in first40
// (migrated to constexpr char[]; not intended for build-time overrides)
// ============================================================================
inline constexpr char FUNKBUS_RC_TYPE[] = "0010"; // Device type (observed)
inline constexpr char FUNKBUS_RC_SUBTYPE[] = "1100"; // Device subtype (observed)
inline constexpr char FUNKBUS_UNKNOWN_29_30[] = "00"; // Bits 29..30 (reserved/unknown)
inline constexpr char FUNKBUS_BATTERY_OK[] = "0"; // Bit 31: 0 = battery OK
inline constexpr char FUNKBUS_UNKNOWN_32_33[] = "00"; // Bits 32..33 (reserved/unknown)
inline constexpr char FUNKBUS_UNKNOWN_39[] = "0"; // Bit 39  (reserved/unknown)

// ============================================================================
// CHANNEL / ACTION MAPS — symbolic 2-bit channels + 1-bit action
// (migrated to constexpr char[])
// ============================================================================
inline constexpr char FUNKBUS_CH_A[] = "00"; // Channel A
inline constexpr char FUNKBUS_CH_B[] = "10"; // Channel B
inline constexpr char FUNKBUS_CH_C[] = "01"; // Channel C
inline constexpr char FUNKBUS_CH_LS[] = "11"; // Channel LS (group)
inline constexpr char FUNKBUS_ACT_ON[] = "1"; // Action ON
inline constexpr char FUNKBUS_ACT_OFF[] = "0"; // Action OFF

// ============================================================================
// TX/RADIO TIMING — preamble, bit cell, trailer, inter-frame gap
// (Public knobs remain macros to allow build-time overrides.)
// ============================================================================
#define FUNKBUS_HALF_BIT_US         500 // Half of a Manchester bit cell (µs)
#define FUNKBUS_PREAMBLE_LEADIN_US  3900 // High “leader” pulse (µs)
#define FUNKBUS_PREAMBLE_LEADOUT_US 100 // Low “leadout” before data (µs)
#define FUNKBUS_INTERFRAME_GAP_US   100000 // Gap between frames in a burst (µs)

// Trailer after final bit (kept as macro for overrides)
#define FUNKBUS_TX_TRAILER_US 800 // Low time at end of frame/burst (µs)

// ============================================================================
// TX SEQUENCING / HOUSEKEEPING — frame count, preroll, cooperative waits
// (these are internal runtime constants → constexpr)
// ============================================================================
inline constexpr int FUNKBUS_TX_FRAMES_PER_BURST = 4; // Frames per single command burst
inline constexpr uint16_t FUNKBUS_TX_PREROLL_US = 10; // Stabilization low before preamble (µs)

inline constexpr uint32_t FUNKBUS_WAIT_YIELD_US = 1000; // Yield if wait > this (µs)
inline constexpr uint32_t FUNKBUS_WAIT_COARSE_SLEEP_US = 500; // Coarse sleep chunk for busy waits (µs)

// ============================================================================
// JSON / DEBUG DEFAULTS — small result acks and carrier test
// (migrated to constexpr)
// ============================================================================
inline constexpr size_t FUNKBUS_JSON_RESERVE_TX = 512; // Reserve when serializing small TX JSON
inline constexpr uint32_t FUNKBUS_DEFAULT_CARRIER_MS = 100; // Default ms for Debug_TxCarrierMs()

// ============================================================================
// CC1101 REGISTER MAP GEOMETRY — counts/ranges for dumps (migrated to constexpr)
// ============================================================================
inline constexpr uint8_t CC1101_NUM_CONFIG_REGS = 0x2F; // Config regs 0x00..0x2E (47)
inline constexpr uint8_t CC1101_FIRST_STATUS_ADDR = 0x30; // First status register address
inline constexpr uint8_t CC1101_LAST_STATUS_ADDR = 0x3B; // Last  status register address
inline constexpr uint8_t CC1101_NUM_STATUS_REGS = (CC1101_LAST_STATUS_ADDR - CC1101_FIRST_STATUS_ADDR + 1); // Derived count
inline constexpr uint8_t CC1101_PATABLE_SIZE = 8; // PA table entries
inline constexpr uint8_t CC1101_HEXBUF_LEN = 6; // “0xNN” with NUL (snprintf helper)

// ============================================================================
// RMT (ESP32) CONVERSION HELPERS — µs ↔ ticks and hardware limits (migrated)
// ============================================================================
inline constexpr uint32_t RMT_TICKS_PER_SEC = 1000000UL; // Fallback ticks/sec when clock not queried
inline constexpr uint32_t RMT_ROUNDING_HALF = 500000UL; // For integer rounding in tick conversion
inline constexpr uint16_t RMT_DURATION_MAX_TICKS = 32767U; // Max per-half-item duration (15-bit field)

// ============================================================================
// API / TOPICS / PINS — compile-time defaults (overridable in config)
// (keep macros to preserve #ifndef override behavior)
// ============================================================================
#ifndef FUNKBUS_CMD_RESULT_TOPIC
#  define FUNKBUS_CMD_RESULT_TOPIC "/FunkbustoMQTT" // Topic for small command acks/results
#endif

#ifndef FUNKBUS_CC1101_GDO0_MCU
#  define FUNKBUS_CC1101_GDO0_MCU 12 // Default MCU GPIO wired to CC1101 GDO0 (OOK TX)
#endif

// Keep alias even if already defined elsewhere (no change intended)
#define FUNKBUS_TX_GPIO FUNKBUS_CC1101_GDO0_MCU // Alias used by GPIO-timed path

#ifndef FUNKBUS_DEFAULT_LISTEN_MHZ
#  define FUNKBUS_DEFAULT_LISTEN_MHZ 433.42f // Default RX listen frequency (MHz)
#endif

// Debug: initial segment dump toggle (kept as macro to match existing usage)
#define FUNKBUS_TX_DUMP_FIRST_SEGS 0 // Non-zero to dump first TX segments

// ============================================================================
// ESP32 RMT DEFAULTS (constexpr) — channel/divider used for TX
// ============================================================================
namespace { // TU-local only
inline constexpr rmt_channel_t kRmtTxChannel = RMT_CHANNEL_7; // Primary RMT TX channel
inline constexpr uint8_t kRmtClkDiv = 80; // APB/80 → 1 MHz RMT tick

class RmtTxSession {
public:
  RmtTxSession(rmt_channel_t ch, gpio_num_t gpio, uint8_t clk_div)
      : ch_(ch), gpio_(gpio), clk_div_(clk_div) {}

  RmtTxSession(const RmtTxSession&) = delete;
  RmtTxSession& operator=(const RmtTxSession&) = delete;
  RmtTxSession(RmtTxSession&&) noexcept = default;
  RmtTxSession& operator=(RmtTxSession&&) noexcept = default;

  bool begin() {
    if (installed_) return true;

    rmt_config_t cfg = {};
    cfg.rmt_mode = RMT_MODE_TX;
    cfg.channel = ch_;
    cfg.gpio_num = gpio_;
    cfg.clk_div = clk_div_;
    cfg.mem_block_num = 1;
    cfg.tx_config.loop_en = false;
    cfg.tx_config.carrier_en = false;
    cfg.tx_config.idle_output_en = true;
    cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    if (rmt_config(&cfg) != ESP_OK) return false;
    if (rmt_set_source_clk(ch_, RMT_BASECLK_APB) != ESP_OK) return false;
    if (rmt_driver_install(ch_, 0, 0) != ESP_OK) return false;

    installed_ = true;
    return true;
  }

  bool write(const std::vector<rmt_item32_t>& items) {
    if (!installed_ || items.empty()) return false;
    if (rmt_write_items(ch_, items.data(), items.size(), false) != ESP_OK) return false;
    return rmt_wait_tx_done(ch_, portMAX_DELAY) == ESP_OK;
  }

  void stop() {
    if (!installed_) return;
    (void)rmt_tx_stop(ch_);
    // line stays LOW (idle level) while installed
  }

  void end() {
    if (!installed_) return;
    stop();
    rmt_driver_uninstall(ch_);
    installed_ = false;
  }

  ~RmtTxSession() { end(); }

  uint32_t counter_hz() const {
    if (!installed_) return 0;
    uint32_t hz = 0;
    (void)rmt_get_counter_clock(ch_, &hz);
    return hz;
  }

private:
  rmt_channel_t ch_;
  gpio_num_t gpio_;
  uint8_t clk_div_;
  bool installed_ = false;
};

} // namespace

#if FUNKBUS_LED_TX_ENABLE
static inline void tx_led_write(bool on) {
  digitalWrite(FUNKBUS_LED_TX_GPIO,
               (FUNKBUS_LED_TX_ACTIVE_HIGH ? (on ? HIGH : LOW)
                                           : (on ? LOW : HIGH)));
}

struct TxLedGuard {
  TxLedGuard() {
    pinMode(FUNKBUS_LED_TX_GPIO, OUTPUT);
    tx_led_write(true);
  }
  ~TxLedGuard() { tx_led_write(false); }
};
#endif

// =====================================================================================
// Tiny TX state machine types
// =====================================================================================
enum class TxState : uint8_t { Idle,
                               ArmRadio,
                               SendFrame,
                               Gap,
                               Done };

struct TxCycle {
  TxState state = TxState::Idle;
  size_t idx = 0; // current frame index
  int64_t next_due_us = 0; // absolute timestamp in micros
};

// -------------------------------------------------------------------------------------
// nowMicros — monotonic microsecond timer (ESP32 uses esp_timer)">
static inline int64_t nowMicros() {
  return esp_timer_get_time();
}

// waitUntilMicros — busy/RTOS-wait until absolute timestamp, minimizing jitter">
static inline void waitUntilMicros(int64_t ts_abs) {
  for (;;) {
    const int64_t t = nowMicros();
    int64_t remain = ts_abs - t;
    if (remain <= 0) break;

    if (remain > FUNKBUS_WAIT_YIELD_US) {
      vTaskDelay(1); // release CPU for a tick
      continue;
    }
    taskYIELD(); // short spin/yield near the edge
  }
}

// =====================================================================================
// MQTT helpers for returning command results
// =====================================================================================

// fb_publish_json — serialize and publish a small result JSON via Theengs/OMG">
static inline void fb_publish_json(const JsonDocument& doc) {
  String out;
  out.reserve(FUNKBUS_JSON_RESERVE_TX);
  serializeJson(doc, out);
  FB_LOG_N(F("[CC1101 CMD->MQTT] topic='%s' payload=%s" CR),
           FUNKBUS_CMD_RESULT_TOPIC, out.c_str());
  pub(FUNKBUS_CMD_RESULT_TOPIC, out.c_str());
}

// =====================================================================================
#ifdef ZradioCC1101
// PublishCC1101RegistersJson — read CC1101 config/status/PA and publish as JSON">
static void PublishCC1101RegistersJson() {
  StaticJsonDocument<2048> d;
  d["type"] = "cc1101_dump";

  uint8_t regs_arr[CC1101_NUM_CONFIG_REGS] = {0}, status_arr[CC1101_NUM_STATUS_REGS] = {0}, pa_tbl[CC1101_PATABLE_SIZE] = {0};
  FunkbusTB::readAllRegisters(regs_arr, status_arr, pa_tbl);

  JsonArray regs = d.createNestedArray("regs");
  for (uint8_t a = 0x00; a <= (CC1101_NUM_CONFIG_REGS - 1); ++a) {
    char buf[CC1101_HEXBUF_LEN];
    snprintf(buf, sizeof(buf), "0x%02X", regs_arr[a]);
    regs.add(buf);
  }
  JsonArray sregs = d.createNestedArray("status");
  for (uint8_t a = CC1101_FIRST_STATUS_ADDR; a <= CC1101_LAST_STATUS_ADDR; ++a) {
    char buf[CC1101_HEXBUF_LEN];
    snprintf(buf, sizeof(buf), "0x%02X", status_arr[a - CC1101_FIRST_STATUS_ADDR]);
    sregs.add(buf);
  }
  JsonArray pa = d.createNestedArray("pa");
  for (int i = 0; i < CC1101_PATABLE_SIZE; ++i) {
    char buf[CC1101_HEXBUF_LEN];
    snprintf(buf, sizeof(buf), "0x%02X", pa_tbl[i]);
    pa.add(buf);
  }

  fb_publish_json(d);
}

#endif

// =====================================================================================
// CC1101 helpers & debug glue
// =====================================================================================
#if FUNKBUS_REG_DUMPS >= 1
#  define DUMP_REGS() FunkbusTB::DumpRegistersHexLogs()
#else
#  define DUMP_REGS() \
    do {              \
    } while (0)
#endif

namespace {
// TxFreqGuard — RAII switch to TX MHz during a TX scope (restores listen MHz on exit)">
struct TxFreqGuard {
  float prev = NAN;
  explicit TxFreqGuard(float tx_mhz) {
#ifdef ZradioCC1101
    prev = FunkbusTB::GetListenMhz();
    FunkbusTB::setMHz(tx_mhz);
    DUMP_REGS();
#endif
  }
  ~TxFreqGuard() {
#ifdef ZradioCC1101
    if (!isnan(prev)) {
      FunkbusTB::setMHz(prev);
    }
    DUMP_REGS();
#endif
  }
};

} // namespace

// =====================================================================================
// Validation & mapping helpers
// =====================================================================================
static inline bool isHexChar(char c) {
  c = (char)toupper((unsigned char)c);
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F');
}
static inline bool isValidChannel(String ch) {
  ch.trim();
  ch.toUpperCase();
  return (ch == "A" || ch == "B" || ch == "C" || ch == "LS");
}
static inline bool isValidAction(String a) {
  a.trim();
  a.toUpperCase();
  return (a == "ON" || a == "OFF");
}
static inline bool isValidDuration(String d) {
  if (d.length() == 0) return true;
  d.trim();
  d.toUpperCase();
  return (d == "S" || d == "L");
}

// ValidatePayload — strict-check JSON fields and normalize strings">
bool FunkbusRemote::ValidatePayload(const String& jsonText, FunkbusPayload* out, String* error) {
  FB_VLOG(F("ValidatePayload: %s" CR), jsonText.c_str());

  StaticJsonDocument<512> doc;
  DeserializationError derr = deserializeJson(doc, jsonText);
  if (derr) {
    if (error) *error = derr.c_str();
    return false;
  }
  JsonObject root = doc.as<JsonObject>();

  if (!root.containsKey("serial")) {
    if (error) *error = F("Missing 'serial'");
    return false;
  }
  String serial = root["serial"].as<const char*>();
  serial.trim();
  serial.toUpperCase();
  if (serial.length() != SERIAL_HEX_LEN) {
    if (error) *error = F("'serial' must be exactly 5 hex chars");
    return false;
  }
  for (int i = 0; i < SERIAL_HEX_LEN; ++i)
    if (!isHexChar(serial[i])) {
      if (error) *error = F("'serial' has non-hex");
      return false;
    }

  if (!root.containsKey("channel")) {
    if (error) *error = F("Missing 'channel'");
    return false;
  }
  String channel = root["channel"].as<const char*>();
  channel.trim();
  channel.toUpperCase();
  if (!isValidChannel(channel)) {
    if (error) *error = F("channel must be A/B/C/LS");
    return false;
  }

  if (!root.containsKey("button")) {
    if (error) *error = F("Missing 'button'");
    return false;
  }
  int btn = root["button"].as<int>();
  if (btn < BUTTON_MIN || btn > BUTTON_MAX) {
    if (error) *error = F("button must be 1..8");
    return false;
  }

  if (!root.containsKey("action")) {
    if (error) *error = F("Missing 'action'");
    return false;
  }
  String action = root["action"].as<const char*>();
  action.trim();
  action.toUpperCase();
  if (!isValidAction(action)) {
    if (error) *error = F("action must be ON/OFF");
    return false;
  }

  String duration;
  if (root.containsKey("duration") && !root["duration"].isNull()) {
    duration = root["duration"].as<const char*>();
    duration.trim();
    duration.toUpperCase();
    if (!isValidDuration(duration)) {
      if (error) *error = F("duration must be S/L");
      return false;
    }
  }

  if (out) {
    out->serial = serial;
    out->channel = channel;
    out->button = (uint8_t)btn;
    out->action = action;
    out->duration = duration;
  }
  return true;
}

static const char* NIBBLE_BIN[16] = {
    "0000", "0001", "0010", "0011", "0100", "0101", "0110", "0111", "1000", "1001", "1010", "1011", "1100", "1101", "1110", "1111"};
static inline uint8_t hexVal(char c) {
  c = (char)toupper((unsigned char)c);
  return (c <= '9') ? (uint8_t)(c - '0') : (uint8_t)(10 + (c - 'A'));
}
static String Hex5To20Bits(const String& hex5) {
  String out;
  out.reserve(SERIAL_BITS);
  for (size_t i = 0; i < SERIAL_HEX_LEN; ++i) out += NIBBLE_BIN[hexVal(hex5[i])];
  return out;
}
static String MapButtonBits(uint8_t button) {
  static const char* BTN[BUTTON_MAX + 1] = {nullptr, "000", "100", "010", "110", "001", "101", "011", "111"};
  return (button >= BUTTON_MIN && button <= BUTTON_MAX) ? String(BTN[button]) : String("000");
}
static String MapChannelBits(const String& ch) {
  if (ch == "A") return String(FUNKBUS_CH_A);
  if (ch == "B") return String(FUNKBUS_CH_B);
  if (ch == "C") return String(FUNKBUS_CH_C);
  return String(FUNKBUS_CH_LS);
}
static String MapActionBit(const String& a) { return (a == "ON") ? String(FUNKBUS_ACT_ON) : String(FUNKBUS_ACT_OFF); }

// BuildFirst40Bits — String-based builder (kept for callers); prefer *_buf version in hot path">
String FunkbusRemote::BuildFirst40Bits(const FunkbusPayload& p, String* error) {
  if (p.serial.length() != SERIAL_HEX_LEN) {
    if (error) *error = F("serial must be 5 hex");
    return String();
  }
  String s20 = Hex5To20Bits(p.serial);
  String bits;
  bits.reserve(FIRST40_BITS);
  bits += FUNKBUS_RC_TYPE;
  bits += FUNKBUS_RC_SUBTYPE;
  bits += s20;
  bits += FUNKBUS_UNKNOWN_29_30;
  bits += FUNKBUS_BATTERY_OK;
  bits += FUNKBUS_UNKNOWN_32_33;
  bits += MapButtonBits(p.button);
  bits += MapChannelBits(p.channel);
  bits += FUNKBUS_UNKNOWN_39;
  bits += MapActionBit(p.action);
  if (bits.length() != FIRST40_BITS) {
    if (error) *error = F("internal length mismatch");
    return String();
  }
  return bits;
}

// ===== SCOM / parity / checksum (String path kept for parity with *_buf logic) =====
static String MapScomBits(uint8_t serial) {
  static const char* SCOM[SCOM_MAX + 1] = {"000", "100", "010", "110", "001", "101", "011", "111"};
  return String((serial < 8) ? SCOM[serial] : "000");
}
static char ComputeParityBit(const String& first43) {
  size_t ones = 0;
  for (size_t i = 0; i < first43.length(); ++i)
    if (first43[i] == '1') ++ones;
  return (ones & 1) ? '1' : '0';
}
static uint8_t XorBytesOverFirstNBits_MSB(const String& s, size_t nbits) {
  uint8_t xorv = 0, cur = 0;
  int cnt = 0;
  for (size_t i = 0; i < nbits; ++i) {
    cur |= (uint8_t)((s[i] == '1') ? (1u << (7 - cnt)) : 0u);
    if (++cnt == 8) {
      xorv ^= cur;
      cur = 0;
      cnt = 0;
    }
  }
  if (cnt) xorv ^= cur;
  return xorv;
}
static String ComputeChecksum4(const String& first43) {
  uint8_t xb = XorBytesOverFirstNBits_MSB(first43, PARITY_RANGE_BITS);
  uint8_t nib = ((xb >> 4) & 0x0F) ^ (xb & 0x0F);
  uint8_t res = 0;
  if (nib & 0x8) res ^= 0x8C;
  if (nib & 0x4) res ^= 0x32;
  if (nib & 0x2) res ^= 0xC8;
  if (nib & 0x1) res ^= 0x23;
  res &= 0x0F;
  return String(NIBBLE_BIN[res]);
}

// Build48BitFrame — String-based builder; prefer *_buf where performance matters">
String FunkbusRemote::Build48BitFrame(const String& first40Bits, uint8_t frameSerial, String* error) {
  if (first40Bits.length() != FIRST40_BITS) {
    if (error) *error = F("first40Bits must be 40 bits");
    return String();
  }
  if (frameSerial > SCOM_MAX) {
    if (error) *error = F("frameSerial 0..7");
    return String();
  }
  String first43 = first40Bits + MapScomBits(frameSerial);
  char parity = ComputeParityBit(first43);
  String chk4 = ComputeChecksum4(first43);
  String out;
  out.reserve(FRAME_BITS);
  out += first43;
  out += parity;
  out += chk4;
  return out;
}

// =====================================================================================
// Zero-alloc bit builders (preferred in hot paths)
// =====================================================================================

// nibble_to_bin / hex_val_uc / map_button_bits / map_channel_bits / parity & checksum helpers">
static inline void nibble_to_bin(uint8_t v, char* dst) {
  dst[0] = (v & 0x8) ? '1' : '0';
  dst[1] = (v & 0x4) ? '1' : '0';
  dst[2] = (v & 0x2) ? '1' : '0';
  dst[3] = (v & 0x1) ? '1' : '0';
}
static inline uint8_t hex_val_uc(char c) {
  return (c <= '9') ? static_cast<uint8_t>(c - '0') : static_cast<uint8_t>(10 + (c - 'A'));
}
static inline void map_button_bits(uint8_t button, char out3[3]) {
  static const char* BTN[BUTTON_MAX + 1] = {nullptr, "000", "100", "010", "110", "001", "101", "011", "111"};
  const char* s = (button >= BUTTON_MIN && button <= BUTTON_MAX) ? BTN[button] : "000";
  out3[0] = s[0];
  out3[1] = s[1];
  out3[2] = s[2];
}
static inline void map_channel_bits(const String& ch, char out2[2]) {
  const char* src =
      (ch == "A") ? FUNKBUS_CH_A : (ch == "B") ? FUNKBUS_CH_B
                               : (ch == "C")   ? FUNKBUS_CH_C
                                               : FUNKBUS_CH_LS;
  out2[0] = src[0];
  out2[1] = src[1];
}
static inline char parity_bit_over_first43(const char first43[PARITY_RANGE_BITS + 1]) {
  size_t ones = 0;
  for (int i = 0; i < PARITY_RANGE_BITS; ++i) ones += (first43[i] == '1');
  return (ones & 1) ? '1' : '0';
}
static uint8_t xor_bytes_over_first_n_bits_msb(const char* bits, size_t nbits) {
  uint8_t xb = 0, cur = 0;
  int cnt = 0;
  for (size_t i = 0; i < nbits; ++i) {
    if (bits[i] == '1') cur |= (uint8_t)(1u << (7 - cnt));
    if (++cnt == 8) {
      xb ^= cur;
      cur = 0;
      cnt = 0;
    }
  }
  if (cnt) xb ^= cur;
  return xb;
}
static inline void checksum4_over_first43(const char first43[PARITY_RANGE_BITS + 1], char out4[4]) {
  uint8_t xb = xor_bytes_over_first_n_bits_msb(first43, PARITY_RANGE_BITS);
  uint8_t nib = ((xb >> 4) & 0x0F) ^ (xb & 0x0F);
  uint8_t res = 0;
  if (nib & 0x8) res ^= 0x8C;
  if (nib & 0x4) res ^= 0x32;
  if (nib & 0x2) res ^= 0xC8;
  if (nib & 0x1) res ^= 0x23;
  res &= 0x0F;
  nibble_to_bin(res, out4);
}
static inline void hex5_to_20bits_uc(const String& hex5_uc, char out20[SERIAL_BITS]) {
  for (int i = 0; i < SERIAL_HEX_LEN; ++i) {
    uint8_t v = hex_val_uc(hex5_uc[i]);
    nibble_to_bin(v, &out20[i * 4]);
  }
}

namespace FunkbusRemote {
// BuildFirst40Bits_buf — zero-alloc first-40 builder for hot path TX">
bool BuildFirst40Bits_buf(const FunkbusPayload& p, char out40[FIRST40_STRLEN], std::string* /*error*/) {
  if (p.serial.length() != 5) {
    out40[0] = '\0';
    return false;
  }

  int idx = 0;

  memcpy(&out40[idx], FUNKBUS_RC_TYPE, 4);
  idx += 4;
  memcpy(&out40[idx], FUNKBUS_RC_SUBTYPE, 4);
  idx += 4;

  char s20[SERIAL_BITS];
  hex5_to_20bits_uc(p.serial, s20);
  memcpy(&out40[idx], s20, SERIAL_BITS);
  idx += SERIAL_BITS;

  memcpy(&out40[idx], FUNKBUS_UNKNOWN_29_30, 2);
  idx += 2;
  memcpy(&out40[idx], FUNKBUS_BATTERY_OK, 1);
  idx += 1;
  memcpy(&out40[idx], FUNKBUS_UNKNOWN_32_33, 2);
  idx += 2;

  char btn3[3];
  map_button_bits(p.button, btn3);
  memcpy(&out40[idx], btn3, 3);
  idx += 3;

  char ch2[2];
  map_channel_bits(p.channel, ch2);
  memcpy(&out40[idx], ch2, 2);
  idx += 2;

  memcpy(&out40[idx], FUNKBUS_UNKNOWN_39, 1);
  idx += 1;

  out40[FIRST40_BITS] = '\0';
  return true;
}

// Build48BitFrame_buf — zero-alloc 48-bit builder with SCOM/parity/checksum">
bool Build48BitFrame_buf(const char first40[FIRST40_STRLEN], uint8_t frameSerial, char out48[FRAME_STRLEN], std::string* /*error*/) {
  if (std::strlen(first40) != 40) {
    out48[0] = '\0';
    return false;
  }
  if (frameSerial > 7) {
    out48[0] = '\0';
    return false;
  }

  char first43[PARITY_RANGE_BITS + 1];
  std::memcpy(first43, first40, FIRST40_BITS);
  static const char* SCOM[SCOM_MAX + 1] = {"000", "100", "010", "110", "001", "101", "011", "111"};
  const char* s = SCOM[frameSerial];
  first43[SCOM_START_BIT + 0] = s[0];
  first43[SCOM_START_BIT + 1] = s[1];
  first43[SCOM_START_BIT + 2] = s[2];
  first43[PARITY_RANGE_BITS] = '\0';

  char chk4[4];
  const char parity = parity_bit_over_first43(first43);
  checksum4_over_first43(first43, chk4);

  std::memcpy(out48, first43, PARITY_RANGE_BITS);
  out48[PARITY_RANGE_BITS + 0] = parity;
  out48[PARITY_RANGE_BITS + 1] = chk4[0];
  out48[PARITY_RANGE_BITS + 2] = chk4[1];
  out48[PARITY_RANGE_BITS + 3] = chk4[2];
  out48[PARITY_RANGE_BITS + 4] = chk4[3];
  out48[FRAME_BITS] = '\0';
  return true;
}

} // namespace FunkbusRemote

// =====================================================================================
// RMT/segment helpers & TX
// =====================================================================================
struct Seg {
  uint8_t level;
  uint16_t us;
};

// SegsToRmt — convert compact (level,µs) segments to rmt_item32_t">
static void SegsToRmt(const std::vector<Seg>& segs, uint32_t clk_hz, std::vector<rmt_item32_t>& out) {
  if (!clk_hz) clk_hz = RMT_TICKS_PER_SEC;
  auto us_to_ticks = [clk_hz](uint32_t us) -> uint32_t {
    uint64_t num = (uint64_t)us * (uint64_t)clk_hz + RMT_ROUNDING_HALF;
    uint32_t t = (uint32_t)(num / 1000000ULL);
    return (t > RMT_DURATION_MAX_TICKS ? RMT_DURATION_MAX_TICKS : t);
  };
  out.clear();
  out.reserve((segs.size() + 1) / 2);
  for (size_t i = 0; i < segs.size();) {
    rmt_item32_t it{};
    it.level0 = segs[i].level;
    it.duration0 = us_to_ticks(segs[i].us);
    if (++i < segs.size()) {
      it.level1 = segs[i].level;
      it.duration1 = us_to_ticks(segs[i].us);
      ++i;
    } else {
      it.level1 = 0;
      it.duration1 = 0;
    }
    out.push_back(it);
  }
}

// pushSeg — coalesce adjacent same-level segments and cap duration to 16-bit">
static inline void pushSeg(std::vector<Seg>& segs, uint8_t level, uint32_t dur) {
  if (!dur) return;
  if (!segs.empty() && segs.back().level == level) {
    uint32_t sum = (uint32_t)segs.back().us + dur;
    segs.back().us = (uint16_t)std::min<uint32_t>(sum, 0xFFFF);
  } else {
    segs.push_back(Seg{level, (uint16_t)std::min<uint32_t>(dur, 0xFFFF)});
  }
}

namespace {
// SendFramesWithStateMachine48 — schedule & transmit four 48-bit frames with precise per-channel gaps">
static void SendFramesWithStateMachine48(const std::vector<const char*>& frames, RmtTxSession& txMain, uint32_t clk_main_hz) {
  auto BitsToSegments_noalloc48 = [](const char* bits, std::vector<Seg>& segs) -> uint32_t {
    segs.clear();
    segs.reserve(128);
    uint32_t total_us = 0;
    const uint16_t PRE_H = FUNKBUS_PREAMBLE_LEADIN_US;
    const uint16_t PRE_L = FUNKBUS_PREAMBLE_LEADOUT_US;
    const uint16_t H = FUNKBUS_HALF_BIT_US;

    pushSeg(segs, 0, FUNKBUS_TX_PREROLL_US);
    total_us += FUNKBUS_TX_PREROLL_US;
    pushSeg(segs, 1, PRE_H);
    total_us += PRE_H;
    pushSeg(segs, 0, PRE_L);
    total_us += PRE_L;

    uint8_t level = 1;
    for (size_t i = 0; i < FRAME_BITS; ++i) {
      const char b = bits[i];
      level ^= 1;
      pushSeg(segs, level, H);
      total_us += H;
      if (b == '1') {
        level ^= 1;
        pushSeg(segs, level, H);
        total_us += H;
      } else {
        pushSeg(segs, level, H);
        total_us += H;
      }
    }

    if (level == 0) {
      pushSeg(segs, 1, H);
      total_us += H;
    }
    pushSeg(segs, 0, H);
    total_us += H;
    pushSeg(segs, 1, H);
    total_us += H;
    pushSeg(segs, 0, H);
    total_us += H;

    return total_us; // trailer added by caller
  };

  std::vector<rmt_item32_t> rmt_main;

  std::vector<Seg> segbuf;

  TxCycle sm;
  sm.state = TxState::Idle;
  sm.idx = 0;
  sm.next_due_us = nowMicros();

  while (true) {
    switch (sm.state) {
      case TxState::Idle: {
        if (sm.idx >= frames.size()) return;
        sm.state = TxState::ArmRadio;
        break;
      }
      case TxState::ArmRadio: {
        sm.state = TxState::SendFrame;
        break;
      }
      case TxState::SendFrame: {
        const char* cur = frames[sm.idx];
        uint32_t frame_us = BitsToSegments_noalloc48(cur, segbuf);
        pushSeg(segbuf, 0, FUNKBUS_TX_TRAILER_US);
        frame_us += FUNKBUS_TX_TRAILER_US;

        SegsToRmt(segbuf, clk_main_hz, rmt_main);

        const int64_t t_before = nowMicros();
        txMain.write(rmt_main); // may block until MAIN frame done
        const int64_t t_after = nowMicros();

        static int64_t s_prev_frame_end = 0;
        int64_t this_frame_end = t_after; // end of MAIN write is a good proxy
        s_prev_frame_end = this_frame_end;

        const bool write_blocked = (t_after - t_before) > (int64_t)(frame_us / 2);

        sm.next_due_us = write_blocked
                             ? (t_after + (int64_t)FUNKBUS_INTERFRAME_GAP_US)
                             : (t_before + (int64_t)frame_us + (int64_t)FUNKBUS_INTERFRAME_GAP_US);

        sm.state = TxState::Gap;
        break;
      }
      case TxState::Gap: {
        const int64_t t = nowMicros();
        if (t >= sm.next_due_us) {
          ++sm.idx;
          sm.state = TxState::Idle;
        } else {
          int64_t remain = sm.next_due_us - t;
          if (remain > FUNKBUS_WAIT_YIELD_US) {
            vTaskDelay(1);
            continue;
          }
          taskYIELD();
        }
        break;
      }
      case TxState::Done:
      default:
        return;
    }
  }
}

} // namespace

// =====================================================================================
// Formatting helpers (debug)
// =====================================================================================
static String BitsToHex(const String& bits) {
  if (bits.length() % 4 != 0) return String("??");
  String hex;
  hex.reserve(bits.length() / 4);
  for (size_t i = 0; i < bits.length(); i += 4) {
    uint8_t v = 0;
    for (int j = 0; j < 4; ++j)
      if (bits[i + j] == '1') v |= (1 << (3 - j));
    hex += (v < 10) ? char('0' + v) : char('A' + (v - 10));
  }
  return hex;
}

// =====================================================================================
// Public: create & transmit frames
// =====================================================================================

// Create_and_TransmitFrames — build 4×48-bit frames from bits40/duration/action and send over RF">
void FunkbusRemote::Create_and_TransmitFrames(const String& bits40,
                                              const String& channel,
                                              uint8_t button,
                                              const String& action,
                                              const String& duration) {
#ifdef ZradioCC1101
  (void)channel;
  (void)button;

  if (bits40.length() != FIRST40_BITS) {
    FB_LOG_E(F("bits40 must be 40 bits (got %d)" CR), bits40.length());
    return;
  }

  static const uint8_t SEQ_S_ON[] = {0, 2, 2, 2};
  static const uint8_t SEQ_S_OFF[] = {1, 3, 3, 3};
  static const uint8_t SEQ_L_ON[] = {0, 4, 4, 6};
  static const uint8_t SEQ_L_OFF[] = {1, 5, 5, 7};

  const uint8_t* seq = (duration == "L")
                           ? ((action == "ON") ? SEQ_L_ON : SEQ_L_OFF)
                           : ((action == "ON") ? SEQ_S_ON : SEQ_S_OFF);

  std::array<std::array<char, FRAME_STRLEN>, FUNKBUS_TX_FRAMES_PER_BURST> frames_buf;
  std::vector<const char*> frames_ptr;
  frames_ptr.reserve(FUNKBUS_TX_FRAMES_PER_BURST);

  char first40[FIRST40_STRLEN];
  std::memcpy(first40, bits40.c_str(), FIRST40_BITS);
  first40[FIRST40_BITS] = '\0';

  for (int i = 0; i < FUNKBUS_TX_FRAMES_PER_BURST; ++i) {
    const uint8_t scom = seq[i];
    if (!Build48BitFrame_buf(first40, scom, frames_buf[i].data(), NULL)) {
      FB_LOG_E(F("Build48BitFrame_buf failed at idx %d" CR), i);
      return;
    }
    frames_ptr.push_back(frames_buf[i].data());
  }

  TxFreqGuard _guard(FUNKBUS_TX_MHZ);
  FunkbusTB::beginTxSession();

#  if FUNKBUS_LED_TX_ENABLE
  TxLedGuard _tx_led_on;
#  endif

  {
    RmtTxSession txMain(
        kRmtTxChannel,
        static_cast<gpio_num_t>(FUNKBUS_CC1101_GDO0_MCU),
        kRmtClkDiv);

    const bool ok_main = txMain.begin();

    if (ok_main) {
      const uint32_t clk_main_hz = txMain.counter_hz();

      for (int i = 0; i < FUNKBUS_TX_FRAMES_PER_BURST; ++i) {
        String bits = String(frames_buf[i].data());
        String hex = BitsToHex(bits);
        char scom3[4];
        scom3[0] = frames_buf[i][SCOM_START_BIT + 0];
        scom3[1] = frames_buf[i][SCOM_START_BIT + 1];
        scom3[2] = frames_buf[i][SCOM_START_BIT + 2];
        scom3[3] = '\0';
        uint8_t scom =
            ((uint8_t)(frames_buf[i][SCOM_START_BIT + 0] == '1') << 2) |
            ((uint8_t)(frames_buf[i][SCOM_START_BIT + 1] == '1') << 1) |
            (uint8_t)(frames_buf[i][SCOM_START_BIT + 2] == '1');

        FB_VLOG(F("[fb tx] frame[%d] scom=%u (%s) hex=%s bits=%s" CR),
                i, scom, scom3, hex.c_str(), frames_buf[i].data());
      }

      SendFramesWithStateMachine48(frames_ptr, txMain, clk_main_hz);

      txMain.end();
      pinMode(FUNKBUS_CC1101_GDO0_MCU, OUTPUT);
      digitalWrite(FUNKBUS_CC1101_GDO0_MCU, LOW);

      // One-liner TX summary at NOTICE (per command)
      FB_LOG_N(F("[funkbus_tx] ch=%s btn=%u action=%s frames=%d duration=%s" CR),
               channel.c_str(), (unsigned)button, action.c_str(),
               FUNKBUS_TX_FRAMES_PER_BURST, (duration == "L") ? "L" : "S");

      FunkbusTB::endTxSession();
      return;
    }
  }

  FB_VLOG(F("GPIO-timed TX (no RMT)" CR));
  pinMode(FUNKBUS_CC1101_GDO0_MCU, OUTPUT);

  for (size_t i = 0; i < frames_ptr.size(); ++i) {
    std::vector<Seg> segs_one;
    {
      const char* bits = frames_ptr[i];
      const uint16_t PRE_H = FUNKBUS_PREAMBLE_LEADIN_US;
      const uint16_t PRE_L = FUNKBUS_PREAMBLE_LEADOUT_US;
      const uint16_t H = FUNKBUS_HALF_BIT_US;
      pushSeg(segs_one, 0, FUNKBUS_TX_PREROLL_US);
      pushSeg(segs_one, 1, PRE_H);
      pushSeg(segs_one, 0, PRE_L);
      uint8_t level = 1;
      for (size_t j = 0; j < FRAME_BITS; ++j) {
        const char b = bits[j];
        level ^= 1;
        pushSeg(segs_one, level, H);
        if (b == '1') {
          level ^= 1;
          pushSeg(segs_one, level, H);
        } else {
          pushSeg(segs_one, level, H);
        }
      }
      if (level == 0) pushSeg(segs_one, 1, H);
      pushSeg(segs_one, 0, H);
      pushSeg(segs_one, 1, H);
      pushSeg(segs_one, 0, H);
      pushSeg(segs_one, 0, FUNKBUS_TX_TRAILER_US);
    }

    for (const auto& s : segs_one) {
      digitalWrite(FUNKBUS_CC1101_GDO0_MCU, s.level ? HIGH : LOW);

      delayMicroseconds(s.us);
    }
    digitalWrite(FUNKBUS_CC1101_GDO0_MCU, LOW);

    if (i + 1 < frames_ptr.size()) delayMicroseconds(FUNKBUS_INTERFRAME_GAP_US);
  }

  // One-liner TX summary at NOTICE (per command)
  FB_LOG_N(F("[funkbus_tx] ch=%s btn=%u action=%s frames=%d duration=%s" CR),
           channel.c_str(), (unsigned)button, action.c_str(),
           FUNKBUS_TX_FRAMES_PER_BURST, (duration == "L") ? "L" : "S");

  FunkbusTB::endTxSession();
#else
  (void)bits40;
  (void)channel;
  (void)button;
  (void)action;
  (void)duration;
#endif // ZradioCC1101
}

// =====================================================================================
// CC1101 command dispatcher
// =====================================================================================

// HandleCc1101Command — dispatch small JSON commands for CC1101 (dump, get/set listen MHz, carrier)">
bool FunkbusRemote::HandleCc1101Command(const String& json) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, json)) return false;

#ifndef ZradioCC1101
  (void)doc;
  return false;
#else
  const char* cmd = doc["cmd"];
  if (!cmd || !*cmd) return false;

  // ICommand received – verbose (TRACE), not NOTICE
  FB_VLOG(F("[CC1101 CMD] received: %s" CR), cmd);

  if (!strcmp(cmd, "cc1101_dump")) {
    PublishCC1101RegistersJson();
    DUMP_REGS();
    return true;
  }

  if (!strcmp(cmd, "get_listen_mhz")) {
    StaticJsonDocument<96> ack;
    ack["type"] = "get_listen_mhz";
    ack["listen_mhz"] = (double)FunkbusTB::GetListenMhz();
    fb_publish_json(ack);
    return true;
  }

  if (!strcmp(cmd, "cc1101_carrier")) {
    unsigned ms = doc["ms"] | FUNKBUS_DEFAULT_CARRIER_MS;
    FunkbusRemote::Debug_TxCarrierMs(ms);
    StaticJsonDocument<96> ack;
    ack["type"] = "cc1101_carrier";
    ack["ms"] = ms;
    fb_publish_json(ack);
    return true;
  }

  return false;
#endif
}

// =====================================================================================
// Debug helpers
// =====================================================================================

// Debug_TxCarrierMs — generate a solid OOK carrier for ms (LED test for RF path)">
void FunkbusRemote::Debug_TxCarrierMs(uint32_t ms) {
#ifdef ZradioCC1101
  FunkbusTB::beginTxSession();
  pinMode(FUNKBUS_CC1101_GDO0_MCU, OUTPUT);
  digitalWrite(FUNKBUS_CC1101_GDO0_MCU, HIGH);
  delay(ms);
  digitalWrite(FUNKBUS_CC1101_GDO0_MCU, LOW);
  FunkbusTB::endTxSession();
#else
  (void)ms;
#endif
}
