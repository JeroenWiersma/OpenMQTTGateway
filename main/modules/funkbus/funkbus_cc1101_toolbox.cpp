#include "funkbus_cc1101_toolbox.h"

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <math.h> // fabsf
#include <stdint.h> // uint*_t

#include "User_config.h"
#include "config_Funkbus.h"
#include "funkbus_log.h" // <-- unified logging shims
#include "funkbus_rx.h"

// ============================================================================
// TX timing readiness
// ============================================================================
// Max age for reuse of last SCAL (ms) — keep as macro so users can override.
#ifndef FUNKBUS_CC1101_SCAL_MAX_AGE_MS
#  define FUNKBUS_CC1101_SCAL_MAX_AGE_MS (15UL * 60UL * 1000UL) // 15 minutes
#endif
// Typed alias used by the code (gives us type-safety & debugger friendliness).
static inline constexpr uint32_t kScalMaxAgeMs =
    static_cast<uint32_t>(FUNKBUS_CC1101_SCAL_MAX_AGE_MS);

// ============================================================================
// Internal typed constants
// ============================================================================
namespace cc1101 {
// Modem & packet format
inline constexpr uint8_t MDMCFG2_OOK_NO_SYNC = 0x30;
inline constexpr uint8_t MDMCFG4_MASK = 0x0F;
inline constexpr uint8_t MDMCFG4_DRATE_E = 0x05;
inline constexpr uint8_t MDMCFG3_DRATE_M = 0x43;

// AGC
inline constexpr uint8_t AGCCTRL2_VAL = 0x07;
inline constexpr uint8_t AGCCTRL1_VAL = 0x00;
inline constexpr uint8_t AGCCTRL0_VAL = 0x91;

// MCSM values
inline constexpr uint8_t MCSM0_RX_AUTOCAL = 0x10;
inline constexpr uint8_t MCSM0_TX_NO_AUTOCAL = 0x00;
inline constexpr uint8_t MCSM1_RX_STAY_RX = 0x03;

// PKTCTRL0 mask/value
inline constexpr uint8_t PKTCTRL0_MODE_MASK = 0x03;
inline constexpr uint8_t PKTCTRL0_ASYNC_VALUE = 0x03;

// Masks
inline constexpr uint8_t MCSM0_FS_AUTOCAL_MASK = 0x30;
inline constexpr uint8_t MCSM1_RXOFF_MODE_MASK = 0x03;

// PATABLE & sentinel
inline constexpr uint8_t PA_OFF_LEVEL = 0x00;
inline constexpr uint8_t PA_ON_LEVEL = 0xC0;
inline constexpr uint8_t PA_ON_INDEX = 1;
inline constexpr uint8_t REG_SKIP = 0xFF;

// Register addresses (config)
inline constexpr uint8_t IOCFG2 = 0x00;
inline constexpr uint8_t IOCFG0 = 0x02;
inline constexpr uint8_t PKTCTRL0 = 0x08;
inline constexpr uint8_t FSCTRL1 = 0x0B;
inline constexpr uint8_t FSCTRL0 = 0x0C;
inline constexpr uint8_t FREQ2 = 0x0D;
inline constexpr uint8_t FREQ1 = 0x0E;
inline constexpr uint8_t FREQ0 = 0x0F;
inline constexpr uint8_t MDMCFG4 = 0x10;
inline constexpr uint8_t MDMCFG3 = 0x11;
inline constexpr uint8_t MDMCFG2 = 0x12;
inline constexpr uint8_t DEVIATN = 0x15; // used if provided in profile
inline constexpr uint8_t MCSM1 = 0x17;
inline constexpr uint8_t MCSM0 = 0x18;
inline constexpr uint8_t AGCCTRL2 = 0x1B;
inline constexpr uint8_t AGCCTRL1 = 0x1C;
inline constexpr uint8_t AGCCTRL0 = 0x1D;
inline constexpr uint8_t FREND0 = 0x22;
inline constexpr uint8_t PATABLE = 0x3E;

// Status & strobes
inline constexpr uint8_t MARCSTATE = 0x35;
inline constexpr uint8_t SRES = 0x30;
inline constexpr uint8_t SFSTXON = 0x31;
inline constexpr uint8_t SCAL = 0x33;
inline constexpr uint8_t SRX = 0x34;
inline constexpr uint8_t STX = 0x35;
inline constexpr uint8_t SIDLE = 0x36;
inline constexpr uint8_t SFRX = 0x3A;
inline constexpr uint8_t SFTX = 0x3B;

// Defaults / wiring values
inline constexpr uint8_t IOCFG2_RX = 0x14;
inline constexpr uint8_t IOCFG0_ASYNC = 0x0D;
inline constexpr uint8_t IOCFG0_HIZ = 0x2E;

// Timing
inline constexpr uint32_t WAIT_TX_READY_MS = 1;
} // namespace cc1101

// ----------------------------------------------------------------------------
// SPI helpers
// ----------------------------------------------------------------------------
static inline uint8_t r8(uint8_t a) { return ELECHOUSE_cc1101.SpiReadReg(a); }
static inline void w8(uint8_t a, uint8_t v) { ELECHOUSE_cc1101.SpiWriteReg(a, v); }
static inline void wB(uint8_t a, const uint8_t* p, size_t n) { ELECHOUSE_cc1101.SpiWriteBurstReg(a, (uint8_t*)p, n); }
static inline void rB(uint8_t a, uint8_t* p, size_t n) { ELECHOUSE_cc1101.SpiReadBurstReg(a, p, n); }
static inline void st(uint8_t s) { ELECHOUSE_cc1101.SpiStrobe(s); }

static inline uint8_t marc5_raw() {
  return (uint8_t)(ELECHOUSE_cc1101.SpiReadStatus(cc1101::MARCSTATE) & 0x1F);
}

bool FunkbusTB::waitMarc(uint8_t target, uint32_t timeout_ms) {
  const uint32_t until = millis() + timeout_ms;
  while ((int32_t)(millis() - until) < 0) {
    if (marc5_raw() == target) return true;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return false;
}

static inline const char* hex8(uint8_t v) {
  static char slot[4][3];
  static uint8_t i = 0;
  char* out = slot[(i++) & 3];
  static const char d[] = "0123456789ABCDEF";
  out[0] = d[(v >> 4) & 0x0F];
  out[1] = d[v & 0x0F];
  out[2] = '\0';
  return out;
}

namespace {
// Cached state to avoid redundant work and to enable fast switches
static float s_current_mhz = 0.0f;
static bool s_has_current_mhz = false;

enum class ProfileTag : uint8_t { Unknown = 0,
                                  RX = 1,
                                  TX = 2 };
static ProfileTag s_current_profile = ProfileTag::Unknown;

// Track last explicit synthesizer calibration (SCAL) conditions
static uint32_t s_last_cal_ms = 0;
static uint32_t s_last_cal_khz = 0; // use kHz to avoid float compares

inline uint32_t khz_from_mhz(float mhz) { return (uint32_t)(mhz * 1000.0f + 0.5f); }

inline bool scal_is_fresh(float mhz, uint32_t now_ms) {
  if (!s_has_current_mhz) return false;
  uint32_t khz = khz_from_mhz(mhz);
  if (khz != s_last_cal_khz) return false;
  return (now_ms - s_last_cal_ms) < kScalMaxAgeMs; // use typed constexpr alias
}

// Single MARC wait wrappers
inline bool wait_rx(uint32_t tmo) { return FunkbusTB::waitMarc(FunkbusTB::MARC_RX, tmo); }
inline bool wait_tx(uint32_t tmo) { return FunkbusTB::waitMarc(FunkbusTB::MARC_TX, tmo); }
inline bool wait_idle(uint32_t tmo) { return FunkbusTB::waitMarc(FunkbusTB::MARC_IDLE, tmo); }

// Send a strobe (assumes ELECHOUSE API)
inline void strobe(uint8_t cmd) { ELECHOUSE_cc1101.SpiStrobe(cmd); }

// Run SCAL only when needed (freq change or stale)
bool scal_if_needed(float mhz) {
  uint32_t now_ms = (uint32_t)millis();
  if (scal_is_fresh(mhz, now_ms)) return true;
  // SCAL: From IDLE, strobe SCAL and wait again for IDLE
  strobe(cc1101::SIDLE);
  (void)wait_idle(5);
  strobe(cc1101::SCAL);
  bool ok = wait_idle(8);
  if (ok) {
    s_last_cal_ms = now_ms;
    s_last_cal_khz = khz_from_mhz(mhz);
  }
  return ok;
}

// Small compare with tolerance to avoid tiny float jitter
inline bool same_mhz(float a, float b) {
  return fabsf(a - b) < 0.0005f; // ~500 Hz tolerance
}
} // namespace

// ============================================================================
// Core utilities (state control, MARC names/waits, MHz set)
// ============================================================================
void FunkbusTB::goIdleAndFlush() {
  st(cc1101::SIDLE);
  st(cc1101::SFTX);
  st(cc1101::SFRX);
}

const char* FunkbusTB::marcName(uint8_t s) {
  switch (s & 0x1F) {
    case 0x00:
      return "SLEEP";
    case 0x01:
      return "IDLE";
    case 0x02:
      return "XOFF";
    case 0x03:
      return "VCOON_MC";
    case 0x04:
      return "REGON_MC";
    case 0x05:
      return "MANCAL";
    case 0x06:
      return "VCOON";
    case 0x07:
      return "REGON";
    case 0x08:
      return "STARTCAL";
    case 0x09:
      return "BWBOOST";
    case 0x0A:
      return "FS_LOCK";
    case 0x0B:
      return "IFADCON";
    case 0x0C:
      return "ENDCAL";
    case 0x0D:
      return "RX";
    case 0x0E:
      return "RX_END";
    case 0x0F:
      return "RX_RST";
    case 0x10:
      return "TXRX_SWITCH";
    case 0x11:
      return "RXFIFO_OVERFLOW";
    case 0x12:
      return "FSTXON";
    case 0x13:
      return "TX";
    case 0x14:
      return "TX_END";
    case 0x15:
      return "RXTX_SWITCH";
    case 0x16:
      return "TXFIFO_UNDERFLOW";
    default:
      return "?";
  }
}

uint8_t FunkbusTB::readMarc5() { return marc5_raw(); }

void FunkbusTB::setMHz(float mhz) {
  // Early out if no change
  if (s_has_current_mhz && same_mhz(mhz, s_current_mhz)) {
    return;
  }
  ELECHOUSE_cc1101.setMHZ(mhz);

  // Update cache
  s_current_mhz = mhz;
  s_has_current_mhz = true;
}

int FunkbusTB::ReadRssiDbm() {
#ifdef ZradioCC1101
  return (int)ELECHOUSE_cc1101.getRssi();
#else
  return 0;
#endif
}

// ============================================================================
// Profile applier (masked writes + optional PATABLE)
// ============================================================================
bool FunkbusTB::applyProfile(const CC1101Profile& p) {
  // Always start from a clean state (IDLE + flush both FIFOs)
  goIdleAndFlush();

  // Small helpers: write only if value would actually change.
  auto write_if_diff = [](uint8_t addr, uint8_t newv) {
    const uint8_t oldv = r8(addr);
    if (oldv != newv) w8(addr, newv);
  };
  auto write_masked_if_diff = [](uint8_t addr, uint8_t mask, uint8_t set_bits) {
    const uint8_t oldv = r8(addr);
    const uint8_t newv = (uint8_t)((oldv & ~mask) | (set_bits & mask));
    if (newv != oldv) w8(addr, newv);
  };

  // 1) IOCFG first (ensures subsequent async/bit-bang uses the right pins)
  write_if_diff(cc1101::IOCFG0, p.iocfg0);
  write_if_diff(cc1101::IOCFG2, p.iocfg2);

#if defined(FUNKBUS_CC1101_GDO0_MCU) && (FUNKBUS_CC1101_GDO0_MCU >= 0)
  // Output if TX Hi-Z (we drive OOK), otherwise input for RX async.
  pinMode(FUNKBUS_CC1101_GDO0_MCU, (p.iocfg0 == cc1101::IOCFG0_HIZ) ? OUTPUT : INPUT);
#endif
#if defined(FUNKBUS_CC1101_GDO2_MCU) && (FUNKBUS_CC1101_GDO2_MCU >= 0)
  pinMode(FUNKBUS_CC1101_GDO2_MCU, INPUT);
#endif

  // 2) PKTCTRL0 masked update (async/whitening bits live here)
  if (p.pktctrl0_mask) {
    write_masked_if_diff(cc1101::PKTCTRL0, p.pktctrl0_mask, p.pktctrl0_value);
  }

  // 3) Core modem setup (OOK + data-rate + AGC)
  write_if_diff(cc1101::MDMCFG2, p.mdmcfg2);
  if (p.mdmcfg4_mask) {
    write_masked_if_diff(cc1101::MDMCFG4, p.mdmcfg4_mask, p.mdmcfg4_value);
  }
  write_if_diff(cc1101::MDMCFG3, p.mdmcfg3);

  write_if_diff(cc1101::AGCCTRL2, p.agcctrl2);
  write_if_diff(cc1101::AGCCTRL1, p.agcctrl1);
  write_if_diff(cc1101::AGCCTRL0, p.agcctrl0);

  if (p.deviatn != cc1101::REG_SKIP) write_if_diff(cc1101::DEVIATN, p.deviatn);
  if (p.fsctrl1 != cc1101::REG_SKIP) write_if_diff(cc1101::FSCTRL1, p.fsctrl1);

  // 4) MCSM behavior (autocal/overflow) as masked writes
  if (p.mcsm0_mask) write_masked_if_diff(cc1101::MCSM0, p.mcsm0_mask, p.mcsm0_value);
  if (p.mcsm1_mask) write_masked_if_diff(cc1101::MCSM1, p.mcsm1_mask, p.mcsm1_value);

  // 5) PA table (optional OOK shaping).
  if (p.set_pa) {
    (void)ensurePaTableOok(p.pa_off_level, p.pa_on_level, p.pa_on_index);
  }

  // 6) No SCAL here: leave calibration to the fast RX↔TX transition helpers.
  FB_LOG_N(F("[FunkbusTB] Profile applied (Δ writes only): %s" CR), p.name);
  return true;
}

// ============================================================================
// Persisted listen MHz (NVS) + helpers
// ============================================================================
static float s_listen_mhz = NAN;

static inline bool inFreqRange(float mhz) { return mhz >= 300.0f && mhz <= 928.0f; }

static inline void applyTxDiffs() {
  // IOCFGs first (they affect pin behavior)
  w8(cc1101::IOCFG0, kProfile_Funkbus_TX.iocfg0);
  w8(cc1101::IOCFG2, kProfile_Funkbus_TX.iocfg2);

#if defined(FUNKBUS_CC1101_GDO0_MCU) && (FUNKBUS_CC1101_GDO0_MCU >= 0)
  // TX: MCU drives OOK => OUTPUT when Hi-Z profile selected
  pinMode(FUNKBUS_CC1101_GDO0_MCU,
          (kProfile_Funkbus_TX.iocfg0 == cc1101::IOCFG0_HIZ) ? OUTPUT : INPUT);
#endif
#if defined(FUNKBUS_CC1101_GDO2_MCU) && (FUNKBUS_CC1101_GDO2_MCU >= 0)
  pinMode(FUNKBUS_CC1101_GDO2_MCU, INPUT);
#endif

  if (kProfile_Funkbus_TX.pktctrl0_mask) {
    uint8_t v = r8(cc1101::PKTCTRL0);
    v = (uint8_t)((v & ~kProfile_Funkbus_TX.pktctrl0_mask) |
                  (kProfile_Funkbus_TX.pktctrl0_value & kProfile_Funkbus_TX.pktctrl0_mask));
    w8(cc1101::PKTCTRL0, v);
  }
}

static inline void applyRxDiffs() {
  // IOCFGs back to RX semantics
  w8(cc1101::IOCFG0, kProfile_Funkbus_RX.iocfg0);
  w8(cc1101::IOCFG2, kProfile_Funkbus_RX.iocfg2);

#if defined(FUNKBUS_CC1101_GDO0_MCU) && (FUNKBUS_CC1101_GDO0_MCU >= 0)
  // RX: ASYNC from chip => INPUT
  pinMode(FUNKBUS_CC1101_GDO0_MCU, INPUT);
#endif
#if defined(FUNKBUS_CC1101_GDO2_MCU) && (FUNKBUS_CC1101_GDO2_MCU >= 0)
  pinMode(FUNKBUS_CC1101_GDO2_MCU, INPUT);
#endif

  if (kProfile_Funkbus_RX.pktctrl0_mask) {
    uint8_t v = r8(cc1101::PKTCTRL0);
    v = (uint8_t)((v & ~kProfile_Funkbus_RX.pktctrl0_mask) |
                  (kProfile_Funkbus_RX.pktctrl0_value & kProfile_Funkbus_RX.pktctrl0_mask));
    w8(cc1101::PKTCTRL0, v);
  }
}

bool FunkbusTB::switchRxToTxFast(uint32_t timeout_ms) {
  st(cc1101::SIDLE);
  if (!wait_idle(2)) return false;

  applyTxDiffs(); // ensure pins/mode are in TX shape
  st(cc1101::STX);

  return wait_tx(timeout_ms);
}

bool FunkbusTB::switchTxToRxFast(uint32_t timeout_ms) {
  st(cc1101::SIDLE);
  if (!wait_idle(2)) return false;

  applyRxDiffs(); // back to RX shape so GDO2/async & RMT work
  st(cc1101::SRX);

  return wait_rx(timeout_ms);
}

void FunkbusTB::EnsureListenFreqInitialized() {
  if (!isnan(s_listen_mhz)) return;
  const uint32_t hz = FunkbusTB::readProgrammedFrequencyHz(FUNKBUS_CC1101_XTAL_HZ);
  s_listen_mhz = (float)hz / 1000000.0f;

  FB_LOG_N(
      F("Listen MHz (cached) = %F (from FREQ regs: %u Hz)" CR),
      (double)s_listen_mhz,
      (unsigned)hz);
}

void FunkbusTB::SetListenMhz(float mhz) {
  if (!inFreqRange(mhz)) return; // silently ignore out-of-range
  s_listen_mhz = mhz; // cache only; OMG is responsible for applying
}

float FunkbusTB::GetListenMhz() {
  FunkbusTB::EnsureListenFreqInitialized();
  return s_listen_mhz;
}

// ============================================================================
// Frequency helpers (program/read/ensure; IF offset)
// ============================================================================
static inline uint32_t freqWordFromHz(uint32_t freq_hz, uint32_t fxosc_hz) {
  uint64_t num = (uint64_t)freq_hz << 16;
  return (uint32_t)((num + (fxosc_hz / 2)) / fxosc_hz) & 0x00FFFFFFUL;
}

static inline uint32_t hzFromFreqWord(uint32_t word, uint32_t fxosc_hz) {
  uint64_t num = (uint64_t)word * (uint64_t)fxosc_hz;
  return (uint32_t)(num >> 16);
}

bool FunkbusTB::programFrequencyHz(uint32_t freq_hz, uint32_t fxosc_hz, bool do_calibrate) {
  st(cc1101::SIDLE);
  (void)waitMarc(MARC_IDLE, 10);

  const uint32_t word = freqWordFromHz(freq_hz, fxosc_hz);
  w8(cc1101::FREQ2, (word >> 16) & 0xFF);
  w8(cc1101::FREQ1, (word >> 8) & 0xFF);
  w8(cc1101::FREQ0, (word >> 0) & 0xFF);

  if (do_calibrate) {
    st(cc1101::SCAL);
    (void)waitMarc(MARC_IDLE, 50);
  }

  const uint32_t rb = FunkbusTB::readProgrammedFrequencyHz(fxosc_hz);
  // Frequency change: keep a concise NOTICE
  FB_LOG_N(F("[FunkbusTB] Program FREQ=%u Hz -> read %u Hz (Δ=%ld Hz)" CR),
           (unsigned)freq_hz, (unsigned)rb, (long)((int32_t)rb - (int32_t)freq_hz));
  return true;
}

uint32_t FunkbusTB::readProgrammedFrequencyHz(uint32_t fxosc_hz) {
  const uint32_t word = ((uint32_t)r8(cc1101::FREQ2) << 16) |
                        ((uint32_t)r8(cc1101::FREQ1) << 8) |
                        ((uint32_t)r8(cc1101::FREQ0) << 0);
  return hzFromFreqWord(word, fxosc_hz);
}

bool FunkbusTB::ensureDefaultFrequency(uint32_t tol_hz, uint32_t fxosc_hz) {
  const uint32_t now_hz = readProgrammedFrequencyHz(fxosc_hz);
  const int32_t d = (int32_t)now_hz - (int32_t)FUNKBUS_DEFAULT_FREQ_HZ;
  if (d >= -(int32_t)tol_hz && d <= (int32_t)tol_hz) {
    FB_VLOG(F("[FunkbusTB] FREQ ok: %u Hz (Δ=%ld)" CR), (unsigned)now_hz, (long)d);
    return true;
  }
  FB_LOG_W(F("[FunkbusTB] FREQ off by %ld Hz; reprogram" CR), (long)d);
  return programFrequencyHz(FUNKBUS_DEFAULT_FREQ_HZ, fxosc_hz, true);
}

void FunkbusTB::setIfFrequencyOffset(int8_t if_khz) {
  uint8_t v = (uint8_t)(if_khz & 0x1F);
  w8(cc1101::FSCTRL1, v);
  FB_VLOG(F("[FunkbusTB] FSCTRL1 (IF) = 0x%02X" CR), v);
}

// ============================================================================
// Driver init + PATABLE helpers
// ============================================================================
bool FunkbusTB::ensureDriverInitialized() {
  static bool s_done = false;
  if (s_done) return true;

#if defined(FUNKBUS_CC1101_GDO0_MCU) && (FUNKBUS_CC1101_GDO0_MCU >= 0)
  ELECHOUSE_cc1101.setGDO(0, FUNKBUS_CC1101_GDO0_MCU);
#endif
#if defined(FUNKBUS_CC1101_GDO2_MCU) && (FUNKBUS_CC1101_GDO2_MCU >= 0)
  ELECHOUSE_cc1101.setGDO(2, FUNKBUS_CC1101_GDO2_MCU);
#endif

  ELECHOUSE_cc1101.Init();
  s_done = true;
  return true;
}

bool FunkbusTB::ensurePaTableOok(uint8_t offLevel, uint8_t onLevel, uint8_t onIndex) {
  if (onIndex > 7) onIndex = 1;
  uint8_t pa[8] = {0};
  pa[0] = offLevel;
  pa[onIndex] = onLevel;
  wB(cc1101::PATABLE, pa, sizeof(pa));

  uint8_t fr0 = r8(cc1101::FREND0);
  fr0 = (uint8_t)((fr0 & ~0x07u) | (onIndex & 0x07u));
  w8(cc1101::FREND0, fr0);

  uint8_t rb[8];
  rB(cc1101::PATABLE, rb, sizeof(rb));
  bool ok = true;
  for (int i = 0; i < 8; ++i) {
    const uint8_t exp = (i == onIndex) ? onLevel : (i == 0 ? offLevel : 0x00);
    if (rb[i] != exp) {
      ok = false;
      break;
    }
  }
  if (!ok) FB_LOG_W(F("[FunkbusTB] PATABLE verify failed (on=%s idx=%u)" CR), hex8(onLevel), (unsigned)onIndex);
  return ok;
}

// ============================================================================
// RX/TX profiles
// ============================================================================
const CC1101Profile kProfile_Funkbus_RX = {
    .name = "Funkbus_RX",
    .iocfg0 = cc1101::IOCFG0_ASYNC,
    .iocfg2 = cc1101::IOCFG2_RX,

    .pktctrl0_mask = cc1101::PKTCTRL0_MODE_MASK, // 0x03 [1:0] = async/sync mode select
    .pktctrl0_value = cc1101::PKTCTRL0_ASYNC_VALUE, // 0x03 async mode

    .mdmcfg2 = cc1101::MDMCFG2_OOK_NO_SYNC, // 0x30 OOK, no sync
    .mdmcfg4_mask = cc1101::MDMCFG4_MASK, // 0x0F keep CHANBW nibble, set DRATE_E nibble
    .mdmcfg4_value = cc1101::MDMCFG4_DRATE_E, // 0x05 ~1 kbps baseline
    .mdmcfg3 = cc1101::MDMCFG3_DRATE_M, // 0x43 DRATE_M for ~1 kbps

    .agcctrl2 = cc1101::AGCCTRL2_VAL, // 0x07 Mild OOK-friendly AGC
    .agcctrl1 = cc1101::AGCCTRL1_VAL,
    .agcctrl0 = cc1101::AGCCTRL0_VAL,

    .deviatn = cc1101::REG_SKIP, // 0xFF unused for OOK; leave as-is
    .fsctrl1 = cc1101::REG_SKIP, // 0xFF keep current IF offset unless explicitly set

    .mcsm0_mask = cc1101::MCSM0_FS_AUTOCAL_MASK, // 0x30 FS_AUTOCAL bits
    .mcsm0_value = cc1101::MCSM0_RX_AUTOCAL, // 0x10 Calibrate from IDLE
    .mcsm1_mask = cc1101::MCSM1_RXOFF_MODE_MASK, // 0x03 RXOFF_MODE bits
    .mcsm1_value = cc1101::MCSM1_RX_STAY_RX, // 0x03 Stay in RX on overflow

    .set_pa = true, // still program a clean OOK table (safe)
    .pa_off_level = cc1101::PA_OFF_LEVEL, // 0x00
    .pa_on_level = cc1101::PA_ON_LEVEL, // 0xC0
    .pa_on_index = cc1101::PA_ON_INDEX, // 1
};

const CC1101Profile kProfile_Funkbus_TX = {
    .name = "Funkbus_TX",
    .iocfg0 = cc1101::IOCFG0_HIZ, // MCU drives OOK on this pin
    .iocfg2 = cc1101::IOCFG2_RX,

    .pktctrl0_mask = cc1101::PKTCTRL0_MODE_MASK,
    .pktctrl0_value = cc1101::PKTCTRL0_ASYNC_VALUE, // async (we're toggling the PA line externally)

    .mdmcfg2 = cc1101::MDMCFG2_OOK_NO_SYNC, // 0x30 OOK, no sync
    .mdmcfg4_mask = cc1101::MDMCFG4_MASK, // 0x0F keep CHANBW nibble, set DRATE_E nibble
    .mdmcfg4_value = cc1101::MDMCFG4_DRATE_E, // 0x05 ~1 kbps baseline
    .mdmcfg3 = cc1101::MDMCFG3_DRATE_M, // 0x43 DRATE_M for ~1 kbps

    .agcctrl2 = cc1101::AGCCTRL2_VAL, // 0x07 Mild OOK-friendly AGC
    .agcctrl1 = cc1101::AGCCTRL1_VAL,
    .agcctrl0 = cc1101::AGCCTRL0_VAL,

    .deviatn = cc1101::REG_SKIP, // 0xFF unused for OOK; leave as-is
    .fsctrl1 = cc1101::REG_SKIP, // 0xFF keep current IF offset unless explicitly set

    .mcsm0_mask = cc1101::MCSM0_FS_AUTOCAL_MASK, // 0x30 FS_AUTOCAL bits
    .mcsm0_value = cc1101::MCSM0_TX_NO_AUTOCAL, // 0x00 Disable FS_AUTOCAL on IDLE->TX; we SCAL explicitly
    .mcsm1_mask = cc1101::MCSM1_RXOFF_MODE_MASK, // 0x03 RXOFF_MODE bits
    .mcsm1_value = cc1101::MCSM1_RX_STAY_RX, // 0x03 Stay in RX on overflow

    .set_pa = true,
    .pa_off_level = cc1101::PA_OFF_LEVEL, // 0x00
    .pa_on_level = cc1101::PA_ON_LEVEL, // 0xC0
    .pa_on_index = cc1101::PA_ON_INDEX, // 1
};

// ============================================================================
// High-level RX/TX readiness helpers
// ============================================================================
bool FunkbusTB::assureRxReady(uint32_t timeout_ms) {
  (void)applyProfile(kProfile_Funkbus_RX); // leaves radio in IDLE

  st(cc1101::SRX);
  if (waitMarc(MARC_RX, timeout_ms)) return true;

  // One quick recovery attempt for odd cases (overflow, missed lock).
  goIdleAndFlush();
  st(cc1101::SCAL);
  (void)waitMarc(MARC_IDLE, 50);
  st(cc1101::SRX);
  return waitMarc(MARC_RX, timeout_ms);
}

bool FunkbusTB::assureTxReady(uint32_t timeout_ms) {
  (void)applyProfile(kProfile_Funkbus_TX);
  // We just need the chip IDLE & configured; the caller will drive OOK via GDO0.
  goIdleAndFlush();
  return waitMarc(MARC_IDLE, timeout_ms);
}

// ============================================================================
// Register dumps (debug utilities)
// ============================================================================
void FunkbusTB::readAllRegisters(uint8_t cfg[0x2F], uint8_t status[0x0C], uint8_t pa[8]) {
  for (uint8_t a = 0x00; a <= 0x2E; ++a) cfg[a] = r8(a);
  for (uint8_t a = 0x30; a <= 0x3B; ++a) status[a - 0x30] = (uint8_t)ELECHOUSE_cc1101.SpiReadStatus(a);
  rB(cc1101::PATABLE, pa, 8);
}

void FunkbusTB::DumpRegistersHexLogs() {
  // Config regs
  String line;
  line.reserve(128);
  for (uint8_t a = 0x00; a <= 0x2E; ++a) {
    char buf[12];
    snprintf(buf, sizeof(buf), "0x%02X:0x%02X ", a, r8(a));
    line += buf;
    if ((a & 0x0F) == 0x0F) {
      FB_VLOG(F("%s" CR), line.c_str());
      line = "";
    }
  }
  if (line.length()) FB_VLOG(F("%s" CR), line.c_str());

  // Status regs
  line = "";
  for (uint8_t a = 0x30; a <= 0x3B; ++a) {
    uint8_t v = (uint8_t)ELECHOUSE_cc1101.SpiReadStatus(a);
    char buf[12];
    snprintf(buf, sizeof(buf), "0x%02X:0x%02X ", a, v);
    line += buf;
    if ((a & 0x0F) == 0x0F) {
      FB_VLOG(F("%s" CR), line.c_str());
      line = "";
    }
  }
  if (line.length()) FB_VLOG(F("%s" CR), line.c_str());

  // PA table
  uint8_t pa_tbl[8] = {0};
  rB(cc1101::PATABLE, pa_tbl, 8);
  line = "[CC1101] PA_TABLE: ";
  for (int i = 0; i < 8; ++i) {
    char b[6];
    snprintf(b, sizeof(b), "0x%02X ", pa_tbl[i]);
    line += b;
  }
  FB_VLOG(F("%s" CR), line.c_str());
}

// ============================================================================
// Self-tests (optional helpers)
// ============================================================================
void FunkbusTB::selfTest_CommonProfile() {
  FB_LOG_N(F("[FunkbusTB][Selftest] CommonProfile" CR));
  ensureDriverInitialized();
  bool ok = assureRxReady(40);
  FB_LOG_N(F("[FunkbusTB][Selftest] assureRxReady: %s (MARC=%s)" CR),
           ok ? "OK" : "FAIL", marcName(readMarc5()));
  // Leave radio idle so normal startup continues cleanly
  st(cc1101::SIDLE);
  (void)waitMarc(MARC_IDLE, 10);
}

void FunkbusTB::selfTest_RxTxTransitions() {
  FB_LOG_N(F("[FunkbusTB][Selftest] Rx↔Tx transitions" CR));
  ensureDriverInitialized();

  // Start TX session, emit a very short OOK burst on GDO0, then return to RX (ESP32-tested; not a protocol TX path)
  beginTxSession();
#if defined(FUNKBUS_CC1101_GDO0_MCU) && (FUNKBUS_CC1101_GDO0_MCU >= 0)
  pinMode(FUNKBUS_CC1101_GDO0_MCU, OUTPUT);
  digitalWrite(FUNKBUS_CC1101_GDO0_MCU, HIGH);
  delay(20);
  digitalWrite(FUNKBUS_CC1101_GDO0_MCU, LOW);
#endif
  endTxSession();

  // Ensure we are back on the listen frequency and RX is ready
  setMHz(GetListenMhz());
  (void)assureRxReady(40);
  FB_LOG_N(F("[FunkbusTB][Selftest] Rx↔Tx done (MARC=%s)" CR), marcName(readMarc5()));
}

void FunkbusTB::selfTest_Frequency(uint32_t freq_hz) {
  FB_LOG_N(F("[FunkbusTB][Selftest] Frequency sweep -> %u Hz" CR), (unsigned)freq_hz);
  ensureDriverInitialized();

  // Remember current programmed frequency
  const uint32_t before_hz = readProgrammedFrequencyHz(FUNKBUS_CC1101_XTAL_HZ);

  // Program requested frequency (calibrate), wait a blink, then restore
  programFrequencyHz(freq_hz, FUNKBUS_CC1101_XTAL_HZ, true);
  delay(30);
  programFrequencyHz(before_hz, FUNKBUS_CC1101_XTAL_HZ, true);

  // Re-apply persisted listen MHz and prep RX
  setMHz(GetListenMhz());
  (void)assureRxReady(40);
  FB_LOG_N(F("[FunkbusTB][Selftest] Frequency restored (%u Hz), MARC=%s" CR),
           (unsigned)readProgrammedFrequencyHz(FUNKBUS_CC1101_XTAL_HZ),
           marcName(readMarc5()));
}

// ============================================================================
// TX session helpers (mirror of Remote’s old begin/end)
// ============================================================================
static inline bool WaitTxReady() {
  const uint32_t until = millis() + cc1101::WAIT_TX_READY_MS;
  uint8_t st5 = FunkbusTB::readMarc5();
  while ((int32_t)(millis() - until) < 0) {
    st5 = FunkbusTB::readMarc5();
    if (st5 == FunkbusTB::MARC_TX || st5 == FunkbusTB::MARC_FSTXON) {
      FB_VLOG(F("CC1101 TX ready: %s (0x%s)" CR), FunkbusTB::marcName(st5), hex8(st5));
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  FB_LOG_W(F("CC1101 TX not ready: %s (0x%s)" CR), FunkbusTB::marcName(st5), hex8(st5));
  return false;
}

void FunkbusTB::beginTxSession() {
  FunkbusRx::pauseForTx();

  // Fast hop RX->TX; NO extra waits here.
  if (!switchRxToTxFast(20)) {
    // Fallback only if fast fails:
    (void)FunkbusTB::assureTxReady(20);
#if FUNKBUS_REG_DUMPS >= 1
    DumpRegistersHexLogs();
#endif
    st(cc1101::STX);
    (void)WaitTxReady(); // single wait here in fallback path
  }
}

void FunkbusTB::endTxSession() {
  // No explicit SIDLE here; resumeAfterTx() will do fast SIDLE→SRX and re-arm RX.
  FunkbusRx::resumeAfterTx();
}

// ============================================================================
// Raw OOK TX session (save → configure → TX → restore)
// ============================================================================

bool FunkbusTB::rawOokBegin(RawTxSession& s, uint32_t tx_freq_hz) {
  ensureDriverInitialized();
  EnsureListenFreqInitialized();

  // Snapshot current state
  s.prev_freq_hz = readProgrammedFrequencyHz(FUNKBUS_CC1101_XTAL_HZ);
  s.prev_listen_mhz = GetListenMhz();
  s.prev_marc5 = readMarc5();
  s.restore_freq = false;

  // If a TX frequency is requested and differs, reprogram (with SCAL)
  if (tx_freq_hz != 0 && tx_freq_hz != s.prev_freq_hz) {
    s.restore_freq = true;
    (void)programFrequencyHz(tx_freq_hz, FUNKBUS_CC1101_XTAL_HZ, true);
  }

  // Enter TX session using your fast path; it applies TX profile + GDO modes
  beginTxSession();

  // We expect TX (or FSTXON) here; tolerate brief settling
  const uint8_t st5 = readMarc5();
  if (st5 != MARC_TX && st5 != MARC_FSTXON) {
    FB_LOG_W(F("[FunkbusTB] rawOokBegin: unexpected MARC=%s (0x%02X)" CR),
             marcName(st5), (unsigned)st5);
  }

  // The caller (FunkbusRemote) now drives OOK on FUNKBUS_CC1101_GDO0_MCU
  return true;
}

bool FunkbusTB::rawOokEndRestore(const RawTxSession& s) {
  // Return to RX path and let your RX module re-arm
  endTxSession(); // this does SIDLE→SRX via switchTxToRxFast() inside resumeAfterTx()

  // If we changed the programmed frequency, restore it exactly
  if (s.restore_freq) {
    (void)programFrequencyHz(s.prev_freq_hz, FUNKBUS_CC1101_XTAL_HZ, true);
  }

  // Re-apply the persisted listen MHz model (your project’s “home” freq)
  setMHz(s.prev_listen_mhz);

  // If we were in RX before the TX, ensure we are back in MARC_RX
  if (s.prev_marc5 == MARC_RX) {
    if (!assureRxReady(40)) {
      FB_LOG_W(F("[FunkbusTB] rawOokEndRestore: assureRxReady() failed; MARC=%s" CR),
               marcName(readMarc5()));
      return false;
    }
  } else {
    // If we weren’t in RX, at least verify IDLE is clean
    (void)waitMarc(MARC_IDLE, 10);
  }
  return true;
}
