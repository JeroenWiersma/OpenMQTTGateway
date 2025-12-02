#pragma once

//
// Funkbus CC1101 Toolbox — centralized low-level radio helpers for OMG
//
// Provides: CC1101 profiles, lightweight MARC helpers, frequency mgmt,
// RX/TX readiness helpers, raw OOK TX sessions, and diagnostics.
// Only declarations live here; implementation is in the .cpp.
//

#include <Arduino.h>

#include "User_config.h"
#include "config_Funkbus.h"

// ---- CC1101 constants -------------------------------------------------------

// Crystal frequency (Hz). Prefer FUNKBUS_CC1101_XTAL_HZ; accept legacy *_MHZ.
#ifndef FUNKBUS_CC1101_XTAL_HZ
#  ifdef FUNKBUS_CC1101_XTAL_MHZ
#    define FUNKBUS_CC1101_XTAL_HZ ((uint32_t)(FUNKBUS_CC1101_XTAL_MHZ * 1000000UL))
#  else
#    define FUNKBUS_CC1101_XTAL_HZ (26000000UL) // default 26 MHz
#  endif
#endif

// Default carrier (Hz) derived from listen MHz; used by frequency helpers.
#ifndef FUNKBUS_DEFAULT_FREQ_HZ
#  define FUNKBUS_DEFAULT_FREQ_HZ ((uint32_t)(FUNKBUS_DEFAULT_LISTEN_MHZ * 1000000.0f + 0.5f))
#endif

// ---- Driver forward-declarations -------------------------------------------

class ELECHOUSE_CC1101;
extern ELECHOUSE_CC1101 ELECHOUSE_cc1101;

// ---- Declarative CC1101 profiles (RX/TX bundles) ---------------------------

struct CC1101Profile {
  const char* name;

  // Pins & I/O behavior
  uint8_t iocfg0; // e.g., ASYNC for RX, Hi-Z for TX bit-banging
  uint8_t iocfg2; // typically "RX status" on GDO2 in this project

  // Packet/async/whitening selection (PKTCTRL0): OR 'value' under 'mask'
  uint8_t pktctrl0_mask;
  uint8_t pktctrl0_value;

  // Modem core
  uint8_t mdmcfg2; // modulation + sync mode (0x30 = OOK, no sync)
  uint8_t mdmcfg4_mask; // CHANBW/DRATE_E nibble control (mask==0 => skip)
  uint8_t mdmcfg4_value;
  uint8_t mdmcfg3; // DRATE_M
  uint8_t agcctrl2;
  uint8_t agcctrl1;
  uint8_t agcctrl0;
  uint8_t deviatn; // 0xFF => leave as-is (useful for OOK)
  uint8_t fsctrl1; // 0xFF => leave as-is (IF setting)

  // MCSM safety (overflow handling, autocal, etc.) as masked writes
  uint8_t mcsm0_mask, mcsm0_value;
  uint8_t mcsm1_mask, mcsm1_value;

  // Optional PA table (handy for TX OOK shaping); skip when set_pa=false
  bool set_pa;
  uint8_t pa_off_level; // usually 0x00
  uint8_t pa_on_level; // e.g., 0xC0
  uint8_t pa_on_index; // which PATABLE slot carries "on" level (1..7)
};

// Canonical profiles for this project
extern const CC1101Profile kProfile_Funkbus_RX;
extern const CC1101Profile kProfile_Funkbus_TX;

// ---- Public API (namespace FunkbusTB) --------------------------------------

namespace FunkbusTB {

// MARC state (5-bit) values used by state/wait helpers.
enum : uint8_t {
  MARC_SLEEP = 0x00,
  MARC_IDLE = 0x01,
  MARC_XOFF = 0x02,
  MARC_VCOON_MC = 0x03,
  MARC_REGON_MC = 0x04,
  MARC_MANCAL = 0x05,
  MARC_VCOON = 0x06,
  MARC_REGON = 0x07,
  MARC_STARTCAL = 0x08,
  MARC_BWBOOST = 0x09,
  MARC_FS_LOCK = 0x0A,
  MARC_IFADCON = 0x0B,
  MARC_ENDCAL = 0x0C,
  MARC_RX = 0x0D,
  MARC_RX_END = 0x0E,
  MARC_RX_RST = 0x0F,
  MARC_TXRX_SWITCH = 0x10,
  MARC_RXFIFO_OVERFLOW = 0x11,
  MARC_FSTXON = 0x12,
  MARC_TX = 0x13,
  MARC_TX_END = 0x14,
  MARC_RXTX_SWITCH = 0x15,
  MARC_TXFIFO_UNDERFLOW = 0x16,
};

// ---- Core state helpers -----------------------------------------------------

// One-time ELECHOUSE driver init and GDO pin mapping.
bool ensureDriverInitialized();

// Poll MARCSTATE until target reached or timeout (ms granularity).
bool waitMarc(uint8_t targetState, uint32_t timeout_ms);

// Force CC1101 to clean IDLE and flush both FIFOs.
void goIdleAndFlush();

// Convert MARCSTATE[4:0] to a human-readable name (for logs).
const char* marcName(uint8_t marc_state_5bit);

// Lightweight accessor for current MARCSTATE[4:0].
uint8_t readMarc5();

// Set RF frequency in MHz via driver helper.
void setMHz(float mhz);

// Apply a CC1101Profile atomically (masked writes + optional PATABLE + SCAL).
// Leaves radio in IDLE; caller decides SRX/STX.
bool applyProfile(const CC1101Profile& p);

// ---- Frequency helpers (+ persisted listen MHz) ----------------------------

// Program carrier frequency (Hz) with optional calibration.
bool programFrequencyHz(uint32_t freq_hz = FUNKBUS_DEFAULT_FREQ_HZ,
                        uint32_t fxosc_hz = FUNKBUS_CC1101_XTAL_HZ,
                        bool do_calibrate = true);

// Read back the currently programmed carrier (Hz).
uint32_t readProgrammedFrequencyHz(uint32_t fxosc_hz = FUNKBUS_CC1101_XTAL_HZ);

// Ensure default Funkbus frequency within tolerance; reprogram if needed.
bool ensureDefaultFrequency(uint32_t tol_hz = 2000,
                            uint32_t fxosc_hz = FUNKBUS_CC1101_XTAL_HZ);

// Adjust IF offset (kHz) via FSCTRL1 (advanced tuning).
void setIfFrequencyOffset(int8_t if_khz);

void EnsureListenFreqInitialized();
void SetListenMhz(float mhz);

// Get current ensured listen-MHz value (reads radio on first use in NEW model).
float GetListenMhz();

// Read/write model of persisted listen MHz (no I/O).
float GetPersistedMhz();

// Get current RSSI dBm value.
int ReadRssiDbm();

// ---- PA table helpers -------------------------------------------------------

// Program an OOK-friendly PATABLE and active index (safe defaults provided).
bool ensurePaTableOok(uint8_t offLevel = 0x00, uint8_t onLevel = 0xC0, uint8_t onIndex = 1);

// ---- Common RX/TX readiness helpers ----------------------------------------

// Configure RX profile and ensure radio reaches MARC_RX (with one retry).
bool assureRxReady(uint32_t timeout_ms = 20);

// Configure TX profile; leave radio in IDLE ready for STX from caller.
bool assureTxReady(uint32_t timeout_ms = 20);

// Fast switch when about to TX and already on listen MHz.
bool switchRxToTxFast(uint32_t timeout_ms = 5);

// Return to RX after a TX burst without reapplying the full RX profile.
bool switchTxToRxFast(uint32_t timeout_ms = 5);

// ---- Register dumps (diagnostics) ------------------------------------------

// Read all config/status registers and PATABLE for diagnostics.
void readAllRegisters(uint8_t cfg_regs[0x2F], uint8_t status_regs[0x0C], uint8_t pa[8]);

// Log registers and PATABLE in hex for quick inspection.
void DumpRegistersHexLogs();

// ---- TX session helpers -----------------------------------------------------

// Pause RX path, prep TX profile, strobe STX, wait until TX is ready.
void beginTxSession();

// Return CC1101 to IDLE and resume RX path after TX session.
void endTxSession();

// ---- Raw OOK TX session (save → configure → TX → restore) ------------------

struct RawTxSession {
  uint32_t prev_freq_hz = 0; // exact FREQ word-derived Hz before TX
  float prev_listen_mhz = 0; // cached listen MHz (model value)
  uint8_t prev_marc5 = MARC_IDLE; // MARC state before TX (RX/IDLE/etc.)
  bool restore_freq = false; // set true if we changed FREQ for TX
};

/**
 * Begin a raw OOK TX session:
 * - snapshot programmed frequency (Hz), cached listen MHz, and MARC state
 * - if tx_freq_hz != 0 and differs, program that freq (+calibrate)
 * - apply TX profile and switch RX→TX (fast path)
 * Leaves the device in MARC_TX (or FSTXON while settling), ready for bit-banging.
 */
bool rawOokBegin(RawTxSession& s, uint32_t tx_freq_hz);

/**
 * End the raw OOK TX session:
 * - stop TX and resume RX path
 * - restore exact programmed frequency if it was changed
 * - reapply listen MHz model and ensure MARC_RX if we were in RX before
 * Radio ends up back in the same functional state as before Begin().
 */
bool rawOokEndRestore(const RawTxSession& s);

// ---- Self-tests (safe, non-blocking) ---------------------------------------

// Smoke-test RX profile application and MARC state sanity.
void selfTest_CommonProfile();

// Exercise TX session begin/end and ensure return to RX readiness.
void selfTest_RxTxTransitions();

// Program a test frequency, verify, and restore previous setting.
void selfTest_Frequency(uint32_t freq_hz);

// ---- TX helpers (shared by tx/gateway) -------------------------------------
// RAII guard: hop to a TX MHz and restore previous listen MHz on destruction.
// Safe no-op on builds without ZradioCC1101.
struct TxFreqGuard {
  float prev = NAN;
  explicit TxFreqGuard(float tx_mhz);
  ~TxFreqGuard();
};

// Debug helper: hold OOK carrier HIGH for 'ms' milliseconds, then restore RX.
// Safe no-op on builds without ZradioCC1101.
void Debug_TxCarrierMs(uint32_t ms);

} // namespace FunkbusTB
