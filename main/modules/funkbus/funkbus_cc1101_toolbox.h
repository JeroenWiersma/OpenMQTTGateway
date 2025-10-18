#pragma once

// =====================================================================================
// Funkbus CC1101 Toolbox — Centralized low-level radio helpers
// =====================================================================================

#include <Arduino.h>

#include "User_config.h"
#include "config_Funkbus.h"

// CC1101 crystal frequency (Hz). Default is 26 MHz.
// Keep a legacy path via FUNKBUS_CC1101_XTAL_MHZ if someone defines it,
// but no build flag is required anymore.
#ifndef FUNKBUS_CC1101_XTAL_HZ
#  ifdef FUNKBUS_CC1101_XTAL_MHZ
#    define FUNKBUS_CC1101_XTAL_HZ ((uint32_t)(FUNKBUS_CC1101_XTAL_MHZ * 1000000UL))
#  else
#    define FUNKBUS_CC1101_XTAL_HZ (26000000UL)
#  endif
#endif

#ifndef FUNKBUS_DEFAULT_FREQ_HZ
#  define FUNKBUS_DEFAULT_FREQ_HZ ((uint32_t)(FUNKBUS_DEFAULT_LISTEN_MHZ * 1000000.0f + 0.5f))
#endif

// ============================================================================
// region Declarative CC1101 Profiles (RX/TX bundles)
// ============================================================================
struct CC1101Profile {
  const char* name;

  // Pins & I/O behavior
  uint8_t iocfg0; // e.g., ASYNC for RX, Hi-Z for TX bit-banging
  uint8_t iocfg2; // usually "RX status" on GDO2 in this project

  // Packet/async/whitening selection (PKTCTRL0)
  // We'll OR in 'pktctrl0_value' under 'pktctrl0_mask'.
  uint8_t pktctrl0_mask;
  uint8_t pktctrl0_value;

  // Modem core
  uint8_t mdmcfg2; // modulation + sync mode (0x30 = OOK, no sync)
  uint8_t mdmcfg4_mask; // CHANBW/DRATE_E nibble control (mask == 0 => skip)
  uint8_t mdmcfg4_value;
  uint8_t mdmcfg3; // DRATE_M
  uint8_t agcctrl2;
  uint8_t agcctrl1;
  uint8_t agcctrl0;
  uint8_t deviatn; // 0xFF means "leave as-is" (useful for OOK)
  uint8_t fsctrl1; // 0xFF means "leave as-is" (IF setting)

  // MCSM safety (overflow handling, autocal, etc.) as masked writes
  uint8_t mcsm0_mask, mcsm0_value;
  uint8_t mcsm1_mask, mcsm1_value;

  // Optional PA table (handy for TX OOK shaping); if set_pa=false we skip.
  bool set_pa;
  uint8_t pa_off_level; // usually 0x00
  uint8_t pa_on_level; // e.g., 0xC0
  uint8_t pa_on_index; // which PATABLE slot carries "on" level (1..7)
};

// Canonical profiles for this project
extern const CC1101Profile kProfile_Funkbus_RX;
extern const CC1101Profile kProfile_Funkbus_TX;
// endregion

// ============================================================================
// region ELECHOUSE driver forward-decl
// ============================================================================
class ELECHOUSE_CC1101;
extern ELECHOUSE_CC1101 ELECHOUSE_cc1101;
// endregion

// =====================================================================================
// region Public API (namespace FunkbusTB)
// =====================================================================================
namespace FunkbusTB {

// ---------------------------------------------------------------------------
// region 1) MARC state (5-bit) values
// Role: Enumerate CC1101 MARCSTATE codes used in state/wait helpers.
// ---------------------------------------------------------------------------
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
// endregion

// ---------------------------------------------------------------------------
// region 2) Core state helpers
// ---------------------------------------------------------------------------

// Role: One-time ELECHOUSE driver init and GDO pin mapping.
bool ensureDriverInitialized();

// Role: Poll MARCSTATE until target reached or timeout (ms granularity).
bool waitMarc(uint8_t targetState, uint32_t timeout_ms);

// Role: Force CC1101 to clean IDLE and flush both FIFOs.
void goIdleAndFlush();

// Role: Convert a MARCSTATE[4:0] to a human-readable name (for logs).
const char* marcName(uint8_t marc_state_5bit);

// Role: Read current MARCSTATE[4:0] (lightweight accessor).
uint8_t readMarc5();

// Role: Set CC1101 RF frequency in MHz via driver helper.
void setMHz(float mhz);

// Role: Apply a CC1101Profile atomically (masked writes + optional PATABLE + SCAL).
// Leaves radio in IDLE for the caller to decide SRX/STX.
bool applyProfile(const CC1101Profile& p);
// endregion

// ---------------------------------------------------------------------------
// region 3) Frequency helpers (+ persisted listen MHz)
// ---------------------------------------------------------------------------

// Role: Program carrier frequency (Hz) with optional calibration.
bool programFrequencyHz(uint32_t freq_hz = FUNKBUS_DEFAULT_FREQ_HZ,
                        uint32_t fxosc_hz = FUNKBUS_CC1101_XTAL_HZ,
                        bool do_calibrate = true);

// Role: Read back the currently programmed carrier (Hz).
uint32_t readProgrammedFrequencyHz(uint32_t fxosc_hz = FUNKBUS_CC1101_XTAL_HZ);

// Role: Ensure default Funkbus frequency within tolerance; reprogram if needed.
bool ensureDefaultFrequency(uint32_t tol_hz = 2000,
                            uint32_t fxosc_hz = FUNKBUS_CC1101_XTAL_HZ);

// Role: Adjust IF offset (kHz) via FSCTRL1 (advanced tuning).
void setIfFrequencyOffset(int8_t if_khz);

void EnsureListenFreqInitialized();

void SetListenMhz(float mhz);

// Role: Get current ensured listen-MHz value (reads radio on first use in NEW model).
float GetListenMhz();

// Role: Get current RSSI Db value.
int ReadRssiDbm();
// endregion

// ---------------------------------------------------------------------------
// region 4) PA table helpers
// ---------------------------------------------------------------------------

// Role: Program an OOK-friendly PATABLE and active index (safe defaults provided).
bool ensurePaTableOok(uint8_t offLevel = 0x00, uint8_t onLevel = 0xC0, uint8_t onIndex = 1);
// endregion

// ---------------------------------------------------------------------------
// region 5) Common RX/TX readiness helpers
// ---------------------------------------------------------------------------

// Role: Configure RX profile and ensure radio reaches MARC_RX (with one retry).
bool assureRxReady(uint32_t timeout_ms = 20);

// Role: Configure TX profile, leave radio in IDLE ready for STX from caller.
bool assureTxReady(uint32_t timeout_ms = 20);

// Call when about to transmit and you're already on the listen MHz.
// Returns true on success (in TX MARC state).
bool switchRxToTxFast(uint32_t timeout_ms = 5);

// Call right after a TX burst to get back to RX without reapplying the full RX profile.
bool switchTxToRxFast(uint32_t timeout_ms = 5);

// endregion

// ---------------------------------------------------------------------------
// region 6) Register dumps (read/print diagnostics)
// ---------------------------------------------------------------------------

// Role: Read all config/status registers and PATABLE for diagnostics.
void readAllRegisters(uint8_t cfg_regs[0x2F], uint8_t status_regs[0x0C], uint8_t pa[8]);

// Role: Log registers and PATABLE in hex for quick inspection.
void DumpRegistersHexLogs();
// endregion

// ---------------------------------------------------------------------------
// region 7) TX session helpers (mirror of legacy begin/end)
// ---------------------------------------------------------------------------

// Role: Pause RX path, prep TX profile, strobe STX, wait until TX is ready.
void beginTxSession();

// Role: Return CC1101 to IDLE and resume RX path after TX session.
void endTxSession();
// endregion

// ---------------------------------------------------------------------------
// region 9) Self-tests (safe, non-blocking)
// ---------------------------------------------------------------------------

// Role: Smoke-test RX profile application and MARC state sanity.
void selfTest_CommonProfile();

// Role: Exercise TX session begin/end and ensure return to RX readiness.
void selfTest_RxTxTransitions();

// Role: Program a test frequency, verify, and restore previous setting.
void selfTest_Frequency(uint32_t freq_hz);
// endregion

} // namespace FunkbusTB
// endregion
