#pragma once

// Public API for building/transmitting Funkbus frames.
// Keep this header dependency-light for faster builds.

#include <Arduino.h> // for String

#include <string> // for std::string

// Theengs/OMG publisher comes from elsewhere
extern bool pub(const char* topic, const char* payload);

// -------------------------------------------------------------------------------------
// Payload model used by ValidatePayload/BuildFirst40Bits
// -------------------------------------------------------------------------------------
struct FunkbusPayload {
  String serial; // 5 hex chars
  String channel; // A/B/C/LS
  uint8_t button; // 1..8
  String action; // ON/OFF
  String duration; // S/L (optional)
};

namespace FunkbusRemote {

// Small JSON command dispatcher for CC1101 debug/controls
bool HandleCc1101Command(const String& json);

/**
 * @brief Handle extended raw TX JSON command.
 * Supports:
 *  (A) Single frame + optional repeats/gaps:
 *      { "ext_raw_v":1, "frequency_hz":868300000, "modulation":"OOK",
 *        "sample_us":1, "timings_us":[ON,OFF,...], "repeats":5, "gaps_us":[...] }
 *
 *  (B) Playlist (multi-frame):
 *      { "ext_raw_v":1, "frequency_hz":868300000, "modulation":"OOK",
 *        "sample_us":1, "frames":[{"timings_us":[...]},...], "gaps_us":[...] }
 *
 * This function only validates & logs for now (no RF TX).
 */
bool HandleExtRawTx(const String& json);

// Validate user JSON and map into FunkbusPayload fields
bool ValidatePayload(const String& jsonText, FunkbusPayload* out, String* error);

// Assemble the first 40 protocol bits from a validated payload
String BuildFirst40Bits(const FunkbusPayload& p, String* error);

// Append SCOM/parity/checksum to 40 bits to get a full 48-bit frame
String Build48BitFrame(const String& first40Bits, uint8_t frameSerial, String* error);

// Zero-alloc variants (callers provide fixed buffers)
bool BuildFirst40Bits_buf(const FunkbusPayload& p, char out[41], std::string* error = nullptr);
bool Build48BitFrame_buf(const char first40[41], uint8_t frameSerial, char out[49], std::string* error = nullptr);

// Build 4 frames (per sequence) and transmit via ESP32 RMT only
void Create_and_TransmitFrames(const String& bits40,
                               const String& channel,
                               uint8_t button,
                               const String& action,
                               const String& duration);

// Hold CC1101 OOK high for ms (sanity check on RF path)
void Debug_TxCarrierMs(uint32_t ms);

} // namespace FunkbusRemote
