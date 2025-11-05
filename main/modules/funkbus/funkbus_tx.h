#pragma once

//
// Funkbus TX — Public API (builder + transmitter)
//
// Lightweight header with minimal deps. Exposes helpers to validate JSON,
// construct Funkbus frames, and transmit via ESP32 RMT.
//
// License: MIT
//

// ---- Includes ---------------------------------------------------------------
#include <Arduino.h> // String

#include <string> // std::string


// ---- OMG publish hook -------------------------------------------------------
// Implemented elsewhere by Theengs/OMG core.
extern bool pub(const char* topic, const char* payload);

// ---- Payload model ----------------------------------------------------------
// Structured result of JSON validation used to build protocol frames.
struct FunkbusPayload {
  String serial; // 5 hex chars
  String channel; // "A" / "B" / "C" / "LS"
  uint8_t button; // 1..8
  String action; // "ON" / "OFF"
  String duration; // "S" / "L" (optional)
};

// ---- API --------------------------------------------------------------------
namespace FunkbusRemote {

// CC1101 diagnostic/tuning command dispatcher (JSON string in, bool out).
bool HandleCc1101Command(const String& json);

// Extended raw TX command (validates & logs; RF TX handled elsewhere).
// Supports:
//   (A) Single frame + optional repeats/gaps:
//     {"ext_raw_v":1,"frequency_hz":868300000,"modulation":"OOK",
//      "sample_us":1,"timings_us":[ON,OFF,...],"repeats":5,"gaps_us":[...]}
//   (B) Playlist (multi-frame):
//     {"ext_raw_v":1,"frequency_hz":868300000,"modulation":"OOK",
//      "sample_us":1,"frames":[{"timings_us":[...]},...],"gaps_us":[...]}
// This function only validates & logs for now (no RF TX).
bool HandleExtRawTx(const String& json);

// Validate user JSON→FunkbusPayload.
// On error, returns false and fills 'error'.
bool ValidatePayload(const String& jsonText, FunkbusPayload* out, String* error);

// Assemble the first 40 protocol bits from a validated payload.
// Returns empty string on error and fills 'error'.
String BuildFirst40Bits(const FunkbusPayload& p, String* error);

// Append SCOM/parity/checksum to 40 bits to get a full 48-bit frame.
String Build48BitFrame(const String& first40Bits, uint8_t frameSerial, String* error);

// Zero-alloc variants (caller-supplied fixed buffers).
bool BuildFirst40Bits_buf(const FunkbusPayload& p,
                          char out[41],
                          std::string* error = nullptr);
bool Build48BitFrame_buf(const char first40[41],
                         uint8_t frameSerial,
                         char out[49],
                         std::string* error = nullptr);

// Build the 4-frame Funkbus sequence and transmit via ESP32 RMT.
void Create_and_TransmitFrames(const String& bits40,
                               const String& channel,
                               uint8_t button,
                               const String& action,
                               const String& duration);

// Simple RF sanity check: hold OOK carrier HIGH for 'ms' milliseconds.
void Debug_TxCarrierMs(uint32_t ms);

} // namespace FunkbusRemote
