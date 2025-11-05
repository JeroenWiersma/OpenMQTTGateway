//
// OpenMQTTGateway — Funkbus TX actuator
//
// Wires Funkbus transmission into OMG when ZactuatorFunkbus is enabled.
// Provides XtoFunkbus() to handle JSON commands. In TX-only builds (no
// ZgatewayFunkbus), also provides setupFunkbus() for link completeness.
//
// Build flags:
//   - ZactuatorFunkbus : enable this TX actuator
//   - ZgatewayFunkbus  : optional; RX lives in gatewayFunkbus.cpp
//
// Notes:
//   - No <ArduinoJson.h> include here; types come transitively from the app.
//   - RX lifecycle is owned by gatewayFunkbus.cpp.
//
// License: MIT
//

#include <Arduino.h>

#include "User_config.h"


#ifdef ZactuatorFunkbus

// ---- Project headers --------------------------------------------------------
#  include "config_Funkbus.h"
#  include "modules/funkbus/funkbus_cc1101_toolbox.h"
#  include "modules/funkbus/funkbus_log.h"
#  include "modules/funkbus/funkbus_tx.h"

// ---- Init guard -------------------------------------------------------------
// Ensure CC1101 toolbox is initialized exactly once.
static void ensureFbInitOnce() {
  static bool done = false;
  if (!done) {
    FunkbusTB::ensureDriverInitialized();
    done = true;
  }
}

// ---- JSON entry point -------------------------------------------------------
// Dispatch Funkbus-related JSON.
// Routes:
//   - {"cmd": "..."}        : CC1101 diagnostic/tuning commands
//   - {"ext_raw_v": ...}    : extended raw OOK TX (non-Funkbus)
//   - Funkbus telegram      : build and transmit protocol frames
void XtoFunkbus(const char* topic, JsonObject& root) {
  (void)topic;
  ensureFbInitOnce();

  String json;
  serializeJson(root, json);

  const bool has_cmd = root.containsKey("cmd");

  // CC1101 diagnostic/tuning commands.
  if (has_cmd) {
    FB_VLOG(F("[Funkbus] route=CMD" CR));
    if (!FunkbusRemote::HandleCc1101Command(json)) {
      FB_LOG_W(F("[Funkbus] Unknown/failed cmd" CR));
    }
    return;
  }

  // Fast path: transmit non-Funkbus raw signals.
  if (root.containsKey("ext_raw_v")) {
    FB_VLOG(F("[Funkbus] route=EXT_RAW_V" CR));
    if (!FunkbusRemote::HandleExtRawTx(json)) {
      FB_LOG_W(F("[Funkbus] Unknown/failed cmd" CR));
    }
    return; // do not fall through to telegram handler
  }

  // Transmit a Funkbus telegram.
  FB_VLOG(F("[Funkbus] route=TELEGRAM -> ValidatePayload" CR));

  FunkbusPayload p;
  String verr;
  if (!FunkbusRemote::ValidatePayload(json, &p, &verr)) {
    FB_VLOG(F("[Funkbus] ValidatePayload FAILED: %s" CR), verr.c_str());
    return;
  }

  String err;
  const String bits40 = FunkbusRemote::BuildFirst40Bits(p, &err);
  if (!bits40.length()) {
    FB_LOG_W(F("[Funkbus] BuildFirst40Bits failed: %s" CR), err.c_str());
    return;
  }

  const double mhz = FunkbusTB::GetListenMhz();
  const double txmhz = (double)FUNKBUS_TX_MHZ;
  FB_VLOG(F("Transmit frequency: %F -> restore to %F" CR), txmhz, mhz);

  FunkbusRemote::Create_and_TransmitFrames(
      bits40, p.channel, p.button, p.action, p.duration);
}

// ---- TX-only setup (for link completeness) ---------------------------------
// When ZgatewayFunkbus is present, the gateway module provides setupFunkbus().
#  ifndef ZgatewayFunkbus
// Provide setup hook in TX-only builds (no RX).
void setupFunkbus() {
  ensureFbInitOnce();
}
#  endif

#endif // ZactuatorFunkbus
