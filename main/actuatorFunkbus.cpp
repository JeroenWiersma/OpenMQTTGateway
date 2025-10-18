/*
  OpenMQTTGateway - Funkbus TX module

  This actuator module wires Funkbus transmission into OMG when the build flag
  ZactuatorFunkbus is enabled. It exposes XtoFunkbus() to handle JSON commands.
  In TX-only builds (no ZgatewayFunkbus), it also provides setupFunkbus() so
  main.cpp links without pulling RX.

  Build flags:
    - ZactuatorFunkbus : enable this TX actuator
    - ZgatewayFunkbus  : optional; when present, RX lives in gatewayFunkbus.cpp

  Notes:
    - No <ArduinoJson.h> include here; types come transitively from the app.
    - This module does not start RX; RX lifecycle is in gatewayFunkbus.cpp.

  License: MIT
*/

#include <Arduino.h>

#include "User_config.h"

#ifdef ZactuatorFunkbus

// --- Project headers --------------------------------------------------------
#  include "config_Funkbus.h"
#  include "modules/funkbus/funkbus_cc1101_toolbox.h"
#  include "modules/funkbus/funkbus_log.h"
#  include "modules/funkbus/funkbus_tx.h"


// --- Init guard -------------------------------------------------------------

/**
 * @brief Ensure CC1101 toolbox and perfmon are initialized exactly once.
 */
static void ensureFbInitOnce() {
  static bool done = false;
  if (!done) {
    FunkbusTB::ensureDriverInitialized();
    done = true;
  }
}

// --- JSON entry point -------------------------------------------------------

/**
 * @brief OMG JSON dispatcher for Funkbus TX.
 *
 * Routes:
 *   - {"cmd": "..."}                 : CC1101 diagnostic/tuning commands
 *   - Funkbus telegram payload       : build & transmit frames
 */
void XtoFunkbus(const char* topic, JsonObject& root) {
  (void)topic;
  ensureFbInitOnce();

  String json;
  serializeJson(root, json);

  const bool has_cmd = root.containsKey("cmd");

  // CC1101 diagnostic/tuning commands
  if (has_cmd) {
    FB_VLOG(F("[Funkbus] route=CMD" CR));
    if (!FunkbusRemote::HandleCc1101Command(json)) {
      FB_LOG_W(F("[Funkbus] Unknown/failed cmd" CR));
    }
    return;
  }

  // Transmit a Funkbus telegram
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

// --- TX-only setup (for link completeness) ---------------------------------
// When ZgatewayFunkbus is present, the gateway module provides setupFunkbus().

#  ifndef ZgatewayFunkbus
/**
 * @brief Provide setup hook in TX-only builds (no RX).
 *
 * Ensures that the CC1101/perf tooling is initialized even if only TX is built.
 */
void setupFunkbus() {
  ensureFbInitOnce();
}
#  endif

#endif // ZactuatorFunkbus
