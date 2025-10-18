/*
  OpenMQTTGateway - Funkbus RX module

  This gateway module wires Funkbus reception into OMG when the build flag
  ZgatewayFunkbus is enabled. It exposes the usual gateway lifecycle
  entry points (setupFunkbus/loopFunkbus) and, in RX-only builds, a minimal
  XtoFunkbus() shim so main.cpp can always dispatch JSON without link errors.

  Build flags:
    - ZgatewayFunkbus : enable this RX gateway
    - ZactuatorFunkbus: optional; when present, TX lives in actuatorFunkbus.cpp

  Notes:
    - This module manages its own CC1101 init and performance metrics.
    - It does NOT use the core RF helpers from OMG (setupCommonRF/stateRFMeasures).

  License: MIT
*/

#include <Arduino.h>

#include "User_config.h"

#ifdef ZgatewayFunkbus

// --- Project headers --------------------------------------------------------
#  include "config_Funkbus.h"
#  include "modules/funkbus/Funkbus_rx.h"
#  include "modules/funkbus/funkbus_cc1101_toolbox.h"

// --- Gateway lifecycle ------------------------------------------------------

/**
 * @brief Initialize Funkbus RX (hardware + tasks).
 *
 * Called by main.cpp during setup when ZgatewayFunkbus is enabled.
 * Initializes the CC1101 toolbox, performance monitoring, and starts the RX
 * worker (RMT, profiles, listen frequency).
 */
void setupFunkbus() {
  static bool s_inited = false;
  if (s_inited) return;
  s_inited = true;

  // Initialize radio/metrics that were originally in the monolithic module
  FunkbusTB::ensureDriverInitialized();

  // Start the RX path (RMT + state machine, applies listen MHz)
  FunkbusRx::begin();
}

/**
 * @brief Pump the Funkbus RX state machine.
 *
 * Called each loop iteration by main.cpp to process inbound RF activity and
 * publish decoded frames.
 */
void loopFunkbus() {
  FunkbusRx::loop();
}

// --- RX-only JSON shim ------------------------------------------------------
// When TX is NOT built, provide a minimal XtoFunkbus() so main.cpp can always
// call it. This accepts tuning/diagnostic commands and ignores TX telegrams.

#  if !defined(ZactuatorFunkbus)
#    include "modules/funkbus/FunkbusRemote.h"

/**
 * @brief RX-only JSON entry point.
 *
 * Accepts:
 *   - {"cmd": "..."}                 : CC1101 diagnostic/tuning commands
 *
 * Any TX telegram payloads are ignored in RX-only builds.
 */
void XtoFunkbus(const char* /*topic*/, JsonObject& root) {
  // Accept tuning commands; ignore everything else on RX-only builds.
  String json;
  serializeJson(root, json);

  if (root.containsKey("cmd")) {
    (void)FunkbusRemote::HandleCc1101Command(json);
    return;
  }

  // No TX in RX-only builds.
}
#  endif // !ZactuatorFunkbus

#endif // ZgatewayFunkbus
