#pragma once

//
// Funkbus RX — Public API (ESP32 RMT-based receiver)
//
// Exposes lifecycle hooks and TX pause/resume used by the OMG/Funkbus stack.
// This header also centralizes platform guards and the FB_IN_ISR() helper.
//
// License: MIT
//

// ---- Project headers --------------------------------------------------------
#include "User_config.h"
#include "config_Funkbus.h"
#include "funkbus_cc1101_toolbox.h"

// ---- Platform wiring & ISR helper ------------------------------------------
#ifdef ARDUINO_ARCH_ESP32
#  include <driver/gpio.h>
#  include <driver/rmt.h>
#  include <freertos/FreeRTOS.h>
#  include <freertos/portmacro.h> // xPortInIsrContext()
// True when called from an ISR context (ESP32 only).
#  define FB_IN_ISR() (xPortInIsrContext())
#else
// Non-ESP32 targets: no ISR-aware RX path here.
#  define FB_IN_ISR() (false)
#endif

// ---- Public API -------------------------------------------------------------
namespace FunkbusRx {

// Initialize the RX path (RMT, gating, worker task). Safe to call once.
void begin();

// Non-blocking pump; decodes frames and publishes grouped results.
void loop();

// Pause RX around TX to avoid self-interference while sending.
void pauseForTx();

// Resume RX after TX; restores listen frequency and gating.
void resumeAfterTx();

} // namespace FunkbusRx
