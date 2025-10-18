#pragma once

// ============================================================================
// Funkbus Rx — Public API (ESP32 RMT-based receiver)
// Role: Exposes begin/loop and TX pause/resume hooks used by OMG/Funkbus stack.
// This header also centralizes tunables and guard macros for RX behavior.
// ============================================================================

#include "User_config.h"
#include "config_Funkbus.h"
#include "funkbus_cc1101_toolbox.h"

#ifdef ARDUINO_ARCH_ESP32
#  include <driver/gpio.h>
#  include <driver/rmt.h>
#  include <freertos/FreeRTOS.h>
#  include <freertos/portmacro.h> // xPortInIsrContext()
#  define FB_IN_ISR() (xPortInIsrContext())
#else
#  define FB_IN_ISR() (false)
#endif

// ============================= Public API =============================
namespace FunkbusRx {

// Initializes the Funkbus RX path (RMT, gating, worker task).
void begin();

// Non-blocking polling step; decodes frames and publishes grouped results.
void loop();

// Pause RX around TX to avoid self-interference while sending.
void pauseForTx();

// Resume RX after TX; restores listen frequency and gating.
void resumeAfterTx();

} // namespace FunkbusRx
