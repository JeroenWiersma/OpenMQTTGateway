#pragma once
#ifndef config_Funkbus_h
#  define config_Funkbus_h

// -----------------------------------------------------------------------------
// config_Funkbus.h — user-facing settings only
// Note: Funkbus TX uses ESP32 RMT; non-ESP32 builds must not enable any TX mirror.
// -----------------------------------------------------------------------------

#  include "TheengsCommon.h"

// -----------------------------------------------------------------------------
// 1) PINS — CC1101 wiring and optional RX activity LED
// -----------------------------------------------------------------------------

// CC1101 → MCU: GDO0 (OOK TX data line). Override per board if needed.
#  ifndef FUNKBUS_CC1101_GDO0_MCU
#    define FUNKBUS_CC1101_GDO0_MCU 12
#  endif

// CC1101 → MCU: GDO2 (carrier sense / RX gate). Override per board if needed.
#  ifndef FUNKBUS_CC1101_GDO2_MCU
#    define FUNKBUS_CC1101_GDO2_MCU 27
#  endif

// Optional RX activity LED (blink during receive).
#  ifndef FUNKBUS_LED_RX_ENABLE
#    define FUNKBUS_LED_RX_ENABLE 1 // 1=enable, 0=disable
#  endif
#  ifndef FUNKBUS_LED_RX_GPIO
#    ifdef LED_BUILTIN
#      define FUNKBUS_LED_RX_GPIO LED_BUILTIN // prefer built-in LED when present
#    else
#      define FUNKBUS_LED_RX_GPIO 2 // fallback GPIO
#    endif
#  endif

// -----------------------------------------------------------------------------
// 2) DEFAULT FREQUENCIES — boot-time RX and TX center frequency (MHz)
// -----------------------------------------------------------------------------

#  ifndef FUNKBUS_DEFAULT_LISTEN_MHZ
#    define FUNKBUS_DEFAULT_LISTEN_MHZ 433.42f // runtime config may override
#  endif

#  ifndef FUNKBUS_TX_MHZ
#    define FUNKBUS_TX_MHZ 433.42f
#  endif

// -----------------------------------------------------------------------------
// 3) TX ACTIVITY LED — simple on/off indicator during transmit
// -----------------------------------------------------------------------------

#  ifndef FUNKBUS_LED_TX_ENABLE
#    define FUNKBUS_LED_TX_ENABLE 1 // 1=LED shows TX activity, 0=off
#  endif

#  ifndef FUNKBUS_LED_TX_GPIO
#    ifdef LED_BUILTIN
#      define FUNKBUS_LED_TX_GPIO LED_BUILTIN
#    else
#      define FUNKBUS_LED_TX_GPIO 2 // choose a safe GPIO wired to an LED
#    endif
#  endif

#  ifndef FUNKBUS_LED_TX_ACTIVE_HIGH
#    define FUNKBUS_LED_TX_ACTIVE_HIGH 1 // 1=LED on is HIGH, 0=LED on is LOW
#  endif

// -----------------------------------------------------------------------------
// 4) INTEGRATION HOOKS & TOPICS — public entry points and result topic
// -----------------------------------------------------------------------------

// Gateway lifecycle hooks (implemented elsewhere).
extern void setupFunkbus(); // called at module setup
extern void loopFunkbus(); // called in main loop

// Actuator entry (implemented in actuatorFunkbus.cpp).
extern void XtoFunkbus(const char* topicOri, JsonObject& RFdata); // handle MQTT→Funkbus

// Optional result topic (keep user-facing for integration flexibility).
#  ifndef FUNKBUS_CMD_RESULT_TOPIC
#    define FUNKBUS_CMD_RESULT_TOPIC "/FunkbustoMQTT" // topic for small command acks
#  endif

// -----------------------------------------------------------------------------
// 4b) Long-press repeat count
// -----------------------------------------------------------------------------

// How many times the repeating frame (6 for ON, 7 for OFF) is sent while
// the button is held. This is in addition to the fixed prefix/suffix frames.
// Remote observation suggests a small value; 2 is a good starting point.
#  ifndef FUNKBUS_LONGPRESS_REPEAT
#    define FUNKBUS_LONGPRESS_REPEAT 2
#  endif

// -----------------------------------------------------------------------------
// 5) LOGGING VERBOSITY
// -----------------------------------------------------------------------------

#  ifndef FUNK_LOG_VERBOSE
#    define FUNK_LOG_VERBOSE 0 // 0=lean logs, 1=extra verbose (TRACE)
#  endif

// Legacy aliases (kept for compatibility); map to the single switch.
#  define FUNKBUS_RX_DEBUG  FUNK_LOG_VERBOSE
#  define FUNKBUS_REG_DUMPS 0

#endif // config_Funkbus_h
