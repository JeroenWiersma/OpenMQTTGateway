#pragma once
#ifndef config_Funkbus_h
#  define config_Funkbus_h

// ============================================================================
// config_Funkbus.h — User-facing settings only
// NOTE: Funkbus TX is ESP32-RMT only; non-ESP32 builds must not enable TX mirror.
// ============================================================================

#  include "TheengsCommon.h"

// ============================================================================
// 1) PINS — CC1101 wiring and optional RX activity LED
// ============================================================================

// CC1101 → MCU: GDO0 pin (OOK data line for TX)
#  ifndef FUNKBUS_CC1101_GDO0_MCU
#    define FUNKBUS_CC1101_GDO0_MCU 12 // Default TX data pin; override per board
#  endif

// CC1101 → MCU: GDO2 pin (often “carrier sense” / RX gating)
#  ifndef FUNKBUS_CC1101_GDO2_MCU
#    define FUNKBUS_CC1101_GDO2_MCU 27 // Default CS/GATE pin; override per board
#  endif

// Optional RX activity LED (flash LED during receive)
#  ifndef FUNKBUS_LED_RX_ENABLE
#    define FUNKBUS_LED_RX_ENABLE 1 // 1=enable RX LED, 0=disable
#  endif
#  ifndef FUNKBUS_LED_RX_GPIO
#    ifdef LED_BUILTIN
#      define FUNKBUS_LED_RX_GPIO LED_BUILTIN // Use built-in LED if available
#    else
#      define FUNKBUS_LED_RX_GPIO 2 // Fallback GPIO for RX LED
#    endif
#  endif

// ============================================================================
// 2) DEFAULT FREQUENCIES — boot-time RX and TX frequency
// ============================================================================
#  ifndef FUNKBUS_DEFAULT_LISTEN_MHZ
#    define FUNKBUS_DEFAULT_LISTEN_MHZ 433.42f // MHz; OMG runtime config may override
#  endif

#  ifndef FUNKBUS_TX_MHZ
#    define FUNKBUS_TX_MHZ 433.42f
#  endif

// ============================================================================
// 3 ) Funkbus TX activity LED (simple on/off during transmit)
// ============================================================================
#  ifndef FUNKBUS_LED_TX_ENABLE
#    define FUNKBUS_LED_TX_ENABLE 1 // 1: LED indicates TX activity, 0: off
#  endif

#  ifndef FUNKBUS_LED_TX_GPIO
#    ifdef LED_BUILTIN
#      define FUNKBUS_LED_TX_GPIO LED_BUILTIN
#    else
#      define FUNKBUS_LED_TX_GPIO 2 // choose any safe GPIO tied to an LED
#    endif
#  endif

#  ifndef FUNKBUS_LED_TX_ACTIVE_HIGH
#    define FUNKBUS_LED_TX_ACTIVE_HIGH 1 // 1: LED on = HIGH, 0: LED on = LOW
#  endif

// ============================================================================
// 4) INTEGRATION HOOKS & TOPICS — public entry points and result topic
// ============================================================================

// Gateway life-cycle hooks (implemented elsewhere)
extern void setupFunkbus(); // Called at module setup
extern void loopFunkbus(); // Called in main loop

// Actuator entry (implemented in actuatorFunkbus.cpp)
extern void XtoFunkbus(const char* topicOri, JsonObject& RFdata); // Handle incoming MQTT→Funkbus

// Optional result topic (kept user-facing for integration flexibility)
#  ifndef FUNKBUS_CMD_RESULT_TOPIC
#    define FUNKBUS_CMD_RESULT_TOPIC "/FunkbustoMQTT" // Topic for small command acks
#  endif

// ============================================================================
// 5) LOGGING VERBOSITY
// ============================================================================
#  ifndef FUNK_LOG_VERBOSE
#    define FUNK_LOG_VERBOSE 0 // 0=default lean logs, 1=extra verbose (TRACE)
#  endif
// Keep legacy aliases for compatibility; they map to the single switch.
#  define FUNKBUS_RX_DEBUG  FUNK_LOG_VERBOSE
#  define FUNKBUS_REG_DUMPS 0

#endif // config_Funkbus_h
