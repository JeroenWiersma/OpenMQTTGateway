#pragma once
#ifndef FUNKBUS_LOG_H
#  define FUNKBUS_LOG_H

//
// Funkbus logging helpers for OpenMQTTGateway
//
// Wraps Theengs logging macros and exposes FB_* aliases used in Funkbus code.
// FB_VLOG compiles to a no-op unless FUNK_LOG_VERBOSE == 1.
//
// License: MIT
//

// ---- Theengs fallbacks ------------------------------------------------------
// If the build doesn't predefine these (e.g., minimal targets), fall back to
// Serial.printf so logs still surface during development.
#  ifndef THEENGS_LOG_TRACE
#    define THEENGS_LOG_TRACE(...) Serial.printf(__VA_ARGS__)
#  endif
#  ifndef THEENGS_LOG_NOTICE
#    define THEENGS_LOG_NOTICE(...) Serial.printf(__VA_ARGS__)
#  endif
#  ifndef THEENGS_LOG_WARNING
#    define THEENGS_LOG_WARNING(...) Serial.printf(__VA_ARGS__)
#  endif
#  ifndef THEENGS_LOG_ERROR
#    define THEENGS_LOG_ERROR(...) Serial.printf(__VA_ARGS__)
#  endif

// ---- Core log levels --------------------------------------------------------
// Keep names short and consistent with the rest of OMG modules.
#  define FB_LOG_N(...) THEENGS_LOG_NOTICE(__VA_ARGS__) // one-line summaries & state changes
#  define FB_LOG_W(...) THEENGS_LOG_WARNING(__VA_ARGS__)
#  define FB_LOG_E(...) THEENGS_LOG_ERROR(__VA_ARGS__)
#  define FB_LOG_T(...) THEENGS_LOG_TRACE(__VA_ARGS__)

// ---- Verbose logs -----------------------------------------------------------
// Compiled only when FUNK_LOG_VERBOSE == 1; always emitted at TRACE.
#  if FUNK_LOG_VERBOSE
#    define FB_VLOG(...) THEENGS_LOG_TRACE(__VA_ARGS__)
#  else
#    define FB_VLOG(...) \
      do {               \
      } while (0)
#  endif

#endif // FUNKBUS_LOG_H
