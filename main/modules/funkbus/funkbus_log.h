#pragma once
#ifndef FUNKBUS_LOG_H
#  define FUNKBUS_LOG_H

// Fallbacks if Theengs macros aren’t pre-provided by the build (already used in rx.cpp)
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

// Always-on levels for behavior you listed:
#  define FB_LOG_N(...) THEENGS_LOG_NOTICE(__VA_ARGS__) // one-line summaries & state changes
#  define FB_LOG_W(...) THEENGS_LOG_WARNING(__VA_ARGS__)
#  define FB_LOG_E(...) THEENGS_LOG_ERROR(__VA_ARGS__)
#  define FB_LOG_T(...) THEENGS_LOG_TRACE(__VA_ARGS__)

// Verbose: compiled in only when FUNK_LOG_VERBOSE==1, and always logged at TRACE.
#  if FUNK_LOG_VERBOSE
#    define FB_VLOG(...) THEENGS_LOG_TRACE(__VA_ARGS__)
#  else
#    define FB_VLOG(...) \
      do {               \
      } while (0)
#  endif

#endif
