#ifndef LOGGING_UTIL
#define LOGGING_UTIL

#include <iostream>

// A logging utility that gives us "log levels." If you want to compile only
// things at level 0, specify -DLOG_LEVEL_0 when compiling. If you want to
// compile only things at level X and below, specify -DLOG_LEVEL_X when
// compiling.
//
// If you want to compile without any logging, just don't specify any of the
// LOG_LEVEL_X flags when compiling (no logging is the default in a sense).
//
// Note that errors, logged with LOG_ERROR, are logged by default. Specify
// -DNO_LOG_ERROR to keep from logging error messages.

#if defined(LOG_LEVEL_0) || defined(LOG_LEVEL_1) || defined(LOG_LEVEL_2) || \
    defined(LOG_LEVEL_3)
#define LOG0(str) std::cout << __func__ << ": " << str << std::endl
#else
#define LOG0(str)
#endif

#if defined(LOG_LEVEL_1) || defined(LOG_LEVEL_2) || defined(LOG_LEVEL_3)
#define LOG1(str) std::cout << __func__ << ": " << str << std::endl
#else
#define LOG1(str)
#endif

#if defined(LOG_LEVEL_2) || defined(LOG_LEVEL_3)
#define LOG2(str) std::cout << __func__ << ": " << str << std::endl
#else
#define LOG2(str)
#endif

#if defined(LOG_LEVEL_3)
#define LOG3(str) std::cout << __func__ << ": " << str << std::endl
#else
#define LOG3(str)
#endif

#if defined(NO_LOG_ERROR)
#define LOG_ERROR(str)
#else
#define LOG_ERROR(str) std::cout << __func__ << ": " << str << std::endl
#endif

// TODO(daddy): Move these into the namespace below and fix everything that
// uses them.
template<typename T>
static std::ostream& cgreen(T s) {
  return std::cout << "\033[1;32m" << s << "\033[0m";
}

template<typename T>
static std::ostream& cred(T s) {
  return std::cout << "\033[1;31m" << s << "\033[0m";
}

namespace logging_utils {
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
static void logging_exception_handler (int sig) {
  void *array[20];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 20);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

static void print_stack_traces() {
  signal(SIGSEGV, logging_exception_handler);
  signal(SIGABRT, logging_exception_handler);
}
} // namespace logging_utils

#define EXPECT_TRUE(x, y, z) \
  if(x) {\
    cgreen(y) << std::endl;\
  } else {\
    cred(z) << std::endl;\
  }

#endif
