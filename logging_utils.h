#ifndef LOGGING_UTIL
#define LOGGING_UTIL

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
#define LOG0(str) cout << __func__ << ": " << str << endl
#else
#define LOG0(str)
#endif

#if defined(LOG_LEVEL_1) || defined(LOG_LEVEL_2) || defined(LOG_LEVEL_3)
#define LOG1(str) cout << __func__ << ": " << str << endl
#else
#define LOG1(str)
#endif

#if defined(LOG_LEVEL_2) || defined(LOG_LEVEL_3)
#define LOG2(str) cout << __func__ << ": " << str << endl
#else
#define LOG2(str)
#endif

#if defined(LOG_LEVEL_3)
#define LOG3(str) cout << __func__ << ": " << str << endl
#else
#define LOG3(str)
#endif

#if defined(NO_LOG_ERROR)
#define LOG_ERROR(str)
#else
#define LOG_ERROR(str) cout << __func__ << ": " << str << endl
#endif

#define EXPECT_TRUE(x, y, z) if(x) {LOG0(y);} else {LOG0(z);}

#endif
