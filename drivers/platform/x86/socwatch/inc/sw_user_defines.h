/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2017 - 2018 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1906 Fox Drive,
  Champaign, IL 61820

  BSD LICENSE

  Copyright(c) 2017 - 2018 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef _PW_USER_DEFINES_H_
#define _PW_USER_DEFINES_H_ 1
#ifndef __KERNEL__

#include <stdio.h> // for "FILE *"
#include <errno.h> // for "PW_PERROR"
#include <string.h> // for "PW_PERROR"
#include <fstream> // for "std::ofstream"
#ifndef _WIN32
    #include <sys/time.h> // for "gettimeofday"
#endif // windows
#if defined (__APPLE__)
    #include <mach/clock.h>
    #include <mach/mach.h>
#endif // __APPLE__
#include "sw_types.h"
#include "plugin_environment.h"


/* ***************************************************
 * The following is valid only for userspace code.
 * ***************************************************
 */
/*
 * Default output file name -- the extensions depend on
 * which program is executing: wuwatch output files have
 * a ".sw1" extension, while wudump output files have a
 * ".txt" extension. The extensions are added in by the
 * respective programs i.e. wuwatch/wudump.
 */
#define DEFAULT_WUWATCH_OUTPUT_FILE_NAME "wuwatch_output"
/*
 * Default wuwatch config file name.
 */
#define DEFAULT_WUWATCH_CONFIG_FILE_NAME "wuwatch_config.txt"
/*
 * Default SWA 2.0 output file prefix. Actual file will
 * be called 'SoCWatchOutput.sw2'
 */
#define DEFAULT_SWA_OUTPUT_FILE_PREFIX "SoCWatchOutput"
/*
 * Default SWA 2.0 config file name.
 */
#define DEFAULT_SWA_CONFIG_FILE_NAME "SOCWatchConfig.txt"
/*
 * Default SWA 2.0 config file directory.
 */
#ifdef SWW_MERGE
#define DEFAULT_SWA_CONFIG_FILE_DIRECTORY "configs\\"
#define DEFAULT_OUTPUT_CONFIG_FILE_DIRECTORY "output_configs\\"
#define SW_COLLECTION_FILE_EXTENSION ".etl" // sww
#define SWW_COLLECTION_FILE_PREFIX "_infoSession"
#else
#define DEFAULT_SWA_CONFIG_FILE_DIRECTORY "configs/"
#define DEFAULT_OUTPUT_CONFIG_FILE_DIRECTORY "output_configs/"
#define SW_COLLECTION_FILE_EXTENSION ".sw2" // swa
#endif

/*
 * Default SWA 2.0 polling interval.
 */
#define DEFAULT_SWA_POLLING_INTERVAL_MSECS 100



/*
 * Macro to convert a {major.minor.other} version into a
 * single 32-bit unsigned version number.
 * This is useful when comparing versions, for example.
 * Pretty much identical to the 'KERNEL_VERSION(...)' macro.
 */
//#define WUWATCH_VERSION(major, minor, other) ( (2^16) * (major) + (2^8) * (minor) + (other) )
// #define COLLECTOR_VERSION(major, minor, other) ( (2^16) * (major) + (2^8) * (minor) + (other) )
#define COLLECTOR_VERSION(major, minor, other) ( ((pw_u32_t)(major) << 16) + ((pw_u32_t)(minor) << 8) + (pw_u32_t)(other) )

#define PW_SWA_RESULT_FILE_MAGIC_HEADER (0x8000000000000000ULL)
#define PW_EXTRACT_SWA_MAGIC_HEADER(m) ( (m) & PW_SWA_RESULT_FILE_MAGIC_HEADER )
#define PW_EXTRACT_SWA_VERSION_NUMBER(m) (pw_u32_t)( (m) & ~PW_SWA_RESULT_FILE_MAGIC_HEADER )
#define PW_GET_MAJOR_FROM_COLLECTOR(v) (pw_u8_t) ( (v) >> 16 & 0xff )
#define PW_GET_MINOR_FROM_COLLECTOR(v) (pw_u8_t) ( (v) >> 8 & 0xff )
#define PW_GET_OTHER_FROM_COLLECTOR(v) (pw_u8_t) ( (v) & 0xff )
#define PW_CONVERT_COLLECTOR_VERSION_TO_STRING(v) ([&]{std::stringstream __stream; \
        __stream << (int)PW_GET_MAJOR_FROM_COLLECTOR(v) << "." << (int)PW_GET_MINOR_FROM_COLLECTOR(v) \
        << "." << (int)PW_GET_OTHER_FROM_COLLECTOR(v); return __stream.str();}())
/*
 * Stringify.
 */
#define __STRING(x) #x
#define TO_STRING(x) __STRING(x)
/*
 * Iterate over vectors, deques and ranges.
 */
#define for_each_ptr_in_vector(ptr, vector) for (size_t __curr=0, __end=(vector).size(); __curr!=__end && (ptr=(vector)[__curr]); ++__curr)
/*
 * Find a string 'str' in an array of 'std::string' instances
 */
#define FIND_STRING_IN_ARRAY(ptr, str, array) \
    ([&]() { \
        int s = SW_ARRAY_SIZE(array); \
        bool found = (ptr = std::find((array), (array)+s, (str))) != &((array)[s]); \
        return found; }())

/* **************************************
 * Debugging tools.
 * **************************************
 */
enum pw_log_level_t {
    PW_LOG_LEVEL_FATAL=0, /* Print FATALs */
    PW_LOG_LEVEL_ERROR=1, /* Print ERRORs */
    PW_LOG_LEVEL_WARNING=2, /* Print ERRORs + WARNINGs */
    PW_LOG_LEVEL_DEBUG=3, /* Print ERRORs + WARNINGs + DEBUGs */
    PW_LOG_LEVEL_INFO=4, /* Print ERRORs + WARNINGs + DEBUGs + INFORMATIONALs */
#ifdef SWW_MERGE
    // Windows cannot stringize a macro and then use the result to stringize
    // another macro like g++ can.
    PW_LOG_LEVEL_0 = PW_LOG_LEVEL_FATAL,
    PW_LOG_LEVEL_1 = PW_LOG_LEVEL_ERROR,
    PW_LOG_LEVEL_2 = PW_LOG_LEVEL_WARNING,
    PW_LOG_LEVEL_3 = PW_LOG_LEVEL_DEBUG,
    PW_LOG_LEVEL_4 = PW_LOG_LEVEL_INFO
#endif // SWW_MERGE
};

//Defines to make the macros below work
// #define PW_LOG_LEVEL_FATAL PW_LOG_LEVEL_ERROR
#define s_FATALLogFP g_pluginEnvironment->getErrorLogFP()
#define s_ERRORLogFP g_pluginEnvironment->getErrorLogFP()
#define s_INFOLogFP g_pluginEnvironment->getDebugLogFP()
#define s_WARNINGLogFP g_pluginEnvironment->getDebugLogFP()
#define s_DEBUGLogFP g_pluginEnvironment->getDebugLogFP()
#define PW_FATAL_STREAM std::cerr
#define PW_ERROR_STREAM std::cerr
#define PW_WARNING_STREAM std::cerr
#define PW_INFO_STREAM std::cout
#define PW_DEBUG_STREAM std::cout

#ifdef SWW_MERGE
    // Windows cannot stringize a macro and then use the result to stringize
    // another macro like g++ can. NOTE: The numbers must match the pw_log_level_t enum!
    #define PW_0_STREAM PW_FATAL_STREAM
    #define PW_1_STREAM PW_ERROR_STREAM
    #define PW_2_STREAM PW_WARNING_STREAM
    #define PW_3_STREAM PW_INFO_STREAM
    #define PW_4_STREAM PW_DEBUG_STREAM
#endif // SWW_MERGE

#define PW_LOG_OUTPUT(level, fp, format, ...) do { if (unlikely((level) <= g_pluginEnvironment->getVerbosity())){ fprintf(fp, format, ##__VA_ARGS__); fflush(fp);}} while(0);
#define PW_GET_STREAM_HELPER(level, stream) ( (g_pluginEnvironment->getVerbosity() && (level) <= g_pluginEnvironment->getVerbosity()) ? (stream) : g_pluginEnvironment->getNullStream())
/*
 * Helper macros to print information
 */
#define PW_LOG_FATAL(format, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_FATAL, g_pluginEnvironment->getErrorLogFP(), "FATAL: " format, ##__VA_ARGS__);
#define PW_LOG_ERROR(format, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_ERROR, g_pluginEnvironment->getErrorLogFP(), "ERROR: " format, ##__VA_ARGS__);
#define PW_LOG_FATAL_LINE(format, ...) \
    do { \
        PW_LOG_OUTPUT(PW_LOG_LEVEL_FATAL, g_pluginEnvironment->getErrorLogFP(), "FATAL: " format, ##__VA_ARGS__); \
        PW_LOG_OUTPUT(PW_LOG_LEVEL_DEBUG, g_pluginEnvironment->getErrorLogFP(), "         at %s:%d\n", __FILE__, __LINE__);\
    } while (0);
#define PW_LOG_ERROR_LINE(format, ...) \
    do { \
        PW_LOG_OUTPUT(PW_LOG_LEVEL_ERROR, g_pluginEnvironment->getErrorLogFP(), "ERROR: " format, ##__VA_ARGS__); \
        PW_LOG_OUTPUT(PW_LOG_LEVEL_DEBUG, g_pluginEnvironment->getErrorLogFP(), "         at %s:%d\n", __FILE__, __LINE__);\
    } while (0);
#define PW_LOG_WARNING(format, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_WARNING, g_pluginEnvironment->getDebugLogFP(), "WARNING: " format, ##__VA_ARGS__)
#define PW_LOG_DEBUG(format, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_DEBUG, g_pluginEnvironment->getDebugLogFP(),     "DEBUG: " format, ##__VA_ARGS__)
#define PW_LOG_INFO(format, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_INFO, g_pluginEnvironment->getDebugLogFP(),       "INFO: " format, ##__VA_ARGS__)
#define PW_LOG(type, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_##type, s_##type##LogFP, __VA_ARGS__)
#define PW_LOG_FORCE(format, ...) fprintf(g_pluginEnvironment->getDebugLogFP(), format, ##__VA_ARGS__) /* Force a debug printf */

/*
 * For "conditional" logging i.e. logging if the log prefix
 * matches one of the strings passed in by the user
 */
#define PW_COND_LOG(type, prefix, ...) do { \
    FILE *_fp = s_##type##LogFP; \
    if ((PW_LOG_LEVEL_##type) <= g_pluginEnvironment->getVerbosity() || g_pluginEnvironment->isDebugLoggingEnabledFor(prefix)) { \
        fprintf(_fp, ##__VA_ARGS__); fflush(_fp); \
    } \
} while (0);

#define PW_COND_LOG_ERROR(prefix, format, ...) PW_COND_LOG(ERROR, prefix, "ERROR: " format, ##__VA_ARGS__)
#define PW_COND_LOG_WARNING(prefix, format, ...) PW_COND_LOG(WARNING, prefix, "WARNING: " format, ##__VA_ARGS__)
#define PW_COND_LOG_DEBUG(prefix, format, ...) PW_COND_LOG(DEBUG, prefix, "DEBUG: " format, ##__VA_ARGS__)
#define PW_COND_LOG_INFO(prefix, format, ...) PW_COND_LOG(INFO, prefix, "INFO: " format, ##__VA_ARGS__)
#define PW_COND_LOG_FATAL(prefix, format, ...) PW_COND_LOG(FATAL, prefix, "FATAL: " format, ##__VA_ARGS__)
#define PW_COND_LOG_FORCE(prefix, format, ...) PW_COND_LOG(FATAL, prefix, format, ##__VA_ARGS__)

/*
 * Macro to queue a text message to be sent to writer plugins once
 * they are initialized.
 * TODO: Find a way for the "PW_LOG_TEXT_MESSAGE" macro to either queue the input string, or create
 * a 'TextMessage' instance depending on whether or not writer plugins have been initialized.
 */
#define PW_LOG_TEXT_MESSAGE(str) g_pluginEnvironment->queueMessage(str)
/*
 * Macro to display a text message to the console as well as queue the
 * text message to be sent to writer plugins once they are initialized.
 */
#define PW_LOG_MESSAGE(str) do { \
    PW_LOG_FORCE("%s\n", (str)); \
    PW_LOG_TEXT_MESSAGE(str); \
} while (0)

#define PW_GET_FATAL_STREAM() PW_GET_STREAM_HELPER(PW_LOG_LEVEL_FATAL, std::cerr << "FATAL: ")
#define PW_GET_ERROR_STREAM() PW_GET_STREAM_HELPER(PW_LOG_LEVEL_ERROR, std::cerr << "ERROR: ")
#define PW_GET_WARNING_STREAM() PW_GET_STREAM_HELPER(PW_LOG_LEVEL_WARNING, std::cerr << "WARNING: ")
#define PW_GET_DEBUG_STREAM() PW_GET_STREAM_HELPER(PW_LOG_LEVEL_DEBUG, std::cout << "DEBUG: ")
#define PW_GET_INFO_STREAM() PW_GET_STREAM_HELPER(PW_LOG_LEVEL_INFO, std::cout << "INFO: ")
#define PW_GET_STREAM(type) PW_GET_STREAM_HELPER(PW_LOG_LEVEL_##type, PW_##type##_STREAM)
#define PW_GET_FORCE_STREAM() (std::cerr)

/*
 * Macros to copy and or assert.
 */
#define PW_DEBUG_COPY(level, ...) do { \
    if (unlikely(g_pluginEnvironment->getVerbosity() && (PW_LOG_LEVEL_##level) <= g_pluginEnvironment->getVerbosity())) { \
        std::copy(__VA_ARGS__); \
    } \
} while (0)
#define PW_DEBUG_ASSERT(level, cond, ...) do { \
    if (unlikely(g_pluginEnvironment->getVerbosity() && (PW_LOG_LEVEL_##level) <= g_pluginEnvironment->getVerbosity() && !(cond))) { \
        PW_LOG_ERROR(__VA_ARGS__); \
        PW_ASSERT(false); \
    } \
} while (0)

/*
 * Helper macro to calculate the diff between current and previous value and handle counter wrap around
 */
#define COUNTER_DELTA(next, prev, mask) (((next) >= (prev)) ? ((next)-(prev)) : ((mask)-(prev)+(next)+1))

/*
 * Check if a TSC indicates a counter reset.
 */
#define IS_RESET_TSC(msg)       (g_pluginEnvironment->isSystemReset((msg)->plugin_id, (msg)->metric_id, (msg)->msg_id, (msg)->tsc))

/*
 * Macros to trace function enters and exits.
 */
#if DEVELOPMENT_MODE // Development code; NOT meant for production
    #define PW_TRACE_FUNCTION_ENTER() do { \
        PW_LOG_INFO("Entering function %s\n", __FUNCTION__); \
    } while(0)

    #define PW_TRACE_FUNCTION_EXIT() do { \
        PW_LOG_INFO("Exiting function %s\n", __FUNCTION__); \
    } while(0)

    #define PW_TRACE_FUNCTION_ENTER_VERBOSE() do { \
        PW_LOG_INFO("Entering function %s\n", __PRETTY_FUNCTION__); \
    } while(0)

    #define PW_TRACE_FUNCTION_EXIT_VERBOSE() do { \
        PW_LOG_INFO("Exiting function %s\n", __PRETTY_FUNCTION__); \
    } while(0)
    /*
     * Basic timer-based profiling functions.
     * Every 'ENTER' MUST be accompanied by
     * a corresponding 'EXIT'!
     */
    #define PW_TIME_FUNCTION_ENTER() { \
        PW_LOG_INFO("Entering function %s\n", __PRETTY_FUNCTION__); \
        pwr::Timer __timer(__FUNCTION__);

    #define PW_TIME_FUNCTION_EXIT() \
        PW_LOG_INFO("Exiting function %s\n", __PRETTY_FUNCTION__); \
    }
#else // Production code
    #define PW_TRACE_FUNCTION_ENTER() /* NOP */

    #define PW_TRACE_FUNCTION_EXIT() /* NOP */

    #define PW_TRACE_FUNCTION_ENTER_VERBOSE() /* NOP */

    #define PW_TRACE_FUNCTION_EXIT_VERBOSE() /* NOP */

    #define PW_TIME_FUNCTION_ENTER() { /* NOP */

    #define PW_TIME_FUNCTION_EXIT() } /* NOP */
#endif // DEVELOPMENT_MODE

#define PW_DO_REPORT_FILE_ERROR(msg, path) do { \
    PW_LOG_WARNING(msg, (path).c_str(), strerror(errno)); \
} while(0)

#define PW_TODO_MSG(msg) do { \
    PW_LOG_WARNING("%s functionality is TODO!\n", (msg)); \
} while(0)

#ifdef SWW_MERGE
#define __PRETTY_FUNCTION__ __FUNCTION__
#endif // SWW_MERGE

#define PW_TODO() do { \
    PW_TODO_MSG(__PRETTY_FUNCTION__); \
} while(0)

#define PW_PERROR_LEVEL(msg, level) do { \
    PW_GET_STREAM(level) << msg << ": " << strerror(errno) << std::endl; \
} while(0)

#define PW_PERROR(msg) PW_PERROR_LEVEL(msg, WARNING)

/*
 * A convenience macro to return the number
 * of micro seconds elapsed since the epoch.
 */
#define PW_GET_CURR_TIME_USECS() ((double)pwr::getTimevalNano() / 1000.0)

/*
 * Macros corresponding to the kernel versions of 'likely()'
 * and 'unlikely()' -- GCC SPECIFIC ONLY!
 */
#if defined (__linux__) || defined (__QNX__)
    #define likely(x) __builtin_expect(!!(x), 1)
    #define unlikely(x) __builtin_expect(!!(x), 0)
#elif defined (__APPLE__)
    #define likely(x)   (x)
    #define unlikely(x) (x)
#else // windows
    #define likely(x) (!!(x))
    #define unlikely(x) (!!(x))

    #define __attribute__(a) // ignore __attribute__ macros on Windows
#endif // linux

/* Nanoseconds in a second */
#define NANOSEC_PER_SEC (1000000000ULL)
#define MILLISEC_PER_SEC (1000ULL)
#define SEC_PER_MILLISEC ((double)1.0/MILLISEC_PER_SEC)
#ifndef USEC_PER_SEC /* avoid redefinition in driver build */
#define USEC_PER_SEC (1000000ULL)
#endif
#define SEC_PER_USEC ((double)1.0/USEC_PER_SEC)

#endif // __KERNEL__

#ifdef SWW_MERGE
#define DOS_DIR_SEPARATOR_CHAR '\\'
#define DOS_DIR_SEPARATOR_STR "\\"
#define DOS_CUR_DIR ".\\"
#endif

#define DIR_SEPARATOR_CHAR '/'
#define DIR_SEPARATOR_STR "/"
#define CUR_DIR "./"

#endif // _PW_USER_DEFINES_H_
