#ifndef USLIB_LOG_CONFIG_H
#define USLIB_LOG_CONFIG_H

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/attributes/timer.hpp>
#include <boost/log/attributes/named_scope.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/core/null_deleter.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace sinks = boost::log::sinks;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

// Define severity levels
enum severity_level
{
    trace,
    debug,
    info,
    warning,
    error,
    fatal
};

// Output operator for severity level
template<typename CharT, typename TraitsT>
std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& strm, severity_level lvl)
{
    static const char* const str[] = {
        "TRACE",
        "DEBUG", 
        "INFO",
        "WARNING",
        "ERROR",
        "FATAL"
    };
    if (static_cast<std::size_t>(lvl) < (sizeof(str) / sizeof(*str)))
        strm << str[lvl];
    else
        strm << static_cast<int>(lvl);
    return strm;
}

class LogConfig
{
public:
    static void init()
    {
        // Add common attributes like timestamp and thread id
        logging::add_common_attributes();
        
        // Create console sink with clean formatting
        auto console_sink = logging::add_console_log(
            std::clog,
            keywords::format = (
                expr::stream
                    << "\033[0;36m" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%H:%M:%S") << "\033[0m "
                    << "[\033[0;32m" << expr::attr<severity_level>("Severity") << "\033[0m] "
                    << expr::smessage
            ),
            keywords::auto_flush = true
        );
        
        // Create file sink for persistent logging
        auto file_sink = logging::add_file_log(
            keywords::file_name = "trajectory_planner_%N.log",
            keywords::rotation_size = 10 * 1024 * 1024, // 10MB
            keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
            keywords::format = (
                expr::stream
                    << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] "
                    << "[" << expr::attr<severity_level>("Severity") << "] "
                    << "[" << expr::attr<attrs::current_thread_id::value_type>("ThreadID") << "] "
                    << expr::smessage
            ),
            keywords::auto_flush = true
        );
        
        // Set default severity level to info
        logging::core::get()->set_filter(
            expr::attr<severity_level>("Severity") >= info
        );
    }
    
    static void setLevel(severity_level level)
    {
        logging::core::get()->set_filter(
            expr::attr<severity_level>("Severity") >= level
        );
    }
    
    static void enableDebug()
    {
        setLevel(debug);
    }
    
    static void enableTrace()
    {
        setLevel(trace);
    }
};

// Global logger instance
extern src::severity_logger<severity_level> g_logger;

// Basic logging macros
#define LOG_TRACE BOOST_LOG_SEV(g_logger, trace)
#define LOG_DEBUG BOOST_LOG_SEV(g_logger, debug)
#define LOG_INFO BOOST_LOG_SEV(g_logger, info)
#define LOG_WARNING BOOST_LOG_SEV(g_logger, warning)
#define LOG_ERROR BOOST_LOG_SEV(g_logger, error)
#define LOG_FATAL BOOST_LOG_SEV(g_logger, fatal)

// General component-based logging macros
// Usage: LOG_INIT("Component") << "message"
#define LOG_INIT(component) LOG_INFO << "[" << component << "] [INIT] "
#define LOG_CONFIG(component) LOG_INFO << "[" << component << "] [CONFIG] "
#define LOG_OPERATION(component) LOG_INFO << "[" << component << "] [OP] "
#define LOG_RESULT(component) LOG_INFO << "[" << component << "] [RESULT] "

// Progress logging with component
#define LOG_PROGRESS(component, current, total) LOG_INFO << "[" << component << "] [PROGRESS] [" << current << "/" << total << "] "

// Performance logging macros
#define LOG_PERF_START(component, timer_name) \
    auto timer_name##_start = std::chrono::high_resolution_clock::now(); \
    LOG_DEBUG << "[" << component << "] [PERF] Starting " << #timer_name

#define LOG_PERF_END(component, timer_name) \
    do { \
        auto timer_name##_end = std::chrono::high_resolution_clock::now(); \
        auto timer_name##_duration = std::chrono::duration_cast<std::chrono::milliseconds>(timer_name##_end - timer_name##_start).count(); \
        LOG_DEBUG << "[" << component << "] [PERF] " << #timer_name << " completed in " << timer_name##_duration << "ms"; \
    } while(0)

// Algorithm lifecycle logging
#define LOG_ALGORITHM_START(component, algorithm) LOG_INFO << "[" << component << "] [ALGO] Starting " << algorithm
#define LOG_ALGORITHM_END(component, algorithm, success) \
    LOG_INFO << "[" << component << "] [ALGO] " << algorithm << " " << (success ? "completed successfully" : "failed")

// Iteration and milestone logging
#define LOG_ITERATION(component, iter, total) LOG_DEBUG << "[" << component << "] [ITER] " << iter << "/" << total
#define LOG_MILESTONE(component, msg) LOG_INFO << "[" << component << "] [MILESTONE] " << msg

// File I/O logging
#define LOG_FILE_READ(component, path) LOG_DEBUG << "[" << component << "] [FILE] Reading: " << path
#define LOG_FILE_WRITE(component, path) LOG_DEBUG << "[" << component << "] [FILE] Writing: " << path
#define LOG_FILE_ERROR(component, path, error) LOG_ERROR << "[" << component << "] [FILE] Error with " << path << ": " << error

// Status and state logging
#define LOG_STATUS(component, status) LOG_INFO << "[" << component << "] [STATUS] " << status
#define LOG_STATE_CHANGE(component, from, to) LOG_DEBUG << "[" << component << "] [STATE] " << from << " -> " << to

// Error and warning logging with context
#define LOG_ERROR_WITH_CONTEXT(component, context, msg) LOG_ERROR << "[" << component << "] [" << context << "] " << msg
#define LOG_WARNING_WITH_CONTEXT(component, context, msg) LOG_WARNING << "[" << component << "] [" << context << "] " << msg

// Convenience macros for common components (optional - can still use the general ones)
#define LOG_TRAJECTORY(msg) LOG_INFO << "[TRAJECTORY] " << msg
#define LOG_PLANNING(msg) LOG_INFO << "[PLANNING] " << msg
#define LOG_MOTION(msg) LOG_INFO << "[MOTION] " << msg
#define LOG_OPTIMIZATION(msg) LOG_DEBUG << "[OPTIMIZATION] " << msg
#define LOG_COLLISION(msg) LOG_WARNING << "[COLLISION] " << msg
#define LOG_KINEMATICS(msg) LOG_DEBUG << "[KINEMATICS] " << msg
#define LOG_GEOMETRY(msg) LOG_DEBUG << "[GEOMETRY] " << msg

// Operation logging macros
#define LOG_OPERATION_START(operation) LOG_INFO << "[OP] Starting " << operation
#define LOG_OPERATION_END(operation, success) LOG_INFO << "[OP] " << operation << " " << (success ? "completed successfully" : "failed")

#endif // USLIB_LOG_CONFIG_H