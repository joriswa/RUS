#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <sstream>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <thread>

// Compile-time enable/disable logging
#ifndef LOGGING_ENABLED
#define LOGGING_ENABLED 0  // DISABLED for performance
#endif

// Runtime enable/disable for different log levels
#ifndef LOG_LEVEL_DEBUG
#define LOG_LEVEL_DEBUG 1
#endif

#ifndef LOG_LEVEL_INFO
#define LOG_LEVEL_INFO 1
#endif

#ifndef LOG_LEVEL_WARNING
#define LOG_LEVEL_WARNING 1
#endif

#ifndef LOG_LEVEL_ERROR
#define LOG_LEVEL_ERROR 1
#endif

enum class LogLevel { DEBUG, INFO, WARNING, ERROR };

class Logger {
public:
    static Logger& instance() {
        static Logger logger;
        return logger;
    }
    
    class LogStream {
    public:
        LogStream(LogLevel level, bool enabled = true) : level_(level), enabled_(enabled) {
            if (enabled_ && Logger::instance().isLevelEnabled(level)) {
                should_log_ = true;
                addPrefix();
            }
        }
        
        ~LogStream() {
            if (should_log_) {
                std::lock_guard<std::mutex> lock(Logger::instance().mutex_);
                std::cout << buffer_.str() << std::endl;
            }
        }
        
        // Disable copy and move to prevent issues
        LogStream(const LogStream&) = delete;
        LogStream& operator=(const LogStream&) = delete;
        LogStream(LogStream&&) = delete;
        LogStream& operator=(LogStream&&) = delete;
        
        template<typename T>
        LogStream& operator<<(const T& value) {
            if (should_log_) {
                buffer_ << value;
            }
            return *this;
        }
        
    private:
        LogLevel level_;
        bool enabled_;
        bool should_log_ = false;
        std::ostringstream buffer_;
        
        void addPrefix() {
            // Add timestamp
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;
            
            buffer_ << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") 
                   << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
            
            // Add thread ID
            buffer_ << "[T:" << std::this_thread::get_id() << "] ";
            
            // Add log level
            switch (level_) {
                case LogLevel::DEBUG:   buffer_ << "[DEBUG] "; break;
                case LogLevel::INFO:    buffer_ << "[INFO]  "; break;
                case LogLevel::WARNING: buffer_ << "[WARN]  "; break;
                case LogLevel::ERROR:   buffer_ << "[ERROR] "; break;
            }
        }
    };
    
    // Simple LogStream without timestamps for performance-critical sections
    class FastLogStream {
    public:
        FastLogStream(LogLevel level, bool enabled = true) : level_(level), enabled_(enabled) {
            if (enabled_ && Logger::instance().isLevelEnabled(level)) {
                should_log_ = true;
            }
        }
        
        ~FastLogStream() {
            if (should_log_) {
                std::lock_guard<std::mutex> lock(Logger::instance().mutex_);
                std::cout << buffer_.str() << std::endl;
            }
        }
        
        // Disable copy and move
        FastLogStream(const FastLogStream&) = delete;
        FastLogStream& operator=(const FastLogStream&) = delete;
        FastLogStream(FastLogStream&&) = delete;
        FastLogStream& operator=(FastLogStream&&) = delete;
        
        template<typename T>
        FastLogStream& operator<<(const T& value) {
            if (should_log_) {
                buffer_ << value;
            }
            return *this;
        }
        
    private:
        LogLevel level_;
        bool enabled_;
        bool should_log_ = false;
        std::ostringstream buffer_;
    };
    
    bool isLevelEnabled(LogLevel level) const {
        switch (level) {
            case LogLevel::DEBUG:   return debug_enabled_;
            case LogLevel::INFO:    return info_enabled_;
            case LogLevel::WARNING: return warning_enabled_;
            case LogLevel::ERROR:   return error_enabled_;
        }
        return false;
    }
    
    void setDebugEnabled(bool enabled) { debug_enabled_ = enabled; }
    void setInfoEnabled(bool enabled) { info_enabled_ = enabled; }
    void setWarningEnabled(bool enabled) { warning_enabled_ = enabled; }
    void setErrorEnabled(bool enabled) { error_enabled_ = enabled; }
    
    void setAllEnabled(bool enabled) {
        debug_enabled_ = enabled;
        info_enabled_ = enabled; 
        warning_enabled_ = enabled;
        error_enabled_ = enabled;
    }
    
private:
    bool debug_enabled_ = LOG_LEVEL_DEBUG;
    bool info_enabled_ = LOG_LEVEL_INFO;
    bool warning_enabled_ = LOG_LEVEL_WARNING;
    bool error_enabled_ = LOG_LEVEL_ERROR;
    mutable std::mutex mutex_;
};

// Regular logging macros with timestamps
#if LOGGING_ENABLED
#define LOG_DEBUG Logger::LogStream(LogLevel::DEBUG)
#define LOG_INFO Logger::LogStream(LogLevel::INFO)
#define LOG_WARNING Logger::LogStream(LogLevel::WARNING)
#define LOG_ERROR Logger::LogStream(LogLevel::ERROR)
#else
// These will be completely optimized away by the compiler
#define LOG_DEBUG if(0) std::cout
#define LOG_INFO if(0) std::cout
#define LOG_WARNING if(0) std::cout
#define LOG_ERROR if(0) std::cout
#endif

// Fast logging macros without timestamps for performance-critical sections
#if LOGGING_ENABLED
#define LOG_FAST_DEBUG Logger::FastLogStream(LogLevel::DEBUG)
#define LOG_FAST_INFO Logger::FastLogStream(LogLevel::INFO)
#define LOG_FAST_WARNING Logger::FastLogStream(LogLevel::WARNING)
#define LOG_FAST_ERROR Logger::FastLogStream(LogLevel::ERROR)
#else
#define LOG_FAST_DEBUG if(0) std::cout
#define LOG_FAST_INFO if(0) std::cout
#define LOG_FAST_WARNING if(0) std::cout
#define LOG_FAST_ERROR if(0) std::cout
#endif

// Convenience macros for enabling/disabling logging at runtime
#define LOGGING_DISABLE_ALL() Logger::instance().setAllEnabled(false)
#define LOGGING_ENABLE_ALL() Logger::instance().setAllEnabled(true)
#define LOGGING_DISABLE_DEBUG() Logger::instance().setDebugEnabled(false)
#define LOGGING_ENABLE_DEBUG() Logger::instance().setDebugEnabled(true)

#endif // LOGGER_H
