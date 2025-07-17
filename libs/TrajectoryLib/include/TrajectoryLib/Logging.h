#ifndef TRAJECTORYLIB_LOGGING_H
#define TRAJECTORYLIB_LOGGING_H

// High-performance, simplified logging system for TrajectoryLib
#include <iostream>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <sstream>

// Compile-time configuration
#ifndef TRAJECTORY_LOGGING_ENABLED
#define TRAJECTORY_LOGGING_ENABLED 1  // Enable by default
#endif

#ifndef TRAJECTORY_LOGGING_TIMESTAMPS
#define TRAJECTORY_LOGGING_TIMESTAMPS 1  // Enable timestamps by default
#endif

// Log levels
enum class LogLevel {
    DEBUG = 0,
    INFO = 1, 
    WARNING = 2,
    ERROR = 3
};

#if TRAJECTORY_LOGGING_ENABLED

// Thread-safe logger implementation
class SimpleLogger {
public:
    static SimpleLogger& instance() {
        static SimpleLogger logger;
        return logger;
    }
    
    class LogStream {
    private:
        std::ostringstream stream_;
        LogLevel level_;
        bool enabled_;
        
    public:
        LogStream(LogLevel level) : level_(level), enabled_(true) {
            if (enabled_) {
                auto& logger = SimpleLogger::instance();
                std::lock_guard<std::mutex> lock(logger.mutex_);
                
#if TRAJECTORY_LOGGING_TIMESTAMPS
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) % 1000;
                
                stream_ << std::put_time(std::localtime(&time_t), "%H:%M:%S");
                stream_ << '.' << std::setfill('0') << std::setw(3) << ms.count() << " ";
#endif
                
                // Add level prefix with color
                switch (level_) {
                    case LogLevel::DEBUG:   stream_ << "\033[36m[DEBUG]\033[0m "; break;  // Cyan
                    case LogLevel::INFO:    stream_ << "\033[32m[INFO]\033[0m ";  break;  // Green
                    case LogLevel::WARNING: stream_ << "\033[33m[WARN]\033[0m ";  break;  // Yellow
                    case LogLevel::ERROR:   stream_ << "\033[31m[ERROR]\033[0m "; break;  // Red
                }
            }
        }
        
        ~LogStream() {
            if (enabled_) {
                auto& logger = SimpleLogger::instance();
                std::lock_guard<std::mutex> lock(logger.mutex_);
                
                if (level_ >= LogLevel::ERROR) {
                    std::cerr << stream_.str() << std::endl;
                } else {
                    std::cout << stream_.str() << std::endl;
                }
            }
        }
        
        template<typename T>
        LogStream& operator<<(const T& value) {
            if (enabled_) {
                stream_ << value;
            }
            return *this;
        }
    };
    
private:
    std::mutex mutex_;
};

// Fast logging macros (compiled out when disabled)
#define LOG_DEBUG SimpleLogger::LogStream(LogLevel::DEBUG)
#define LOG_INFO SimpleLogger::LogStream(LogLevel::INFO)
#define LOG_WARNING SimpleLogger::LogStream(LogLevel::WARNING)
#define LOG_ERROR SimpleLogger::LogStream(LogLevel::ERROR)

#else

// Disabled logging - zero overhead
#define LOG_DEBUG if(0) std::cout
#define LOG_INFO if(0) std::cout  
#define LOG_WARNING if(0) std::cout
#define LOG_ERROR if(0) std::cout

#endif

// Specialized macros for performance-critical STOMP optimization
#define STOMP_LOG_DEBUG LOG_DEBUG << "[STOMP] "
#define STOMP_LOG_INFO LOG_INFO << "[STOMP] "
#define STOMP_LOG_WARNING LOG_WARNING << "[STOMP] "
#define STOMP_LOG_ERROR LOG_ERROR << "[STOMP] "

// High-frequency logging that can be disabled for performance
#ifndef ENABLE_ITERATION_LOGGING
#define ENABLE_ITERATION_LOGGING 0  // Disabled by default for performance
#endif

#if ENABLE_ITERATION_LOGGING && TRAJECTORY_LOGGING_ENABLED
#define LOG_ITERATION(iter, cost) LOG_DEBUG << "[STOMP] Iteration " << iter << ", cost: " << cost
#define LOG_COLLISION_CHECK(result) LOG_DEBUG << "[COLLISION] Check result: " << (result ? "collision" : "free")
#else
#define LOG_ITERATION(iter, cost) do {} while(0)
#define LOG_COLLISION_CHECK(result) do {} while(0)
#endif

// Component-specific convenience macros
#define ROBOT_LOG_DEBUG LOG_DEBUG << "[ROBOT] "
#define ROBOT_LOG_INFO LOG_INFO << "[ROBOT] "
#define ROBOT_LOG_WARNING LOG_WARNING << "[ROBOT] "
#define ROBOT_LOG_ERROR LOG_ERROR << "[ROBOT] "

#define TRAJECTORY_LOG_DEBUG LOG_DEBUG << "[TRAJECTORY] "
#define TRAJECTORY_LOG_INFO LOG_INFO << "[TRAJECTORY] "
#define TRAJECTORY_LOG_WARNING LOG_WARNING << "[TRAJECTORY] "
#define TRAJECTORY_LOG_ERROR LOG_ERROR << "[TRAJECTORY] "

#define COLLISION_LOG_DEBUG LOG_DEBUG << "[COLLISION] "
#define COLLISION_LOG_INFO LOG_INFO << "[COLLISION] "
#define COLLISION_LOG_WARNING LOG_WARNING << "[COLLISION] "
#define COLLISION_LOG_ERROR LOG_ERROR << "[COLLISION] "

// Runtime logging control
#if TRAJECTORY_LOGGING_ENABLED
#define DISABLE_LOGGING() /* No runtime disabling in this simple implementation */
#define ENABLE_LOGGING() /* No runtime enabling in this simple implementation */
#else
#define DISABLE_LOGGING() do {} while(0)
#define ENABLE_LOGGING() do {} while(0)
#endif

#endif // TRAJECTORYLIB_LOGGING_H