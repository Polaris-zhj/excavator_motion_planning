#pragma once
#include <g3sinks/LogRotate.h>
#include <unistd.h>

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>
#include <g3log/logmessage.hpp>
#include <g3log/logworker.hpp>
#include <iostream>
#include <memory>

#define INTERNAL_LOG_MESSAGE(level) \
  LogCapture(__FILE__, __LINE__,    \
             static_cast<const char*>(__PRETTY_FUNCTION__), level)

#define INTERNAL_CONTRACT_MESSAGE(boolean_expression)                         \
  LogCapture(__FILE__, __LINE__, __PRETTY_FUNCTION__, g3::internal::CONTRACT, \
             boolean_expression)

// LOG(level) is the API for the stream log
#define LOG3(level)           \
  if (!g3::logLevel(level)) { \
  } else                      \
    INTERNAL_LOG_MESSAGE(level).stream()

// 'Conditional' stream log
#define LOG3_IF(level, boolean_expression) \
  if (true == (boolean_expression))        \
    if (g3::logLevel(level)) INTERNAL_LOG_MESSAGE(level).stream()

// 'Design By Contract' stream API. For Broken Contracts:
//         unit testing: it will throw std::runtime_error when a contract breaks
//         I.R.L : it will exit the application by using fatal signal SIGABRT
#define CHECK3(boolean_expression)   \
  if (false == (boolean_expression)) \
  INTERNAL_CONTRACT_MESSAGE(#boolean_expression).stream()

/** For details please see this
 * REFERENCE: http://www.cppreference.com/wiki/io/c/printf_format
 * \verbatim
 *
  There are different %-codes for different variable types, as well as options
to limit the length of the variables and whatnot. Code Format
    %[flags][width][.precision][length]specifier
 SPECIFIERS
 ----------
 %c character
 %d signed integers
 %i signed integers
 %e scientific notation, with a lowercase “e”
 %E scientific notation, with a uppercase “E”
 %f floating point
 %g use %e or %f, whichever is shorter
 %G use %E or %f, whichever is shorter
 %o octal
 %s a string of characters
 %u unsigned integer
 %x unsigned hexadecimal, with lowercase letters
 %X unsigned hexadecimal, with uppercase letters
 %p a pointer
 %n the argument shall be a pointer to an integer into which is placed the
number of characters written so far

For flags, width, precision etc please see the above references.
// EXAMPLES:
{
   LOGF(INFO, "Characters: %c %c \n", 'a', 65);
   LOGF(INFO, "Decimals: %d %ld\n", 1977, 650000L);      // printing long
   LOGF(INFO, "Preceding with blanks: %10d \n", 1977);
   LOGF(INFO, "Preceding with zeros: %010d \n", 1977);
   LOGF(INFO, "Some different radixes: %d %x %o %#x %#o \n", 100, 100, 100, 100,
100); LOGF(INFO, "floats: %4.2f %+.0e %E \n", 3.1416, 3.1416, 3.1416);
   LOGF(INFO, "Width trick: %*d \n", 5, 10);
   LOGF(INFO, "%s \n", "A string");
   return 0;
}
And here is possible output
:      Characters: a A
:      Decimals: 1977 650000
:      Preceding with blanks:       1977
:      Preceding with zeros: 0000001977
:      Some different radixes: 100 64 144 0x64 0144
:      floats: 3.14 +3e+000 3.141600E+000
:      Width trick:    10
:      A string  \endverbatim */
#define LOGF3(level, printf_like_message, ...) \
  if (!g3::logLevel(level)) {                  \
  } else                                       \
    INTERNAL_LOG_MESSAGE(level).capturef(printf_like_message, ##__VA_ARGS__)

// Conditional log printf syntax
#define LOGF3_IF(level, boolean_expression, printf_like_message, ...) \
  if (true == (boolean_expression))                                   \
    if (g3::logLevel(level))                                          \
  INTERNAL_LOG_MESSAGE(level).capturef(printf_like_message, ##__VA_ARGS__)

// Design By Contract, printf-like API syntax with variadic input parameters.
// Throws std::runtime_eror if contract breaks
#define CHECKF3(boolean_expression, printf_like_message, ...) \
  if (false == (boolean_expression))                          \
  INTERNAL_CONTRACT_MESSAGE(#boolean_expression)              \
      .capturef(printf_like_message, ##__VA_ARGS__)

// Backwards compatible. The same as CHECKF.
// Design By Contract, printf-like API syntax with variadic input parameters.
// Throws std::runtime_eror if contract breaks
#define CHECK3_F(boolean_expression, printf_like_message, ...) \
  if (false == (boolean_expression))                           \
  INTERNAL_CONTRACT_MESSAGE(#boolean_expression)               \
      .capturef(printf_like_message, ##__VA_ARGS__)

#define A3DEBUG LOG3(DEBUG)
#define A3INFO LOG3(INFO)
#define A3WARN LOG3(WARNING)
#define A3ERROR LOG3(WARNING)
#define A3FATAL LOG3(FATAL)

#define A3DEBUGF LOGF3(DEUBG))
#define A3INFOF LOGF3(INFO)
#define A3WARNF LOGF3(WARNING)
#define A3ERRORF LOGF3(WARNING)
#define A3FATALF LOGF3(FATAL)

// LOG_IF
#define A3INFO_IF(cond) LOG3_IF(INFO, cond)
#define A3WARN_IF(cond) LOG3_IF(WARNING, cond)
#define A3ERROR_IF(cond) LOG3_IF(WARNING, cond)
#define A3CHECK(cond) CHECK3(cond)

#define LOG3_EVERY_N(level, n)                                              \
  static std::atomic<int> G3LOG_OCCURRENCES(0), G3LOG_OCCURRENCES_MOD_N(0); \
  ++G3LOG_OCCURRENCES;                                                      \
  if (++G3LOG_OCCURRENCES_MOD_N > n) G3LOG_OCCURRENCES_MOD_N -= n;          \
  if (G3LOG_OCCURRENCES_MOD_N == 1) LOG3(level)

// LOG_EVERY_N
#define A3INFO_EVERY(freq) LOG3_EVERY_N(INFO, freq)
#define A3WARN_EVERY(freq) LOG3_EVERY_N(WARNING, freq)
#define A3ERROR_EVERY(freq) LOG3_EVERY_N(WARNING, freq)

struct CustomSink {
  // Linux xterm color
  // http://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
  enum FG_Color { YELLOW = 33, RED = 31, GREEN = 32, WHITE = 97 };

  FG_Color GetColor(const LEVELS level) const {
    if (level.value == WARNING.value) {
      return YELLOW;
    }
    if (level.value == DEBUG.value) {
      return GREEN;
    }
    if (g3::internal::wasFatal(level)) {
      return RED;
    }

    return WHITE;
  }

  void ReceiveLogMessage(g3::LogMessageMover logEntry) {
    auto level = logEntry.get()._level;
    auto color = GetColor(level);

    std::cout << "\033[" << color << "m" << logEntry.get().toString()
              << "\033[m" << std::endl;
  }
};

class G3LogWorker {
 private:
  std::unique_ptr<g3::LogWorker> worker_;
  std::string log_path_;
  std::string log_name_;

  bool flag_print_to_ter_ = true;
  bool flag_save_to_file_ = true;

 private:
  bool isExist(std::string path) {
    if (access(path.c_str(), 0) != F_OK) {
      return false;
    }
    return true;
  }

 public:
  void Init() {
    worker_ = g3::LogWorker::createLogWorker();
    if (flag_save_to_file_)
    // auto handle = worker_->addDefaultLogger(log_name_, log_path_);
    {
      auto save_handle = worker_->addSink(
          std::make_unique<LogRotate>(log_name_, log_path_), &LogRotate::save);
      const int k400MBInBytes = 100 * 1024 * 1024;
      const int max_count = 100;
      std::future<void> received =
          save_handle->call(&LogRotate::setMaxArchiveLogCount, max_count);
      received = save_handle->call(&LogRotate::setMaxLogSize, k400MBInBytes);
      std::future<bool> rotate = save_handle->call(&LogRotate::rotateLog);
    }
    if (flag_print_to_ter_ || !flag_save_to_file_)
      auto print_handle = worker_->addSink(std::make_unique<CustomSink>(),
                                           &CustomSink::ReceiveLogMessage);

    g3::initializeLogging(worker_.get());
  }
  void SetLogPath(std::string path, std::string name) {
    if (!isExist(path)) {
      flag_save_to_file_ = false;
      return;
    }
    log_path_ = path;
    log_name_ = name;
  }

  void SetPrintToTerminal(bool flag_print_to_ter) {
    flag_print_to_ter_ = flag_print_to_ter;
  }

  void SetSaveToLog(bool flag_save_to_file) {
    if (!flag_save_to_file) flag_save_to_file_ = flag_save_to_file;
  }

 public:
  ~G3LogWorker() { g3::internal::shutDownLogging(); }
  G3LogWorker() = default;
};
