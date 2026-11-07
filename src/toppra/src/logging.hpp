#ifndef TOPPRA_LOGGING_HPP
#define TOPPRA_LOGGING_HPP

#include <ostream>
#include <sstream>
#include <string>

namespace toppra {
namespace log {

/** \brief Severity levels, from least to most verbose.
 *
 *  The logger prints a message when its level is <= the currently
 *  configured level. The default is \ref WARN unless the library was
 *  built with -DTOPPRA_DEBUG_ON, in which case it is \ref DEBUG.
 */
enum Level {
  NONE = 0,
  ERROR = 1,
  WARN = 2,
  INFO = 3,
  DEBUG = 4,
  TRACE = 5,
};

/// \brief Set the global log level. Not thread-safe.
void setLevel(Level level);

/// \brief Get the global log level.
Level getLevel();

/** \brief Whether a message at \p level would currently be printed.
 *
 *  Kept out-of-line on purpose. Exposing the level as a mutable global in
 *  this header and inlining the test made the Seidel solver ~10x slower:
 *  the compiler can no longer prove the level does not alias the solver's
 *  working matrices, so it stops vectorising and starts reloading them.
 *  An opaque call carries no such aliasing hazard.
 *
 *  When this is called in a hot loop, hoist the result into a local bool
 *  and log through TOPPRA_LOG_UNCONDITIONAL instead of testing per
 *  iteration -- see solve_lp2d in solver/seidel.cpp.
 */
bool isEnabled(Level level);

/// \brief Emit a already-formatted message. Prefer the TOPPRA_LOG* macros.

/// \brief Redirect log output. Defaults to std::cout.
///
/// The caller keeps ownership of the stream; it must outlive any
/// subsequent logging call. Pass a std::ofstream to log to a file.
void setStream(std::ostream &stream);

/// \brief Restore the default stream (std::cout).
void resetStream();

/// \brief Prefix each line with a wall-clock timestamp. Off by default.
///
/// Enabling this costs a localtime_r + snprintf per line, which is
/// noticeable when DEBUG logging inside the stage loop.
void setTimestampEnabled(bool enabled);

/// \brief Emit a already-formatted message. Prefer the TOPPRA_LOG* macros.
void write(Level level, const std::string &msg);

/// \brief Single-character tag used in the output prefix, e.g. "[D]".
const char *levelTag(Level level);

/** \brief Format a value with operator<< into a std::string.
 *
 *  For composing a log message in a cold function and handing it to
 *  reportLp2dNote-style helpers, so that the hot function itself never
 *  forms a streaming expression.
 */
template <typename T>
std::string toString(const T &value) {
  std::ostringstream oss;
  oss << value;
  return oss.str();
}

/** \brief Severity-tagged message with no formatting attached.
 *
 *  The pre-formatted string is built by the caller. A std::string typed
 *  as a non-template *variable* carries no destructor into the enclosing
 *  scope, so unlike a local Sink it adds no unwinding state where it is
 *  used -- which is exactly what the solver's inner loops need.
 */
struct Message {
  Level level;
  std::string text;
};

/** \brief Buffers one log line and emits it on destruction.
 *
 *  This exists so the expensive parts of logging -- allocating and
 *  formatting a std::ostringstream, then emitting a std::string -- live in
 *  one translation unit instead of being inlined into every call site.
 *  The constructor, destructor and buffer() are accordingly defined
 *  out-of-line in logging.cpp.
 *
 *  Use through the TOPPRA_LOG* macros rather than directly.
 */
class Sink {
 public:
  explicit Sink(Level level);
  ~Sink();
  Sink(const Sink &) = delete;
  Sink &operator=(const Sink &) = delete;

  template <typename T>
  Sink &operator<<(const T &value) {
    buffer() << value;
    return *this;
  }

  /** \brief Heap-allocate a Sink, to be released with dispose().
   *
   *  Declaring a Sink as a local with a destructor inside a hot function
   *  costs ~16% on the bundled example even when the call is never taken
   *  at runtime: the compiler still emits unwind tables and stack-frame
   *  setup for the local. Going through an opaque allocating call avoids
   *  that. If the streaming expression throws, the Sink leaks -- an
   *  acceptable trade for a logger.
   */
  static Sink *make(Level level) { return new Sink(level); }

  /// \brief Emit and destroy a Sink obtained from make().
  static void dispose(Sink *sink) { delete sink; }

 private:
  std::ostream &buffer();
  Level m_level;
  std::ostringstream *m_oss;
};

}  // namespace log
}  // namespace toppra

/** \brief Log an expression at the given level, if that level is enabled.
 *
 *  The expression is only evaluated when the level is enabled, so it is
 *  safe to write expensive debug output inline.
 *
 *  \code
 *    TOPPRA_LOG(INFO, "gridpoints: " << gridpoints.transpose());
 *  \endcode
 */
#define TOPPRA_LOG(lvl, X)                       \
  do {                                           \
    if (::toppra::log::isEnabled(lvl)) {         \
      TOPPRA_LOG_UNCONDITIONAL(lvl, X);          \
    }                                            \
  } while (0)

#define TOPPRA_LOG_UNCONDITIONAL(lvl, X)                     \
  do {                                                       \
    ::toppra::log::Sink *_toppra_log_sink =                  \
        ::toppra::log::Sink::make((lvl));                    \
    *_toppra_log_sink << X;                                  \
    ::toppra::log::Sink::dispose(_toppra_log_sink);          \
  } while (0)

/** \brief Log a message that has already been formatted.
 *
 *  The level test happens *after* formatting, so prefer TOPPRA_LOG in
 *  ordinary code, where the test is free and skips the formatting too.
 *  Reserve this for places that have already tested the level.
 */
#define TOPPRA_LOG_FORMATTED(lvl, text)    \
  do {                                     \
    if (::toppra::log::isEnabled(lvl))     \
      ::toppra::log::write((lvl), (text)); \
  } while (0)

/** \brief Build a Message early, emit it once the level is known.
 *
 *  Declares a struct containing a std::string, so the compiler emits no
 *  destructor here and no unwinding state in the enclosing function.
 *
 *  \code
 *    TOPPRA_LOG_MSG(m, ::toppra::log::DEBUG, "x = " << x);
 *    for (...) { if (cond) TOPPRA_LOG_SEND(m); }
 *  \endcode
 */
#define TOPPRA_LOG_MSG(name, lvl, X)                     \
  ::toppra::log::Message name{(lvl), ::toppra::log::toString(X)}

/// \brief Emit a Message built by TOPPRA_LOG_MSG.
#define TOPPRA_LOG_SEND(name) \
  TOPPRA_LOG_FORMATTED((name).level, (name).text)

#define TOPPRA_LOG_ERROR(X) TOPPRA_LOG(::toppra::log::ERROR, X)
#define TOPPRA_LOG_WARN(X) TOPPRA_LOG(::toppra::log::WARN, X)
#define TOPPRA_LOG_INFO(X) TOPPRA_LOG(::toppra::log::INFO, X)
#define TOPPRA_LOG_DEBUG(X) TOPPRA_LOG(::toppra::log::DEBUG, X)
#define TOPPRA_LOG_TRACE(X) TOPPRA_LOG(::toppra::log::TRACE, X)

#endif
