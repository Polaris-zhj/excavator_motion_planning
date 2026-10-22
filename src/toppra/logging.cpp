#include <toppra/logging.hpp>

#include <chrono>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <mutex>

namespace toppra {
namespace log {

namespace {

/// Default level, preserving the pre-existing compile-time behaviour:
/// debug builds printed both WARN and DEBUG, warn builds printed WARN
/// only, and otherwise the library was silent.
#if defined(TOPPRA_DEBUG_ON)
constexpr Level kDefaultLevel = DEBUG;
#elif defined(TOPPRA_WARN_ON)
constexpr Level kDefaultLevel = WARN;
#else
constexpr Level kDefaultLevel = NONE;
#endif

std::ostream *g_stream = &std::cout;
bool g_timestamp = false;

/// Guards g_stream writes so that concurrent solvers do not interleave
/// partial lines. Deliberately a separate lock from the level read,
/// which stays lock-free on the hot path.
std::mutex g_writeMutex;

void writeTimestamp(std::ostream &os) {
  using namespace std::chrono;
  auto now = system_clock::now();
  auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  std::time_t t = system_clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min,
                tm.tm_sec);
  os << buf << '.' << ms.count() << ' ';
}

}  // namespace

/// Definition of the level declared in the header, where isEnabled() reads
/// it inline.
Level g_level = kDefaultLevel;

void setLevel(Level level) { g_level = level; }

Level getLevel() { return g_level; }

void setStream(std::ostream &stream) { g_stream = &stream; }

void resetStream() { g_stream = &std::cout; }

bool isEnabled(Level level) { return level <= g_level; }

const char *levelTag(Level level) {
  switch (level) {
    case ERROR:
      return "E";
    case WARN:
      return "W";
    case INFO:
      return "I";
    case DEBUG:
      return "D";
    case TRACE:
      return "T";
    default:
      return "?";
  }
}

void write(Level level, const std::string &msg) {
  std::lock_guard<std::mutex> lock(g_writeMutex);
  std::ostream &os = *g_stream;
  os << '[';
  if (g_timestamp) writeTimestamp(os);
  os << levelTag(level) << "] " << msg << '\n';
  os.flush();
}

Sink::Sink(Level level) : m_level(level), m_oss(new std::ostringstream) {}

Sink::~Sink() {
  if (m_oss) {
    write(m_level, m_oss->str());
    delete m_oss;
  }
}

std::ostream &Sink::buffer() { return *m_oss; }

/// \brief Enable or disable the wall-clock timestamp prefix.
///
/// Off by default because it costs a localtime_r + snprintf per line,
/// which is noticeable when DEBUG logging inside the stage loop.
void setTimestampEnabled(bool enabled) { g_timestamp = enabled; }

}  // namespace log
}  // namespace toppra
